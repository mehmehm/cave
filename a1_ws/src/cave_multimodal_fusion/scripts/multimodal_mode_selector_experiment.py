#!/usr/bin/env python3
"""Select RGB, thermal, or LiDAR colouring for a fused PointCloud2.

operating_mode values:
  ADAPTIVE       brightness-gated RGB/thermal selection with hysteresis

Adaptive thresholds
  The pilot runs in the MIXED cave never exceeded a mean image brightness of
  about 0.20 (median ~0.127, 75th percentile ~0.13, 10th percentile ~0.105),
  so the original 0.15 / 0.20 thresholds kept the selector in THERMAL_LIDAR for
  the entire run.  The defaults below are calibrated to that distribution:
    brightness >= light_threshold (0.130) for frames_to_switch frames -> RGB_LIDAR
    brightness <  dark_threshold  (0.118) for frames_to_switch frames -> THERMAL_LIDAR
  Between the two is a hysteresis band that keeps the current mode.  Report
  these values (and how they were chosen) in the thesis methodology.

  Setting threshold_mode:=auto instead calibrates both thresholds from the
  brightness percentiles seen during the first calibration_seconds of a run
  (defaults: dark = 35th percentile, light = 65th percentile).
  RGB_LIDAR      force RGB where valid, LiDAR greyscale otherwise
  THERMAL_LIDAR  force thermal where valid, LiDAR greyscale otherwise
  LIDAR_ONLY     always use LiDAR intensity greyscale
"""

import struct
import threading
import time

import cv2
import numpy as np
import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, PointCloud2, PointField
from std_msgs.msg import Float32, String


VALID_MODES = {"ADAPTIVE", "RGB_LIDAR", "THERMAL_LIDAR", "LIDAR_ONLY"}

# PointField datatype -> numpy dtype (sensor_msgs/PointField constants).
_POINTFIELD_DTYPES = {
    1: np.int8, 2: np.uint8, 3: np.int16, 4: np.uint16,
    5: np.int32, 6: np.uint32, 7: np.float32, 8: np.float64,
}

OUTPUT_FIELDS = ["x", "y", "z", "intensity", "rgb", "modality", "confidence"]
OUTPUT_DTYPE = np.dtype([(name, np.float32) for name in OUTPUT_FIELDS])


def _packed_rgb(red, green, blue):
    packed = (int(red) << 16) | (int(green) << 8) | int(blue)
    return struct.unpack("f", struct.pack("I", packed))[0]


def _build_thermal_lut():
    bgr = cv2.applyColorMap(
        np.arange(256, dtype=np.uint8).reshape(256, 1), cv2.COLORMAP_JET
    ).reshape(256, 3).astype(np.uint32)
    packed = (bgr[:, 2] << 16) | (bgr[:, 1] << 8) | bgr[:, 0]
    return packed.astype(np.uint32).view(np.float32)


THERMAL_LUT = _build_thermal_lut()
GREY_RGB = np.float32(_packed_rgb(180, 180, 180))


def cloud_to_array(msg, field_names):
    """Read named PointCloud2 fields into a structured array, dropping NaN xyz.

    Equivalent to point_cloud2.read_points(..., skip_nans=True) but without a
    Python-level loop over points.
    """
    lookup = {field.name: field for field in msg.fields}
    endian = ">" if msg.is_bigendian else "<"
    formats, offsets = [], []
    for name in field_names:
        field = lookup[name]
        formats.append(np.dtype(_POINTFIELD_DTYPES[field.datatype]).newbyteorder(endian))
        offsets.append(field.offset)
    dtype = np.dtype({
        "names": list(field_names), "formats": formats,
        "offsets": offsets, "itemsize": msg.point_step,
    })
    count = msg.width * msg.height
    if count == 0:
        return np.zeros(0, dtype=dtype)
    if msg.row_step == msg.width * msg.point_step:
        raw = np.frombuffer(msg.data, dtype=dtype, count=count)
    else:  # padded rows
        buffer = np.frombuffer(msg.data, dtype=np.uint8).reshape(
            msg.height, msg.row_step
        )[:, :msg.width * msg.point_step]
        raw = np.frombuffer(buffer.tobytes(), dtype=dtype, count=count)
    keep = np.ones(count, dtype=bool)
    for axis in ("x", "y", "z"):
        if axis in field_names:
            keep &= np.isfinite(raw[axis])
    return raw[keep]


def array_to_cloud(header, array):
    """Pack an OUTPUT_DTYPE array as an unorganised PointCloud2."""
    cloud = PointCloud2()
    cloud.header = header
    cloud.height = 1
    cloud.width = int(array.shape[0])
    cloud.fields = [
        PointField(name, 4 * index, PointField.FLOAT32, 1)
        for index, name in enumerate(OUTPUT_FIELDS)
    ]
    cloud.is_bigendian = False
    cloud.point_step = OUTPUT_DTYPE.itemsize
    cloud.row_step = cloud.point_step * cloud.width
    cloud.is_dense = True
    cloud.data = np.ascontiguousarray(array).tobytes()
    return cloud


class ModeSelector:
    def __init__(self):
        self.bridge = CvBridge()
        self.lock = threading.Lock()
        self.latest_brightness = 0.0
        self.latest_rgb_quality = 0.0
        self.latest_image_received = rospy.Time(0)
        # Start conservatively until enough image frames establish that the
        # scene is bright. This prevents a dark cave from briefly appearing as
        # RGB solely because the selector has just started.
        self.mode = "THERMAL_LIDAR"
        self.dark_count = 0
        self.light_count = 0

        self.operating_mode = str(
            rospy.get_param("~operating_mode", "ADAPTIVE")
        ).strip().upper()
        if self.operating_mode not in VALID_MODES:
            raise ValueError(
                "operating_mode must be one of: %s" % sorted(VALID_MODES)
            )

        self.dark_threshold = float(rospy.get_param("~dark_threshold", 0.118))
        self.light_threshold = float(rospy.get_param("~light_threshold", 0.130))
        if self.light_threshold <= self.dark_threshold:
            raise ValueError("light_threshold must be greater than dark_threshold")
        self.frames_to_switch = max(
            1, int(rospy.get_param("~frames_to_switch", 8))
        )
        self.threshold_mode = str(
            rospy.get_param("~threshold_mode", "fixed")
        ).strip().lower()
        if self.threshold_mode not in ("fixed", "auto"):
            raise ValueError("threshold_mode must be 'fixed' or 'auto'")
        self.calibration_seconds = float(
            rospy.get_param("~calibration_seconds", 60.0)
        )
        self.dark_percentile = float(rospy.get_param("~dark_percentile", 35.0))
        self.light_percentile = float(rospy.get_param("~light_percentile", 65.0))
        self.calibration_samples = []
        self.calibration_start = None
        self.calibrated = self.threshold_mode == "fixed"
        self.max_image_age = float(rospy.get_param("~max_image_age", 2.0))
        image_topic = rospy.get_param(
            "~rgb_image_topic", "/rgb_camera/image_raw"
        )
        cloud_topic = rospy.get_param(
            "~fused_cloud_topic", "/cave/fusion/rgb_thermal_lidar_points"
        )
        output_topic = rospy.get_param(
            "~output_topic", "/cave/fusion/active_points"
        )

        self.mode_pub = rospy.Publisher(
            "/cave/perception/mode", String, queue_size=1, latch=True
        )
        self.mode_event_pub = rospy.Publisher(
            "/cave/perception/mode_event", String, queue_size=10
        )
        self.brightness_pub = rospy.Publisher(
            "/cave/perception/brightness", Float32, queue_size=1
        )
        self.rgb_quality_pub = rospy.Publisher(
            "/cave/perception/rgb_quality", Float32, queue_size=1
        )
        self.thermal_quality_pub = rospy.Publisher(
            "/cave/perception/thermal_quality", Float32, queue_size=1
        )
        self.rgb_valid_pub = rospy.Publisher(
            "/cave/perception/rgb_valid_fraction", Float32, queue_size=1
        )
        self.thermal_valid_pub = rospy.Publisher(
            "/cave/perception/thermal_valid_fraction", Float32, queue_size=1
        )
        self.selected_valid_pub = rospy.Publisher(
            "/cave/perception/selected_valid_fraction", Float32, queue_size=1
        )
        self.processing_ms_pub = rospy.Publisher(
            "/cave/perception/selector_processing_ms", Float32, queue_size=1
        )
        self.threshold_pub = rospy.Publisher(
            "/cave/perception/brightness_thresholds", String,
            queue_size=1, latch=True
        )
        self.cloud_pub = rospy.Publisher(output_topic, PointCloud2, queue_size=1)

        rospy.Subscriber(
            image_topic, Image, self.image_cb, queue_size=1, buff_size=2 ** 24
        )
        rospy.Subscriber(
            cloud_topic, PointCloud2, self.cloud_cb,
            queue_size=1, buff_size=2 ** 26
        )

        rospy.loginfo("Multimodal mode selector started")
        rospy.loginfo("Operating mode: %s", self.operating_mode)
        rospy.loginfo(
            "Adaptive brightness thresholds: dark < %.3f, bright >= %.3f",
            self.dark_threshold,
            self.light_threshold,
        )
        rospy.loginfo("Threshold mode: %s", self.threshold_mode)
        self.publish_thresholds()
        rospy.loginfo("Input image:   %s", image_topic)
        rospy.loginfo("Input cloud:   %s", cloud_topic)
        rospy.loginfo("Output cloud:  %s", output_topic)

    def image_cb(self, msg):
        try:
            image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            gray_u8 = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
            gray = gray_u8.astype(np.float32) / 255.0

            brightness = float(np.mean(gray))
            contrast = min(float(np.std(gray)) / 0.25, 1.0)
            usable = float(np.mean((gray > 0.02) & (gray < 0.98)))
            rgb_quality = float(np.clip(
                0.55 * brightness + 0.25 * contrast + 0.20 * usable,
                0.0,
                1.0,
            ))

            with self.lock:
                self.latest_brightness = brightness
                self.latest_rgb_quality = rgb_quality
                self.latest_image_received = rospy.Time.now()

            self.brightness_pub.publish(brightness)
            self.rgb_quality_pub.publish(rgb_quality)

            rospy.loginfo_throttle(
                2.0,
                "RGB brightness=%.3f quality=%.3f",
                brightness,
                rgb_quality,
            )
        except (CvBridgeError, cv2.error, ValueError) as exc:
            rospy.logwarn_throttle(5.0, "RGB conversion failed: %s", str(exc))

    @staticmethod
    def rgb_float(red, green, blue):
        return _packed_rgb(red, green, blue)

    @staticmethod
    def thermal_colour(value):
        temperature = int(np.clip(float(value), 0.0, 255.0))
        return float(THERMAL_LUT[temperature])

    def publish_thresholds(self):
        self.threshold_pub.publish(String(data=(
            "mode=%s dark=%.4f light=%.4f frames=%d calibrated=%s"
            % (self.threshold_mode, self.dark_threshold, self.light_threshold,
               self.frames_to_switch, self.calibrated)
        )))

    def calibrate(self, brightness):
        """Collect brightness for calibration_seconds, then set thresholds.

        Returns True once thresholds are final.  Until then the selector keeps
        its conservative starting mode (THERMAL_LIDAR).
        """
        if self.calibrated:
            return True
        now = rospy.Time.now()
        if self.calibration_start is None:
            self.calibration_start = now
        if brightness > 0.0:
            self.calibration_samples.append(brightness)
        if (now - self.calibration_start).to_sec() < self.calibration_seconds:
            return False
        if len(self.calibration_samples) < 10:
            rospy.logwarn("Brightness calibration had too few samples; "
                          "keeping thresholds %.3f / %.3f",
                          self.dark_threshold, self.light_threshold)
        else:
            samples = np.asarray(self.calibration_samples, dtype=np.float64)
            dark = float(np.percentile(samples, self.dark_percentile))
            light = float(np.percentile(samples, self.light_percentile))
            # Keep a minimum hysteresis gap so noise cannot cause flapping.
            if light - dark < 0.005:
                mid = 0.5 * (light + dark)
                dark, light = mid - 0.0025, mid + 0.0025
            self.dark_threshold, self.light_threshold = dark, light
        self.calibrated = True
        self.publish_thresholds()
        rospy.logwarn("Brightness thresholds calibrated: dark < %.4f, "
                      "bright >= %.4f (%d samples)", self.dark_threshold,
                      self.light_threshold, len(self.calibration_samples))
        return True

    def update_adaptive_mode(self, rgb_quality, thermal_quality, brightness):
        previous_mode = self.mode
        if not self.calibrate(brightness):
            return

        # The experiment definition is illumination based: use RGB in a bright
        # scene and thermal in a dark scene.  rgb_quality is still published as
        # a diagnostic, but contrast can make a very dark image score highly,
        # so it must not drive the mode decision.
        if brightness < self.dark_threshold:
            self.dark_count += 1
            self.light_count = 0
        elif brightness >= self.light_threshold:
            self.light_count += 1
            self.dark_count = 0
        else:
            # Hysteresis band: retain the current mode and reset confirmation.
            self.dark_count = 0
            self.light_count = 0

        if self.dark_count >= self.frames_to_switch:
            self.mode = (
                "THERMAL_LIDAR" if thermal_quality > 0.01 else "LIDAR_ONLY"
            )
            self.dark_count = 0
        elif self.light_count >= self.frames_to_switch:
            self.mode = "RGB_LIDAR"
            self.light_count = 0

        if self.mode != previous_mode:
            event = (
                "%.6f %s->%s brightness=%.4f rgb_quality=%.4f "
                "thermal_quality=%.4f"
                % (
                    rospy.Time.now().to_sec(),
                    previous_mode,
                    self.mode,
                    brightness,
                    rgb_quality,
                    thermal_quality,
                )
            )
            self.mode_event_pub.publish(event)
            rospy.logwarn("MODE SWITCH: %s", event)

    def cloud_cb(self, msg):
        start_time = time.perf_counter()
        names = {field.name for field in msg.fields}
        required = {
            "x", "y", "z", "intensity", "rgb", "thermal",
            "rgb_valid", "thermal_valid",
        }
        missing = required.difference(names)
        if missing:
            rospy.logerr_throttle(
                5.0, "Fused cloud missing fields: %s", sorted(missing)
            )
            return

        with self.lock:
            brightness = self.latest_brightness
            rgb_quality = self.latest_rgb_quality
            image_received = self.latest_image_received

        if image_received == rospy.Time(0):
            image_age = float("inf")
            brightness = 0.0
            rgb_quality = 0.0
        else:
            image_age = max(0.0, (rospy.Time.now() - image_received).to_sec())
            if image_age > self.max_image_age:
                rospy.logwarn_throttle(
                    2.0, "No recent RGB image: callback age %.3f seconds", image_age
                )
                brightness = 0.0
                rgb_quality = 0.0

        points = cloud_to_array(msg, [
            "x", "y", "z", "intensity", "rgb", "thermal",
            "rgb_valid", "thermal_valid",
        ])
        point_count = int(points.shape[0])
        rgb_valid = points["rgb_valid"] > 0.5
        thermal_valid = points["thermal_valid"] > 0.5
        if point_count:
            rgb_valid_fraction = float(np.mean(rgb_valid))
            thermal_valid_fraction = float(np.mean(thermal_valid))
        else:
            rgb_valid_fraction = 0.0
            thermal_valid_fraction = 0.0

        thermal_quality = thermal_valid_fraction

        if self.operating_mode == "ADAPTIVE":
            self.update_adaptive_mode(rgb_quality, thermal_quality, brightness)
        else:
            self.mode = self.operating_mode

        # Vectorised colouring: identical output to the former per-point loop
        # (grey LiDAR shade unless the active modality is valid for a point),
        # but ~50-100x faster than calling cv2.applyColorMap once per point.
        output = np.zeros(point_count, dtype=OUTPUT_DTYPE)
        for name in ("x", "y", "z", "intensity"):
            output[name] = points[name]
        output["rgb"] = GREY_RGB
        output["modality"] = 0.0
        output["confidence"] = 1.0

        if self.mode == "RGB_LIDAR":
            selected = rgb_valid
            output["rgb"][selected] = points["rgb"][selected]
            output["modality"][selected] = 1.0
            output["confidence"][selected] = rgb_quality
        elif self.mode == "THERMAL_LIDAR":
            selected = thermal_valid
            index = np.clip(
                np.nan_to_num(points["thermal"][selected], nan=0.0), 0.0, 255.0
            ).astype(np.uint8)
            output["rgb"][selected] = THERMAL_LUT[index]
            output["modality"][selected] = 2.0
            output["confidence"][selected] = thermal_quality
        else:
            selected = np.zeros(point_count, dtype=bool)

        selected_valid_count = int(np.count_nonzero(selected))
        selected_valid_fraction = (
            float(selected_valid_count) / point_count if point_count else 0.0
        )

        self.cloud_pub.publish(array_to_cloud(msg.header, output))

        processing_ms = 1000.0 * (time.perf_counter() - start_time)
        self.mode_pub.publish(self.mode)
        self.brightness_pub.publish(brightness)
        self.rgb_quality_pub.publish(rgb_quality)
        self.thermal_quality_pub.publish(thermal_quality)
        self.rgb_valid_pub.publish(rgb_valid_fraction)
        self.thermal_valid_pub.publish(thermal_valid_fraction)
        self.selected_valid_pub.publish(selected_valid_fraction)
        self.processing_ms_pub.publish(processing_ms)

        rospy.loginfo_throttle(
            2.0,
            "mode=%s requested=%s brightness=%.3f rgb_q=%.3f thermal_q=%.3f "
            "selected_valid=%.1f%% image_age=%.3fs points=%d processing=%.1fms",
            self.mode,
            self.operating_mode,
            brightness,
            rgb_quality,
            thermal_quality,
            100.0 * selected_valid_fraction,
            image_age,
            point_count,
            processing_ms,
        )


if __name__ == "__main__":
    rospy.init_node("multimodal_mode_selector")
    ModeSelector()
    rospy.spin()
