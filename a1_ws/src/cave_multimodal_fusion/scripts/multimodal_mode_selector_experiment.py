#!/usr/bin/env python3
"""Select RGB, thermal, or LiDAR colouring for a fused PointCloud2.

operating_mode values:
  ADAPTIVE       brightness-gated RGB/thermal selection with hysteresis
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
from sensor_msgs import point_cloud2
from sensor_msgs.msg import Image, PointCloud2, PointField
from std_msgs.msg import Float32, String


VALID_MODES = {"ADAPTIVE", "RGB_LIDAR", "THERMAL_LIDAR", "LIDAR_ONLY"}


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

        self.dark_threshold = float(rospy.get_param("~dark_threshold", 0.15))
        self.light_threshold = float(rospy.get_param("~light_threshold", 0.20))
        self.frames_to_switch = max(
            1, int(rospy.get_param("~frames_to_switch", 5))
        )
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
        packed = (int(red) << 16) | (int(green) << 8) | int(blue)
        return struct.unpack("f", struct.pack("I", packed))[0]

    @staticmethod
    def thermal_colour(value):
        temperature = int(np.clip(float(value), 0.0, 255.0))
        bgr = cv2.applyColorMap(
            np.array([[temperature]], dtype=np.uint8), cv2.COLORMAP_JET
        )[0, 0]
        return ModeSelector.rgb_float(bgr[2], bgr[1], bgr[0])

    def update_adaptive_mode(self, rgb_quality, thermal_quality, brightness):
        previous_mode = self.mode

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

        rows = list(point_cloud2.read_points(
            msg,
            field_names=[
                "x", "y", "z", "intensity", "rgb", "thermal",
                "rgb_valid", "thermal_valid",
            ],
            skip_nans=True,
        ))

        point_count = len(rows)
        if point_count:
            rgb_valid_fraction = float(np.mean([
                float(row[6]) > 0.5 for row in rows
            ]))
            thermal_valid_fraction = float(np.mean([
                float(row[7]) > 0.5 for row in rows
            ]))
        else:
            rgb_valid_fraction = 0.0
            thermal_valid_fraction = 0.0

        thermal_quality = thermal_valid_fraction

        if self.operating_mode == "ADAPTIVE":
            self.update_adaptive_mode(rgb_quality, thermal_quality, brightness)
        else:
            self.mode = self.operating_mode

        output = []
        selected_valid_count = 0
        for x, y, z, intensity, rgb, thermal, rgb_valid, thermal_valid in rows:
            if self.mode == "RGB_LIDAR" and float(rgb_valid) > 0.5:
                colour = rgb
                modality = 1.0
                confidence = rgb_quality
                selected_valid_count += 1
            elif self.mode == "THERMAL_LIDAR" and float(thermal_valid) > 0.5:
                colour = self.thermal_colour(thermal)
                modality = 2.0
                confidence = thermal_quality
                selected_valid_count += 1
            else:
                shade = 180
                colour = self.rgb_float(shade, shade, shade)
                modality = 0.0
                confidence = 1.0
            output.append((x, y, z, intensity, colour, modality, confidence))

        selected_valid_fraction = (
            float(selected_valid_count) / point_count if point_count else 0.0
        )

        fields = [
            PointField("x", 0, PointField.FLOAT32, 1),
            PointField("y", 4, PointField.FLOAT32, 1),
            PointField("z", 8, PointField.FLOAT32, 1),
            PointField("intensity", 12, PointField.FLOAT32, 1),
            PointField("rgb", 16, PointField.FLOAT32, 1),
            PointField("modality", 20, PointField.FLOAT32, 1),
            PointField("confidence", 24, PointField.FLOAT32, 1),
        ]
        self.cloud_pub.publish(point_cloud2.create_cloud(msg.header, fields, output))

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
