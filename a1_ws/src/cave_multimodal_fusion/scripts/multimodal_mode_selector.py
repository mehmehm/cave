#!/usr/bin/env python3
import struct
import threading

import cv2
import numpy as np
import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, PointCloud2, PointField
from sensor_msgs import point_cloud2
from std_msgs.msg import Float32, String


class ModeSelector:
    def __init__(self):
        self.bridge = CvBridge()
        self.lock = threading.Lock()
        self.latest_brightness = 0.0
        self.latest_rgb_quality = 0.0
        self.latest_image_received = rospy.Time(0)
        self.mode = "RGB_LIDAR"
        self.dark_count = 0
        self.light_count = 0

        self.dark_threshold = rospy.get_param("~dark_threshold", 0.15)
        self.light_threshold = rospy.get_param("~light_threshold", 0.25)
        self.frames_to_switch = rospy.get_param("~frames_to_switch", 10)
        self.max_image_age = rospy.get_param("~max_image_age", 2.0)
        image_topic = rospy.get_param("~rgb_image_topic", "/rgb_camera/image_raw")
        cloud_topic = rospy.get_param("~fused_cloud_topic", "/cave/fusion/rgb_thermal_lidar_points")
        output_topic = rospy.get_param("~output_topic", "/cave/fusion/active_points")

        self.mode_pub = rospy.Publisher("/cave/perception/mode", String, queue_size=1, latch=True)
        self.rgb_quality_pub = rospy.Publisher("/cave/perception/rgb_quality", Float32, queue_size=1)
        self.thermal_quality_pub = rospy.Publisher("/cave/perception/thermal_quality", Float32, queue_size=1)
        self.cloud_pub = rospy.Publisher(output_topic, PointCloud2, queue_size=1)
        rospy.Subscriber(image_topic, Image, self.image_cb, queue_size=1, buff_size=2 ** 24)
        rospy.Subscriber(cloud_topic, PointCloud2, self.cloud_cb, queue_size=1, buff_size=2 ** 26)

        rospy.loginfo("Multimodal mode selector started (latest-image mode)")
        rospy.loginfo("Input image: %s", image_topic)
        rospy.loginfo("Input cloud: %s", cloud_topic)
        rospy.loginfo("Output cloud: %s", output_topic)

    def image_cb(self, msg):
        try:
            # Always request BGR8 so the encoding is handled consistently.
            image = self.bridge.imgmsg_to_cv2(
                msg,
                desired_encoding="bgr8"
            )

            gray_u8 = cv2.cvtColor(
                image,
                cv2.COLOR_BGR2GRAY
            )

            gray = gray_u8.astype(np.float32) / 255.0

            brightness = float(np.mean(gray))
            contrast = min(
                float(np.std(gray)) / 0.25,
                1.0
            )

            usable = float(
                np.mean(
                    (gray > 0.02)
                    & (gray < 0.98)
                )
            )

            rgb_quality = float(
                np.clip(
                    0.55 * brightness
                    + 0.25 * contrast
                    + 0.20 * usable,
                    0.0,
                    1.0
                )
            )

            with self.lock:
                self.latest_brightness = brightness
                self.latest_rgb_quality = rgb_quality

                # Record when this callback actually received the image.
                self.latest_image_received = rospy.Time.now()

            # Publish directly from the image callback. This lets you test the
            # camera quality even if the fused cloud is delayed or unavailable.
            self.rgb_quality_pub.publish(rgb_quality)

            rospy.loginfo_throttle(
                2.0,
                "RGB image brightness=%.3f quality=%.3f",
                brightness,
                rgb_quality
            )

        except (CvBridgeError, cv2.error, ValueError) as exc:
            rospy.logwarn_throttle(
                5.0,
                "RGB conversion failed: %s",
                str(exc)
            )

    @staticmethod
    def packed_rgb(value):
        if isinstance(value, float):
            return struct.unpack("I", struct.pack("f", value))[0]
        return int(value) & 0xFFFFFFFF

    @staticmethod
    def rgb_float(r, g, b):
        packed = (int(r) << 16) | (int(g) << 8) | int(b)
        return struct.unpack("f", struct.pack("I", packed))[0]

    @staticmethod
    def thermal_colour(value):
        t = int(np.clip(float(value), 0.0, 255.0))
        bgr = cv2.applyColorMap(np.array([[t]], dtype=np.uint8), cv2.COLORMAP_JET)[0, 0]
        return ModeSelector.rgb_float(bgr[2], bgr[1], bgr[0])

    def cloud_cb(self, msg):
        names = [field.name for field in msg.fields]
        required = {"x", "y", "z", "intensity", "rgb", "thermal", "rgb_valid", "thermal_valid"}
        missing = required.difference(names)
        if missing:
            rospy.logerr_throttle(5.0, "Fused cloud missing fields: %s", sorted(missing))
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
            image_age = max(
                0.0,
                (rospy.Time.now() - image_received).to_sec()
            )

            if image_age > self.max_image_age:
                rospy.logwarn_throttle(
                    2.0,
                    "No recent RGB image: callback age %.3f seconds",
                    image_age
                )

                brightness = 0.0
                rgb_quality = 0.0

        rows = list(point_cloud2.read_points(msg, field_names=["x", "y", "z", "intensity", "rgb", "thermal", "rgb_valid", "thermal_valid"], skip_nans=True))
        thermal_valid_fraction = float(np.mean([float(row[7]) > 0.5 for row in rows])) if rows else 0.0
        thermal_quality = thermal_valid_fraction

        if rgb_quality < self.dark_threshold:
            self.dark_count += 1
            self.light_count = 0
        elif rgb_quality >= self.light_threshold:
            self.light_count += 1
            self.dark_count = 0
        else:
            self.dark_count = self.light_count = 0

        if self.dark_count >= self.frames_to_switch:
            self.mode = "THERMAL_LIDAR" if thermal_quality > 0.01 else "LIDAR_ONLY"
        elif self.light_count >= self.frames_to_switch:
            self.mode = "RGB_LIDAR"

        output = []
        for x, y, z, intensity, rgb, thermal, rgb_valid, thermal_valid in rows:
            if self.mode == "RGB_LIDAR" and float(rgb_valid) > 0.5:
                colour, modality, confidence = rgb, 1.0, rgb_quality
            elif self.mode == "THERMAL_LIDAR" and float(thermal_valid) > 0.5:
                colour, modality, confidence = self.thermal_colour(thermal), 2.0, thermal_quality
            else:
                shade = int(np.clip(float(intensity), 0.0, 255.0))
                colour, modality, confidence = self.rgb_float(shade, shade, shade), 0.0, 1.0
            output.append((x, y, z, intensity, colour, modality, confidence))

        fields = [
            PointField("x", 0, PointField.FLOAT32, 1), PointField("y", 4, PointField.FLOAT32, 1),
            PointField("z", 8, PointField.FLOAT32, 1), PointField("intensity", 12, PointField.FLOAT32, 1),
            PointField("rgb", 16, PointField.FLOAT32, 1), PointField("modality", 20, PointField.FLOAT32, 1),
            PointField("confidence", 24, PointField.FLOAT32, 1),
        ]
        self.cloud_pub.publish(point_cloud2.create_cloud(msg.header, fields, output))
        self.mode_pub.publish(self.mode)
        self.rgb_quality_pub.publish(rgb_quality)
        self.thermal_quality_pub.publish(thermal_quality)
        rospy.loginfo_throttle(2.0, "mode=%s brightness=%.3f rgb_quality=%.3f thermal_quality=%.3f image_age=%.3fs points=%d", self.mode, brightness, rgb_quality, thermal_quality, image_age, len(output))


if __name__ == "__main__":
    rospy.init_node("multimodal_mode_selector")
    ModeSelector()
    rospy.spin()
