#!/usr/bin/env python3

import rospy
import cv2
import struct
import numpy as np

from sensor_msgs.msg import Image, CameraInfo, PointCloud2, PointField
from std_msgs.msg import Header
from cv_bridge import CvBridge
from image_geometry import PinholeCameraModel
import sensor_msgs.point_cloud2 as pc2

import tf2_ros
import tf2_sensor_msgs.tf2_sensor_msgs


class AdaptiveRGBThermalLidarFusion:
    def __init__(self):
        self.bridge = CvBridge()

        self.rgb_image = None
        self.thermal_image = None

        self.rgb_model = PinholeCameraModel()
        self.thermal_model = PinholeCameraModel()

        self.rgb_info_ready = False
        self.thermal_info_ready = False

        # Change these topic names if your topics are different.
        self.rgb_image_topic = rospy.get_param("~rgb_image_topic", "/camera/rgb/image_raw")
        self.rgb_info_topic = rospy.get_param("~rgb_info_topic", "/camera/rgb/camera_info")

        self.thermal_image_topic = rospy.get_param("~thermal_image_topic", "/thermal_camera/image_raw")
        self.thermal_info_topic = rospy.get_param("~thermal_info_topic", "/thermal_camera/camera_info")

        self.lidar_topic = rospy.get_param("~lidar_topic", "/velodyne_points")

        self.rgb_frame = rospy.get_param("~rgb_frame", "rgb_camera_optical_frame")
        self.thermal_frame = rospy.get_param("~thermal_frame", "thermal_camera_optical_frame")
        self.output_frame = rospy.get_param("~output_frame", "odom")

        self.dark_threshold = rospy.get_param("~dark_threshold", 45)
        self.point_skip = rospy.get_param("~point_skip", 3)
        self.max_range = rospy.get_param("~max_range", 25.0)

        self.tf_buffer = tf2_ros.Buffer(rospy.Duration(10.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        rospy.Subscriber(self.rgb_image_topic, Image, self.rgb_image_cb, queue_size=1)
        rospy.Subscriber(self.rgb_info_topic, CameraInfo, self.rgb_info_cb, queue_size=1)

        rospy.Subscriber(self.thermal_image_topic, Image, self.thermal_image_cb, queue_size=1)
        rospy.Subscriber(self.thermal_info_topic, CameraInfo, self.thermal_info_cb, queue_size=1)

        rospy.Subscriber(self.lidar_topic, PointCloud2, self.cloud_cb, queue_size=1, buff_size=2**24)

        self.pub = rospy.Publisher("/fusion/adaptive_lidar_points", PointCloud2, queue_size=1)

        rospy.loginfo("Adaptive RGB/Thermal LiDAR fusion started.")
        rospy.loginfo("RGB image topic: %s", self.rgb_image_topic)
        rospy.loginfo("Thermal image topic: %s", self.thermal_image_topic)
        rospy.loginfo("LiDAR topic: %s", self.lidar_topic)

    def rgb_image_cb(self, msg):
        self.rgb_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

    def thermal_image_cb(self, msg):
        try:
            self.thermal_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="mono8")
        except Exception:
            thermal_bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            self.thermal_image = cv2.cvtColor(thermal_bgr, cv2.COLOR_BGR2GRAY)

    def rgb_info_cb(self, msg):
        self.rgb_model.fromCameraInfo(msg)
        self.rgb_info_ready = True

    def thermal_info_cb(self, msg):
        self.thermal_model.fromCameraInfo(msg)
        self.thermal_info_ready = True

    def pack_rgb_float(self, r, g, b):
        rgb_uint32 = (int(r) << 16) | (int(g) << 8) | int(b)
        return struct.unpack("f", struct.pack("I", rgb_uint32))[0]

    def transform_cloud(self, cloud_msg, target_frame):
        transform = self.tf_buffer.lookup_transform(
            target_frame,
            cloud_msg.header.frame_id,
            cloud_msg.header.stamp,
            rospy.Duration(0.2)
        )

        return tf2_sensor_msgs.tf2_sensor_msgs.do_transform_cloud(
            cloud_msg,
            transform
        )

    def sample_rgb(self, point):
        x, y, z = point

        if z <= 0 or z > self.max_range:
            return None

        u, v = self.rgb_model.project3dToPixel((x, y, z))
        u = int(round(u))
        v = int(round(v))

        h, w = self.rgb_image.shape[:2]

        if 0 <= u < w and 0 <= v < h:
            b, g, r = self.rgb_image[v, u]
            brightness = 0.299 * r + 0.587 * g + 0.114 * b
            return int(r), int(g), int(b), brightness

        return None

    def thermal_to_colour(self, thermal_value):
        """
        Converts thermal intensity into a visible colour.
        Low thermal = blue/purple.
        High thermal = yellow/red.
        """
        t = max(0, min(255, int(thermal_value)))

        # Simple heatmap.
        if t < 85:
            r = 0
            g = int(3 * t)
            b = 255
        elif t < 170:
            r = int(3 * (t - 85))
            g = 255
            b = int(255 - 3 * (t - 85))
        else:
            r = 255
            g = int(255 - 3 * (t - 170))
            b = 0

        r = max(0, min(255, r))
        g = max(0, min(255, g))
        b = max(0, min(255, b))

        return r, g, b

    def sample_thermal(self, point):
        x, y, z = point

        if z <= 0 or z > self.max_range:
            return None

        u, v = self.thermal_model.project3dToPixel((x, y, z))
        u = int(round(u))
        v = int(round(v))

        h, w = self.thermal_image.shape[:2]

        if 0 <= u < w and 0 <= v < h:
            thermal_value = int(self.thermal_image[v, u])
            return self.thermal_to_colour(thermal_value)

        return None

    def cloud_cb(self, cloud_msg):
        if self.rgb_image is None or self.thermal_image is None:
            return

        if not self.rgb_info_ready or not self.thermal_info_ready:
            return

        try:
            cloud_rgb_frame = self.transform_cloud(cloud_msg, self.rgb_frame)
            cloud_thermal_frame = self.transform_cloud(cloud_msg, self.thermal_frame)
            cloud_output_frame = self.transform_cloud(cloud_msg, self.output_frame)

        except Exception as e:
            rospy.logwarn_throttle(1.0, "Adaptive fusion TF failed: %s", str(e))
            return

        rgb_points = list(pc2.read_points(cloud_rgb_frame, field_names=("x", "y", "z"), skip_nans=True))
        thermal_points = list(pc2.read_points(cloud_thermal_frame, field_names=("x", "y", "z"), skip_nans=True))
        output_points = list(pc2.read_points(cloud_output_frame, field_names=("x", "y", "z"), skip_nans=True))

        fused_points = []

        count = min(len(rgb_points), len(thermal_points), len(output_points))

        for i in range(0, count, self.point_skip):
            rgb_sample = self.sample_rgb(rgb_points[i])
            thermal_sample = self.sample_thermal(thermal_points[i])

            if rgb_sample is None and thermal_sample is None:
                continue

            x, y, z = output_points[i]

            # Use RGB when it is bright enough.
            # Use thermal colour when RGB is too dark.
            if rgb_sample is not None:
                r, g, b, brightness = rgb_sample

                if brightness < self.dark_threshold and thermal_sample is not None:
                    r, g, b = thermal_sample

            elif thermal_sample is not None:
                r, g, b = thermal_sample

            else:
                continue

            rgb_float = self.pack_rgb_float(r, g, b)
            fused_points.append([x, y, z, rgb_float])

        header = Header()
        header.stamp = cloud_msg.header.stamp
        header.frame_id = self.output_frame

        fields = [
            PointField("x", 0, PointField.FLOAT32, 1),
            PointField("y", 4, PointField.FLOAT32, 1),
            PointField("z", 8, PointField.FLOAT32, 1),
            PointField("rgb", 12, PointField.FLOAT32, 1),
        ]

        fused_cloud = pc2.create_cloud(header, fields, fused_points)
        self.pub.publish(fused_cloud)


if __name__ == "__main__":
    rospy.init_node("adaptive_rgb_thermal_lidar_points")
    AdaptiveRGBThermalLidarFusion()
    rospy.spin()