#!/usr/bin/env python3

import rospy
import cv2
import struct

from sensor_msgs.msg import Image, CameraInfo, PointCloud2, PointField
from std_msgs.msg import Header
from cv_bridge import CvBridge
from image_geometry import PinholeCameraModel
import sensor_msgs.point_cloud2 as pc2

import tf2_ros
import tf2_sensor_msgs.tf2_sensor_msgs


class RGBLidarFusionCloud:
    def __init__(self):
        self.bridge = CvBridge()

        self.rgb_image = None
        self.rgb_model = PinholeCameraModel()
        self.rgb_info_ready = False

        self.rgb_frame = rospy.get_param("~rgb_frame", "a1/rgb_camera_optical_frame")
        self.output_frame = rospy.get_param("~output_frame", "a1/odom")

        self.max_range = rospy.get_param("~max_range", 25.0)
        self.point_skip = rospy.get_param("~point_skip", 3)

        self.tf_buffer = tf2_ros.Buffer(rospy.Duration(10.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        rospy.Subscriber("/a1/rgb_camera/image_raw", Image, self.rgb_image_cb, queue_size=1)
        rospy.Subscriber("/rgb_camera/camera_info", CameraInfo, self.rgb_info_cb, queue_size=1)
        rospy.Subscriber("/a1/velodyne_points", PointCloud2, self.cloud_cb, queue_size=1)

        self.pub = rospy.Publisher("/fusion/rgb_lidar_points", PointCloud2, queue_size=1)

    def rgb_image_cb(self, msg):
        self.rgb_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

    def rgb_info_cb(self, msg):
        self.rgb_model.fromCameraInfo(msg)
        self.rgb_info_ready = True

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

    def sample_rgb(self, point_in_rgb_frame):
        x, y, z = point_in_rgb_frame

        # In optical camera frame, z is forward/depth.
        if z <= 0:
            return None

        if z > self.max_range:
            return None

        u, v = self.rgb_model.project3dToPixel((x, y, z))
        u = int(round(u))
        v = int(round(v))

        h, w = self.rgb_image.shape[:2]

        if 0 <= u < w and 0 <= v < h:
            b, g, r = self.rgb_image[v, u]
            return int(r), int(g), int(b)

        return None

    def cloud_cb(self, cloud_msg):
        if self.rgb_image is None or not self.rgb_info_ready:
            return

        try:
            cloud_rgb_frame = self.transform_cloud(cloud_msg, self.rgb_frame)
            cloud_output_frame = self.transform_cloud(cloud_msg, self.output_frame)

        except Exception as e:
            rospy.logwarn("RGB-LiDAR fusion TF failed: %s", str(e))
            return

        rgb_frame_points = list(
            pc2.read_points(
                cloud_rgb_frame,
                field_names=("x", "y", "z"),
                skip_nans=True
            )
        )

        output_frame_points = list(
            pc2.read_points(
                cloud_output_frame,
                field_names=("x", "y", "z"),
                skip_nans=True
            )
        )

        fused_points = []
        count = min(len(rgb_frame_points), len(output_frame_points))

        for i in range(0, count, self.point_skip):
            rgb_sample = self.sample_rgb(rgb_frame_points[i])

            if rgb_sample is None:
                continue

            x, y, z = output_frame_points[i]
            r, g, b = rgb_sample
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
    rospy.init_node("rgb_lidar_fusion_cloud")
    RGBLidarFusionCloud()
    rospy.spin()