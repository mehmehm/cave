#!/usr/bin/env python3

import rospy
import cv2
import numpy as np

from sensor_msgs.msg import Image, CameraInfo, PointCloud2
from cv_bridge import CvBridge
from image_geometry import PinholeCameraModel
import sensor_msgs.point_cloud2 as pc2

import tf2_ros
import tf2_sensor_msgs.tf2_sensor_msgs


class LidarToRGBProjection:
    def __init__(self):
        self.bridge = CvBridge()

        self.camera_model = PinholeCameraModel()
        self.camera_info_received = False
        self.latest_image = None

        self.target_frame = "a1/thermal_camera_optical_frame"

        self.tf_buffer = tf2_ros.Buffer(rospy.Duration(10.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        rospy.Subscriber("/thermal_camera/camera_info", CameraInfo, self.camera_info_callback, queue_size=1)
        rospy.Subscriber("/thermal_camera/image_raw", Image, self.image_callback, queue_size=1)
        rospy.Subscriber("/velodyne_points", PointCloud2, self.cloud_callback, queue_size=1)

        self.pub = rospy.Publisher("/fusion/lidar_on_thermal", Image, queue_size=1)

    def camera_info_callback(self, msg):
        self.camera_model.fromCameraInfo(msg)
        self.camera_info_received = True

    def image_callback(self, msg):
        self.latest_image = msg

    def cloud_callback(self, cloud_msg):
        if not self.camera_info_received or self.latest_image is None:
            return

        try:
            transform = self.tf_buffer.lookup_transform(
                self.target_frame,
                cloud_msg.header.frame_id,
                cloud_msg.header.stamp,
                rospy.Duration(0.2)
            )

            cloud_in_camera = tf2_sensor_msgs.tf2_sensor_msgs.do_transform_cloud(
                cloud_msg,
                transform
            )

        except Exception as e:
            rospy.logwarn("TF transform failed: %s", str(e))
            return

        cv_image = self.bridge.imgmsg_to_cv2(self.latest_image, desired_encoding="bgr8")
        height, width = cv_image.shape[:2]

        for point in pc2.read_points(cloud_in_camera, field_names=("x", "y", "z"), skip_nans=True):
            x, y, z = point

            # In optical camera frame, z is depth forward.
            if z <= 0:
                continue

            u, v = self.camera_model.project3dToPixel((x, y, z))
            u = int(round(u))
            v = int(round(v))

            if 0 <= u < width and 0 <= v < height:
                depth = min(z, 30.0)
                radius = 1
                cv2.circle(cv_image, (u, v), radius, (0, 255, 0), -1)

        out_msg = self.bridge.cv2_to_imgmsg(cv_image, encoding="bgr8")
        out_msg.header = self.latest_image.header
        self.pub.publish(out_msg)


if __name__ == "__main__":
    rospy.init_node("project_lidar_to_thermal")
    node = LidarToRGBProjection()
    rospy.spin()