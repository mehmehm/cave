#!/usr/bin/env python3

"""Project A1 Velodyne points into the RGB camera and publish a coloured cloud.

The output XYZ coordinates remain in the input LiDAR frame.  The original
LiDAR timestamp is preserved so the cloud can be transformed correctly later.
"""

import struct
import time

import message_filters
import numpy as np
import rospy
import sensor_msgs.point_cloud2 as pc2
import tf2_ros

from cv_bridge import CvBridge, CvBridgeError
from image_geometry import PinholeCameraModel
from sensor_msgs.msg import CameraInfo, Image, PointCloud2, PointField
from std_msgs.msg import Header
from tf.transformations import quaternion_matrix


class RgbLidarProjection:
    def __init__(self):
        self.bridge = CvBridge()
        self.rgb_model = PinholeCameraModel()
        self.rgb_info_ready = False
        self.rgb_info_frame = ""

        self.image_topic = rospy.get_param(
            "~image_topic", "/rgb_camera/image_raw"
        )
        self.camera_info_topic = rospy.get_param(
            "~camera_info_topic", "/rgb_camera/camera_info"
        )
        self.lidar_topic = rospy.get_param(
            "~lidar_topic", "/velodyne_points"
        )
        self.output_topic = rospy.get_param(
            "~output_topic", "/cave/fusion/rgb_lidar_points"
        )

        self.camera_frame_override = rospy.get_param("~camera_frame", "")

        self.point_skip = max(1, int(rospy.get_param("~point_skip", 3)))
        self.sync_queue_size = max(
            2, int(rospy.get_param("~sync_queue_size", 20))
        )
        self.sync_slop = max(
            0.0, float(rospy.get_param("~sync_slop", 0.05))
        )
        self.tf_timeout = max(
            0.01, float(rospy.get_param("~tf_timeout", 0.2))
        )
        self.keep_uncoloured_points = bool(
            rospy.get_param("~keep_uncoloured_points", True)
        )

        self.tf_buffer = tf2_ros.Buffer(rospy.Duration(30.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.publisher = rospy.Publisher(
            self.output_topic, PointCloud2, queue_size=1
        )

        self.camera_info_subscriber = rospy.Subscriber(
            self.camera_info_topic,
            CameraInfo,
            self.camera_info_callback,
            queue_size=1,
        )

        self.image_subscriber = message_filters.Subscriber(
            self.image_topic, Image, queue_size=1
        )
        self.lidar_subscriber = message_filters.Subscriber(
            self.lidar_topic, PointCloud2, queue_size=1
        )

        self.synchronizer = message_filters.ApproximateTimeSynchronizer(
            [self.image_subscriber, self.lidar_subscriber],
            queue_size=self.sync_queue_size,
            slop=self.sync_slop,
            allow_headerless=False,
        )
        self.synchronizer.registerCallback(self.fusion_callback)

        rospy.loginfo("RGB-LiDAR projection started")
        rospy.loginfo("  RGB image:  %s", self.image_topic)
        rospy.loginfo("  RGB info:   %s", self.camera_info_topic)
        rospy.loginfo("  LiDAR:      %s", self.lidar_topic)
        rospy.loginfo("  Output:     %s", self.output_topic)
        rospy.loginfo("  Point skip: %d", self.point_skip)
        rospy.loginfo("  Sync slop:  %.3f s", self.sync_slop)

    def camera_info_callback(self, message):
        self.rgb_model.fromCameraInfo(message)
        self.rgb_info_frame = message.header.frame_id
        self.rgb_info_ready = True

    @staticmethod
    def pack_rgb_float(red, green, blue):
        """Pack 8-bit RGB values into the float32 layout expected by RViz/PCL."""
        rgb_uint32 = (
            (int(red) << 16) | (int(green) << 8) | int(blue)
        )
        return struct.unpack("<f", struct.pack("<I", rgb_uint32))[0]

    def sample_rgb(self, image, point_camera):
        """Return (r, g, b) for a point expressed in the RGB optical frame."""
        camera_x, camera_y, camera_z = point_camera

        # In a ROS optical frame, +Z points forwards.
        if not np.isfinite(camera_z) or camera_z <= 0.0:
            return None

        pixel_u, pixel_v = self.rgb_model.project3dToPixel(
            (camera_x, camera_y, camera_z)
        )

        if not np.isfinite(pixel_u) or not np.isfinite(pixel_v):
            return None

        pixel_u = int(round(pixel_u))
        pixel_v = int(round(pixel_v))
        image_height, image_width = image.shape[:2]

        if not (
            0 <= pixel_u < image_width and 0 <= pixel_v < image_height
        ):
            return None

        blue, green, red = image[pixel_v, pixel_u]
        return int(red), int(green), int(blue)

    def lookup_lidar_to_camera(self, camera_frame, cloud_message):
        """Return R and t for p_camera = R * p_lidar + t."""
        transform = self.tf_buffer.lookup_transform(
            camera_frame,
            cloud_message.header.frame_id,
            cloud_message.header.stamp,
            rospy.Duration(self.tf_timeout),
        )

        translation = transform.transform.translation
        rotation = transform.transform.rotation

        transform_matrix = quaternion_matrix(
            [rotation.x, rotation.y, rotation.z, rotation.w]
        )
        transform_matrix[0, 3] = translation.x
        transform_matrix[1, 3] = translation.y
        transform_matrix[2, 3] = translation.z

        return transform_matrix[:3, :3], transform_matrix[:3, 3]

    def fusion_callback(self, image_message, cloud_message):
        if not self.rgb_info_ready:
            rospy.logwarn_throttle(
                2.0, "Waiting for RGB CameraInfo on %s", self.camera_info_topic
            )
            return

        if not cloud_message.header.frame_id:
            rospy.logwarn_throttle(2.0, "LiDAR message has an empty frame_id")
            return

        camera_frame = (
            self.camera_frame_override
            or image_message.header.frame_id
            or self.rgb_info_frame
        )

        if not camera_frame:
            rospy.logwarn_throttle(2.0, "RGB message has an empty frame_id")
            return

        try:
            rgb_image = self.bridge.imgmsg_to_cv2(
                image_message, desired_encoding="bgr8"
            )
        except CvBridgeError as error:
            rospy.logwarn_throttle(
                2.0, "Could not convert RGB image: %s", str(error)
            )
            return

        try:
            rotation, translation = self.lookup_lidar_to_camera(
                camera_frame, cloud_message
            )
        except (
            tf2_ros.LookupException,
            tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException,
        ) as error:
            rospy.logwarn_throttle(
                2.0,
                "RGB-LiDAR TF failed (%s -> %s): %s",
                cloud_message.header.frame_id,
                camera_frame,
                str(error),
            )
            return

        input_fields = {field.name for field in cloud_message.fields}
        has_intensity = "intensity" in input_fields
        requested_fields = (
            ("x", "y", "z", "intensity")
            if has_intensity
            else ("x", "y", "z")
        )

        fused_points = []
        processed_count = 0
        coloured_count = 0
        start_time = time.perf_counter()

        points = pc2.read_points(
            cloud_message,
            field_names=requested_fields,
            skip_nans=True,
        )

        for point_index, point in enumerate(points):
            if point_index % self.point_skip != 0:
                continue

            lidar_x = float(point[0])
            lidar_y = float(point[1])
            lidar_z = float(point[2])
            intensity = float(point[3]) if has_intensity else 0.0
            processed_count += 1

            point_lidar = np.array(
                [lidar_x, lidar_y, lidar_z], dtype=np.float64
            )
            point_camera = rotation.dot(point_lidar) + translation
            rgb_sample = self.sample_rgb(rgb_image, point_camera)

            if rgb_sample is None:
                if not self.keep_uncoloured_points:
                    continue

                packed_rgb = self.pack_rgb_float(0, 0, 0)
                rgb_valid = 0.0
            else:
                red, green, blue = rgb_sample
                packed_rgb = self.pack_rgb_float(red, green, blue)
                rgb_valid = 1.0
                coloured_count += 1

            # XYZ remains in cloud_message.header.frame_id. Only RGB is sampled
            # from the transformed camera-frame copy of the point.
            fused_points.append(
                [
                    lidar_x,
                    lidar_y,
                    lidar_z,
                    intensity,
                    packed_rgb,
                    rgb_valid,
                ]
            )

        output_header = Header()
        output_header.stamp = cloud_message.header.stamp
        output_header.frame_id = cloud_message.header.frame_id

        output_fields = [
            PointField("x", 0, PointField.FLOAT32, 1),
            PointField("y", 4, PointField.FLOAT32, 1),
            PointField("z", 8, PointField.FLOAT32, 1),
            PointField("intensity", 12, PointField.FLOAT32, 1),
            PointField("rgb", 16, PointField.FLOAT32, 1),
            PointField("rgb_valid", 20, PointField.FLOAT32, 1),
        ]

        output_cloud = pc2.create_cloud(
            output_header, output_fields, fused_points
        )
        self.publisher.publish(output_cloud)

        valid_percentage = (
            100.0 * coloured_count / processed_count
            if processed_count
            else 0.0
        )
        time_difference = abs(
            (image_message.header.stamp - cloud_message.header.stamp).to_sec()
        )
        processing_time_ms = 1000.0 * (time.perf_counter() - start_time)

        rospy.loginfo_throttle(
            2.0,
            "RGB-LiDAR: %d/%d coloured (%.1f%%), dt=%.4fs, processing=%.1fms",
            coloured_count,
            processed_count,
            valid_percentage,
            time_difference,
            processing_time_ms,
        )


if __name__ == "__main__":
    rospy.init_node("rgb_lidar_projection")
    RgbLidarProjection()
    rospy.spin()
