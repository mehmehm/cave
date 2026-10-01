#!/usr/bin/env python3

"""Synchronised RGB, thermal and LiDAR fusion for the A1 robot."""

import struct
import time

import cv2
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


class RgbThermalLidarSynchronizer:
    def __init__(self):
        self.bridge = CvBridge()

        self.rgb_model = PinholeCameraModel()
        self.thermal_model = PinholeCameraModel()

        self.rgb_info_ready = False
        self.thermal_info_ready = False
        self.rgb_info_frame = ""
        self.thermal_info_frame = ""

        # Topics
        self.rgb_image_topic = rospy.get_param(
            "~rgb_image_topic",
            "/rgb_camera/image_raw"
        )

        self.rgb_info_topic = rospy.get_param(
            "~rgb_info_topic",
            "/rgb_camera/camera_info"
        )

        self.thermal_image_topic = rospy.get_param(
            "~thermal_image_topic",
            "/thermal_camera/image_raw"
        )

        self.thermal_info_topic = rospy.get_param(
            "~thermal_info_topic",
            "/thermal_camera/camera_info"
        )

        self.lidar_topic = rospy.get_param(
            "~lidar_topic",
            "/velodyne_points"
        )

        self.output_topic = rospy.get_param(
            "~output_topic",
            "/cave/fusion/rgb_thermal_lidar_points"
        )

        # Transform configuration
        self.fixed_frame = rospy.get_param(
            "~fixed_frame",
            "lio_odom"
        )

        self.rgb_frame_override = rospy.get_param(
            "~rgb_frame",
            ""
        )

        self.thermal_frame_override = rospy.get_param(
            "~thermal_frame",
            ""
        )

        # Processing configuration
        self.point_skip = max(
            1,
            int(rospy.get_param("~point_skip", 3))
        )

        self.sync_queue_size = max(
            3,
            int(rospy.get_param("~sync_queue_size", 30))
        )

        self.sync_slop = max(
            0.0,
            float(rospy.get_param("~sync_slop", 0.08))
        )

        self.tf_timeout = max(
            0.01,
            float(rospy.get_param("~tf_timeout", 0.20))
        )

        self.keep_unassociated_points = bool(
            rospy.get_param(
                "~keep_unassociated_points",
                True
            )
        )

        # Statistics
        self.rgb_received = 0
        self.thermal_received = 0
        self.lidar_received = 0
        self.synchronised_callbacks = 0
        self.tf_failures = 0
        self.published_clouds = 0

        # TF
        self.tf_buffer = tf2_ros.Buffer(
            cache_time=rospy.Duration(60.0)
        )

        self.tf_listener = tf2_ros.TransformListener(
            self.tf_buffer
        )

        # Publisher
        self.publisher = rospy.Publisher(
            self.output_topic,
            PointCloud2,
            queue_size=1
        )

        # CameraInfo subscribers
        self.rgb_info_subscriber = rospy.Subscriber(
            self.rgb_info_topic,
            CameraInfo,
            self.rgb_info_callback,
            queue_size=1
        )

        self.thermal_info_subscriber = rospy.Subscriber(
            self.thermal_info_topic,
            CameraInfo,
            self.thermal_info_callback,
            queue_size=1
        )

        # Synchronised sensor subscribers
        self.rgb_subscriber = message_filters.Subscriber(
            self.rgb_image_topic,
            Image,
            queue_size=1
        )

        self.thermal_subscriber = message_filters.Subscriber(
            self.thermal_image_topic,
            Image,
            queue_size=1
        )

        self.lidar_subscriber = message_filters.Subscriber(
            self.lidar_topic,
            PointCloud2,
            queue_size=1
        )

        self.rgb_subscriber.registerCallback(
            self.count_rgb
        )

        self.thermal_subscriber.registerCallback(
            self.count_thermal
        )

        self.lidar_subscriber.registerCallback(
            self.count_lidar
        )

        self.synchronizer = (
            message_filters.ApproximateTimeSynchronizer(
                [
                    self.rgb_subscriber,
                    self.thermal_subscriber,
                    self.lidar_subscriber
                ],
                queue_size=self.sync_queue_size,
                slop=self.sync_slop,
                allow_headerless=False
            )
        )

        self.synchronizer.registerCallback(
            self.fusion_callback
        )

        rospy.loginfo(
            "RGB-thermal-LiDAR synchronizer started"
        )
        rospy.loginfo(
            "RGB image:      %s",
            self.rgb_image_topic
        )
        rospy.loginfo(
            "Thermal image:  %s",
            self.thermal_image_topic
        )
        rospy.loginfo(
            "LiDAR:          %s",
            self.lidar_topic
        )
        rospy.loginfo(
            "Fixed frame:    %s",
            self.fixed_frame
        )
        rospy.loginfo(
            "Output:         %s",
            self.output_topic
        )
        rospy.loginfo(
            "Point skip:     %d",
            self.point_skip
        )
        rospy.loginfo(
            "Sync slop:      %.3f seconds",
            self.sync_slop
        )

    def count_rgb(self, _message):
        self.rgb_received += 1

    def count_thermal(self, _message):
        self.thermal_received += 1

    def count_lidar(self, _message):
        self.lidar_received += 1

    def rgb_info_callback(self, message):
        self.rgb_model.fromCameraInfo(message)
        self.rgb_info_frame = message.header.frame_id
        self.rgb_info_ready = True

    def thermal_info_callback(self, message):
        self.thermal_model.fromCameraInfo(message)
        self.thermal_info_frame = message.header.frame_id
        self.thermal_info_ready = True

    @staticmethod
    def pack_rgb_float(red, green, blue):
        red = max(0, min(255, int(red)))
        green = max(0, min(255, int(green)))
        blue = max(0, min(255, int(blue)))

        packed_integer = (
            (red << 16)
            | (green << 8)
            | blue
        )

        return struct.unpack(
            "<f",
            struct.pack("<I", packed_integer)
        )[0]

    @staticmethod
    def transform_to_matrix(transform):
        translation = transform.transform.translation
        rotation = transform.transform.rotation

        matrix = quaternion_matrix(
            [
                rotation.x,
                rotation.y,
                rotation.z,
                rotation.w
            ]
        )

        matrix[0, 3] = translation.x
        matrix[1, 3] = translation.y
        matrix[2, 3] = translation.z

        return matrix[:3, :3], matrix[:3, 3]

    def lookup_lidar_to_camera(
        self,
        camera_frame,
        camera_stamp,
        lidar_frame,
        lidar_stamp
    ):
        transform = self.tf_buffer.lookup_transform_full(
            target_frame=camera_frame,
            target_time=camera_stamp,
            source_frame=lidar_frame,
            source_time=lidar_stamp,
            fixed_frame=self.fixed_frame,
            timeout=rospy.Duration(self.tf_timeout)
        )

        return self.transform_to_matrix(transform)

    @staticmethod
    def project_point(model, image, point_camera):
        camera_x = float(point_camera[0])
        camera_y = float(point_camera[1])
        camera_z = float(point_camera[2])

        if not np.isfinite(camera_z) or camera_z <= 0.0:
            return None

        pixel_u, pixel_v = model.project3dToPixel(
            (camera_x, camera_y, camera_z)
        )

        if not (
            np.isfinite(pixel_u)
            and np.isfinite(pixel_v)
        ):
            return None

        pixel_u = int(round(pixel_u))
        pixel_v = int(round(pixel_v))

        image_height, image_width = image.shape[:2]

        if not (
            0 <= pixel_u < image_width
            and 0 <= pixel_v < image_height
        ):
            return None

        return pixel_u, pixel_v, camera_z

    @staticmethod
    def sample_rgb(image, projection):
        if projection is None:
            return None

        pixel_u, pixel_v, _depth = projection

        # cv_bridge provides BGR, while ROS packed RGB expects RGB.
        blue, green, red = image[pixel_v, pixel_u]

        return (
            int(red),
            int(green),
            int(blue)
        )

    @staticmethod
    def sample_thermal(image, projection):
        if projection is None:
            return None

        pixel_u, pixel_v, _depth = projection

        return float(image[pixel_v, pixel_u])

    def convert_thermal_image(self, message):
        try:
            return self.bridge.imgmsg_to_cv2(
                message,
                desired_encoding="mono8"
            )

        except CvBridgeError:
            thermal_bgr = self.bridge.imgmsg_to_cv2(
                message,
                desired_encoding="bgr8"
            )

            return cv2.cvtColor(
                thermal_bgr,
                cv2.COLOR_BGR2GRAY
            )

    @staticmethod
    def create_depth_buffer(candidates, projection_name):
        depth_buffer = {}

        for candidate in candidates:
            projection = candidate[projection_name]

            if projection is None:
                continue

            pixel_u, pixel_v, depth = projection
            pixel_key = (pixel_u, pixel_v)

            current_depth = depth_buffer.get(pixel_key)

            if current_depth is None or depth < current_depth:
                depth_buffer[pixel_key] = depth

        return depth_buffer

    @staticmethod
    def passes_depth_buffer(projection, depth_buffer):
        if projection is None:
            return False

        pixel_u, pixel_v, depth = projection
        closest_depth = depth_buffer.get((pixel_u, pixel_v))

        if closest_depth is None:
            return False

        return depth <= closest_depth + 1e-6

    def fusion_callback(
        self,
        rgb_message,
        thermal_message,
        lidar_message
    ):
        self.synchronised_callbacks += 1

        if not (
            self.rgb_info_ready
            and self.thermal_info_ready
        ):
            rospy.logwarn_throttle(
                2.0,
                "Waiting for RGB and thermal CameraInfo"
            )
            return

        # CameraInfo normally provides the correct optical frame.
        rgb_frame = (
            self.rgb_frame_override
            or self.rgb_info_frame
            or rgb_message.header.frame_id
        )

        thermal_frame = (
            self.thermal_frame_override
            or self.thermal_info_frame
            or thermal_message.header.frame_id
        )

        lidar_frame = lidar_message.header.frame_id

        if not rgb_frame or not thermal_frame or not lidar_frame:
            rospy.logwarn_throttle(
                2.0,
                "A synchronized message has an empty frame_id"
            )
            return

        try:
            rgb_image = self.bridge.imgmsg_to_cv2(
                rgb_message,
                desired_encoding="bgr8"
            )

            thermal_image = self.convert_thermal_image(
                thermal_message
            )

        except CvBridgeError as error:
            rospy.logwarn_throttle(
                2.0,
                "Image conversion failed: %s",
                str(error)
            )
            return

        try:
            (
                rgb_rotation,
                rgb_translation
            ) = self.lookup_lidar_to_camera(
                rgb_frame,
                rgb_message.header.stamp,
                lidar_frame,
                lidar_message.header.stamp
            )

            (
                thermal_rotation,
                thermal_translation
            ) = self.lookup_lidar_to_camera(
                thermal_frame,
                thermal_message.header.stamp,
                lidar_frame,
                lidar_message.header.stamp
            )

        except (
            tf2_ros.LookupException,
            tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException
        ) as error:
            self.tf_failures += 1

            rospy.logwarn_throttle(
                2.0,
                "Fusion TF failed via '%s': %s",
                self.fixed_frame,
                str(error)
            )
            return

        lidar_field_names = {
            field.name
            for field in lidar_message.fields
        }

        has_intensity = "intensity" in lidar_field_names

        requested_fields = (
            ("x", "y", "z", "intensity")
            if has_intensity
            else ("x", "y", "z")
        )

        candidates = []
        start_time = time.perf_counter()

        lidar_points = pc2.read_points(
            lidar_message,
            field_names=requested_fields,
            skip_nans=True
        )

        for point_index, point in enumerate(lidar_points):
            if point_index % self.point_skip != 0:
                continue

            lidar_x = float(point[0])
            lidar_y = float(point[1])
            lidar_z = float(point[2])

            intensity = (
                float(point[3])
                if has_intensity
                else 0.0
            )

            point_lidar = np.array(
                [lidar_x, lidar_y, lidar_z],
                dtype=np.float64
            )

            point_in_rgb_camera = (
                rgb_rotation.dot(point_lidar)
                + rgb_translation
            )

            point_in_thermal_camera = (
                thermal_rotation.dot(point_lidar)
                + thermal_translation
            )

            rgb_projection = self.project_point(
                self.rgb_model,
                rgb_image,
                point_in_rgb_camera
            )

            thermal_projection = self.project_point(
                self.thermal_model,
                thermal_image,
                point_in_thermal_camera
            )

            candidates.append(
                {
                    "x": lidar_x,
                    "y": lidar_y,
                    "z": lidar_z,
                    "intensity": intensity,
                    "rgb_projection": rgb_projection,
                    "thermal_projection": thermal_projection
                }
            )

        rgb_depth_buffer = self.create_depth_buffer(
            candidates,
            "rgb_projection"
        )

        thermal_depth_buffer = self.create_depth_buffer(
            candidates,
            "thermal_projection"
        )

        fused_points = []
        rgb_valid_count = 0
        thermal_valid_count = 0

        for candidate in candidates:
            rgb_projection = candidate["rgb_projection"]
            thermal_projection = candidate["thermal_projection"]

            rgb_visible = self.passes_depth_buffer(
                rgb_projection,
                rgb_depth_buffer
            )

            thermal_visible = self.passes_depth_buffer(
                thermal_projection,
                thermal_depth_buffer
            )

            rgb_sample = (
                self.sample_rgb(
                    rgb_image,
                    rgb_projection
                )
                if rgb_visible
                else None
            )

            thermal_sample = (
                self.sample_thermal(
                    thermal_image,
                    thermal_projection
                )
                if thermal_visible
                else None
            )

            rgb_valid = (
                1.0 if rgb_sample is not None else 0.0
            )

            thermal_valid = (
                1.0 if thermal_sample is not None else 0.0
            )

            if rgb_sample is not None:
                red, green, blue = rgb_sample

                packed_rgb = self.pack_rgb_float(
                    red,
                    green,
                    blue
                )

                rgb_valid_count += 1
            else:
                packed_rgb = self.pack_rgb_float(
                    0,
                    0,
                    0
                )

            if thermal_sample is not None:
                thermal_value = float(thermal_sample)
                thermal_valid_count += 1
            else:
                thermal_value = 0.0

            if (
                not self.keep_unassociated_points
                and rgb_sample is None
                and thermal_sample is None
            ):
                continue

            fused_points.append(
                [
                    candidate["x"],
                    candidate["y"],
                    candidate["z"],
                    candidate["intensity"],
                    packed_rgb,
                    thermal_value,
                    rgb_valid,
                    thermal_valid
                ]
            )

        output_header = Header()
        output_header.stamp = lidar_message.header.stamp
        output_header.frame_id = lidar_frame

        output_fields = [
            PointField(
                "x", 0, PointField.FLOAT32, 1
            ),
            PointField(
                "y", 4, PointField.FLOAT32, 1
            ),
            PointField(
                "z", 8, PointField.FLOAT32, 1
            ),
            PointField(
                "intensity", 12, PointField.FLOAT32, 1
            ),
            PointField(
                "rgb", 16, PointField.FLOAT32, 1
            ),
            PointField(
                "thermal", 20, PointField.FLOAT32, 1
            ),
            PointField(
                "rgb_valid", 24, PointField.FLOAT32, 1
            ),
            PointField(
                "thermal_valid", 28, PointField.FLOAT32, 1
            )
        ]

        output_cloud = pc2.create_cloud(
            output_header,
            output_fields,
            fused_points
        )

        self.publisher.publish(output_cloud)
        self.published_clouds += 1

        processed_count = len(candidates)

        rgb_percentage = (
            100.0 * rgb_valid_count / processed_count
            if processed_count
            else 0.0
        )

        thermal_percentage = (
            100.0 * thermal_valid_count / processed_count
            if processed_count
            else 0.0
        )

        rgb_time_difference = abs(
            (
                rgb_message.header.stamp
                - lidar_message.header.stamp
            ).to_sec()
        )

        thermal_time_difference = abs(
            (
                thermal_message.header.stamp
                - lidar_message.header.stamp
            ).to_sec()
        )

        processing_time_ms = (
            1000.0
            * (time.perf_counter() - start_time)
        )

        approximately_unmatched_lidar = max(
            0,
            self.lidar_received
            - self.synchronised_callbacks
        )

        rospy.loginfo_throttle(
            2.0,
            "Sync #%d: RGB dt=%.4fs thermal dt=%.4fs; "
            "RGB visible=%.1f%% thermal visible=%.1f%%; "
            "processing=%.1fms; TF failures=%d; "
            "unmatched LiDAR~%d",
            self.synchronised_callbacks,
            rgb_time_difference,
            thermal_time_difference,
            rgb_percentage,
            thermal_percentage,
            processing_time_ms,
            self.tf_failures,
            approximately_unmatched_lidar
        )


if __name__ == "__main__":
    rospy.init_node(
        "rgb_thermal_lidar_sync"
    )

    RgbThermalLidarSynchronizer()
    rospy.spin()