#!/usr/bin/env python3

import math
import struct
import threading

import tf2_ros

from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud

import rospy
import sensor_msgs.point_cloud2 as pc2

from geometry_msgs.msg import Point
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray


class MultimodalVoxelAccumulator:
    def __init__(self):
        self.input_topic = rospy.get_param(
            "~input_topic",
            "/cave/fusion/active_points"
        )

        self.output_topic = rospy.get_param(
            "~output_topic",
            "/cave/map/multimodal_voxels"
        )

        self.fixed_frame = rospy.get_param(
            "~fixed_frame",
            "lio_odom"
        )

        self.resolution = float(rospy.get_param(
            "~resolution",
            0.10
        ))

        self.publish_rate = float(rospy.get_param(
            "~publish_rate",
            1.0
        ))

        self.maximum_observations = int(rospy.get_param(
            "~maximum_observations",
            20
        ))

        self.maximum_voxels = int(rospy.get_param(
            "~maximum_voxels",
            250000
        ))

        # key -> [mean_r, mean_g, mean_b, observations]
        self.voxels = {}
        self.lock = threading.Lock()
        self.rgb_datatype = PointField.FLOAT32

        self.tf_timeout = float(rospy.get_param(
            "~tf_timeout",
            0.25
        ))

        self.tf_buffer = tf2_ros.Buffer(
            cache_time=rospy.Duration(30.0)
        )

        self.tf_listener = tf2_ros.TransformListener(
            self.tf_buffer
        )
        self.received_clouds = 0
        self.rejected_clouds = 0

        self.publisher = rospy.Publisher(
            self.output_topic,
            MarkerArray,
            queue_size=1,
            latch=True
        )

        self.subscriber = rospy.Subscriber(
            self.input_topic,
            PointCloud2,
            self.cloud_callback,
            queue_size=2,
            buff_size=2 ** 25,
            tcp_nodelay=True
        )

        rospy.Timer(
            rospy.Duration(1.0 / max(self.publish_rate, 0.1)),
            self.publish_voxels
        )

        rospy.loginfo("Multimodal voxel accumulator started")
        rospy.loginfo("Input:      %s", self.input_topic)
        rospy.loginfo("Output:     %s", self.output_topic)
        rospy.loginfo("Frame:      %s", self.fixed_frame)
        rospy.loginfo("Resolution: %.3f m", self.resolution)

    def coordinate_to_key(self, x, y, z):
        return (
            int(math.floor(x / self.resolution)),
            int(math.floor(y / self.resolution)),
            int(math.floor(z / self.resolution))
        )

    def key_to_centre(self, key):
        return (
            (key[0] + 0.5) * self.resolution,
            (key[1] + 0.5) * self.resolution,
            (key[2] + 0.5) * self.resolution
        )

    def unpack_rgb(self, value):
        if self.rgb_datatype == PointField.FLOAT32:
            packed = struct.unpack(
                "I",
                struct.pack("f", float(value))
            )[0]
        else:
            packed = int(value)

        red = (packed >> 16) & 0xFF
        green = (packed >> 8) & 0xFF
        blue = packed & 0xFF

        return red, green, blue

    def cloud_callback(self, cloud):
        source_frame = cloud.header.frame_id.lstrip("/")
        target_frame = self.fixed_frame.lstrip("/")

        if source_frame != target_frame:
            try:
                transform = self.tf_buffer.lookup_transform(
                    target_frame,
                    source_frame,
                    cloud.header.stamp,
                    rospy.Duration(self.tf_timeout)
                )

                cloud = do_transform_cloud(
                    cloud,
                    transform
                )

                cloud.header.frame_id = target_frame

            except (
                tf2_ros.LookupException,
                tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException
            ) as error:
                self.rejected_clouds += 1

                rospy.logwarn_throttle(
                    5.0,
                    "Could not transform active cloud from '%s' "
                    "to '%s': %s",
                    source_frame,
                    target_frame,
                    str(error)
                )
                return

        field_map = {field.name: field for field in cloud.fields}

        if "rgb" not in field_map:
            rospy.logerr_throttle(
                5.0,
                "Input cloud has no packed 'rgb' field. Fields: %s",
                sorted(field_map.keys())
            )
            return

        self.rgb_datatype = field_map["rgb"].datatype
        local_updates = {}

        try:
            points = pc2.read_points(
                cloud,
                field_names=("x", "y", "z", "rgb"),
                skip_nans=True
            )

            for x, y, z, packed_rgb in points:
                if not (
                    math.isfinite(x)
                    and math.isfinite(y)
                    and math.isfinite(z)
                ):
                    continue

                red, green, blue = self.unpack_rgb(packed_rgb)
                key = self.coordinate_to_key(x, y, z)

                if key not in local_updates:
                    local_updates[key] = [
                        float(red),
                        float(green),
                        float(blue),
                        1
                    ]
                else:
                    entry = local_updates[key]
                    entry[0] += red
                    entry[1] += green
                    entry[2] += blue
                    entry[3] += 1

        except Exception as error:
            rospy.logerr_throttle(
                5.0,
                "Could not read active cloud: %s",
                str(error)
            )
            return

        with self.lock:
            for key, update in local_updates.items():
                if (
                    key not in self.voxels
                    and len(self.voxels) >= self.maximum_voxels
                ):
                    continue

                observed_r = update[0] / update[3]
                observed_g = update[1] / update[3]
                observed_b = update[2] / update[3]

                if key not in self.voxels:
                    self.voxels[key] = [
                        observed_r,
                        observed_g,
                        observed_b,
                        1
                    ]
                    continue

                voxel = self.voxels[key]
                weight = min(
                    voxel[3],
                    self.maximum_observations
                )

                alpha = 1.0 / float(weight + 1)

                voxel[0] = (
                    (1.0 - alpha) * voxel[0]
                    + alpha * observed_r
                )
                voxel[1] = (
                    (1.0 - alpha) * voxel[1]
                    + alpha * observed_g
                )
                voxel[2] = (
                    (1.0 - alpha) * voxel[2]
                    + alpha * observed_b
                )
                voxel[3] = min(
                    voxel[3] + 1,
                    self.maximum_observations
                )

        self.received_clouds += 1

        rospy.loginfo_throttle(
            5.0,
            "Clouds=%d, coloured voxels=%d, rejected=%d",
            self.received_clouds,
            len(self.voxels),
            self.rejected_clouds
        )

    def publish_voxels(self, _event):
        with self.lock:
            snapshot = list(self.voxels.items())

        if not snapshot:
            return

        marker = Marker()
        marker.header.stamp = rospy.Time.now()
        marker.header.frame_id = self.fixed_frame
        marker.ns = "multimodal_surface"
        marker.id = 0
        marker.type = Marker.CUBE_LIST
        marker.action = Marker.ADD

        marker.pose.orientation.w = 1.0

        display_scale = self.resolution * 1.20

        marker.scale.x = display_scale
        marker.scale.y = display_scale
        marker.scale.z = display_scale

        marker.color.a = 1.0
        marker.lifetime = rospy.Duration(0)

        for key, voxel in snapshot:
            centre_x, centre_y, centre_z = self.key_to_centre(key)

            point = Point()
            point.x = centre_x
            point.y = centre_y
            point.z = centre_z
            marker.points.append(point)

            colour = ColorRGBA()
            colour.r = max(0.0, min(1.0, voxel[0] / 255.0))
            colour.g = max(0.0, min(1.0, voxel[1] / 255.0))
            colour.b = max(0.0, min(1.0, voxel[2] / 255.0))
            colour.a = 1.0
            marker.colors.append(colour)

        output = MarkerArray()
        output.markers.append(marker)
        self.publisher.publish(output)


if __name__ == "__main__":
    rospy.init_node("multimodal_voxel_accumulator")
    MultimodalVoxelAccumulator()
    rospy.spin()#!/usr/bin/env python3

import math
import struct
import threading

import rospy
import sensor_msgs.point_cloud2 as pc2

from geometry_msgs.msg import Point
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray


class MultimodalVoxelAccumulator:
    def __init__(self):
        self.input_topic = rospy.get_param(
            "~input_topic",
            "/cave/fusion/active_points"
        )

        self.output_topic = rospy.get_param(
            "~output_topic",
            "/cave/map/multimodal_voxels"
        )

        self.fixed_frame = rospy.get_param(
            "~fixed_frame",
            "lio_odom"
        )

        self.resolution = float(rospy.get_param(
            "~resolution",
            0.10
        ))

        self.publish_rate = float(rospy.get_param(
            "~publish_rate",
            1.0
        ))

        self.maximum_observations = int(rospy.get_param(
            "~maximum_observations",
            20
        ))

        self.maximum_voxels = int(rospy.get_param(
            "~maximum_voxels",
            250000
        ))

        # key -> [mean_r, mean_g, mean_b, observations]
        self.voxels = {}
        self.lock = threading.Lock()
        self.rgb_datatype = PointField.FLOAT32
        self.received_clouds = 0
        self.rejected_clouds = 0

        self.publisher = rospy.Publisher(
            self.output_topic,
            MarkerArray,
            queue_size=1,
            latch=True
        )

        self.subscriber = rospy.Subscriber(
            self.input_topic,
            PointCloud2,
            self.cloud_callback,
            queue_size=2,
            buff_size=2 ** 25,
            tcp_nodelay=True
        )

        rospy.Timer(
            rospy.Duration(1.0 / max(self.publish_rate, 0.1)),
            self.publish_voxels
        )

        rospy.loginfo("Multimodal voxel accumulator started")
        rospy.loginfo("Input:      %s", self.input_topic)
        rospy.loginfo("Output:     %s", self.output_topic)
        rospy.loginfo("Frame:      %s", self.fixed_frame)
        rospy.loginfo("Resolution: %.3f m", self.resolution)

    def coordinate_to_key(self, x, y, z):
        return (
            int(math.floor(x / self.resolution)),
            int(math.floor(y / self.resolution)),
            int(math.floor(z / self.resolution))
        )

    def key_to_centre(self, key):
        return (
            (key[0] + 0.5) * self.resolution,
            (key[1] + 0.5) * self.resolution,
            (key[2] + 0.5) * self.resolution
        )

    def unpack_rgb(self, value):
        if self.rgb_datatype == PointField.FLOAT32:
            packed = struct.unpack(
                "I",
                struct.pack("f", float(value))
            )[0]
        else:
            packed = int(value)

        red = (packed >> 16) & 0xFF
        green = (packed >> 8) & 0xFF
        blue = packed & 0xFF

        return red, green, blue

    def cloud_callback(self, cloud):
        if cloud.header.frame_id != self.fixed_frame:
            self.rejected_clouds += 1

            rospy.logwarn_throttle(
                5.0,
                "Rejected active cloud in frame '%s'; expected '%s'",
                cloud.header.frame_id,
                self.fixed_frame
            )
            return

        field_map = {field.name: field for field in cloud.fields}

        if "rgb" not in field_map:
            rospy.logerr_throttle(
                5.0,
                "Input cloud has no packed 'rgb' field. Fields: %s",
                sorted(field_map.keys())
            )
            return

        self.rgb_datatype = field_map["rgb"].datatype
        local_updates = {}

        try:
            points = pc2.read_points(
                cloud,
                field_names=("x", "y", "z", "rgb"),
                skip_nans=True
            )

            for x, y, z, packed_rgb in points:
                if not (
                    math.isfinite(x)
                    and math.isfinite(y)
                    and math.isfinite(z)
                ):
                    continue

                red, green, blue = self.unpack_rgb(packed_rgb)
                key = self.coordinate_to_key(x, y, z)

                if key not in local_updates:
                    local_updates[key] = [
                        float(red),
                        float(green),
                        float(blue),
                        1
                    ]
                else:
                    entry = local_updates[key]
                    entry[0] += red
                    entry[1] += green
                    entry[2] += blue
                    entry[3] += 1

        except Exception as error:
            rospy.logerr_throttle(
                5.0,
                "Could not read active cloud: %s",
                str(error)
            )
            return

        with self.lock:
            for key, update in local_updates.items():
                if (
                    key not in self.voxels
                    and len(self.voxels) >= self.maximum_voxels
                ):
                    continue

                observed_r = update[0] / update[3]
                observed_g = update[1] / update[3]
                observed_b = update[2] / update[3]

                if key not in self.voxels:
                    self.voxels[key] = [
                        observed_r,
                        observed_g,
                        observed_b,
                        1
                    ]
                    continue

                voxel = self.voxels[key]
                weight = min(
                    voxel[3],
                    self.maximum_observations
                )

                alpha = 1.0 / float(weight + 1)

                voxel[0] = (
                    (1.0 - alpha) * voxel[0]
                    + alpha * observed_r
                )
                voxel[1] = (
                    (1.0 - alpha) * voxel[1]
                    + alpha * observed_g
                )
                voxel[2] = (
                    (1.0 - alpha) * voxel[2]
                    + alpha * observed_b
                )
                voxel[3] = min(
                    voxel[3] + 1,
                    self.maximum_observations
                )

        self.received_clouds += 1

        rospy.loginfo_throttle(
            5.0,
            "Clouds=%d, coloured voxels=%d, rejected=%d",
            self.received_clouds,
            len(self.voxels),
            self.rejected_clouds
        )

    def publish_voxels(self, _event):
        with self.lock:
            snapshot = list(self.voxels.items())

        if not snapshot:
            return

        marker = Marker()
        marker.header.stamp = rospy.Time.now()
        marker.header.frame_id = self.fixed_frame
        marker.ns = "multimodal_surface"
        marker.id = 0
        marker.type = Marker.CUBE_LIST
        marker.action = Marker.ADD

        marker.pose.orientation.w = 1.0

        marker.scale.x = self.resolution
        marker.scale.y = self.resolution
        marker.scale.z = self.resolution

        marker.color.a = 1.0
        marker.lifetime = rospy.Duration(0)

        for key, voxel in snapshot:
            centre_x, centre_y, centre_z = self.key_to_centre(key)

            point = Point()
            point.x = centre_x
            point.y = centre_y
            point.z = centre_z
            marker.points.append(point)

            colour = ColorRGBA()
            colour.r = max(0.0, min(1.0, voxel[0] / 255.0))
            colour.g = max(0.0, min(1.0, voxel[1] / 255.0))
            colour.b = max(0.0, min(1.0, voxel[2] / 255.0))
            colour.a = 1.0
            marker.colors.append(colour)

        output = MarkerArray()
        output.markers.append(marker)
        self.publisher.publish(output)


if __name__ == "__main__":
    rospy.init_node("multimodal_voxel_accumulator")
    MultimodalVoxelAccumulator()
    rospy.spin()#!/usr/bin/env python3

import math
import struct
import threading

import tf2_ros

from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud

import rospy
import sensor_msgs.point_cloud2 as pc2

from geometry_msgs.msg import Point
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray


class MultimodalVoxelAccumulator:
    def __init__(self):
        self.input_topic = rospy.get_param(
            "~input_topic",
            "/cave/fusion/active_points"
        )

        self.output_topic = rospy.get_param(
            "~output_topic",
            "/cave/map/multimodal_voxels"
        )

        self.fixed_frame = rospy.get_param(
            "~fixed_frame",
            "lio_odom"
        )

        self.resolution = float(rospy.get_param(
            "~resolution",
            0.10
        ))

        self.publish_rate = float(rospy.get_param(
            "~publish_rate",
            1.0
        ))

        self.maximum_observations = int(rospy.get_param(
            "~maximum_observations",
            20
        ))

        self.maximum_voxels = int(rospy.get_param(
            "~maximum_voxels",
            250000
        ))

        # key -> [mean_r, mean_g, mean_b, observations]
        self.voxels = {}
        self.lock = threading.Lock()
        self.rgb_datatype = PointField.FLOAT32
        self.received_clouds = 0
        self.rejected_clouds = 0

        self.publisher = rospy.Publisher(
            self.output_topic,
            MarkerArray,
            queue_size=1,
            latch=True
        )

        self.subscriber = rospy.Subscriber(
            self.input_topic,
            PointCloud2,
            self.cloud_callback,
            queue_size=2,
            buff_size=2 ** 25,
            tcp_nodelay=True
        )

        rospy.Timer(
            rospy.Duration(1.0 / max(self.publish_rate, 0.1)),
            self.publish_voxels
        )

        rospy.loginfo("Multimodal voxel accumulator started")
        rospy.loginfo("Input:      %s", self.input_topic)
        rospy.loginfo("Output:     %s", self.output_topic)
        rospy.loginfo("Frame:      %s", self.fixed_frame)
        rospy.loginfo("Resolution: %.3f m", self.resolution)

    def coordinate_to_key(self, x, y, z):
        return (
            int(math.floor(x / self.resolution)),
            int(math.floor(y / self.resolution)),
            int(math.floor(z / self.resolution))
        )

    def key_to_centre(self, key):
        return (
            (key[0] + 0.5) * self.resolution,
            (key[1] + 0.5) * self.resolution,
            (key[2] + 0.5) * self.resolution
        )

    def unpack_rgb(self, value):
        if self.rgb_datatype == PointField.FLOAT32:
            packed = struct.unpack(
                "I",
                struct.pack("f", float(value))
            )[0]
        else:
            packed = int(value)

        red = (packed >> 16) & 0xFF
        green = (packed >> 8) & 0xFF
        blue = packed & 0xFF

        return red, green, blue

    def cloud_callback(self, cloud):
        if cloud.header.frame_id != self.fixed_frame:
            self.rejected_clouds += 1

            rospy.logwarn_throttle(
                5.0,
                "Rejected active cloud in frame '%s'; expected '%s'",
                cloud.header.frame_id,
                self.fixed_frame
            )
            return

        field_map = {field.name: field for field in cloud.fields}

        if "rgb" not in field_map:
            rospy.logerr_throttle(
                5.0,
                "Input cloud has no packed 'rgb' field. Fields: %s",
                sorted(field_map.keys())
            )
            return

        self.rgb_datatype = field_map["rgb"].datatype
        local_updates = {}

        try:
            points = pc2.read_points(
                cloud,
                field_names=("x", "y", "z", "rgb"),
                skip_nans=True
            )

            for x, y, z, packed_rgb in points:
                if not (
                    math.isfinite(x)
                    and math.isfinite(y)
                    and math.isfinite(z)
                ):
                    continue

                red, green, blue = self.unpack_rgb(packed_rgb)
                key = self.coordinate_to_key(x, y, z)

                if key not in local_updates:
                    local_updates[key] = [
                        float(red),
                        float(green),
                        float(blue),
                        1
                    ]
                else:
                    entry = local_updates[key]
                    entry[0] += red
                    entry[1] += green
                    entry[2] += blue
                    entry[3] += 1

        except Exception as error:
            rospy.logerr_throttle(
                5.0,
                "Could not read active cloud: %s",
                str(error)
            )
            return

        with self.lock:
            for key, update in local_updates.items():
                if (
                    key not in self.voxels
                    and len(self.voxels) >= self.maximum_voxels
                ):
                    continue

                observed_r = update[0] / update[3]
                observed_g = update[1] / update[3]
                observed_b = update[2] / update[3]

                if key not in self.voxels:
                    self.voxels[key] = [
                        observed_r,
                        observed_g,
                        observed_b,
                        1
                    ]
                    continue

                voxel = self.voxels[key]
                weight = min(
                    voxel[3],
                    self.maximum_observations
                )

                alpha = 1.0 / float(weight + 1)

                voxel[0] = (
                    (1.0 - alpha) * voxel[0]
                    + alpha * observed_r
                )
                voxel[1] = (
                    (1.0 - alpha) * voxel[1]
                    + alpha * observed_g
                )
                voxel[2] = (
                    (1.0 - alpha) * voxel[2]
                    + alpha * observed_b
                )
                voxel[3] = min(
                    voxel[3] + 1,
                    self.maximum_observations
                )

        self.received_clouds += 1

        rospy.loginfo_throttle(
            5.0,
            "Clouds=%d, coloured voxels=%d, rejected=%d",
            self.received_clouds,
            len(self.voxels),
            self.rejected_clouds
        )

    def publish_voxels(self, _event):
        with self.lock:
            snapshot = list(self.voxels.items())

        if not snapshot:
            return

        marker = Marker()
        marker.header.stamp = rospy.Time.now()
        marker.header.frame_id = self.fixed_frame
        marker.ns = "multimodal_surface"
        marker.id = 0
        marker.type = Marker.CUBE_LIST
        marker.action = Marker.ADD

        marker.pose.orientation.w = 1.0

        marker.scale.x = self.resolution
        marker.scale.y = self.resolution
        marker.scale.z = self.resolution

        marker.color.a = 1.0
        marker.lifetime = rospy.Duration(0)

        for key, voxel in snapshot:
            centre_x, centre_y, centre_z = self.key_to_centre(key)

            point = Point()
            point.x = centre_x
            point.y = centre_y
            point.z = centre_z
            marker.points.append(point)

            colour = ColorRGBA()
            colour.r = max(0.0, min(1.0, voxel[0] / 255.0))
            colour.g = max(0.0, min(1.0, voxel[1] / 255.0))
            colour.b = max(0.0, min(1.0, voxel[2] / 255.0))
            colour.a = 1.0
            marker.colors.append(colour)

        output = MarkerArray()
        output.markers.append(marker)
        self.publisher.publish(output)


if __name__ == "__main__":
    rospy.init_node("multimodal_voxel_accumulator")
    MultimodalVoxelAccumulator()
    rospy.spin()#!/usr/bin/env python3

import math
import struct
import threading

import rospy
import sensor_msgs.point_cloud2 as pc2

from geometry_msgs.msg import Point
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray


class MultimodalVoxelAccumulator:
    def __init__(self):
        self.input_topic = rospy.get_param(
            "~input_topic",
            "/cave/fusion/active_points"
        )

        self.output_topic = rospy.get_param(
            "~output_topic",
            "/cave/map/multimodal_voxels"
        )

        self.fixed_frame = rospy.get_param(
            "~fixed_frame",
            "lio_odom"
        )

        self.resolution = float(rospy.get_param(
            "~resolution",
            0.10
        ))

        self.publish_rate = float(rospy.get_param(
            "~publish_rate",
            1.0
        ))

        self.maximum_observations = int(rospy.get_param(
            "~maximum_observations",
            20
        ))

        self.maximum_voxels = int(rospy.get_param(
            "~maximum_voxels",
            250000
        ))

        # key -> [mean_r, mean_g, mean_b, observations]
        self.voxels = {}
        self.lock = threading.Lock()
        self.rgb_datatype = PointField.FLOAT32
        self.received_clouds = 0
        self.rejected_clouds = 0

        self.publisher = rospy.Publisher(
            self.output_topic,
            MarkerArray,
            queue_size=1,
            latch=True
        )

        self.subscriber = rospy.Subscriber(
            self.input_topic,
            PointCloud2,
            self.cloud_callback,
            queue_size=2,
            buff_size=2 ** 25,
            tcp_nodelay=True
        )

        rospy.Timer(
            rospy.Duration(1.0 / max(self.publish_rate, 0.1)),
            self.publish_voxels
        )

        rospy.loginfo("Multimodal voxel accumulator started")
        rospy.loginfo("Input:      %s", self.input_topic)
        rospy.loginfo("Output:     %s", self.output_topic)
        rospy.loginfo("Frame:      %s", self.fixed_frame)
        rospy.loginfo("Resolution: %.3f m", self.resolution)

    def coordinate_to_key(self, x, y, z):
        return (
            int(math.floor(x / self.resolution)),
            int(math.floor(y / self.resolution)),
            int(math.floor(z / self.resolution))
        )

    def key_to_centre(self, key):
        return (
            (key[0] + 0.5) * self.resolution,
            (key[1] + 0.5) * self.resolution,
            (key[2] + 0.5) * self.resolution
        )

    def unpack_rgb(self, value):
        if self.rgb_datatype == PointField.FLOAT32:
            packed = struct.unpack(
                "I",
                struct.pack("f", float(value))
            )[0]
        else:
            packed = int(value)

        red = (packed >> 16) & 0xFF
        green = (packed >> 8) & 0xFF
        blue = packed & 0xFF

        return red, green, blue

    def cloud_callback(self, cloud):
        if cloud.header.frame_id != self.fixed_frame:
            self.rejected_clouds += 1

            rospy.logwarn_throttle(
                5.0,
                "Rejected active cloud in frame '%s'; expected '%s'",
                cloud.header.frame_id,
                self.fixed_frame
            )
            return

        field_map = {field.name: field for field in cloud.fields}

        if "rgb" not in field_map:
            rospy.logerr_throttle(
                5.0,
                "Input cloud has no packed 'rgb' field. Fields: %s",
                sorted(field_map.keys())
            )
            return

        self.rgb_datatype = field_map["rgb"].datatype
        local_updates = {}

        try:
            points = pc2.read_points(
                cloud,
                field_names=("x", "y", "z", "rgb"),
                skip_nans=True
            )

            for x, y, z, packed_rgb in points:
                if not (
                    math.isfinite(x)
                    and math.isfinite(y)
                    and math.isfinite(z)
                ):
                    continue

                red, green, blue = self.unpack_rgb(packed_rgb)
                key = self.coordinate_to_key(x, y, z)

                if key not in local_updates:
                    local_updates[key] = [
                        float(red),
                        float(green),
                        float(blue),
                        1
                    ]
                else:
                    entry = local_updates[key]
                    entry[0] += red
                    entry[1] += green
                    entry[2] += blue
                    entry[3] += 1

        except Exception as error:
            rospy.logerr_throttle(
                5.0,
                "Could not read active cloud: %s",
                str(error)
            )
            return

        with self.lock:
            for key, update in local_updates.items():
                if (
                    key not in self.voxels
                    and len(self.voxels) >= self.maximum_voxels
                ):
                    continue

                observed_r = update[0] / update[3]
                observed_g = update[1] / update[3]
                observed_b = update[2] / update[3]

                if key not in self.voxels:
                    self.voxels[key] = [
                        observed_r,
                        observed_g,
                        observed_b,
                        1
                    ]
                    continue

                voxel = self.voxels[key]
                weight = min(
                    voxel[3],
                    self.maximum_observations
                )

                alpha = 1.0 / float(weight + 1)

                voxel[0] = (
                    (1.0 - alpha) * voxel[0]
                    + alpha * observed_r
                )
                voxel[1] = (
                    (1.0 - alpha) * voxel[1]
                    + alpha * observed_g
                )
                voxel[2] = (
                    (1.0 - alpha) * voxel[2]
                    + alpha * observed_b
                )
                voxel[3] = min(
                    voxel[3] + 1,
                    self.maximum_observations
                )

        self.received_clouds += 1

        rospy.loginfo_throttle(
            5.0,
            "Clouds=%d, coloured voxels=%d, rejected=%d",
            self.received_clouds,
            len(self.voxels),
            self.rejected_clouds
        )

    def publish_voxels(self, _event):
        with self.lock:
            snapshot = list(self.voxels.items())

        if not snapshot:
            return

        marker = Marker()
        marker.header.stamp = rospy.Time.now()
        marker.header.frame_id = self.fixed_frame
        marker.ns = "multimodal_surface"
        marker.id = 0
        marker.type = Marker.CUBE_LIST
        marker.action = Marker.ADD

        marker.pose.orientation.w = 1.0

        marker.scale.x = self.resolution
        marker.scale.y = self.resolution
        marker.scale.z = self.resolution

        marker.color.a = 1.0
        marker.lifetime = rospy.Duration(0)

        for key, voxel in snapshot:
            centre_x, centre_y, centre_z = self.key_to_centre(key)

            point = Point()
            point.x = centre_x
            point.y = centre_y
            point.z = centre_z
            marker.points.append(point)

            colour = ColorRGBA()
            colour.r = max(0.0, min(1.0, voxel[0] / 255.0))
            colour.g = max(0.0, min(1.0, voxel[1] / 255.0))
            colour.b = max(0.0, min(1.0, voxel[2] / 255.0))
            colour.a = 1.0
            marker.colors.append(colour)

        output = MarkerArray()
        output.markers.append(marker)
        self.publisher.publish(output)


if __name__ == "__main__":
    rospy.init_node("multimodal_voxel_accumulator")
    MultimodalVoxelAccumulator()
    rospy.spin()