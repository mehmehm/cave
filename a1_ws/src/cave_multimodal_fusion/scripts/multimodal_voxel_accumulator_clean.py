#!/usr/bin/env python3
"""Persistent coloured voxel accumulator with an optional RViz-only Z slice."""

import math
import struct
import threading

import rospy
import sensor_msgs.point_cloud2 as pc2
import tf2_ros
from geometry_msgs.msg import Point
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import ColorRGBA, Float32, Int32
from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud
from visualization_msgs.msg import Marker, MarkerArray


class MultimodalVoxelAccumulator:
    def __init__(self):
        self.input_topic = rospy.get_param(
            "~input_topic", "/cave/fusion/active_points"
        )
        self.output_topic = rospy.get_param(
            "~output_topic", "/cave/map/multimodal_voxels"
        )
        self.fixed_frame = rospy.get_param("~fixed_frame", "lio_odom")
        self.resolution = float(rospy.get_param("~resolution", 0.15))
        self.publish_rate = float(rospy.get_param("~publish_rate", 1.0))
        self.maximum_observations = max(
            1, int(rospy.get_param("~maximum_observations", 20))
        )
        self.maximum_voxels = max(
            1, int(rospy.get_param("~maximum_voxels", 500000))
        )
        self.tf_timeout = float(rospy.get_param("~tf_timeout", 0.25))

        # These limits affect only the published RViz marker. All voxels remain
        # stored, so roof filtering does not alter the research map.
        self.visualization_min_z = float(rospy.get_param(
            "~visualization_min_z", -1.0e9
        ))
        self.visualization_max_z = float(rospy.get_param(
            "~visualization_max_z", 1.0e9
        ))

        # key -> one colour estimate per modality. Keeping the evidence
        # separate prevents grey LiDAR fallback samples from washing out RGB
        # and thermal colours in the persistent map.
        # Each modality entry is [mean_r, mean_g, mean_b, observation_count].
        self.voxels = {}
        self.lock = threading.Lock()
        self.rgb_datatype = PointField.FLOAT32
        self.received_clouds = 0
        self.rejected_clouds = 0
        self.capacity_warnings = 0

        self.tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(30.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.publisher = rospy.Publisher(
            self.output_topic, MarkerArray, queue_size=1, latch=True
        )
        self.voxel_count_pub = rospy.Publisher(
            "/cave/map/voxel_count", Int32, queue_size=1, latch=True
        )
        self.visible_voxel_count_pub = rospy.Publisher(
            "/cave/map/visible_voxel_count", Int32, queue_size=1, latch=True
        )
        self.lidar_voxel_count_pub = rospy.Publisher(
            "/cave/map/lidar_voxel_count", Int32, queue_size=1, latch=True
        )
        self.rgb_voxel_count_pub = rospy.Publisher(
            "/cave/map/rgb_voxel_count", Int32, queue_size=1, latch=True
        )
        self.thermal_voxel_count_pub = rospy.Publisher(
            "/cave/map/thermal_voxel_count", Int32, queue_size=1, latch=True
        )
        self.capacity_percent_pub = rospy.Publisher(
            "/cave/map/capacity_percent", Float32, queue_size=1, latch=True
        )
        self.subscriber = rospy.Subscriber(
            self.input_topic,
            PointCloud2,
            self.cloud_callback,
            queue_size=2,
            buff_size=2 ** 25,
            tcp_nodelay=True,
        )

        rospy.Timer(
            rospy.Duration(1.0 / max(self.publish_rate, 0.1)),
            self.publish_voxels,
        )

        rospy.loginfo("Multimodal voxel accumulator started")
        rospy.loginfo("Input:      %s", self.input_topic)
        rospy.loginfo("Output:     %s", self.output_topic)
        rospy.loginfo("Frame:      %s", self.fixed_frame)
        rospy.loginfo("Resolution: %.3f m", self.resolution)
        rospy.loginfo("Capacity:   %d voxels", self.maximum_voxels)
        rospy.loginfo(
            "RViz Z slice: [%.3f, %.3f] m",
            self.visualization_min_z,
            self.visualization_max_z,
        )

    def coordinate_to_key(self, x, y, z):
        return (
            int(math.floor(x / self.resolution)),
            int(math.floor(y / self.resolution)),
            int(math.floor(z / self.resolution)),
        )

    def key_to_centre(self, key):
        return (
            (key[0] + 0.5) * self.resolution,
            (key[1] + 0.5) * self.resolution,
            (key[2] + 0.5) * self.resolution,
        )

    def unpack_rgb(self, value):
        if self.rgb_datatype == PointField.FLOAT32:
            packed = struct.unpack("I", struct.pack("f", float(value)))[0]
        else:
            packed = int(value)
        return (
            (packed >> 16) & 0xFF,
            (packed >> 8) & 0xFF,
            packed & 0xFF,
        )

    def transform_cloud(self, cloud):
        source_frame = cloud.header.frame_id.lstrip("/")
        target_frame = self.fixed_frame.lstrip("/")
        if source_frame == target_frame:
            return cloud

        transform = self.tf_buffer.lookup_transform(
            target_frame,
            source_frame,
            cloud.header.stamp,
            rospy.Duration(self.tf_timeout),
        )
        transformed = do_transform_cloud(cloud, transform)
        transformed.header.frame_id = target_frame
        return transformed

    def cloud_callback(self, cloud):
        try:
            cloud = self.transform_cloud(cloud)
        except (
            tf2_ros.LookupException,
            tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException,
        ) as error:
            self.rejected_clouds += 1
            rospy.logwarn_throttle(
                5.0,
                "Could not transform active cloud from '%s' to '%s': %s",
                cloud.header.frame_id,
                self.fixed_frame,
                str(error),
            )
            return

        field_map = {field.name: field for field in cloud.fields}
        if "rgb" not in field_map or "modality" not in field_map:
            rospy.logerr_throttle(
                5.0,
                "Input cloud must contain 'rgb' and 'modality'. Fields: %s",
                sorted(field_map.keys()),
            )
            return

        self.rgb_datatype = field_map["rgb"].datatype
        local_updates = {}
        try:
            points = pc2.read_points(
                cloud,
                field_names=("x", "y", "z", "rgb", "modality"),
                skip_nans=True,
            )
            for x, y, z, packed_rgb, modality_value in points:
                if not all(math.isfinite(v) for v in (x, y, z)):
                    continue
                red, green, blue = self.unpack_rgb(packed_rgb)
                modality = int(round(float(modality_value)))
                if modality not in (0, 1, 2):
                    modality = 0
                key = self.coordinate_to_key(x, y, z)
                modes = local_updates.setdefault(key, {})
                update = modes.setdefault(modality, [0.0, 0.0, 0.0, 0])
                update[0] += red
                update[1] += green
                update[2] += blue
                update[3] += 1
        except Exception as error:
            rospy.logerr_throttle(5.0, "Could not read active cloud: %s", error)
            return

        with self.lock:
            for key, mode_updates in local_updates.items():
                if key not in self.voxels and len(self.voxels) >= self.maximum_voxels:
                    self.capacity_warnings += 1
                    continue

                if key not in self.voxels:
                    self.voxels[key] = {}

                voxel_modes = self.voxels[key]
                for modality, update in mode_updates.items():
                    observed_r = update[0] / update[3]
                    observed_g = update[1] / update[3]
                    observed_b = update[2] / update[3]

                    if modality not in voxel_modes:
                        voxel_modes[modality] = [
                            observed_r, observed_g, observed_b, 1
                        ]
                        continue

                    voxel = voxel_modes[modality]
                    weight = min(voxel[3], self.maximum_observations)
                    alpha = 1.0 / float(weight + 1)
                    voxel[0] = (1.0 - alpha) * voxel[0] + alpha * observed_r
                    voxel[1] = (1.0 - alpha) * voxel[1] + alpha * observed_g
                    voxel[2] = (1.0 - alpha) * voxel[2] + alpha * observed_b
                    voxel[3] = min(voxel[3] + 1, self.maximum_observations)

            stored_count = len(self.voxels)

        self.received_clouds += 1
        self.voxel_count_pub.publish(stored_count)
        self.capacity_percent_pub.publish(
            100.0 * float(stored_count) / float(self.maximum_voxels)
        )

        if self.capacity_warnings:
            rospy.logwarn_throttle(
                5.0,
                "Voxel capacity reached (%d); new voxels are being skipped",
                self.maximum_voxels,
            )
        rospy.loginfo_throttle(
            5.0,
            "Clouds=%d stored_voxels=%d rejected=%d",
            self.received_clouds,
            stored_count,
            self.rejected_clouds,
        )

    def publish_voxels(self, _event):
        with self.lock:
            snapshot = list(self.voxels.items())
        if not snapshot:
            return

        now = rospy.Time.now()
        display_scale = self.resolution * 1.05
        namespace = {0: "lidar_only", 1: "rgb_lidar", 2: "thermal_lidar"}
        markers = {}
        for modality in (0, 1, 2):
            marker = Marker()
            marker.header.stamp = now
            marker.header.frame_id = self.fixed_frame
            marker.ns = namespace[modality]
            marker.id = modality
            marker.type = Marker.CUBE_LIST
            marker.action = Marker.ADD
            marker.pose.orientation.w = 1.0
            marker.scale.x = display_scale
            marker.scale.y = display_scale
            marker.scale.z = display_scale
            marker.color.a = 1.0
            marker.lifetime = rospy.Duration(0)
            markers[modality] = marker

        for key, voxel_modes in snapshot:
            centre_x, centre_y, centre_z = self.key_to_centre(key)
            if not (
                self.visualization_min_z
                <= centre_z
                <= self.visualization_max_z
            ):
                continue

            # Dominant observation count determines how this voxel is shown.
            # RGB wins an exact tie, then thermal, then LiDAR.
            tie_priority = {0: 0, 2: 1, 1: 2}
            modality = max(
                voxel_modes.keys(),
                key=lambda mode: (
                    voxel_modes[mode][3], tie_priority.get(mode, -1)
                ),
            )
            voxel = voxel_modes[modality]
            marker = markers[modality]
            marker.points.append(Point(centre_x, centre_y, centre_z))
            marker.colors.append(ColorRGBA(
                max(0.0, min(1.0, voxel[0] / 255.0)),
                max(0.0, min(1.0, voxel[1] / 255.0)),
                max(0.0, min(1.0, voxel[2] / 255.0)),
                1.0,
            ))

        output = MarkerArray()
        output.markers.extend(markers[mode] for mode in (0, 1, 2))
        self.publisher.publish(output)
        self.voxel_count_pub.publish(len(snapshot))
        counts = {mode: len(markers[mode].points) for mode in (0, 1, 2)}
        self.visible_voxel_count_pub.publish(sum(counts.values()))
        self.lidar_voxel_count_pub.publish(counts[0])
        self.rgb_voxel_count_pub.publish(counts[1])
        self.thermal_voxel_count_pub.publish(counts[2])


if __name__ == "__main__":
    rospy.init_node("multimodal_voxel_accumulator")
    MultimodalVoxelAccumulator()
    rospy.spin()
