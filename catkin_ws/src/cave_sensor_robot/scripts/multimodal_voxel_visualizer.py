#!/usr/bin/env python3

import rospy
import struct
import sensor_msgs.point_cloud2 as pc2

from sensor_msgs.msg import PointCloud2
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point


class MultimodalVoxelVisualizer:
    def __init__(self):
        self.voxel_size = rospy.get_param("~voxel_size", 0.20)
        self.frame_id = rospy.get_param("~frame_id", "odom")
        self.max_voxels = rospy.get_param("~max_voxels", 6000)

        self.voxels = {}

        rospy.Subscriber(
            "/fusion/rgb_thermal_lidar_points",
            PointCloud2,
            self.cloud_cb,
            queue_size=1
        )

        self.rgb_pub = rospy.Publisher(
            "/fusion/rgb_voxel_markers",
            MarkerArray,
            queue_size=1
        )

        self.thermal_pub = rospy.Publisher(
            "/fusion/thermal_voxel_markers",
            MarkerArray,
            queue_size=1
        )

    def unpack_rgb_float(self, rgb_float):
        rgb_uint32 = struct.unpack("I", struct.pack("f", rgb_float))[0]
        r = (rgb_uint32 >> 16) & 255
        g = (rgb_uint32 >> 8) & 255
        b = rgb_uint32 & 255
        return r, g, b

    def voxel_key(self, x, y, z):
        return (
            int(x / self.voxel_size),
            int(y / self.voxel_size),
            int(z / self.voxel_size)
        )

    def cloud_cb(self, msg):
        field_names = [field.name for field in msg.fields]

        if "rgb" not in field_names or "thermal" not in field_names:
            rospy.logwarn_throttle(2.0, "Fused cloud needs rgb and thermal fields.")
            return

        for p in pc2.read_points(
            msg,
            field_names=("x", "y", "z", "rgb", "thermal"),
            skip_nans=True
        ):
            x, y, z, rgb_float, thermal = p
            key = self.voxel_key(x, y, z)

            r, g, b = self.unpack_rgb_float(rgb_float)

            if key not in self.voxels:
                self.voxels[key] = {
                    "count": 0,
                    "x": 0.0,
                    "y": 0.0,
                    "z": 0.0,
                    "r": 0.0,
                    "g": 0.0,
                    "b": 0.0,
                    "thermal": 0.0
                }

            v = self.voxels[key]
            v["count"] += 1
            v["x"] += x
            v["y"] += y
            v["z"] += z
            v["r"] += r
            v["g"] += g
            v["b"] += b
            v["thermal"] += thermal

        self.publish_markers(msg.header.stamp)

    def make_marker(self, marker_id, position, color, namespace):
        marker = Marker()
        marker.header.frame_id = self.frame_id
        marker.header.stamp = rospy.Time.now()
        marker.ns = namespace
        marker.id = marker_id
        marker.type = Marker.CUBE
        marker.action = Marker.ADD

        marker.pose.position.x = position[0]
        marker.pose.position.y = position[1]
        marker.pose.position.z = position[2]
        marker.pose.orientation.w = 1.0

        marker.scale.x = self.voxel_size
        marker.scale.y = self.voxel_size
        marker.scale.z = self.voxel_size

        marker.color.r = color[0]
        marker.color.g = color[1]
        marker.color.b = color[2]
        marker.color.a = 0.8

        marker.lifetime = rospy.Duration(0.0)
        return marker

    def publish_markers(self, stamp):
        rgb_array = MarkerArray()
        thermal_array = MarkerArray()

        items = list(self.voxels.items())[-self.max_voxels:]

        for i, (key, v) in enumerate(items):
            c = float(v["count"])

            x = v["x"] / c
            y = v["y"] / c
            z = v["z"] / c

            r = (v["r"] / c) / 255.0
            g = (v["g"] / c) / 255.0
            b = (v["b"] / c) / 255.0

            thermal = (v["thermal"] / c) / 255.0

            rgb_marker = self.make_marker(
                i,
                (x, y, z),
                (r, g, b),
                "rgb_voxels"
            )

            # Simple thermal colour ramp:
            # low thermal = blue-ish, high thermal = red-ish
            thermal_marker = self.make_marker(
                i,
                (x, y, z),
                (thermal, 0.0, 1.0 - thermal),
                "thermal_voxels"
            )

            rgb_array.markers.append(rgb_marker)
            thermal_array.markers.append(thermal_marker)

        self.rgb_pub.publish(rgb_array)
        self.thermal_pub.publish(thermal_array)


if __name__ == "__main__":
    rospy.init_node("multimodal_voxel_visualizer")
    MultimodalVoxelVisualizer()
    rospy.spin()