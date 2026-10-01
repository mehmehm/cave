#!/usr/bin/env python3

import rospy
import struct
import math
import threading

from collections import OrderedDict

import sensor_msgs.point_cloud2 as pc2

from sensor_msgs.msg import PointCloud2
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA


class FastRGBVoxelVisualizer:
    def __init__(self):

        # ============================================================
        # Parameters
        # ============================================================

        self.input_topic = rospy.get_param(
            "~input_topic",
            "/fusion/rgb_lidar_points"
        )

        self.output_topic = rospy.get_param(
            "~output_topic",
            "/fusion/rgb_voxel_markers"
        )

        # IMPORTANT:
        # This must match the frame of /fusion/rgb_lidar_points.
        #
        # For the A1 this will usually be either:
        #   odom
        # or
        #   a1/odom
        #
        # Check with:
        # rostopic echo -n 1 /fusion/rgb_lidar_points/header
        self.frame_id = rospy.get_param(
            "~frame_id",
            "odom"
        )

        self.voxel_size = rospy.get_param(
            "~voxel_size",
            0.10
        )

        self.max_voxels = rospy.get_param(
            "~max_voxels",
            20000
        )

        self.publish_rate = rospy.get_param(
            "~publish_rate",
            1.0
        )

        if self.voxel_size <= 0:
            raise ValueError("voxel_size must be > 0")

        if self.publish_rate <= 0:
            raise ValueError("publish_rate must be > 0")

        # OrderedDict is used so recently observed voxels can be kept.
        #
        # key:
        #   (voxel_x, voxel_y, voxel_z)
        #
        # value:
        #   accumulated colour information
        self.voxels = OrderedDict()

        self.dirty = False

        # Subscriber callback and timer can run independently.
        self.lock = threading.Lock()

        # ============================================================
        # Subscriber
        # ============================================================

        rospy.Subscriber(
            self.input_topic,
            PointCloud2,
            self.cloud_cb,
            queue_size=1,
            buff_size=2**24
        )

        # ============================================================
        # Publisher
        # ============================================================

        self.pub = rospy.Publisher(
            self.output_topic,
            Marker,
            queue_size=1
        )

        # ============================================================
        # Timer
        # ============================================================

        rospy.Timer(
            rospy.Duration(1.0 / self.publish_rate),
            self.publish_timer
        )

        rospy.loginfo("A1 RGB voxel visualizer started.")

        rospy.loginfo(
            "Input cloud: %s",
            self.input_topic
        )

        rospy.loginfo(
            "Output marker: %s",
            self.output_topic
        )

        rospy.loginfo(
            "Voxel frame: %s",
            self.frame_id
        )

        rospy.loginfo(
            "voxel_size: %.3f m, max_voxels: %d, publish_rate: %.2f Hz",
            self.voxel_size,
            self.max_voxels,
            self.publish_rate
        )

    # ================================================================
    # RGB conversion
    # ================================================================

    def unpack_rgb(self, rgb_value):
        """
        Convert packed PointCloud2/PCL RGB value into R, G, B.

        RGB is commonly stored as FLOAT32 even though its underlying
        bits actually represent a packed uint32 value:

            0x00RRGGBB
        """

        if isinstance(rgb_value, float):

            rgb_uint32 = struct.unpack(
                "<I",
                struct.pack("<f", rgb_value)
            )[0]

        else:

            rgb_uint32 = int(rgb_value)

        r = (rgb_uint32 >> 16) & 255
        g = (rgb_uint32 >> 8) & 255
        b = rgb_uint32 & 255

        return r, g, b

    # ================================================================
    # Voxel indexing
    # ================================================================

    def voxel_key(self, x, y, z):
        """
        Convert XYZ position into a discrete voxel index.

        floor() is important because the cave contains negative
        world coordinates.
        """

        return (
            math.floor(x / self.voxel_size),
            math.floor(y / self.voxel_size),
            math.floor(z / self.voxel_size)
        )

    # ================================================================
    # Point cloud callback
    # ================================================================

    def cloud_cb(self, msg):

        # ------------------------------------------------------------
        # Validate RGB field
        # ------------------------------------------------------------

        field_names = [
            field.name
            for field in msg.fields
        ]

        if "rgb" not in field_names:

            rospy.logwarn_throttle(
                2.0,
                "Input cloud '%s' has no 'rgb' field.",
                self.input_topic
            )

            return

        # ------------------------------------------------------------
        # Validate coordinate frame
        # ------------------------------------------------------------

        cloud_frame = msg.header.frame_id.lstrip("/")
        expected_frame = self.frame_id.lstrip("/")

        if cloud_frame != expected_frame:

            rospy.logerr_throttle(
                2.0,
                "Cannot voxelise RGB cloud: "
                "cloud frame='%s', expected='%s'. "
                "Transform the RGB-LiDAR cloud into the mapping frame first.",
                msg.header.frame_id,
                self.frame_id
            )

            return

        # ------------------------------------------------------------
        # Accumulate RGB voxels
        # ------------------------------------------------------------

        points_processed = 0

        with self.lock:

            for p in pc2.read_points(
                msg,
                field_names=("x", "y", "z", "rgb"),
                skip_nans=True
            ):

                x, y, z, rgb_value = p

                key = self.voxel_key(
                    x,
                    y,
                    z
                )

                r, g, b = self.unpack_rgb(
                    rgb_value
                )

                if key not in self.voxels:

                    self.voxels[key] = {
                        "count": 0,
                        "r": 0.0,
                        "g": 0.0,
                        "b": 0.0
                    }

                voxel = self.voxels[key]

                voxel["count"] += 1

                voxel["r"] += r
                voxel["g"] += g
                voxel["b"] += b

                # Mark this voxel as recently observed.
                self.voxels.move_to_end(key)

                points_processed += 1

            # --------------------------------------------------------
            # Limit memory usage
            # --------------------------------------------------------

            if self.max_voxels > 0:

                while len(self.voxels) > self.max_voxels:

                    # Remove oldest observed voxel.
                    self.voxels.popitem(last=False)

            if points_processed > 0:
                self.dirty = True

        rospy.loginfo_throttle(
            2.0,
            "RGB voxel map: %d voxels",
            len(self.voxels)
        )

    # ================================================================
    # RViz publisher
    # ================================================================

    def publish_timer(self, event):

        # ------------------------------------------------------------
        # Copy current voxel state
        # ------------------------------------------------------------

        with self.lock:

            if not self.dirty:
                return

            items = list(
                self.voxels.items()
            )

            self.dirty = False

        # ------------------------------------------------------------
        # Build RViz marker
        # ------------------------------------------------------------

        marker = Marker()

        marker.header.frame_id = self.frame_id
        marker.header.stamp = rospy.Time.now()

        marker.ns = "rgb_voxel_map"

        marker.id = 0

        marker.type = Marker.CUBE_LIST
        marker.action = Marker.ADD

        marker.pose.orientation.w = 1.0

        # Each cube is one voxel.
        marker.scale.x = self.voxel_size
        marker.scale.y = self.voxel_size
        marker.scale.z = self.voxel_size

        marker.points = []
        marker.colors = []

        # ------------------------------------------------------------
        # Convert voxel grid into cubes
        # ------------------------------------------------------------

        for key, voxel in items:

            ix, iy, iz = key

            # Centre of this voxel.
            point = Point()

            point.x = (
                ix + 0.5
            ) * self.voxel_size

            point.y = (
                iy + 0.5
            ) * self.voxel_size

            point.z = (
                iz + 0.5
            ) * self.voxel_size

            count = float(
                voxel["count"]
            )

            # Average RGB colour of all points that entered this voxel.
            colour = ColorRGBA()

            colour.r = (
                voxel["r"] / count
            ) / 255.0

            colour.g = (
                voxel["g"] / count
            ) / 255.0

            colour.b = (
                voxel["b"] / count
            ) / 255.0

            colour.a = 1.0

            marker.points.append(
                point
            )

            marker.colors.append(
                colour
            )

        # ------------------------------------------------------------
        # Publish
        # ------------------------------------------------------------

        self.pub.publish(marker)


if __name__ == "__main__":

    rospy.init_node(
        "fast_rgb_voxel_visualizer"
    )

    FastRGBVoxelVisualizer()

    rospy.spin()