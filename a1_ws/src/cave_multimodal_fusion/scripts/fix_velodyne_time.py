#!/usr/bin/env python3

import math


import rospy
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs import point_cloud2


SCAN_PERIOD = 0.1  # 10 Hz LiDAR


def callback(msg):
    output_points = []

    for x, y, z, intensity, ring, _ in point_cloud2.read_points(
        msg,
        field_names=("x", "y", "z", "intensity", "ring", "time"),
        skip_nans=True
    ):
        angle = math.atan2(y, x)

        # Map horizontal angle from [-pi, pi] to [0, SCAN_PERIOD).
        relative_time = ((angle + math.pi) / (2.0 * math.pi)) * scan_period

        output_points.append((
            float(x),
            float(y),
            float(z),
            float(intensity),
            int(ring),
            float(relative_time)
        ))

    fields = [
        PointField("x", 0, PointField.FLOAT32, 1),
        PointField("y", 4, PointField.FLOAT32, 1),
        PointField("z", 8, PointField.FLOAT32, 1),
        PointField("intensity", 12, PointField.FLOAT32, 1),
        PointField("ring", 16, PointField.UINT16, 1),
        PointField("time", 18, PointField.FLOAT32, 1),
    ]

    output = point_cloud2.create_cloud(msg.header, fields, output_points)
    publisher.publish(output)


rospy.init_node("fix_velodyne_time")

scan_period = rospy.get_param("~scan_period", SCAN_PERIOD)

publisher = rospy.Publisher(
    "/velodyne_points_timed",
    PointCloud2,
    queue_size=2
)

rospy.Subscriber(
    "/velodyne_points",
    PointCloud2,
    callback,
    queue_size=2,
    buff_size=2**24
)

rospy.loginfo(
    "Repairing Velodyne point times: /velodyne_points -> "
    "/velodyne_points_timed"
)

rospy.spin()