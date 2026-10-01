#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Imu


class ImuTimestampFilter:
    def __init__(self):
        self.last_stamp = None
        self.accepted = 0
        self.dropped = 0

        input_topic = rospy.get_param(
            "~input_topic",
            "/a1/trunk_imu"
        )

        output_topic = rospy.get_param(
            "~output_topic",
            "/a1/imu/lio_filtered"
        )

        self.output_frame = rospy.get_param(
            "~output_frame",
            "a1/imu_link"
        )

        self.publisher = rospy.Publisher(
            output_topic,
            Imu,
            queue_size=2000
        )

        self.subscriber = rospy.Subscriber(
            input_topic,
            Imu,
            self.imu_callback,
            queue_size=2000,
            tcp_nodelay=True
        )

        rospy.loginfo("LIO-SAM IMU timestamp filter started")
        rospy.loginfo("Input:  %s", input_topic)
        rospy.loginfo("Output: %s", output_topic)
        rospy.loginfo("Frame:  %s", self.output_frame)

    def imu_callback(self, message):
        current_stamp = message.header.stamp

        if current_stamp == rospy.Time(0):
            self.dropped += 1
            return

        if (
            self.last_stamp is not None
            and current_stamp <= self.last_stamp
        ):
            self.dropped += 1

            rospy.logwarn_throttle(
                5.0,
                "Dropped non-increasing IMU timestamp. "
                "Accepted=%d Dropped=%d",
                self.accepted,
                self.dropped
            )
            return

        message.header.frame_id = self.output_frame

        self.publisher.publish(message)

        self.last_stamp = current_stamp
        self.accepted += 1


if __name__ == "__main__":
    rospy.init_node("lio_imu_timestamp_filter")
    ImuTimestampFilter()
    rospy.spin()