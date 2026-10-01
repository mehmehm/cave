#!/usr/bin/env python3

"""Conservative planar path follower for the simulated CHAMP A1 robot."""

import math
import threading

import rospy

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import String
from tf.transformations import euler_from_quaternion


def normalise_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


class PathFollower:
    def __init__(self):
        self.path_topic = rospy.get_param(
            "~path_topic", "/cave/exploration/path"
        )
        self.odom_topic = rospy.get_param(
            "~odom_topic", "/lio_sam/mapping/odometry"
        )
        self.cmd_vel_topic = rospy.get_param("~cmd_vel_topic", "/cmd_vel")
        self.status_topic = rospy.get_param(
            "~status_topic", "/cave/exploration/follower_status"
        )

        self.control_rate = float(rospy.get_param("~control_rate", 10.0))
        self.lookahead_distance = float(
            rospy.get_param("~lookahead_distance", 0.45)
        )
        self.goal_tolerance = float(rospy.get_param("~goal_tolerance", 0.25))
        self.maximum_linear_speed = float(
            rospy.get_param("~maximum_linear_speed", 0.12)
        )
        self.maximum_angular_speed = float(
            rospy.get_param("~maximum_angular_speed", 0.45)
        )
        self.angular_gain = float(rospy.get_param("~angular_gain", 1.4))
        self.rotate_in_place_angle = float(
            rospy.get_param("~rotate_in_place_angle", 0.65)
        )
        self.slowdown_distance = float(
            rospy.get_param("~slowdown_distance", 0.8)
        )
        self.path_timeout = float(rospy.get_param("~path_timeout", 5.0))
        self.odom_timeout = float(rospy.get_param("~odom_timeout", 1.0))

        self.lock = threading.Lock()
        self.latest_path = None
        self.latest_odom = None
        self.path_received_time = rospy.Time(0)
        self.odom_received_time = rospy.Time(0)
        self.goal_reached = False
        self.last_status = None

        self.cmd_pub = rospy.Publisher(
            self.cmd_vel_topic, Twist, queue_size=1
        )
        self.status_pub = rospy.Publisher(
            self.status_topic, String, queue_size=1, latch=True
        )
        self.path_sub = rospy.Subscriber(
            self.path_topic, Path, self.path_callback, queue_size=1
        )
        self.odom_sub = rospy.Subscriber(
            self.odom_topic, Odometry, self.odom_callback, queue_size=1
        )

        period = 1.0 / max(self.control_rate, 1.0)
        self.timer = rospy.Timer(rospy.Duration(period), self.timer_callback)
        rospy.on_shutdown(self.stop_robot)

        rospy.loginfo("A1 path follower started")
        rospy.loginfo("Path: %s", self.path_topic)
        rospy.loginfo("Velocity output: %s", self.cmd_vel_topic)

    def publish_status(self, status):
        if status == self.last_status:
            return
        self.last_status = status
        self.status_pub.publish(String(data=status))
        rospy.loginfo_throttle(2.0, "Path follower: %s", status)

    def path_callback(self, message):
        with self.lock:
            self.latest_path = message
            self.path_received_time = rospy.Time.now()
            self.goal_reached = False
        if not message.poses:
            self.stop_robot()
            self.publish_status("STOPPED: planner published an empty path")

    def odom_callback(self, message):
        with self.lock:
            self.latest_odom = message
            self.odom_received_time = rospy.Time.now()

    def stop_robot(self):
        self.cmd_pub.publish(Twist())

    @staticmethod
    def robot_yaw(odometry):
        q = odometry.pose.pose.orientation
        return euler_from_quaternion([q.x, q.y, q.z, q.w])[2]

    @staticmethod
    def distance_to_pose(robot_x, robot_y, pose):
        return math.hypot(
            pose.pose.position.x - robot_x,
            pose.pose.position.y - robot_y,
        )

    def select_lookahead(self, path, robot_x, robot_y):
        closest_index = min(
            range(len(path.poses)),
            key=lambda index: self.distance_to_pose(
                robot_x, robot_y, path.poses[index]
            ),
        )

        accumulated = 0.0
        target_index = closest_index
        for index in range(closest_index + 1, len(path.poses)):
            previous = path.poses[index - 1].pose.position
            current = path.poses[index].pose.position
            accumulated += math.hypot(
                current.x - previous.x, current.y - previous.y
            )
            target_index = index
            if accumulated >= self.lookahead_distance:
                break

        return path.poses[target_index]

    def timer_callback(self, _event):
        with self.lock:
            path = self.latest_path
            odometry = self.latest_odom
            path_time = self.path_received_time
            odom_time = self.odom_received_time
            goal_reached = self.goal_reached

        now = rospy.Time.now()

        if path is None or odometry is None:
            self.stop_robot()
            self.publish_status("WAITING: path or odometry missing")
            return

        if goal_reached:
            self.stop_robot()
            return

        if not path.poses:
            self.stop_robot()
            return

        if (now - odom_time).to_sec() > self.odom_timeout:
            self.stop_robot()
            self.publish_status("STOPPED: odometry timeout")
            return

        if (now - path_time).to_sec() > self.path_timeout:
            self.stop_robot()
            self.publish_status("STOPPED: path timeout")
            return

        path_frame = path.header.frame_id.lstrip("/")
        odom_frame = odometry.header.frame_id.lstrip("/")
        if not path_frame or path_frame != odom_frame:
            self.stop_robot()
            self.publish_status("STOPPED: path/odometry frame mismatch")
            return

        robot_x = odometry.pose.pose.position.x
        robot_y = odometry.pose.pose.position.y
        robot_yaw = self.robot_yaw(odometry)
        final_pose = path.poses[-1]
        goal_distance = self.distance_to_pose(robot_x, robot_y, final_pose)

        if goal_distance <= self.goal_tolerance:
            with self.lock:
                self.goal_reached = True
            self.stop_robot()
            self.publish_status("GOAL_REACHED")
            return

        lookahead = self.select_lookahead(path, robot_x, robot_y)
        target_x = lookahead.pose.position.x
        target_y = lookahead.pose.position.y
        desired_yaw = math.atan2(target_y - robot_y, target_x - robot_x)
        heading_error = normalise_angle(desired_yaw - robot_yaw)

        command = Twist()
        command.angular.z = max(
            -self.maximum_angular_speed,
            min(self.maximum_angular_speed, self.angular_gain * heading_error),
        )

        if abs(heading_error) >= self.rotate_in_place_angle:
            command.linear.x = 0.0
            mode = "ROTATING"
        else:
            heading_scale = max(0.0, math.cos(heading_error))
            distance_scale = min(
                1.0,
                max(0.25, goal_distance / max(self.slowdown_distance, 0.01)),
            )
            command.linear.x = (
                self.maximum_linear_speed * heading_scale * distance_scale
            )
            mode = "FOLLOWING"

        self.cmd_pub.publish(command)
        self.publish_status(
            "{} goal_distance={:.2f}m heading_error={:.2f}rad".format(
                mode, goal_distance, heading_error
            )
        )


if __name__ == "__main__":
    rospy.init_node("path_follower")
    PathFollower()
    rospy.spin()
