#!/usr/bin/env python3

"""Conservative planar path follower for the simulated CHAMP A1 robot."""

import math
import threading

import rospy

from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import Bool, Float32, String
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
        self.approach_slope_topic = rospy.get_param(
            '~approach_slope_topic', '/cave/terrain/approach_slope_deg')
        self.unsafe_ahead_topic = rospy.get_param(
            '~unsafe_ahead_topic', '/cave/terrain/unsafe_ahead')
        self.slope_message_timeout = float(rospy.get_param('~slope_message_timeout', 2.0))
        self.slowdown_slope_deg = float(rospy.get_param('~slowdown_slope_deg', 7.0))
        self.moderate_climb_deg = float(rospy.get_param('~moderate_climb_deg', 7.0))
        self.maximum_climb_pitch_deg = float(rospy.get_param('~maximum_climb_pitch_deg', 17.0))
        self.maximum_body_pitch_deg = float(rospy.get_param('~maximum_body_pitch_deg', 22.0))
        self.maximum_body_roll_deg = float(rospy.get_param('~maximum_body_roll_deg', 16.0))
        self.approach_speed_factor = float(rospy.get_param('~approach_speed_factor', 0.55))
        self.climb_speed_factor = float(rospy.get_param('~climb_speed_factor', 1.15))
        self.maximum_climb_speed = float(rospy.get_param('~maximum_climb_speed', 0.20))
        self.slope_guard_required = bool(rospy.get_param('~slope_guard_required', True))

        self.lock = threading.Lock()
        self.latest_path = None
        self.latest_odom = None
        self.path_received_time = rospy.Time(0)
        self.odom_received_time = rospy.Time(0)
        self.goal_reached = False
        self.completed_endpoint = None
        self.last_status = None
        self.approach_slope = float('nan')
        self.unsafe_ahead = False
        self.slope_received_time = rospy.Time(0)
        self.unsafe_received_time = rospy.Time(0)

        self.cmd_pub = rospy.Publisher(
            self.cmd_vel_topic, Twist, queue_size=1
        )
        self.reached_pub = rospy.Publisher('/cave/exploration/reached_goal', PoseStamped, queue_size=1)
        self.status_pub = rospy.Publisher(
            self.status_topic, String, queue_size=1, latch=True
        )
        self.path_sub = rospy.Subscriber(
            self.path_topic, Path, self.path_callback, queue_size=1
        )
        self.odom_sub = rospy.Subscriber(
            self.odom_topic, Odometry, self.odom_callback, queue_size=1
        )
        self.slope_sub = rospy.Subscriber(self.approach_slope_topic, Float32,
                                          self.slope_callback, queue_size=1)
        self.unsafe_sub = rospy.Subscriber(self.unsafe_ahead_topic, Bool,
                                           self.unsafe_callback, queue_size=1)

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
            if message.poses and self.completed_endpoint is not None:
                frame, x, y = self.completed_endpoint
                point = message.poses[-1].pose.position
                self.goal_reached = (frame == message.header.frame_id.lstrip('/')
                                     and math.hypot(point.x-x, point.y-y) <= 0.10)
                if not self.goal_reached:
                    self.completed_endpoint = None
            else:
                self.goal_reached = False
        if not message.poses:
            self.stop_robot()
            self.publish_status("STOPPED: planner published an empty path")

    def odom_callback(self, message):
        with self.lock:
            self.latest_odom = message
            self.odom_received_time = rospy.Time.now()

    def slope_callback(self, message):
        with self.lock:
            self.approach_slope = message.data
            self.slope_received_time = rospy.Time.now()

    def unsafe_callback(self, message):
        with self.lock:
            self.unsafe_ahead = message.data
            self.unsafe_received_time = rospy.Time.now()

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
        """Return a point a fixed arc distance ahead on the path.

        A* removes collinear grid cells, leaving some long segments. Jumping
        directly to the next vertex can overshoot the requested lookahead and
        cause a large turn whenever the path is replanned.
        """
        points = [pose.pose.position for pose in path.poses]
        if len(points) == 1:
            return points[0].x, points[0].y

        closest = None
        for index in range(len(points) - 1):
            a, b = points[index], points[index + 1]
            dx, dy = b.x - a.x, b.y - a.y
            length_squared = dx * dx + dy * dy
            if length_squared <= 1e-12:
                continue
            fraction = max(0.0, min(1.0,
                ((robot_x - a.x) * dx + (robot_y - a.y) * dy)
                / length_squared))
            px, py = a.x + fraction * dx, a.y + fraction * dy
            candidate = ((robot_x - px) ** 2 + (robot_y - py) ** 2,
                         index, fraction)
            if closest is None or candidate[0] < closest[0]:
                closest = candidate

        if closest is None:
            return points[-1].x, points[-1].y

        _, start_index, fraction = closest
        remaining = max(0.0, self.lookahead_distance)
        for index in range(start_index, len(points) - 1):
            a, b = points[index], points[index + 1]
            dx, dy = b.x - a.x, b.y - a.y
            length = math.hypot(dx, dy)
            if length <= 1e-12:
                continue
            start_fraction = fraction if index == start_index else 0.0
            available = (1.0 - start_fraction) * length
            if remaining <= available:
                target_fraction = start_fraction + remaining / length
                return (a.x + target_fraction * dx,
                        a.y + target_fraction * dy)
            remaining -= available

        return points[-1].x, points[-1].y

    def timer_callback(self, _event):
        with self.lock:
            path = self.latest_path
            odometry = self.latest_odom
            path_time = self.path_received_time
            odom_time = self.odom_received_time
            goal_reached = self.goal_reached
            slope = self.approach_slope
            unsafe_ahead = self.unsafe_ahead
            slope_time = self.slope_received_time
            unsafe_time = self.unsafe_received_time

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

        if self.slope_guard_required and (
            (now - slope_time).to_sec() > self.slope_message_timeout
            or (now - unsafe_time).to_sec() > self.slope_message_timeout
        ):
            self.stop_robot()
            self.publish_status('STOPPED: slope guard missing or stale')
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
        q = odometry.pose.pose.orientation
        roll, pitch, _ = euler_from_quaternion([q.x, q.y, q.z, q.w])
        roll_deg = abs(math.degrees(roll))
        pitch_deg = math.degrees(pitch)
        if unsafe_ahead or abs(pitch_deg) >= self.maximum_body_pitch_deg or roll_deg >= self.maximum_body_roll_deg:
            self.stop_robot()
            self.publish_status('STOPPED: slope hazard ahead or body tilt exceeded limit')
            return
        final_pose = path.poses[-1]
        goal_distance = self.distance_to_pose(robot_x, robot_y, final_pose)

        if goal_distance <= self.goal_tolerance:
            with self.lock:
                if self.latest_path is not path:
                    return
                self.goal_reached = True
                self.completed_endpoint = (path_frame, final_pose.pose.position.x, final_pose.pose.position.y)
            reached = PoseStamped()
            reached.header.frame_id = path.header.frame_id
            reached.header.stamp = rospy.Time.now()
            reached.pose = final_pose.pose
            self.reached_pub.publish(reached)
            self.stop_robot()
            self.publish_status("GOAL_REACHED")
            return

        target_x, target_y = self.select_lookahead(path, robot_x, robot_y)
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
            mode = "FOLLOWING"
            heading_scale = max(0.0, math.cos(heading_error))
            distance_scale = min(
                1.0,
                max(0.25, goal_distance / max(self.slowdown_distance, 0.01)),
            )
            command.linear.x = (
                self.maximum_linear_speed * heading_scale * distance_scale
            )
            if math.isfinite(slope) and abs(slope) >= self.slowdown_slope_deg:
                command.linear.x *= self.approach_speed_factor
                mode = 'APPROACHING_SLOPE'
            # Increase speed only for a measured, moderate uphill while
            # upright. On a steeper climb, keep the conservative approach speed.
            if (self.moderate_climb_deg <= pitch_deg < self.maximum_climb_pitch_deg
                    and roll_deg < self.maximum_body_roll_deg * 0.5):
                command.linear.x = min(
                    self.maximum_climb_speed,
                    self.maximum_linear_speed * self.climb_speed_factor
                    * heading_scale * distance_scale,
                )
                mode = 'CLIMBING'

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
