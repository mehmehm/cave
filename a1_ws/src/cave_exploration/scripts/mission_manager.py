#!/usr/bin/env python3
"""Time-boxed exploration mission with return to the start pose.

Sits between the frontier detector and the A* planner:

    frontier_detector --/cave/exploration/best_frontier--> mission_manager
    frontier_detector --/cave/exploration/goal_status----> mission_manager
    mission_manager   --/cave/mission/goal---------------> astar_planner
    mission_manager   --/cave/mission/goal_status--------> astar_planner

Phases (published on /cave/mission/phase):
  WAITING    no odometry yet
  EXPLORING  frontier goals are passed through unchanged
  RETURNING  the start pose is sent to the planner instead
  RETURNED   robot is within return_tolerance of the start; robot stops
  TIMEOUT    max_run_seconds reached before getting home

The robot turns for home when ANY of these holds:
  * elapsed >= max_run_seconds - return_reserve, where return_reserve grows with
    the straight-line distance home (distance * tortuosity / return_speed)
  * the frontier detector has reported NO_GOAL for no_goal_seconds
    (exploration finished early)

With shutdown_on_finish:=true the node exits after RETURNED/TIMEOUT. Launch it
with required="true" so the whole run (and the metrics logger) ends cleanly.
All times are ROS (simulation) time.
"""

import math
import threading

import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32, String


class MissionManager:
    def __init__(self):
        self.odom_topic = rospy.get_param("~odom_topic", "/lio_sam/mapping/odometry")
        self.frontier_goal_topic = rospy.get_param(
            "~frontier_goal_topic", "/cave/exploration/best_frontier")
        self.frontier_status_topic = rospy.get_param(
            "~frontier_status_topic", "/cave/exploration/goal_status")
        self.goal_topic = rospy.get_param("~goal_topic", "/cave/mission/goal")
        self.goal_status_topic = rospy.get_param(
            "~goal_status_topic", "/cave/mission/goal_status")

        self.max_run_seconds = float(rospy.get_param("~max_run_seconds", 2400.0))
        self.minimum_return_reserve = float(
            rospy.get_param("~minimum_return_reserve", 240.0))
        self.return_speed = float(rospy.get_param("~return_speed", 0.25))
        self.tortuosity = float(rospy.get_param("~tortuosity", 1.8))
        self.no_goal_seconds = float(rospy.get_param("~no_goal_seconds", 60.0))
        self.minimum_explore_seconds = float(
            rospy.get_param("~minimum_explore_seconds", 120.0))
        self.return_tolerance = float(rospy.get_param("~return_tolerance", 1.0))
        self.publish_rate = float(rospy.get_param("~publish_rate", 2.0))
        self.shutdown_on_finish = bool(rospy.get_param("~shutdown_on_finish", True))
        self.finish_delay = float(rospy.get_param("~finish_delay", 5.0))

        self.lock = threading.Lock()
        self.phase = "WAITING"
        self.start_pose = None          # (frame, x, y)
        self.start_time = None
        self.robot = None               # (frame, x, y)
        self.frontier_goal = None
        self.frontier_status = "NO_GOAL"
        self.no_goal_since = None
        self.finished_time = None
        self.return_reason = ""
        self.return_started = None

        self.goal_pub = rospy.Publisher(self.goal_topic, PoseStamped, queue_size=1)
        self.status_pub = rospy.Publisher(
            self.goal_status_topic, String, queue_size=1, latch=True)
        self.phase_pub = rospy.Publisher(
            "/cave/mission/phase", String, queue_size=1, latch=True)
        self.home_distance_pub = rospy.Publisher(
            "/cave/mission/distance_to_start", Float32, queue_size=1)
        self.event_pub = rospy.Publisher(
            "/cave/mission/event", String, queue_size=10, latch=True)

        rospy.Subscriber(self.odom_topic, Odometry, self.odom_callback, queue_size=5)
        rospy.Subscriber(self.frontier_goal_topic, PoseStamped,
                         self.frontier_goal_callback, queue_size=1)
        rospy.Subscriber(self.frontier_status_topic, String,
                         self.frontier_status_callback, queue_size=1)

        self.phase_pub.publish(String(data=self.phase))
        rospy.Timer(rospy.Duration(1.0 / max(self.publish_rate, 0.5)), self.tick)
        rospy.loginfo("Mission manager: max run %.0fs, return reserve >= %.0fs",
                      self.max_run_seconds, self.minimum_return_reserve)

    # ------------------------------------------------------------- callbacks
    def odom_callback(self, msg):
        frame = msg.header.frame_id.lstrip("/")
        p = msg.pose.pose.position
        with self.lock:
            self.robot = (frame, p.x, p.y)
            if self.start_pose is None and rospy.Time.now().to_sec() > 0.0:
                self.start_pose = (frame, p.x, p.y)
                self.start_time = rospy.Time.now()
                self.set_phase("EXPLORING", "start=(%.2f, %.2f) frame=%s" % (p.x, p.y, frame))

    def frontier_goal_callback(self, msg):
        with self.lock:
            self.frontier_goal = msg
            if self.phase == "EXPLORING":
                self.goal_pub.publish(msg)

    def frontier_status_callback(self, msg):
        with self.lock:
            self.frontier_status = msg.data
            now = rospy.Time.now()
            if msg.data == "NO_GOAL":
                if self.no_goal_since is None:
                    self.no_goal_since = now
            else:
                self.no_goal_since = None
            if self.phase == "EXPLORING":
                self.status_pub.publish(msg)

    # ----------------------------------------------------------------- logic
    def set_phase(self, phase, detail=""):
        """Caller holds self.lock."""
        if phase == self.phase:
            return
        elapsed = self.elapsed()
        text = "%.1f %s->%s %s" % (elapsed, self.phase, phase, detail)
        self.phase = phase
        self.phase_pub.publish(String(data=phase))
        self.event_pub.publish(String(data=text))
        rospy.logwarn("MISSION: %s", text)

    def elapsed(self):
        if self.start_time is None:
            return 0.0
        return max(0.0, (rospy.Time.now() - self.start_time).to_sec())

    def distance_home(self):
        if self.robot is None or self.start_pose is None:
            return float("nan")
        return math.hypot(self.robot[1] - self.start_pose[1],
                          self.robot[2] - self.start_pose[2])

    def return_reserve(self, distance):
        travel = self.tortuosity * distance / max(self.return_speed, 0.05)
        return max(self.minimum_return_reserve, 1.5 * travel)

    def home_goal(self):
        goal = PoseStamped()
        goal.header.frame_id = self.start_pose[0]
        goal.header.stamp = rospy.Time.now()
        goal.pose.position.x = self.start_pose[1]
        goal.pose.position.y = self.start_pose[2]
        goal.pose.orientation.w = 1.0
        return goal

    def tick(self, _event):
        with self.lock:
            if self.phase == "WAITING":
                return
            elapsed = self.elapsed()
            distance = self.distance_home()
            if math.isfinite(distance):
                self.home_distance_pub.publish(Float32(data=distance))

            if self.phase == "EXPLORING":
                reserve = self.return_reserve(distance if math.isfinite(distance) else 0.0)
                out_of_time = elapsed >= self.max_run_seconds - reserve
                done = (
                    self.no_goal_since is not None
                    and elapsed >= self.minimum_explore_seconds
                    and (rospy.Time.now() - self.no_goal_since).to_sec() >= self.no_goal_seconds
                )
                if out_of_time or done:
                    self.return_reason = "time budget" if out_of_time else "no frontiers left"
                    self.return_started = elapsed
                    self.set_phase("RETURNING", "reason=%s distance_home=%.1fm reserve=%.0fs"
                                   % (self.return_reason, distance, reserve))

            if self.phase == "RETURNING":
                if distance <= self.return_tolerance:
                    self.finished_time = rospy.Time.now()
                    self.set_phase("RETURNED", "distance_home=%.2fm return_time=%.0fs"
                                   % (distance, elapsed - self.return_started))
                else:
                    self.goal_pub.publish(self.home_goal())
                    self.status_pub.publish(String(data="ACTIVE"))

            if self.phase == "RETURNING" and elapsed >= self.max_run_seconds:
                self.finished_time = rospy.Time.now()
                self.set_phase("TIMEOUT", "distance_home=%.1fm" % distance)

            if self.phase in ("RETURNED", "TIMEOUT"):
                # Withdraw the goal so the planner publishes an empty path and
                # the follower stops the robot.
                self.status_pub.publish(String(data="NO_GOAL"))
                if (self.shutdown_on_finish and self.finished_time is not None
                        and (rospy.Time.now() - self.finished_time).to_sec() >= self.finish_delay):
                    rospy.signal_shutdown("mission %s" % self.phase.lower())


if __name__ == "__main__":
    rospy.init_node("mission_manager")
    MissionManager()
    rospy.spin()
