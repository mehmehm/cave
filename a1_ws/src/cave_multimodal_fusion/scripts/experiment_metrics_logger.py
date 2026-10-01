#!/usr/bin/env python3
"""Record repeatable cave-perception and exploration metrics to CSV/JSON."""

import csv
import json
import math
import os
import threading

import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Float32, Int32, String
from visualization_msgs.msg import MarkerArray


class RunningMetric:
    def __init__(self):
        self.count = 0
        self.total = 0.0
        self.minimum = None
        self.maximum = None

    def add(self, value):
        value = float(value)
        if not math.isfinite(value):
            return
        self.count += 1
        self.total += value
        self.minimum = value if self.minimum is None else min(self.minimum, value)
        self.maximum = value if self.maximum is None else max(self.maximum, value)

    def summary(self):
        return {
            "count": self.count,
            "mean": self.total / self.count if self.count else None,
            "min": self.minimum,
            "max": self.maximum,
        }


class ExperimentMetricsLogger:
    COLUMNS = [
        "sim_time_s", "configuration", "condition", "run_id", "mode",
        "brightness", "rgb_quality", "thermal_quality",
        "rgb_valid_fraction", "thermal_valid_fraction",
        "selected_valid_fraction", "selector_processing_ms",
        "active_cloud_messages", "active_points", "voxel_count",
        "visible_voxel_count", "distance_m", "planner_failures",
        "plans_succeeded", "goals_reached", "mode_switches",
    ]

    def __init__(self):
        self.configuration = rospy.get_param("~configuration", "ADAPTIVE")
        self.condition = rospy.get_param("~condition", "MIXED")
        self.run_id = rospy.get_param("~run_id", "run01")
        output_directory = os.path.expanduser(rospy.get_param(
            "~output_directory", "~/a1_ws/experiments/results"
        ))
        os.makedirs(output_directory, exist_ok=True)
        stem = "%s_%s_%s" % (
            self.configuration.lower(), self.condition.lower(), self.run_id
        )
        self.csv_path = os.path.join(output_directory, stem + ".csv")
        self.summary_path = os.path.join(output_directory, stem + "_summary.json")

        self.lock = threading.Lock()
        self.values = {
            "mode": "UNKNOWN",
            "brightness": float("nan"),
            "rgb_quality": float("nan"),
            "thermal_quality": float("nan"),
            "rgb_valid_fraction": float("nan"),
            "thermal_valid_fraction": float("nan"),
            "selected_valid_fraction": float("nan"),
            "selector_processing_ms": float("nan"),
            "voxel_count": 0,
            "visible_voxel_count": 0,
        }
        self.metrics = {
            name: RunningMetric() for name in (
                "brightness", "rgb_quality", "thermal_quality",
                "rgb_valid_fraction", "thermal_valid_fraction",
                "selected_valid_fraction", "selector_processing_ms",
                "voxel_count", "visible_voxel_count",
            )
        }
        self.active_cloud_messages = 0
        self.active_points = 0
        self.distance_m = 0.0
        self.last_position = None
        self.planner_failures = 0
        self.plans_succeeded = 0
        self.goals_reached = 0
        self.last_follower_status = ""
        self.mode_switches = 0
        self.last_mode = None
        self.start_time = rospy.Time.now()

        self.csv_file = open(self.csv_path, "w", newline="")
        self.writer = csv.DictWriter(self.csv_file, fieldnames=self.COLUMNS)
        self.writer.writeheader()
        self.csv_file.flush()

        scalar_topics = {
            "/cave/perception/brightness": "brightness",
            "/cave/perception/rgb_quality": "rgb_quality",
            "/cave/perception/thermal_quality": "thermal_quality",
            "/cave/perception/rgb_valid_fraction": "rgb_valid_fraction",
            "/cave/perception/thermal_valid_fraction": "thermal_valid_fraction",
            "/cave/perception/selected_valid_fraction": "selected_valid_fraction",
            "/cave/perception/selector_processing_ms": "selector_processing_ms",
        }
        for topic, name in scalar_topics.items():
            rospy.Subscriber(
                topic, Float32, self.scalar_callback,
                callback_args=name, queue_size=10
            )

        rospy.Subscriber(
            "/cave/perception/mode", String, self.mode_callback, queue_size=10
        )
        rospy.Subscriber(
            "/cave/fusion/active_points", PointCloud2,
            self.cloud_callback, queue_size=10
        )
        rospy.Subscriber(
            "/cave/map/voxel_count", Int32,
            self.int_callback, callback_args="voxel_count", queue_size=10
        )
        rospy.Subscriber(
            "/cave/map/visible_voxel_count", Int32,
            self.int_callback, callback_args="visible_voxel_count", queue_size=10
        )
        rospy.Subscriber(
            "/cave/map/multimodal_voxels", MarkerArray,
            self.marker_callback, queue_size=2
        )
        rospy.Subscriber(
            "/lio_sam/mapping/odometry", Odometry,
            self.odom_callback, queue_size=20
        )
        rospy.Subscriber(
            "/cave/exploration/planner_status", String,
            self.planner_callback, queue_size=20
        )
        rospy.Subscriber(
            "/cave/exploration/follower_status", String,
            self.follower_callback, queue_size=20
        )

        rospy.Timer(rospy.Duration(1.0), self.write_row)
        rospy.on_shutdown(self.finish)
        rospy.loginfo("Experiment metrics logger started")
        rospy.loginfo("CSV:     %s", self.csv_path)
        rospy.loginfo("Summary: %s", self.summary_path)

    def scalar_callback(self, message, name):
        with self.lock:
            self.values[name] = float(message.data)
            self.metrics[name].add(message.data)

    def int_callback(self, message, name):
        with self.lock:
            self.values[name] = int(message.data)
            self.metrics[name].add(message.data)

    def mode_callback(self, message):
        with self.lock:
            mode = message.data
            if self.last_mode is not None and mode != self.last_mode:
                self.mode_switches += 1
            self.last_mode = mode
            self.values["mode"] = mode

    def cloud_callback(self, message):
        with self.lock:
            self.active_cloud_messages += 1
            self.active_points += int(message.width) * int(message.height)

    def marker_callback(self, message):
        count = sum(len(marker.points) for marker in message.markers)
        with self.lock:
            # Fallback for an older accumulator that lacks count topics.
            self.values["visible_voxel_count"] = count
            self.metrics["visible_voxel_count"].add(count)

    def odom_callback(self, message):
        position = message.pose.pose.position
        current = (position.x, position.y, position.z)
        with self.lock:
            if self.last_position is not None:
                dx = current[0] - self.last_position[0]
                dy = current[1] - self.last_position[1]
                dz = current[2] - self.last_position[2]
                step = math.sqrt(dx * dx + dy * dy + dz * dz)
                # Ignore localisation resets or large loop-closure jumps.
                if step <= 1.0:
                    self.distance_m += step
            self.last_position = current

    def planner_callback(self, message):
        with self.lock:
            if message.data.startswith("FAILED"):
                self.planner_failures += 1
            elif message.data.startswith("PLANNED"):
                self.plans_succeeded += 1

    def follower_callback(self, message):
        with self.lock:
            if (
                message.data.startswith("GOAL_REACHED")
                and not self.last_follower_status.startswith("GOAL_REACHED")
            ):
                self.goals_reached += 1
            self.last_follower_status = message.data

    def current_row(self):
        return {
            "sim_time_s": "%.6f" % rospy.Time.now().to_sec(),
            "configuration": self.configuration,
            "condition": self.condition,
            "run_id": self.run_id,
            "mode": self.values["mode"],
            "brightness": self.values["brightness"],
            "rgb_quality": self.values["rgb_quality"],
            "thermal_quality": self.values["thermal_quality"],
            "rgb_valid_fraction": self.values["rgb_valid_fraction"],
            "thermal_valid_fraction": self.values["thermal_valid_fraction"],
            "selected_valid_fraction": self.values["selected_valid_fraction"],
            "selector_processing_ms": self.values["selector_processing_ms"],
            "active_cloud_messages": self.active_cloud_messages,
            "active_points": self.active_points,
            "voxel_count": self.values["voxel_count"],
            "visible_voxel_count": self.values["visible_voxel_count"],
            "distance_m": self.distance_m,
            "planner_failures": self.planner_failures,
            "plans_succeeded": self.plans_succeeded,
            "goals_reached": self.goals_reached,
            "mode_switches": self.mode_switches,
        }

    def write_row(self, _event):
        with self.lock:
            row = self.current_row()
        self.writer.writerow(row)
        self.csv_file.flush()

    def finish(self):
        with self.lock:
            elapsed = max(0.0, (rospy.Time.now() - self.start_time).to_sec())
            summary = {
                "configuration": self.configuration,
                "condition": self.condition,
                "run_id": self.run_id,
                "duration_s": elapsed,
                "active_cloud_messages": self.active_cloud_messages,
                "active_cloud_hz": (
                    self.active_cloud_messages / elapsed if elapsed > 0.0 else None
                ),
                "active_points": self.active_points,
                "distance_m": self.distance_m,
                "planner_failures": self.planner_failures,
                "plans_succeeded": self.plans_succeeded,
                "goals_reached": self.goals_reached,
                "mode_switches": self.mode_switches,
                "metrics": {
                    name: metric.summary()
                    for name, metric in self.metrics.items()
                },
            }
        with open(self.summary_path, "w") as summary_file:
            json.dump(summary, summary_file, indent=2, sort_keys=True)
        if not self.csv_file.closed:
            self.csv_file.flush()
            self.csv_file.close()
        rospy.loginfo("Experiment summary saved: %s", self.summary_path)


if __name__ == "__main__":
    rospy.init_node("experiment_metrics_logger")
    ExperimentMetricsLogger()
    rospy.spin()
