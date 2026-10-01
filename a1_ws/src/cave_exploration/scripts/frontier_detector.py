#!/usr/bin/env python3

"""Reachable BFS frontier detection for the cave occupancy grid."""

import math
import threading
from collections import deque

import numpy as np
import rospy
import tf2_ros

from geometry_msgs.msg import Point, PoseStamped
from nav_msgs.msg import OccupancyGrid, Odometry
from tf.transformations import euler_from_quaternion
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import String


class FrontierDetector:
    def __init__(self):
        self.map_topic = rospy.get_param(
            "~map_topic",
            "/projected_map"
        )

        self.odom_topic = rospy.get_param(
            "~odom_topic",
            "/lio_sam/mapping/odometry"
        )

        self.robot_frame = rospy.get_param(
            "~robot_frame",
            "a1/base"
        )

        self.frontier_topic = rospy.get_param(
            "~frontier_topic",
            "/cave/exploration/frontiers"
        )

        self.best_frontier_topic = rospy.get_param(
            "~best_frontier_topic",
            "/cave/exploration/best_frontier"
        )

        self.rejected_goal_topic = rospy.get_param(
            "~rejected_goal_topic",
            "/cave/exploration/rejected_goal"
        )

        self.update_rate = float(
            rospy.get_param("~update_rate", 1.0)
        )

        self.free_threshold = int(
            rospy.get_param("~free_threshold", 20)
        )

        self.minimum_cluster_size = int(
            rospy.get_param("~minimum_cluster_size", 8)
        )

        self.distance_weight = float(
            rospy.get_param("~distance_weight", 0.5)
        )

        # A projected 3-D OctoMap can temporarily contain no free-to-unknown
        # boundary in the robot's connected component (for example while the
        # robot is in the middle of a scanned chamber).  In that case, move to
        # a distant known-free viewpoint so a new scan can reveal space around
        # corners.  This never permits travel through occupied/unknown cells.
        self.enable_recovery_goal = bool(
            rospy.get_param("~enable_recovery_goal", True)
        )

        self.minimum_recovery_distance = float(
            rospy.get_param("~minimum_recovery_distance", 1.0)
        )

        self.recovery_goal_tolerance = float(
            rospy.get_param("~recovery_goal_tolerance", 0.35)
        )

        self.recovery_clearance_cells = int(
            rospy.get_param("~recovery_clearance_cells", 2)
        )

        self.recovery_revisit_radius = float(
            rospy.get_param("~recovery_revisit_radius", 1.0)
        )

        self.blacklist_radius = float(
            rospy.get_param("~blacklist_radius", 1.0)
        )

        self.blacklist_duration = float(
            rospy.get_param("~blacklist_duration", 60.0)
        )

        self.minimum_goal_distance = float(
            rospy.get_param("~minimum_goal_distance", 1.0)
        )
        self.frontier_goal_tolerance = float(
            rospy.get_param("~frontier_goal_tolerance", 0.25)
        )

        self.completed_goal_radius = float(rospy.get_param("~completed_goal_radius", 1.0))
        self.completed_goal_hold_seconds = float(rospy.get_param("~completed_goal_hold_seconds", 120.0))
        self.completion_match_radius = float(rospy.get_param("~completion_match_radius", 1.35))
        self.recovery_information_radius = float(rospy.get_param("~recovery_information_radius", 3.0))
        self.completed_goals = []
        self.visited_viewpoints = []
        self.pending_reached = None
        self.information_integral = None
        self.status_pub = rospy.Publisher('/cave/exploration/goal_status', String, queue_size=1, latch=True)
        self.map_lock = threading.Lock()
        self.latest_map = None
        self.latest_odom = None
        self.active_frontier_goal = None
        self.active_recovery_goal = None
        self.visited_recovery_goals = []
        self.blacklist_lock = threading.Lock()
        self.goal_blacklist = []

        self.tf_buffer = tf2_ros.Buffer(
            cache_time=rospy.Duration(30.0)
        )

        self.tf_listener = tf2_ros.TransformListener(
            self.tf_buffer
        )

        self.marker_pub = rospy.Publisher(
            self.frontier_topic,
            MarkerArray,
            queue_size=1,
            latch=True
        )

        self.best_pub = rospy.Publisher(
            self.best_frontier_topic,
            PoseStamped,
            queue_size=1,
            latch=True
        )

        self.map_sub = rospy.Subscriber(
            self.map_topic,
            OccupancyGrid,
            self.map_callback,
            queue_size=1
        )

        self.odom_sub = rospy.Subscriber(
            self.odom_topic,
            Odometry,
            self.odom_callback,
            queue_size=1
        )

        self.rejected_goal_sub = rospy.Subscriber(
            self.rejected_goal_topic,
            PoseStamped,
            self.rejected_goal_callback,
            queue_size=10
        )

        self.reached_sub = rospy.Subscriber('/cave/exploration/reached_goal', PoseStamped,
                                             self.reached_callback, queue_size=1)
        period = 1.0 / max(self.update_rate, 0.1)

        self.timer = rospy.Timer(
            rospy.Duration(period),
            self.timer_callback
        )

        rospy.loginfo("BFS frontier detector started")
        rospy.loginfo("Map: %s", self.map_topic)
        rospy.loginfo("Robot frame: %s", self.robot_frame)
        rospy.loginfo("Frontiers: %s", self.frontier_topic)
        rospy.loginfo(
            "Best frontier: %s",
            self.best_frontier_topic
        )
        rospy.loginfo("Rejected goals: %s", self.rejected_goal_topic)
        rospy.loginfo(
            "Goal blacklist: radius=%.2fm duration=%.1fs",
            self.blacklist_radius,
            self.blacklist_duration,
        )

    def reached_callback(self, message):
        with self.map_lock:
            self.pending_reached = message

    def remember_completed(self, goal):
        x, y = goal['world_x'], goal['world_y']
        self.completed_goals.append((x, y, rospy.Time.now().to_sec() + self.completed_goal_hold_seconds))
        self.visited_recovery_goals.append((x, y))

    def recently_completed(self, x, y):
        now = rospy.Time.now().to_sec()
        self.completed_goals = [g for g in self.completed_goals if g[2] > now]
        return any(math.hypot(x-gx, y-gy) < self.completed_goal_radius
                   for gx, gy, _ in self.completed_goals)

    def information_count(self, grid, x, y, radius):
        # Count stored unknown cells, not merely distance from the robot.
        h, w = grid.shape
        x0, x1 = max(0, x-radius), min(w, x+radius+1)
        y0, y1 = max(0, y-radius), min(h, y+radius+1)
        a = self.information_integral
        return int(a[y1, x1] - a[y0, x1] - a[y1, x0] + a[y0, x0])

    def map_callback(self, message):
        with self.map_lock:
            self.latest_map = message

    def odom_callback(self, message):
        with self.map_lock:
            self.latest_odom = message

    def rejected_goal_callback(self, message):
        """Temporarily exclude a goal rejected by the inflated A* map."""
        now = rospy.Time.now().to_sec()
        rejected_x = message.pose.position.x
        rejected_y = message.pose.position.y

        with self.blacklist_lock:
            self.goal_blacklist = [
                item for item in self.goal_blacklist if item[2] > now
            ]
            self.goal_blacklist.append(
                (
                    rejected_x,
                    rejected_y,
                    now + max(self.blacklist_duration, 0.0),
                )
            )

        if self.active_recovery_goal is not None:
            active_x = self.active_recovery_goal["world_x"]
            active_y = self.active_recovery_goal["world_y"]
            if (
                math.hypot(active_x - rejected_x, active_y - rejected_y)
                < self.blacklist_radius
            ):
                self.active_recovery_goal = None

        if self.active_frontier_goal is not None:
            active_x = self.active_frontier_goal["world_x"]
            active_y = self.active_frontier_goal["world_y"]
            if math.hypot(active_x - rejected_x, active_y - rejected_y) < self.blacklist_radius:
                self.active_frontier_goal = None

        rospy.logwarn(
            "Blacklisted rejected exploration goal (%.2f, %.2f) for %.1fs",
            rejected_x,
            rejected_y,
            self.blacklist_duration,
        )

    def is_blacklisted(self, world_x, world_y):
        now = rospy.Time.now().to_sec()
        with self.blacklist_lock:
            self.goal_blacklist = [
                item for item in self.goal_blacklist if item[2] > now
            ]
            entries = list(self.goal_blacklist)

        return any(
            math.hypot(world_x - item_x, world_y - item_y)
            < self.blacklist_radius
            for item_x, item_y, _expiry in entries
        )

    @staticmethod
    def origin_yaw(message):      
        orientation = message.info.origin.orientation

        norm_squared = (
            orientation.x * orientation.x
            + orientation.y * orientation.y
            + orientation.z * orientation.z
            + orientation.w * orientation.w
        )

        # OctoMap can publish an uninitialised all-zero quaternion.
        # Interpret it as the identity rotation.
        if norm_squared < 1e-12:
            return 0.0

        quaternion = [
            orientation.x,
            orientation.y,
            orientation.z,
            orientation.w
        ]

        return euler_from_quaternion(quaternion)[2]

    def world_to_grid(self, message, world_x, world_y):
        origin = message.info.origin.position
        yaw = self.origin_yaw(message)

        relative_x = world_x - origin.x
        relative_y = world_y - origin.y

        cosine = math.cos(yaw)
        sine = math.sin(yaw)

        local_x = cosine * relative_x + sine * relative_y
        local_y = -sine * relative_x + cosine * relative_y

        grid_x = int(
            math.floor(local_x / message.info.resolution)
        )

        grid_y = int(
            math.floor(local_y / message.info.resolution)
        )

        return grid_x, grid_y

    def grid_to_world(self, message, grid_x, grid_y):
        origin = message.info.origin.position
        yaw = self.origin_yaw(message)
        resolution = message.info.resolution

        local_x = (grid_x + 0.5) * resolution
        local_y = (grid_y + 0.5) * resolution

        cosine = math.cos(yaw)
        sine = math.sin(yaw)

        world_x = (
            origin.x
            + cosine * local_x
            - sine * local_y
        )

        world_y = (
            origin.y
            + sine * local_x
            + cosine * local_y
        )

        return world_x, world_y

    @staticmethod
    def inside(grid, grid_x, grid_y):
        height, width = grid.shape

        return (
            0 <= grid_x < width
            and 0 <= grid_y < height
        )

    def is_free(self, grid, grid_x, grid_y):
        return (
            self.inside(grid, grid_x, grid_y)
            and 0 <= grid[grid_y, grid_x] <= self.free_threshold
        )

    def is_frontier(self, grid, grid_x, grid_y):
        if not self.is_free(grid, grid_x, grid_y):
            return False

        for offset_y in (-1, 0, 1):
            for offset_x in (-1, 0, 1):
                if offset_x == 0 and offset_y == 0:
                    continue

                neighbour_x = grid_x + offset_x
                neighbour_y = grid_y + offset_y

                # Space outside a dynamically sized OctoMap grid
                # has not been observed and is therefore unknown.
                if not self.inside(
                    grid,
                    neighbour_x,
                    neighbour_y
                ):
                    return True

                if grid[neighbour_y, neighbour_x] == -1:
                    return True

        return False

    def find_nearest_free_cell(
        self,
        grid,
        start_x,
        start_y,
        maximum_radius=30
    ):
        if self.is_free(grid, start_x, start_y):
            return start_x, start_y

        for radius in range(1, maximum_radius + 1):
            for offset_y in range(-radius, radius + 1):
                for offset_x in range(-radius, radius + 1):
                    if (
                        abs(offset_x) != radius
                        and abs(offset_y) != radius
                    ):
                        continue

                    candidate_x = start_x + offset_x
                    candidate_y = start_y + offset_y

                    if self.is_free(
                        grid,
                        candidate_x,
                        candidate_y
                    ):
                        return candidate_x, candidate_y

        return None

    def reachable_frontiers(
        self,
        grid,
        start_x,
        start_y
    ):
        queue = deque([(start_x, start_y)])
        visited = {(start_x, start_y)}
        distances = {(start_x, start_y): 0}
        frontier_cells = set()

        neighbours = [
            (1, 0),
            (-1, 0),
            (0, 1),
            (0, -1)
        ]

        while queue:
            current_x, current_y = queue.popleft()

            if self.is_frontier(
                grid,
                current_x,
                current_y
            ):
                frontier_cells.add(
                    (current_x, current_y)
                )

            for offset_x, offset_y in neighbours:
                neighbour_x = current_x + offset_x
                neighbour_y = current_y + offset_y
                neighbour = (neighbour_x, neighbour_y)

                if neighbour in visited:
                    continue

                if not self.is_free(
                    grid,
                    neighbour_x,
                    neighbour_y
                ):
                    continue

                visited.add(neighbour)
                distances[neighbour] = (
                    distances[(current_x, current_y)] + 1
                )
                queue.append(neighbour)

        return frontier_cells, visited, distances

    def cell_clearance(self, grid, grid_x, grid_y):
        """Return the free square clearance around a grid cell."""
        maximum = max(self.recovery_clearance_cells, 0)

        for radius in range(1, maximum + 1):
            for offset_y in range(-radius, radius + 1):
                for offset_x in range(-radius, radius + 1):
                    if not self.is_free(
                        grid,
                        grid_x + offset_x,
                        grid_y + offset_y
                    ):
                        return radius - 1

        return maximum

    def recovery_candidate(
        self,
        message,
        grid,
        reachable_cells,
        distances,
        robot_x,
        robot_y
    ):
        """Select or retain a safe known-free next-best-view goal."""
        if not self.enable_recovery_goal:
            self.active_recovery_goal = None
            return None

        # Keep one recovery target stable until the robot reaches it.  Without
        # this, the farthest cell can change every map update and cause goal
        # oscillation.
        if self.active_recovery_goal is not None:
            active_x = self.active_recovery_goal["world_x"]
            active_y = self.active_recovery_goal["world_y"]
            remaining = math.hypot(
                active_x - robot_x,
                active_y - robot_y
            )

            active_grid_x, active_grid_y = self.world_to_grid(
                message,
                active_x,
                active_y
            )

            if (
                remaining > self.recovery_goal_tolerance
                and (active_grid_x, active_grid_y) in reachable_cells
                and self.is_free(grid, active_grid_x, active_grid_y)
            ):
                candidate = dict(self.active_recovery_goal)
                candidate["grid_x"] = active_grid_x
                candidate["grid_y"] = active_grid_y
                candidate["distance"] = remaining
                return candidate

            if remaining <= self.recovery_goal_tolerance:
                self.remember_completed(self.active_recovery_goal)

            self.active_recovery_goal = None

        resolution = message.info.resolution
        minimum_steps = int(math.ceil(
            self.minimum_recovery_distance / resolution
        ))

        choices = []
        information_radius = max(1, int(math.ceil(self.recovery_information_radius / resolution)))

        for grid_x, grid_y in reachable_cells:
            path_steps = distances[(grid_x, grid_y)]

            if path_steps < minimum_steps:
                continue

            clearance = self.cell_clearance(
                grid,
                grid_x,
                grid_y
            )

            if clearance < self.recovery_clearance_cells:
                continue

            world_x, world_y = self.grid_to_world(
                message,
                grid_x,
                grid_y
            )

            if any(
                math.hypot(
                    world_x - visited_x,
                    world_y - visited_y
                ) < self.recovery_revisit_radius
                for visited_x, visited_y in self.visited_recovery_goals
            ):
                continue

            if self.is_blacklisted(world_x, world_y) or self.recently_completed(world_x, world_y):
                continue
            if any(math.hypot(world_x-vx, world_y-vy) < self.recovery_revisit_radius
                   for vx, vy in self.visited_viewpoints):
                continue
            information = self.information_count(grid, grid_x, grid_y, information_radius)
            if information == 0:
                continue

            choices.append({
                "cluster": [(grid_x, grid_y)],
                "grid_x": grid_x,
                "grid_y": grid_y,
                "world_x": world_x,
                "world_y": world_y,
                "distance": math.hypot(
                    world_x - robot_x,
                    world_y - robot_y
                ),
                "path_distance": path_steps * resolution,
                "score": information / (1.0 + path_steps * resolution),
                "mode": "RECOVERY"
            })

        if not choices:
            return None

        candidate = max(
            choices,
            key=lambda item: (
                item["score"],
                -item["path_distance"]
            )
        )

        self.active_recovery_goal = dict(candidate)
        return candidate

    @staticmethod
    def cluster_frontiers(frontier_cells):
        remaining = set(frontier_cells)
        clusters = []

        neighbour_offsets = [
            (-1, -1), (0, -1), (1, -1),
            (-1, 0),           (1, 0),
            (-1, 1),  (0, 1),  (1, 1)
        ]

        while remaining:
            first_cell = remaining.pop()
            cluster = [first_cell]
            queue = deque([first_cell])

            while queue:
                current_x, current_y = queue.popleft()

                for offset_x, offset_y in neighbour_offsets:
                    neighbour = (
                        current_x + offset_x,
                        current_y + offset_y
                    )

                    if neighbour not in remaining:
                        continue

                    remaining.remove(neighbour)
                    cluster.append(neighbour)
                    queue.append(neighbour)

            clusters.append(cluster)

        return clusters

    def select_best_cluster(
        self,
        message,
        clusters,
        robot_x,
        robot_y
    ):
        candidates = []

        for cluster in clusters:
            if len(cluster) < self.minimum_cluster_size:
                continue

            # A rounded cluster mean may lie outside a curved frontier or on
            # the far side of a wall.  Choose the actual reachable frontier
            # cell nearest to the geometric centre instead.
            centre_x = (
                sum(cell[0] for cell in cluster) / float(len(cluster))
            )
            centre_y = (
                sum(cell[1] for cell in cluster) / float(len(cluster))
            )
            centre_grid_x, centre_grid_y = min(
                cluster,
                key=lambda cell: (
                    (cell[0] - centre_x) ** 2
                    + (cell[1] - centre_y) ** 2
                ),
            )

            world_x, world_y = self.grid_to_world(
                message,
                centre_grid_x,
                centre_grid_y
            )

            distance = math.hypot(
                world_x - robot_x,
                world_y - robot_y
            )

            if distance < self.minimum_goal_distance:
                continue

            if self.is_blacklisted(world_x, world_y) or self.recently_completed(world_x, world_y):
                continue

            information_gain = (
                len(cluster)
                * message.info.resolution
            )

            score = (
                information_gain
                - self.distance_weight * distance
            )

            candidates.append(
                {
                    "cluster": cluster,
                    "grid_x": centre_grid_x,
                    "grid_y": centre_grid_y,
                    "world_x": world_x,
                    "world_y": world_y,
                    "distance": distance,
                    "score": score,
                    "mode": "FRONTIER"
                }
            )

        if not candidates:
            return None, []

        best_candidate = max(
            candidates,
            key=lambda candidate: candidate["score"]
        )

        return best_candidate, candidates

    def publish_markers(
        self,
        message,
        frontier_cells,
        candidates,
        best_candidate
    ):
        marker_array = MarkerArray()

        delete_marker = Marker()
        delete_marker.action = Marker.DELETEALL
        marker_array.markers.append(delete_marker)

        cells_marker = Marker()
        cells_marker.header.frame_id = message.header.frame_id
        cells_marker.header.stamp = rospy.Time.now()
        cells_marker.ns = "frontier_cells"
        cells_marker.id = 0
        cells_marker.type = Marker.POINTS
        cells_marker.action = Marker.ADD
        cells_marker.pose.orientation.w = 1.0
        cells_marker.scale.x = max(
            message.info.resolution * 0.7,
            0.04
        )
        cells_marker.scale.y = cells_marker.scale.x
        cells_marker.color.r = 0.0
        cells_marker.color.g = 1.0
        cells_marker.color.b = 0.2
        cells_marker.color.a = 1.0

        for grid_x, grid_y in frontier_cells:
            world_x, world_y = self.grid_to_world(
                message,
                grid_x,
                grid_y
            )

            point = Point()
            point.x = world_x
            point.y = world_y
            point.z = 0.10
            cells_marker.points.append(point)

        marker_array.markers.append(cells_marker)

        for marker_id, candidate in enumerate(
            candidates,
            start=1
        ):
            marker = Marker()
            marker.header.frame_id = message.header.frame_id
            marker.header.stamp = rospy.Time.now()
            marker.ns = "frontier_centres"
            marker.id = marker_id
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD

            marker.pose.position.x = candidate["world_x"]
            marker.pose.position.y = candidate["world_y"]
            marker.pose.position.z = 0.20
            marker.pose.orientation.w = 1.0

            marker.scale.x = 0.22
            marker.scale.y = 0.22
            marker.scale.z = 0.22

            marker.color.r = 0.1
            marker.color.g = 0.4
            marker.color.b = 1.0
            marker.color.a = 1.0

            marker_array.markers.append(marker)

        if best_candidate is not None:
            best_marker = Marker()
            best_marker.header.frame_id = message.header.frame_id
            best_marker.header.stamp = rospy.Time.now()
            best_marker.ns = "best_frontier"
            best_marker.id = 100000
            best_marker.type = Marker.SPHERE
            best_marker.action = Marker.ADD

            best_marker.pose.position.x = (
                best_candidate["world_x"]
            )
            best_marker.pose.position.y = (
                best_candidate["world_y"]
            )
            best_marker.pose.position.z = 0.30
            best_marker.pose.orientation.w = 1.0

            best_marker.scale.x = 0.40
            best_marker.scale.y = 0.40
            best_marker.scale.z = 0.40

            if best_candidate.get("mode") == "RECOVERY":
                best_marker.color.r = 1.0
                best_marker.color.g = 0.75
                best_marker.color.b = 0.0
            else:
                best_marker.color.r = 1.0
                best_marker.color.g = 0.1
                best_marker.color.b = 0.1
            best_marker.color.a = 1.0

            marker_array.markers.append(best_marker)

        self.marker_pub.publish(marker_array)

    def timer_callback(self, _event):
        with self.map_lock:
            message = self.latest_map
            odometry = self.latest_odom

        if message is None:
            rospy.logwarn_throttle(
                5.0,
                "Waiting for occupancy grid on %s",
                self.map_topic
            )
            return

        if odometry is None:
            rospy.logwarn_throttle(
                5.0,
                "Waiting for odometry on %s",
                self.odom_topic
            )
            return

        if not message.header.frame_id:
            rospy.logwarn_throttle(
                5.0,
                "Occupancy grid has no frame_id"
            )
            return

        map_frame = message.header.frame_id.lstrip("/")
        odom_frame = odometry.header.frame_id.lstrip("/")

        if map_frame != odom_frame:
            rospy.logwarn_throttle(
                2.0,
                "Map frame '%s' does not match odometry frame '%s'",
                map_frame,
                odom_frame
            )
            return

        robot_x = odometry.pose.pose.position.x
        robot_y = odometry.pose.pose.position.y

        expected_size = (
            message.info.width
            * message.info.height
        )

        if len(message.data) != expected_size:
            rospy.logwarn_throttle(
                5.0,
                "Occupancy grid data size is invalid"
            )
            return

        grid = np.asarray(
            message.data,
            dtype=np.int16
        ).reshape(
            message.info.height,
            message.info.width
        )

        self.information_integral = np.pad((grid == -1).astype(np.int32), ((1, 0), (1, 0))).cumsum(0).cumsum(1)
        if not self.visited_viewpoints or math.hypot(robot_x-self.visited_viewpoints[-1][0],
                                                     robot_y-self.visited_viewpoints[-1][1]) >= 0.5:
            self.visited_viewpoints.append((robot_x, robot_y))
        with self.map_lock:
            reached = self.pending_reached
            self.pending_reached = None
        if reached is not None and reached.header.frame_id.lstrip('/') == map_frame:
            for attribute in ('active_frontier_goal', 'active_recovery_goal'):
                active = getattr(self, attribute)
                if active is not None and math.hypot(active['world_x']-reached.pose.position.x,
                                                     active['world_y']-reached.pose.position.y) <= self.completion_match_radius:
                    self.remember_completed(active)
                    setattr(self, attribute, None)

        start_x, start_y = self.world_to_grid(
            message,
            robot_x,
            robot_y
        )

        start_cell = self.find_nearest_free_cell(
            grid,
            start_x,
            start_y
        )

        if start_cell is None:
            rospy.logwarn_throttle(
                2.0,
                "No free cell near the robot at grid (%d, %d)",
                start_x,
                start_y
            )
            return

        frontier_cells, reachable_cells, distances = self.reachable_frontiers(
            grid,
            start_cell[0],
            start_cell[1]
        )

        clusters = self.cluster_frontiers(
            frontier_cells
        )

        best_candidate, candidates = (
            self.select_best_cluster(
                message,
                clusters,
                robot_x,
                robot_y
            )
        )

        # Keep a selected frontier until arrival or invalidation. Re-ranking
        # clusters on every map update otherwise steers the follower toward a
        # different destination while it is still approaching the first one.
        if self.active_frontier_goal is not None:
            active = self.active_frontier_goal
            remaining = math.hypot(
                active["world_x"] - robot_x,
                active["world_y"] - robot_y,
            )
            gx, gy = self.world_to_grid(
                message, active["world_x"], active["world_y"]
            )
            if (
                remaining > self.frontier_goal_tolerance
                and (gx, gy) in reachable_cells
                and self.is_free(grid, gx, gy)
                and self.is_frontier(grid, gx, gy)
                and not self.is_blacklisted(active["world_x"], active["world_y"])
            ):
                best_candidate = dict(active)
                best_candidate["grid_x"] = gx
                best_candidate["grid_y"] = gy
                best_candidate["distance"] = remaining
                candidates = [best_candidate]
            else:
                if remaining <= self.frontier_goal_tolerance:
                    self.remember_completed(active)
                self.active_frontier_goal = None
                best_candidate, candidates = self.select_best_cluster(message, clusters, robot_x, robot_y)

        if self.active_frontier_goal is None and best_candidate is not None:
            self.active_frontier_goal = dict(best_candidate)

        if best_candidate is not None:
            # A real frontier supersedes any active recovery target.
            self.active_recovery_goal = None
        else:
            best_candidate = self.recovery_candidate(
                message,
                grid,
                reachable_cells,
                distances,
                robot_x,
                robot_y
            )

            if best_candidate is not None:
                candidates = [best_candidate]

        self.publish_markers(
            message,
            frontier_cells,
            candidates,
            best_candidate
        )

        if best_candidate is not None:
            target = PoseStamped()
            target.header.frame_id = message.header.frame_id
            target.header.stamp = rospy.Time.now()
            target.pose.position.x = (
                best_candidate["world_x"]
            )
            target.pose.position.y = (
                best_candidate["world_y"]
            )
            target.pose.position.z = 0.0
            target.pose.orientation.w = 1.0

            self.best_pub.publish(target)
            self.status_pub.publish(String(data="ACTIVE"))

            rospy.loginfo_throttle(
                2.0,
                "mode=%s reachable=%d frontier cells=%d clusters=%d "
                "valid=%d goal=(%.2f, %.2f) distance=%.2fm score=%.2f",
                best_candidate.get("mode", "FRONTIER"),
                len(reachable_cells),
                len(frontier_cells),
                len(clusters),
                len(candidates),
                best_candidate["world_x"],
                best_candidate["world_y"],
                best_candidate["distance"],
                best_candidate["score"]
            )

        else:
            self.status_pub.publish(String(data="NO_GOAL"))
            rospy.logwarn_throttle(
                2.0,
                "No valid exploration goal: reachable=%d cells=%d "
                "clusters=%d",
                len(reachable_cells),
                len(frontier_cells),
                len(clusters)
            )


if __name__ == "__main__":
    rospy.init_node("frontier_detector")
    FrontierDetector()
    rospy.spin()
