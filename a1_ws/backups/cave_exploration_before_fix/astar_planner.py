#!/usr/bin/env python3

"""A* global planner for the A1 cave-exploration occupancy grid."""

import heapq
import math
import threading

import numpy as np
import rospy

from geometry_msgs.msg import PoseStamped, Quaternion
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from std_msgs.msg import String
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from collections import deque

class AStarPlanner:
    def __init__(self):
        self.map_topic = rospy.get_param("~map_topic", "/projected_map")
        self.odom_topic = rospy.get_param(
            "~odom_topic", "/lio_sam/mapping/odometry"
        )
        self.goal_topic = rospy.get_param(
            "~goal_topic", "/cave/exploration/best_frontier"
        )
        self.path_topic = rospy.get_param(
            "~path_topic", "/cave/exploration/path"
        )
        self.status_topic = rospy.get_param(
            "~status_topic", "/cave/exploration/planner_status"
        )

        self.free_threshold = int(rospy.get_param("~free_threshold", 20))
        self.robot_radius = float(rospy.get_param("~robot_radius", 0.30))
        self.nearest_free_radius = float(
            rospy.get_param("~nearest_free_radius", 1.0)
        )
        self.replan_rate = float(rospy.get_param("~replan_rate", 1.0))
        self.periodic_replan_seconds = float(
            rospy.get_param("~periodic_replan_seconds", 2.0)
        )
        self.maximum_expansions = int(
            rospy.get_param("~maximum_expansions", 300000)
        )

        self.lock = threading.Lock()
        self.latest_map = None
        self.latest_odom = None
        self.latest_goal = None
        self.goal_revision = 0
        self.planned_goal_revision = -1
        self.last_plan_time = rospy.Time(0)

        self.path_pub = rospy.Publisher(
            self.path_topic, Path, queue_size=1, latch=True
        )
        self.status_pub = rospy.Publisher(
            self.status_topic, String, queue_size=1, latch=True
        )

        self.map_sub = rospy.Subscriber(
            self.map_topic, OccupancyGrid, self.map_callback, queue_size=1
        )
        self.odom_sub = rospy.Subscriber(
            self.odom_topic, Odometry, self.odom_callback, queue_size=1
        )
        self.goal_sub = rospy.Subscriber(
            self.goal_topic, PoseStamped, self.goal_callback, queue_size=1
        )

        period = 1.0 / max(self.replan_rate, 0.1)
        self.timer = rospy.Timer(rospy.Duration(period), self.timer_callback)

        rospy.loginfo("A* cave planner started")
        rospy.loginfo("Map: %s", self.map_topic)
        rospy.loginfo("Goal: %s", self.goal_topic)
        rospy.loginfo("Path: %s", self.path_topic)
        rospy.loginfo("Robot inflation radius: %.2fm", self.robot_radius)

    def map_callback(self, message):
        with self.lock:
            self.latest_map = message

    def odom_callback(self, message):
        with self.lock:
            self.latest_odom = message

    def goal_callback(self, message):
        with self.lock:
            if self.latest_goal is not None:
                old = self.latest_goal.pose.position
                new = message.pose.position
                same_frame = (
                    self.latest_goal.header.frame_id.lstrip("/")
                    == message.header.frame_id.lstrip("/")
                )
                if same_frame and math.hypot(new.x - old.x, new.y - old.y) < 0.03:
                    self.latest_goal = message
                    return

            self.latest_goal = message
            self.goal_revision += 1

    @staticmethod
    def origin_yaw(message):
        q = message.info.origin.orientation
        norm_squared = q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w
        if norm_squared < 1e-12:
            return 0.0
        return euler_from_quaternion([q.x, q.y, q.z, q.w])[2]

    def world_to_grid(self, message, world_x, world_y):
        origin = message.info.origin.position
        yaw = self.origin_yaw(message)
        relative_x = world_x - origin.x
        relative_y = world_y - origin.y
        cosine = math.cos(yaw)
        sine = math.sin(yaw)
        local_x = cosine * relative_x + sine * relative_y
        local_y = -sine * relative_x + cosine * relative_y
        return (
            int(math.floor(local_x / message.info.resolution)),
            int(math.floor(local_y / message.info.resolution)),
        )

    def grid_to_world(self, message, grid_x, grid_y):
        origin = message.info.origin.position
        yaw = self.origin_yaw(message)
        resolution = message.info.resolution
        local_x = (grid_x + 0.5) * resolution
        local_y = (grid_y + 0.5) * resolution
        cosine = math.cos(yaw)
        sine = math.sin(yaw)
        return (
            origin.x + cosine * local_x - sine * local_y,
            origin.y + sine * local_x + cosine * local_y,
        )

    @staticmethod
    def inside(mask, cell):
        x, y = cell
        height, width = mask.shape
        return 0 <= x < width and 0 <= y < height

    def build_traversable_mask(self, message):
        grid = np.asarray(message.data, dtype=np.int16).reshape(
            message.info.height, message.info.width
        )
        known_free = (grid >= 0) & (grid <= self.free_threshold)
        occupied = grid > self.free_threshold

        radius_cells = int(math.ceil(
            self.robot_radius / message.info.resolution
        ))
        if radius_cells <= 0:
            return known_free

        inflated_occupied = occupied.copy()
        height, width = occupied.shape

        for dy in range(-radius_cells, radius_cells + 1):
            for dx in range(-radius_cells, radius_cells + 1):
                if dx * dx + dy * dy > radius_cells * radius_cells:
                    continue

                source_x0 = max(0, -dx)
                source_x1 = min(width, width - dx)
                source_y0 = max(0, -dy)
                source_y1 = min(height, height - dy)
                target_x0 = source_x0 + dx
                target_x1 = source_x1 + dx
                target_y0 = source_y0 + dy
                target_y1 = source_y1 + dy

                inflated_occupied[
                    target_y0:target_y1, target_x0:target_x1
                ] |= occupied[
                    source_y0:source_y1, source_x0:source_x1
                ]

        # The map exterior is unknown even though it has no stored cells.
        # Keep the robot footprint inside the represented map boundary.
        traversable = known_free & ~inflated_occupied
        traversable[:radius_cells, :] = False
        traversable[-radius_cells:, :] = False
        traversable[:, :radius_cells] = False
        traversable[:, -radius_cells:] = False

        return traversable

    def nearest_traversable(self, mask, cell, maximum_cells):
        if self.inside(mask, cell) and mask[cell[1], cell[0]]:
            return cell

        start_x, start_y = cell
        best = None

        for radius in range(1, maximum_cells + 1):
            candidates = []
            for dy in range(-radius, radius + 1):
                for dx in range(-radius, radius + 1):
                    if abs(dx) != radius and abs(dy) != radius:
                        continue
                    candidate = (start_x + dx, start_y + dy)
                    if self.inside(mask, candidate) and mask[candidate[1], candidate[0]]:
                        candidates.append((dx * dx + dy * dy, candidate))
            if candidates:
                best = min(candidates, key=lambda item: item[0])[1]
                break

        return best

    def nearest_reachable_target(
        self,
        traversable,
        start,
        desired_goal
    ):
        """Return the cell in the robot's component nearest the goal."""
        if (
            not self.inside(traversable, start)
            or not traversable[start[1], start[0]]
        ):
            return None

        queue = deque([start])
        visited = {start}
        best = start
        best_distance_squared = (
            (start[0] - desired_goal[0]) ** 2
            + (start[1] - desired_goal[1]) ** 2
        )
        neighbours = (
            (1, 0), (-1, 0), (0, 1), (0, -1),
            (1, 1), (1, -1), (-1, 1), (-1, -1),
        )

        while queue:
            x, y = queue.popleft()
            distance_squared = (
                (x - desired_goal[0]) ** 2
                + (y - desired_goal[1]) ** 2
            )
            if distance_squared < best_distance_squared:
                best = (x, y)
                best_distance_squared = distance_squared

            for dx, dy in neighbours:
                nx, ny = x + dx, y + dy
                neighbour = (nx, ny)
                if neighbour in visited:
                    continue
                if not self.inside(traversable, neighbour):
                    continue
                if not traversable[ny, nx]:
                    continue
                if dx != 0 and dy != 0:
                    if not traversable[y, nx] or not traversable[ny, x]:
                        continue
                visited.add(neighbour)
                queue.append(neighbour)

        return best

    @staticmethod
    def heuristic(cell, goal):
        dx = abs(cell[0] - goal[0])
        dy = abs(cell[1] - goal[1])
        return max(dx, dy) + (math.sqrt(2.0) - 1.0) * min(dx, dy)

    def astar(self, traversable, start, goal):
        height, width = traversable.shape
        g_score = np.full((height, width), np.inf, dtype=np.float64)
        parent_x = np.full((height, width), -1, dtype=np.int32)
        parent_y = np.full((height, width), -1, dtype=np.int32)
        closed = np.zeros((height, width), dtype=np.bool_)

        g_score[start[1], start[0]] = 0.0
        queue = [(self.heuristic(start, goal), 0.0, start[0], start[1])]
        expansions = 0

        neighbours = (
            (1, 0, 1.0), (-1, 0, 1.0),
            (0, 1, 1.0), (0, -1, 1.0),
            (1, 1, math.sqrt(2.0)), (1, -1, math.sqrt(2.0)),
            (-1, 1, math.sqrt(2.0)), (-1, -1, math.sqrt(2.0)),
        )

        while queue and expansions < self.maximum_expansions:
            _, current_cost, x, y = heapq.heappop(queue)
            if closed[y, x]:
                continue
            closed[y, x] = True
            expansions += 1

            if (x, y) == goal:
                path = [(x, y)]
                while (x, y) != start:
                    previous_x = int(parent_x[y, x])
                    previous_y = int(parent_y[y, x])
                    if previous_x < 0 or previous_y < 0:
                        return None, expansions
                    x, y = previous_x, previous_y
                    path.append((x, y))
                path.reverse()
                return path, expansions

            for dx, dy, step_cost in neighbours:
                nx, ny = x + dx, y + dy
                if not (0 <= nx < width and 0 <= ny < height):
                    continue
                if closed[ny, nx] or not traversable[ny, nx]:
                    continue

                # Prevent diagonal corner cutting between two obstacles.
                if dx != 0 and dy != 0:
                    if not traversable[y, nx] or not traversable[ny, x]:
                        continue

                tentative = current_cost + step_cost
                if tentative >= g_score[ny, nx]:
                    continue

                g_score[ny, nx] = tentative
                parent_x[ny, nx] = x
                parent_y[ny, nx] = y
                priority = tentative + self.heuristic((nx, ny), goal)
                heapq.heappush(queue, (priority, tentative, nx, ny))

        return None, expansions

    @staticmethod
    def simplify_collinear(path):
        if path is None or len(path) <= 2:
            return path
        simplified = [path[0]]
        previous_direction = None
        for index in range(1, len(path)):
            direction = (
                path[index][0] - path[index - 1][0],
                path[index][1] - path[index - 1][1],
            )
            if previous_direction is not None and direction != previous_direction:
                simplified.append(path[index - 1])
            previous_direction = direction
        simplified.append(path[-1])
        return simplified

    def publish_empty_path(self, frame_id, reason):
        path = Path()
        path.header.frame_id = frame_id
        path.header.stamp = rospy.Time.now()
        self.path_pub.publish(path)
        self.status_pub.publish(String(data="FAILED: " + reason))
        rospy.logwarn_throttle(2.0, "A* planning failed: %s", reason)

    def publish_path(self, message, cells, expansions):
        output = Path()
        output.header.frame_id = message.header.frame_id
        output.header.stamp = rospy.Time.now()

        world_points = [self.grid_to_world(message, x, y) for x, y in cells]

        for index, (world_x, world_y) in enumerate(world_points):
            pose = PoseStamped()
            pose.header = output.header
            pose.pose.position.x = world_x
            pose.pose.position.y = world_y
            pose.pose.position.z = 0.0

            if index + 1 < len(world_points):
                next_x, next_y = world_points[index + 1]
                yaw = math.atan2(next_y - world_y, next_x - world_x)
            elif index > 0:
                old_x, old_y = world_points[index - 1]
                yaw = math.atan2(world_y - old_y, world_x - old_x)
            else:
                yaw = 0.0

            q = quaternion_from_euler(0.0, 0.0, yaw)
            pose.pose.orientation = Quaternion(*q)
            output.poses.append(pose)

        self.path_pub.publish(output)
        length = 0.0
        for index in range(1, len(world_points)):
            length += math.hypot(
                world_points[index][0] - world_points[index - 1][0],
                world_points[index][1] - world_points[index - 1][1],
            )
        status = "PLANNED poses={} length={:.2f}m expansions={}".format(
            len(output.poses), length, expansions
        )
        self.status_pub.publish(String(data=status))
        rospy.loginfo_throttle(2.0, status)

    def timer_callback(self, _event):
        with self.lock:
            message = self.latest_map
            odometry = self.latest_odom
            goal = self.latest_goal
            goal_revision = self.goal_revision

        if message is None or odometry is None or goal is None:
            rospy.logwarn_throttle(5.0, "A* waiting for map, odometry, or goal")
            return

        now = rospy.Time.now()
        elapsed = (now - self.last_plan_time).to_sec()
        if (
            goal_revision == self.planned_goal_revision
            and elapsed < self.periodic_replan_seconds
        ):
            return

        map_frame = message.header.frame_id.lstrip("/")
        odom_frame = odometry.header.frame_id.lstrip("/")
        goal_frame = goal.header.frame_id.lstrip("/")
        if not map_frame or map_frame != odom_frame or map_frame != goal_frame:
            self.publish_empty_path(
                message.header.frame_id,
                "frame mismatch map='{}' odom='{}' goal='{}'".format(
                    map_frame, odom_frame, goal_frame
                ),
            )
            return

        expected_size = message.info.width * message.info.height
        if len(message.data) != expected_size:
            self.publish_empty_path(map_frame, "invalid occupancy-grid size")
            return

        traversable = self.build_traversable_mask(message)
        start_raw = self.world_to_grid(
            message,
            odometry.pose.pose.position.x,
            odometry.pose.pose.position.y,
        )
        goal_raw = self.world_to_grid(
            message, goal.pose.position.x, goal.pose.position.y
        )
        search_cells = int(math.ceil(
            self.nearest_free_radius / message.info.resolution
        ))
        start = self.nearest_traversable(traversable, start_raw, search_cells)

        target = self.nearest_traversable(
            traversable,
            goal_raw,
            search_cells
        )

        self.last_plan_time = now
        self.planned_goal_revision = goal_revision

        if start is None:
            self.publish_empty_path(
                map_frame,
                "no inflated-free start near robot"
            )
            return

        cells = None
        expansions = 0
        used_reachable_fallback = False

        # First attempt the cell nearest to the requested frontier.
        if target is not None:
            cells, expansions = self.astar(
                traversable,
                start,
                target
            )

        # The closest cell may lie across a wall or disconnected map
        # component. Find the closest target in the robot's own component.
        if not cells:
            target = self.nearest_reachable_target(
                traversable,
                start,
                goal_raw
            )
            used_reachable_fallback = True

            if target is None or target == start:
                self.publish_empty_path(
                    map_frame,
                    "no safe reachable progress toward goal"
                )
                return

            cells, expansions = self.astar(
                traversable,
                start,
                target
            )

        if not cells:
            self.publish_empty_path(
                map_frame,
                "reachable fallback planning failed"
            )
            return

        if used_reachable_fallback:
            target_x, target_y = self.grid_to_world(
                message,
                target[0],
                target[1]
            )

            rospy.logwarn_throttle(
                2.0,
                "Requested frontier unavailable; using reachable "
                "standoff target=(%.2f, %.2f)",
                target_x,
                target_y
            )

        cells = self.simplify_collinear(cells)
        self.publish_path(message, cells, expansions)


if __name__ == "__main__":
    rospy.init_node("astar_planner")
    AStarPlanner()
    rospy.spin()
