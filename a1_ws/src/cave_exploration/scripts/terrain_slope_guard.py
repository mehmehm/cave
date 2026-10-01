#!/usr/bin/env python3
"""Local LiDAR ground-slope layer for the A1 cave exploration grid.

Publishes an occupancy grid with observed steep transitions marked occupied.
The original /projected_map is left untouched. Unknown terrain remains unknown.
"""

import copy
import math
import threading
from collections import defaultdict

import numpy as np
import rospy
import tf2_ros
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from sensor_msgs.msg import PointCloud2
from sensor_msgs import point_cloud2
from std_msgs.msg import Bool, Float32
from tf.transformations import euler_from_quaternion, quaternion_matrix


def grid_yaw(grid):
    q = grid.info.origin.orientation
    if q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w < 1e-12:
        return 0.0
    return euler_from_quaternion((q.x, q.y, q.z, q.w))[2]


def world_to_cell(grid, x, y):
    origin = grid.info.origin.position
    angle = grid_yaw(grid)
    dx, dy = x - origin.x, y - origin.y
    c, s = math.cos(angle), math.sin(angle)
    return (int(math.floor((c*dx + s*dy)/grid.info.resolution)),
            int(math.floor((-s*dx + c*dy)/grid.info.resolution)))


def cell_to_world(grid, x, y):
    origin = grid.info.origin.position
    angle = grid_yaw(grid)
    dx = (x + 0.5) * grid.info.resolution
    dy = (y + 0.5) * grid.info.resolution
    return (origin.x + math.cos(angle)*dx - math.sin(angle)*dy,
            origin.y + math.sin(angle)*dx + math.cos(angle)*dy)


def estimate_ground_heights(samples, minimum_points, roughness_limit):
    """Use the low part of each cell's return distribution as a floor estimate."""
    heights = {}
    for cell, values in samples.items():
        if len(values) < minimum_points:
            continue
        low, middle = np.percentile(values, (10, 45))
        if middle - low <= roughness_limit:
            heights[cell] = float(np.percentile(values, 20))
    return heights


def steep_cells(heights, resolution, maximum_degrees):
    unsafe = set()
    limit = math.tan(math.radians(maximum_degrees))
    for (x, y), z in heights.items():
        for dx, dy in ((1, 0), (0, 1), (1, 1), (1, -1)):
            other = (x + dx, y + dy)
            if other not in heights:
                continue
            rise = abs(heights[other] - z)
            run = resolution * math.hypot(dx, dy)
            if rise / run > limit:
                unsafe.update(((x, y), other))
    return unsafe


class TerrainSlopeGuard:
    def __init__(self):
        self.map_topic = rospy.get_param('~map_topic', '/projected_map')
        self.cloud_topic = rospy.get_param('~cloud_topic', '/velodyne_points')
        self.odom_topic = rospy.get_param('~odom_topic', '/lio_sam/mapping/odometry')
        self.path_topic = rospy.get_param('~path_topic', '/cave/exploration/path')
        self.output_topic = rospy.get_param('~output_topic', '/cave/map/slope_safe_projected_map')
        self.max_slope_degrees = float(rospy.get_param('~max_slope_degrees', 22.0))
        self.max_grid_resolution = float(rospy.get_param('~max_grid_resolution', 0.55))
        self.local_radius = float(rospy.get_param('~local_radius', 8.0))
        self.minimum_points = int(rospy.get_param('~minimum_points_per_cell', 5))
        self.point_skip = max(1, int(rospy.get_param('~point_skip', 3)))
        self.roughness_limit = float(rospy.get_param('~roughness_limit', 0.30))
        self.approach_distance = float(rospy.get_param('~approach_distance', 2.0))
        # A single scan is sparse. Keep a detected steep cell occupied long
        # enough that a missed return cannot open/close the route every scan.
        self.unsafe_cell_hold_seconds = max(0.0, float(
            rospy.get_param('~unsafe_cell_hold_seconds', 15.0)))
        self.unsafe_cell_expiry = {}
        self.unsafe_grid_signature = None
        self.lock = threading.Lock()
        self.grid = self.cloud = self.odom = self.path = None
        self.tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(15.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.map_pub = rospy.Publisher(self.output_topic, OccupancyGrid, queue_size=1)
        self.slope_pub = rospy.Publisher('/cave/terrain/approach_slope_deg', Float32, queue_size=1)
        self.unsafe_pub = rospy.Publisher('/cave/terrain/unsafe_ahead', Bool, queue_size=1)
        self.grid_sub = rospy.Subscriber(self.map_topic, OccupancyGrid, self.set_grid, queue_size=1)
        self.cloud_sub = rospy.Subscriber(self.cloud_topic, PointCloud2, self.set_cloud, queue_size=1)
        self.odom_sub = rospy.Subscriber(self.odom_topic, Odometry, self.set_odom, queue_size=1)
        self.path_sub = rospy.Subscriber(self.path_topic, Path, self.set_path, queue_size=1)
        self.timer = rospy.Timer(rospy.Duration(0.5), self.update)
        rospy.loginfo('Slope guard: %s + %s -> %s', self.map_topic,
                      self.cloud_topic, self.output_topic)

    def set_grid(self, msg):
        with self.lock: self.grid = msg

    def set_cloud(self, msg):
        with self.lock: self.cloud = msg

    def set_odom(self, msg):
        with self.lock: self.odom = msg

    def set_path(self, msg):
        with self.lock: self.path = msg

    def ground_samples(self, grid, cloud, odom):
        target_frame = grid.header.frame_id.lstrip('/')
        source_frame = cloud.header.frame_id.lstrip('/')
        if not target_frame or not source_frame:
            return None
        try:
            transform = self.tf_buffer.lookup_transform(
                target_frame, source_frame, cloud.header.stamp,
                rospy.Duration(0.15))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException):
            rospy.logwarn_throttle(5.0, 'Slope guard waiting for cloud TF at scan timestamp')
            return None
        q = transform.transform.rotation
        rotation = quaternion_matrix((q.x, q.y, q.z, q.w))[:3, :3]
        translation = transform.transform.translation
        tx, ty, tz = translation.x, translation.y, translation.z
        robot = odom.pose.pose.position
        samples = defaultdict(list)
        width, height = grid.info.width, grid.info.height
        for index, point in enumerate(point_cloud2.read_points(
                cloud, field_names=('x', 'y', 'z'), skip_nans=True)):
            if index % self.point_skip:
                continue
            px, py, pz = point
            x = rotation[0,0]*px + rotation[0,1]*py + rotation[0,2]*pz + tx
            y = rotation[1,0]*px + rotation[1,1]*py + rotation[1,2]*pz + ty
            if abs(x - robot.x) > self.local_radius or abs(y - robot.y) > self.local_radius:
                continue
            z = rotation[2,0]*px + rotation[2,1]*py + rotation[2,2]*pz + tz
            # Discard ceilings and most wall returns. This is a heuristic;
            # unknown cells are never declared traversable by this layer.
            if z < robot.z - 1.2 or z > robot.z + 0.9:
                continue
            cx, cy = world_to_cell(grid, x, y)
            if 0 <= cx < width and 0 <= cy < height and len(samples[cx, cy]) < 100:
                samples[cx, cy].append(z)
        return samples

    def approach(self, grid, heights, unsafe, path, odom):
        if path is None or not path.poses or path.header.frame_id.lstrip('/') != grid.header.frame_id.lstrip('/'):
            return float('nan'), False
        robot = odom.pose.pose.position
        # Evaluate the closest path segment and up to approach_distance ahead.
        points = [(p.pose.position.x, p.pose.position.y) for p in path.poses]
        if len(points) < 2:
            return float('nan'), False
        nearest = None
        for i in range(len(points)-1):
            ax, ay = points[i]
            bx, by = points[i+1]
            dx, dy = bx-ax, by-ay
            length_sq = dx*dx + dy*dy
            if length_sq < 1e-12:
                continue
            t = max(0.0, min(1.0,
                ((robot.x-ax)*dx + (robot.y-ay)*dy)/length_sq))
            px, py = ax+t*dx, ay+t*dy
            candidate = ((px-robot.x)**2+(py-robot.y)**2, i, (px, py))
            if nearest is None or candidate[0] < nearest[0]:
                nearest = candidate
        if nearest is None:
            return float('nan'), False
        samples = [(robot.x, robot.y)]
        remaining = self.approach_distance
        previous = nearest[2]
        samples.append(previous)
        for end in points[nearest[1]+1:]:
            length = math.hypot(end[0]-previous[0], end[1]-previous[1])
            if length < 1e-6:
                continue
            travel = min(length, remaining)
            steps = max(1, int(math.ceil(travel / max(grid.info.resolution / 2.0, 0.1))))
            for step in range(1, steps + 1):
                fraction = (travel * step / steps) / length
                samples.append((previous[0] + fraction*(end[0]-previous[0]),
                                previous[1] + fraction*(end[1]-previous[1])))
            remaining -= travel
            if remaining <= 1e-6:
                break
            previous = end
        cells = [world_to_cell(grid, x, y) for x, y in samples]
        current_cell = world_to_cell(grid, robot.x, robot.y)
        # The robot can already be inside the one-cell planning buffer when a
        # slope is first observed. Stopping because its *current* cell is in
        # that buffer deadlocks a short escape path. Only halt for an observed
        # steep cell a short distance ahead on the commanded route.
        hazard = any(
            cell != current_cell and cell in unsafe
            and 0.30 <= math.hypot(x-robot.x, y-robot.y) <= 1.0
            for (x, y), cell in zip(samples, cells)
        )
        known = [(i, heights[cell]) for i, cell in enumerate(cells) if cell in heights]
        if len(known) < 2:
            return float('nan'), hazard
        first_index, first_z = known[0]
        last_index, last_z = known[-1]
        x0, y0 = samples[first_index]
        x1, y1 = samples[last_index]
        distance = math.hypot(x1-x0, y1-y0)
        if distance < grid.info.resolution:
            return float('nan'), hazard
        return math.degrees(math.atan2(last_z-first_z, distance)), hazard

    def update(self, _event):
        with self.lock:
            grid, cloud, odom, path = self.grid, self.cloud, self.odom, self.path
        if grid is None or cloud is None or odom is None:
            return
        if grid.info.resolution <= 0 or grid.info.resolution > self.max_grid_resolution:
            rospy.logerr_throttle(5.0, 'Slope guard: grid %.2fm is too coarse; refusing to publish a safe map', grid.info.resolution)
            return
        if len(grid.data) != grid.info.width * grid.info.height:
            return
        if grid.header.frame_id.lstrip('/') != odom.header.frame_id.lstrip('/'):
            rospy.logerr_throttle(5.0, 'Slope guard: map and odometry frames differ')
            return
        if (rospy.Time.now() - cloud.header.stamp).to_sec() > 2.0:
            rospy.logwarn_throttle(5.0, 'Slope guard: stale LiDAR scan')
            return
        samples = self.ground_samples(grid, cloud, odom)
        if samples is None:
            return
        heights = estimate_ground_heights(samples, self.minimum_points, self.roughness_limit)
        observed_unsafe = steep_cells(heights, grid.info.resolution, self.max_slope_degrees)
        origin = grid.info.origin
        orientation = origin.orientation
        signature = (grid.header.frame_id, grid.info.width, grid.info.height,
                     grid.info.resolution, origin.position.x, origin.position.y,
                     orientation.x, orientation.y, orientation.z, orientation.w)
        if signature != self.unsafe_grid_signature:
            # Cell indices from an old map geometry must never be reused.
            self.unsafe_cell_expiry.clear()
            self.unsafe_grid_signature = signature
        now_seconds = rospy.Time.now().to_sec()
        self.unsafe_cell_expiry = {
            cell: expiry for cell, expiry in self.unsafe_cell_expiry.items()
            if expiry > now_seconds
        }
        for cell in observed_unsafe:
            self.unsafe_cell_expiry[cell] = now_seconds + self.unsafe_cell_hold_seconds
        unsafe = observed_unsafe | set(self.unsafe_cell_expiry)
        # Buffer observed hazards by one grid cell for the A1 footprint.
        buffered = set()
        for x, y in unsafe:
            buffered.update((x+dx, y+dy) for dx in (-1,0,1) for dy in (-1,0,1))
        output = copy.deepcopy(grid)
        output.header.stamp = rospy.Time.now()
        data = list(output.data)
        for x, y in buffered:
            if 0 <= x < grid.info.width and 0 <= y < grid.info.height:
                data[y*grid.info.width+x] = 100
        output.data = data
        self.map_pub.publish(output)
        slope, hazard = self.approach(grid, heights, unsafe, path, odom)
        self.slope_pub.publish(Float32(data=slope))
        self.unsafe_pub.publish(Bool(data=hazard))
        rospy.loginfo_throttle(5.0, 'Slope guard: floor cells=%d observed_steep=%d retained_steep=%d ahead=%s',
                               len(heights), len(observed_unsafe), len(unsafe),
                               'unknown' if math.isnan(slope) else '%.1f deg' % slope)


if __name__ == '__main__':
    rospy.init_node('terrain_slope_guard')
    TerrainSlopeGuard()
    rospy.spin()
