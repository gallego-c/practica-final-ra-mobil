#!/usr/bin/env python3
"""
Single-robot frontier exploration on the SLAM map.
Goal: maximize mapped area (unknown -> free/occupied).
Optimized for safety and obstacle avoidance.
"""
from __future__ import division

import math
import threading
import time

import actionlib
import rospy
import tf2_ros
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Bool

FREE_THRESH = 25
NEIGHBORS_4 = ((1, 0), (-1, 0), (0, 1), (0, -1))


def yaw_to_quaternion(yaw):
    from tf.transformations import quaternion_from_euler
    q = quaternion_from_euler(0, 0, yaw)
    return q[0], q[1], q[2], q[3]


def quat_to_yaw(q):
    from tf.transformations import euler_from_quaternion
    return euler_from_quaternion([q.x, q.y, q.z, q.w])[2]


def cell_index(mx, my, width):
    return my * width + mx


def world_to_map(wx, wy, info):
    mx = int((wx - info.origin.position.x) / info.resolution)
    my = int((wy - info.origin.position.y) / info.resolution)
    return mx, my


def map_to_world(mx, my, info):
    wx = info.origin.position.x + (mx + 0.5) * info.resolution
    wy = info.origin.position.y + (my + 0.5) * info.resolution
    return wx, wy


def is_free(value):
    return 0 <= value < FREE_THRESH


class SlamSingleFrontierExplorer:
    def __init__(self):
        rospy.init_node('slam_frontier_explorer')

        self.map_frame = rospy.get_param('~map_frame', 'map')
        self.map_topic = rospy.get_param('~map_topic', '/map')
        self.exploration_timeout = float(rospy.get_param('~exploration_timeout', 300.0))
        self.goal_timeout = float(rospy.get_param('~goal_timeout', 50.0))
        self.min_frontier_points = int(rospy.get_param('~min_frontier_points', 6))
        self.min_coverage = float(rospy.get_param('~min_coverage', 0.75))
        self.gain_scale = float(rospy.get_param('~gain_scale', 2.0))
        self.progress_timeout = float(rospy.get_param('~progress_timeout', 15.0))
        self.min_progress_distance = float(rospy.get_param('~min_progress_distance', 0.15))
        self.startup_delay = float(rospy.get_param('~startup_delay', 15.0))
        self.move_base_timeout = float(rospy.get_param('~move_base_timeout', 120.0))
        self.idle_frontier_cycles = int(rospy.get_param('~idle_frontier_cycles', 8))

        self.robot_ns = rospy.get_param('~robot_ns', 'robot1')
        self.base_frame = rospy.get_param('~base_frame', 'robot1/base_footprint')

        self._map = None
        self._map_lock = threading.Lock()
        self._stop = threading.Event()
        self._blacklist = {}  # (wx, wy) -> expiry monotonic time
        self._blacklist_lock = threading.Lock()
        self._stats_lock = threading.Lock()
        self._idle_cycles = 0
        self._last_coverage = 0.0

        rospy.Subscriber(self.map_topic, OccupancyGrid, self._map_cb, queue_size=1)
        self._done_pub = rospy.Publisher(
            '/slam_exploration/complete', Bool, queue_size=1, latch=True)

        self._tf_buffer = tf2_ros.Buffer()
        tf2_ros.TransformListener(self._tf_buffer)

        self._client = actionlib.SimpleActionClient(
            '/' + self.robot_ns + '/move_base', MoveBaseAction)
        self._cmd_pub = rospy.Publisher(
            '/' + self.robot_ns + '/cmd_vel', Twist, queue_size=1)

        rospy.loginfo('Waiting %.0f s for navigation stack...', self.startup_delay)
        rospy.sleep(self.startup_delay)
        if not self._client.wait_for_server(rospy.Duration(self.move_base_timeout)):
            raise rospy.ROSException('move_base not ready for ' + self.robot_ns)

        rospy.loginfo('Frontier explorer ready on %s', self.map_topic)

    def _map_cb(self, msg):
        with self._map_lock:
            self._map = msg

    def _get_map(self):
        with self._map_lock:
            return self._map

    def _coverage(self, grid):
        if grid is None or not grid.data:
            return 0.0
        known = sum(1 for v in grid.data if v >= 0)
        return float(known) / float(len(grid.data))

    def _unknown_count(self, grid):
        if grid is None:
            return 10 ** 9
        return sum(1 for v in grid.data if v < 0)

    def _get_robot_pose(self):
        try:
            tf = self._tf_buffer.lookup_transform(
                self.map_frame, self.base_frame, rospy.Time(0), rospy.Duration(0.3))
            yaw = quat_to_yaw(tf.transform.rotation)
            return (tf.transform.translation.x,
                    tf.transform.translation.y, yaw)
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException):
            return None

    def _blacklist_key(self, wx, wy):
        return (round(wx, 1), round(wy, 1))

    def _is_blacklisted(self, wx, wy):
        key = self._blacklist_key(wx, wy)
        now = time.monotonic()
        with self._blacklist_lock:
            exp = self._blacklist.get(key)
            if exp is None:
                return False
            if now >= exp:
                del self._blacklist[key]
                return False
            return True

    def _blacklist_goal(self, wx, wy, duration=50.0):
        key = self._blacklist_key(wx, wy)
        with self._blacklist_lock:
            self._blacklist[key] = time.monotonic() + duration

    def _find_frontiers(self, grid):
        info = grid.info
        w, h = info.width, info.height
        data = grid.data
        frontier_cells = set()

        for my in range(1, h - 1):
            for mx in range(1, w - 1):
                idx = cell_index(mx, my, w)
                if not is_free(data[idx]):
                    continue
                for dx, dy in NEIGHBORS_4:
                    if data[cell_index(mx + dx, my + dy, w)] < 0:
                        frontier_cells.add((mx, my))
                        break

        if not frontier_cells:
            return []

        # Proper BFS connected-component clustering (prevents splitting frontiers across rigid bins)
        clusters = []
        visited = set()
        
        for cell in frontier_cells:
            if cell in visited:
                continue
            
            cluster = []
            queue = [cell]
            visited.add(cell)
            
            while queue:
                curr = queue.pop(0)
                cluster.append(curr)
                cx, cy = curr
                
                # Check 8-connected neighbors within 2 cells (bridges small gaps in SLAM noise)
                for dy in range(-2, 3):
                    for dx in range(-2, 3):
                        nx, ny = cx + dx, cy + dy
                        n_cell = (nx, ny)
                        if n_cell in frontier_cells and n_cell not in visited:
                            visited.add(n_cell)
                            queue.append(n_cell)
            
            if len(cluster) >= self.min_frontier_points:
                clusters.append(cluster)

        centroids = []
        for cluster in clusters:
            sx = sum(c[0] for c in cluster)
            sy = sum(c[1] for c in cluster)
            n = len(cluster)
            cmx, cmy = int(round(sx / n)), int(round(sy / n))
            
            wx, wy = map_to_world(cmx, cmy, info)
            mx, my = world_to_map(wx, wy, info)
            
            # Ensure the centroid itself is free, fallback to first free cell in cluster if needed
            if 0 <= mx < w and 0 <= my < h and is_free(data[cell_index(mx, my, w)]):
                centroids.append((wx, wy, len(cluster)))
            else:
                for cx, cy in cluster:
                    if is_free(data[cell_index(cx, cy, w)]):
                        gwx, gwy = map_to_world(cx, cy, info)
                        centroids.append((gwx, gwy, len(cluster)))
                        break
        return centroids


    def _cell_has_clearance(self, grid, mx, my, obstacle_clearance, unknown_clearance):
        """Checks if a cell is free and has specified clearances from known obstacles and unknown space."""
        info = grid.info
        w, h = info.width, info.height
        data = grid.data

        # Target cell must be free
        if not is_free(data[cell_index(mx, my, w)]):
            return False

        # Convert clearance in meters to cell grid distance
        clear_cells_obs = int(math.ceil(obstacle_clearance / info.resolution))
        clear_cells_unkn = int(math.ceil(unknown_clearance / info.resolution))
        max_r = max(clear_cells_obs, clear_cells_unkn)

        if mx < max_r or my < max_r or mx >= w - max_r or my >= h - max_r:
            return False

        # Scan surrounding bounding box
        for dy in range(-max_r, max_r + 1):
            for dx in range(-max_r, max_r + 1):
                dist_m = math.hypot(dx, dy) * info.resolution
                val = data[cell_index(mx + dx, my + dy, w)]

                if dist_m <= obstacle_clearance and val >= FREE_THRESH:
                    return False # Too close to an obstacle
                if dist_m <= unknown_clearance and val < 0:
                    return False # Too close to unknown space (where hidden walls may lie)

        return True

    def _safe_goal_near_frontier(self, grid, wx, wy):
        """Move the goal off the frontier edge and into a nearby clear known cell.
        Uses a layered fallback search to balance safety and navigation ability in tight spaces."""
        info = grid.info
        mx, my = world_to_map(wx, wy, info)
        
        # Clearance levels: (obstacle_clearance, unknown_clearance) in meters
        clearance_levels = [
            (0.32, 0.30),  # Level 1: Highly safe, robot fully in known free space
            (0.28, 0.22),  # Level 2: Safe clearance, still allows rotation
            (0.26, 0.15)   # Level 3: Tight fallback, absolute minimum clearance to rotate safely
        ]

        # Search window: search up to 1.2 meters away from frontier centroid
        search_cells = max(2, int(round(1.2 / info.resolution)))

        for obs_clear, unkn_clear in clearance_levels:
            best = None
            best_dist = float('inf')
            
            # Spiral search out from the center mx, my
            for radius in range(0, search_cells + 1):
                for dy in range(-radius, radius + 1):
                    for dx in range(-radius, radius + 1):
                        if abs(dx) != radius and abs(dy) != radius:
                            continue
                        cx = mx + dx
                        cy = my + dy
                        if not self._cell_has_clearance(grid, cx, cy, obs_clear, unkn_clear):
                            continue
                        gx, gy = map_to_world(cx, cy, info)
                        d = math.hypot(gx - wx, gy - wy)
                        if d < best_dist:
                            best_dist = d
                            best = (gx, gy)
                
                # If we found a valid pose at this radius for the current safety level, use it!
                if best is not None:
                    return best

        return None

    def _pick_frontier(self, robot_xy, frontiers):
        """Prefer large frontiers close to the robot."""
        best = None
        best_score = float('inf')

        for wx, wy, size in frontiers:
            if self._is_blacklisted(wx, wy):
                continue

            my_dist = math.hypot(wx - robot_xy[0], wy - robot_xy[1])
            
            # Skip frontiers that are too close to the robot. If we are within 0.65m,
            # the laser scan has already cleared everything it could. If the frontier
            # remains, it is likely behind a wall or inside an obstacle.
            if my_dist < 0.65:
                continue

            gain = self.gain_scale * math.sqrt(float(size))
            score = my_dist - gain
            if score < best_score:
                best_score = score
                best = (wx, wy, size)

        return best

    def _make_goal(self, wx, wy, yaw):
        qx, qy, qz, qw = yaw_to_quaternion(yaw)
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = self.map_frame
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = wx
        goal.target_pose.pose.position.y = wy
        goal.target_pose.pose.orientation.x = qx
        goal.target_pose.pose.orientation.y = qy
        goal.target_pose.pose.orientation.z = qz
        goal.target_pose.pose.orientation.w = qw
        return goal

    def _wait_for_goal(self, goal_xy, timeout):
        deadline = rospy.Time.now() + rospy.Duration(timeout)
        last_progress = rospy.Time.now()
        pose = self._get_robot_pose()
        best_dist = (math.hypot(goal_xy[0] - pose[0], goal_xy[1] - pose[1])
                     if pose is not None else float('inf'))
        rate = rospy.Rate(5.0)
        while rospy.Time.now() < deadline and not rospy.is_shutdown() and not self._stop.is_set():
            state = self._client.get_state()
            if state in (GoalStatus.SUCCEEDED, GoalStatus.ABORTED,
                         GoalStatus.REJECTED, GoalStatus.PREEMPTED):
                return state == GoalStatus.SUCCEEDED

            pose = self._get_robot_pose()
            if pose is not None:
                dist = math.hypot(goal_xy[0] - pose[0], goal_xy[1] - pose[1])
                if dist + self.min_progress_distance < best_dist:
                    best_dist = dist
                    last_progress = rospy.Time.now()

            if (rospy.Time.now() - last_progress).to_sec() > self.progress_timeout:
                rospy.logwarn('Goal stalled: no progress for %.1f s', self.progress_timeout)
                self._client.cancel_goal()
                return False

            rate.sleep()
        self._client.cancel_goal()
        return False

    def _spin_to_scan(self, duration=3.0):
        """Rotate in place to let SLAM capture more of the surroundings."""
        twist = Twist()
        twist.angular.z = 0.8
        end = rospy.Time.now() + rospy.Duration(duration)
        rate = rospy.Rate(10)
        while rospy.Time.now() < end and not rospy.is_shutdown() and not self._stop.is_set():
            self._cmd_pub.publish(twist)
            rate.sleep()
        self._cmd_pub.publish(Twist())

    def _robot_loop(self):
        rospy.loginfo('%s exploration loop started', self.robot_ns)
        consecutive_fails = 0

        while not self._stop.is_set() and not rospy.is_shutdown():
            grid = self._get_map()
            if grid is None:
                rospy.sleep(1.0)
                continue

            pose = self._get_robot_pose()
            if pose is None:
                rospy.sleep(1.0)
                continue

            frontiers = self._find_frontiers(grid)
            if not frontiers:
                rospy.loginfo_throttle(15.0, '%s: no frontiers visible', self.robot_ns)
                self._spin_to_scan(4.0)
                continue

            pick = self._pick_frontier(pose, frontiers)
            if pick is None:
                rospy.logwarn_throttle(10.0, '%s: no assignable frontier, spinning', self.robot_ns)
                self._spin_to_scan(3.0)
                continue

            fx, fy, size = pick
            safe_goal = self._safe_goal_near_frontier(grid, fx, fy)
            if safe_goal is None:
                self._blacklist_goal(fx, fy, duration=30.0)
                rospy.sleep(0.2)
                continue

            wx, wy = safe_goal
            yaw = math.atan2(wy - pose[1], wx - pose[0])
            rospy.loginfo('%s -> frontier (%.2f, %.2f) size=%d', self.robot_ns, wx, wy, size)
            self._client.send_goal(self._make_goal(wx, wy, yaw))
            ok = self._wait_for_goal((wx, wy), self.goal_timeout)

            if ok:
                consecutive_fails = 0
                # Blacklist the reached frontier to prevent "reach and repeat" loops
                self._blacklist_goal(fx, fy, duration=45.0)
            else:
                consecutive_fails += 1
                # Blacklist the failed frontier so we don't try it again immediately
                self._blacklist_goal(fx, fy, duration=40.0)
                if consecutive_fails >= 3:
                    rospy.logwarn('%s: %d consecutive fails, spinning to recover',
                                 self.robot_ns, consecutive_fails)
                    self._spin_to_scan(5.0)
                    consecutive_fails = 0

            rospy.sleep(0.2)

        self._client.cancel_all_goals()
        rospy.loginfo('%s exploration loop stopped', self.robot_ns)

    def run(self):
        rospy.loginfo(
            'Max-coverage frontier exploration (timeout=%.0fs, target coverage=%.0f%%)',
            self.exploration_timeout, self.min_coverage * 100.0)

        t = threading.Thread(target=self._robot_loop)
        t.daemon = True
        t.start()

        deadline = rospy.Time.now() + rospy.Duration(self.exploration_timeout)
        rate = rospy.Rate(1.0)

        while rospy.Time.now() < deadline and not rospy.is_shutdown():
            grid = self._get_map()
            cov = self._coverage(grid)
            unknown = self._unknown_count(grid)
            frontiers = self._find_frontiers(grid) if grid else []

            with self._stats_lock:
                self._last_coverage = cov
                if frontiers:
                    self._idle_cycles = 0
                else:
                    self._idle_cycles += 1

            rospy.loginfo_throttle(
                10.0,
                'Exploration: coverage=%.1f%% unknown=%d frontiers=%d',
                cov * 100.0, unknown, len(frontiers))

            if cov >= self.min_coverage:
                rospy.loginfo('Coverage target reached (%.1f%% >= %.1f%%)',
                               cov * 100.0, self.min_coverage * 100.0)
                break

            if self._idle_cycles >= self.idle_frontier_cycles:
                rospy.loginfo('No frontiers for %d cycles — exploration done',
                               self.idle_frontier_cycles)
                break

            rate.sleep()

        self._stop.set()
        t.join(timeout=5.0)
        self._client.cancel_all_goals()

        cov = self._last_coverage
        self._done_pub.publish(Bool(data=True))
        rospy.loginfo('Frontier exploration finished (coverage=%.1f%%)', cov * 100.0)


def main():
    try:
        SlamSingleFrontierExplorer().run()
    except rospy.ROSInterruptException:
        pass


if __name__ == '__main__':
    main()
