#!/usr/bin/env python3
"""
Multi-robot frontier exploration and Capture the Flag (CTF) game loop on the merged SLAM map.
Combines:
1. Voronoi-partitioned frontier exploration (SLAM map).
2. Vision-based flag detection, pursuit, and capture.
3. Return/chase phase (carrier heads home, pursuer chases carrier).
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
from visualization_msgs.msg import Marker, MarkerArray

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


class SlamFrontierExplorerCtf:
    def __init__(self):
        rospy.init_node('slam_frontier_explorer_ctf')

        self.map_frame = rospy.get_param('~map_frame', 'map')
        self.map_topic = rospy.get_param('~map_topic', '/merged_map')
        self.exploration_timeout = float(rospy.get_param('~exploration_timeout', 300.0))
        self.goal_timeout = float(rospy.get_param('~goal_timeout', 50.0))
        self.min_frontier_points = int(rospy.get_param('~min_frontier_points', 6))
        self.min_coverage = float(rospy.get_param('~min_coverage', 0.72))
        self.gain_scale = float(rospy.get_param('~gain_scale', 2.0))
        self.voronoi_margin = float(rospy.get_param('~voronoi_margin', 0.25))
        self.min_goal_clearance = float(rospy.get_param('~min_goal_clearance', 0.15))
        self.progress_timeout = float(rospy.get_param('~progress_timeout', 15.0))
        self.min_progress_distance = float(rospy.get_param('~min_progress_distance', 0.15))
        self.startup_delay = float(rospy.get_param('~startup_delay', 30.0))
        self.move_base_timeout = float(rospy.get_param('~move_base_timeout', 120.0))
        self.idle_frontier_cycles = int(rospy.get_param('~idle_frontier_cycles', 8))

        # CTF parameters
        self.flag_capture_distance = float(rospy.get_param('~flag_capture_distance', 0.45))
        self.flag_standoff_distance = float(rospy.get_param('~flag_standoff_distance', 0.20))
        self.catch_distance = float(rospy.get_param('~catch_distance', 0.65))
        self.chase_goal_period = float(rospy.get_param('~chase_goal_period', 1.0))
        self.capture_pause_sec = float(rospy.get_param('~capture_pause_sec', 3.0))
        self.flag_memory_timeout = float(rospy.get_param('~flag_memory_timeout', 5.0))

        self.robots = rospy.get_param('~robots', [
            {'ns': 'robot1', 'base_frame': 'robot1/base_footprint'},
            {'ns': 'robot2', 'base_frame': 'robot2/base_footprint'},
        ])

        # Game states: 'EXPLORING', 'CAPTURED', 'CHASE', 'FINISHED'
        self.game_state = 'EXPLORING'
        self.carrier_ns = None
        self.pursuer_ns = None

        # Flag estimate and detection status
        self.flag_found = {'robot1': False, 'robot2': False}
        self.flag_estimate = {'robot1': None, 'robot2': None}
        self.flag_estimate_time = {'robot1': 0.0, 'robot2': 0.0}
        self.flag_lock = threading.Lock()

        # Goal throttling
        self._last_flag_goal = {}
        self._last_flag_goal_time = {}

        # Robot homes
        self.homes = {
            'robot1': rospy.get_param('~robot1_home', [-3.0, -3.0, 0.0]),
            'robot2': rospy.get_param('~robot2_home', [3.0, 3.0, 3.1416]),
        }

        self._map = None
        self._map_lock = threading.Lock()
        self._stop = threading.Event()
        self._blacklist = {}  # (wx, wy) -> expiry monotonic time
        self._blacklist_lock = threading.Lock()
        self._stats_lock = threading.Lock()
        self._idle_cycles = 0
        self._last_coverage = 0.0

        # Subscriptions
        rospy.Subscriber(self.map_topic, OccupancyGrid, self._map_cb, queue_size=1)
        self._done_pub = rospy.Publisher('/slam_exploration/complete', Bool, queue_size=1, latch=True)
        self._flag_captured_pub = rospy.Publisher('/ctf/flag_captured', Bool, queue_size=1, latch=True)
        self._marker_pub = rospy.Publisher('/ctf/markers', MarkerArray, queue_size=1, latch=True)

        self._tf_buffer = tf2_ros.Buffer()
        tf2_ros.TransformListener(self._tf_buffer)

        self._clients = {}
        self._cmd_pubs = {}
        self._flag_found_subs = {}
        self._flag_est_subs = {}

        for cfg in self.robots:
            ns = cfg['ns']
            self._clients[ns] = actionlib.SimpleActionClient('/' + ns + '/move_base', MoveBaseAction)
            self._cmd_pubs[ns] = rospy.Publisher('/' + ns + '/cmd_vel', Twist, queue_size=1)
            
            # Subscribe to flag detectors (topic: /robotN/flag_detector/...)
            self._flag_found_subs[ns] = rospy.Subscriber(
                '/' + ns + '/flag_detector/flag_found', Bool, self._flag_found_cb, callback_args=ns, queue_size=1)
            self._flag_est_subs[ns] = rospy.Subscriber(
                '/' + ns + '/flag_detector/flag_estimate', PoseStamped, self._flag_estimate_cb, callback_args=ns, queue_size=1)

        rospy.loginfo('Waiting %.0f s for navigation stack...', self.startup_delay)
        rospy.sleep(self.startup_delay)
        if not self._wait_for_all_move_base():
            raise rospy.ROSException('move_base not ready for all robots')

        # Publish initial capture state (False)
        self._flag_captured_pub.publish(Bool(data=False))
        rospy.loginfo('SlamFrontierExplorerCtf: initialized and ready.')

    def _wait_for_all_move_base(self):
        deadline = rospy.Time.now() + rospy.Duration(self.move_base_timeout)
        pending = set(self._clients.keys())
        rate = rospy.Rate(2.0)
        while pending and rospy.Time.now() < deadline and not rospy.is_shutdown():
            for ns in list(pending):
                if self._clients[ns].wait_for_server(rospy.Duration(0.1)):
                    rospy.loginfo('%s move_base ready', ns)
                    pending.discard(ns)
            rate.sleep()
        return not pending

    def _flag_found_cb(self, msg, ns):
        with self.flag_lock:
            self.flag_found[ns] = msg.data

    def _flag_estimate_cb(self, msg, ns):
        with self.flag_lock:
            self.flag_estimate[ns] = (msg.pose.position.x, msg.pose.position.y)
            self.flag_estimate_time[ns] = rospy.Time.now().to_sec()

    def _get_fresh_flag_estimate(self, ns):
        with self.flag_lock:
            if self.flag_estimate[ns] is None:
                return False, None
            age = rospy.Time.now().to_sec() - self.flag_estimate_time[ns]
            if age <= self.flag_memory_timeout:
                return True, self.flag_estimate[ns]
            return False, None

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

    def _get_robot_pose(self, base_frame):
        try:
            tf = self._tf_buffer.lookup_transform(
                self.map_frame, base_frame, rospy.Time(0), rospy.Duration(0.3))
            yaw = quat_to_yaw(tf.transform.rotation)
            return (tf.transform.translation.x,
                    tf.transform.translation.y, yaw)
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException):
            return None

    def _all_robot_poses(self):
        poses = {}
        for cfg in self.robots:
            p = self._get_robot_pose(cfg['base_frame'])
            if p is not None:
                poses[cfg['ns']] = p
        return poses

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
        frontier_cells = []

        for my in range(1, h - 1):
            for mx in range(1, w - 1):
                idx = cell_index(mx, my, w)
                if not is_free(data[idx]):
                    continue
                for dx, dy in NEIGHBORS_4:
                    if data[cell_index(mx + dx, my + dy, w)] < 0:
                        frontier_cells.append((mx, my))
                        break

        if not frontier_cells:
            return []

        bucket = max(3, int(0.30 / info.resolution))
        clusters = {}
        for mx, my in frontier_cells:
            key = (mx // bucket, my // bucket)
            clusters.setdefault(key, []).append((mx, my))

        centroids = []
        for cells in clusters.values():
            if len(cells) < self.min_frontier_points:
                continue
            sx = sum(c[0] for c in cells)
            sy = sum(c[1] for c in cells)
            n = len(cells)
            cmx, cmy = sx // n, sy // n
            wx, wy = map_to_world(cmx, cmy, info)
            mx, my = world_to_map(wx, wy, info)
            if 0 <= mx < w and 0 <= my < h and is_free(data[cell_index(mx, my, w)]):
                centroids.append((wx, wy, len(cells)))
        return centroids

    def _cell_has_clearance(self, grid, mx, my):
        info = grid.info
        w, h = info.width, info.height
        data = grid.data
        clear_cells = max(1, int(round(self.min_goal_clearance / info.resolution)))

        if mx < clear_cells or my < clear_cells or mx >= w - clear_cells or my >= h - clear_cells:
            return False

        center_val = data[cell_index(mx, my, w)]
        if not is_free(center_val):
            return False

        for dy in range(-clear_cells, clear_cells + 1):
            for dx in range(-clear_cells, clear_cells + 1):
                value = data[cell_index(mx + dx, my + dy, w)]
                if value >= FREE_THRESH:
                    return False
        return True

    def _safe_goal_near_frontier(self, grid, wx, wy):
        info = grid.info
        mx, my = world_to_map(wx, wy, info)
        search_cells = max(2, int(round(0.8 / info.resolution)))

        best = None
        best_dist = float('inf')
        for radius in range(0, search_cells + 1):
            for dy in range(-radius, radius + 1):
                for dx in range(-radius, radius + 1):
                    if abs(dx) != radius and abs(dy) != radius:
                        continue
                    cx = mx + dx
                    cy = my + dy
                    if not self._cell_has_clearance(grid, cx, cy):
                        continue
                    gx, gy = map_to_world(cx, cy, info)
                    d = math.hypot(gx - wx, gy - wy)
                    if d < best_dist:
                        best_dist = d
                        best = (gx, gy)
            if best is not None:
                return best
        return None

    def _pick_frontier(self, ns, robot_xy, frontiers, all_poses, strict_voronoi=True):
        best = None
        best_score = float('inf')

        for wx, wy, size in frontiers:
            if self._is_blacklisted(wx, wy):
                continue

            my_dist = math.hypot(wx - robot_xy[0], wy - robot_xy[1])

            if strict_voronoi:
                blocked = False
                for other_ns, other_xy in all_poses.items():
                    if other_ns == ns:
                        continue
                    if math.hypot(wx - other_xy[0], wy - other_xy[1]) + self.voronoi_margin < my_dist:
                        blocked = True
                        break
                if blocked:
                    continue

            gain = self.gain_scale * math.sqrt(float(size))
            score = my_dist - gain
            if score < best_score:
                best_score = score
                best = (wx, wy, size)

        if best is None and strict_voronoi:
            return self._pick_frontier(ns, robot_xy, frontiers, all_poses, strict_voronoi=False)
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

    def _wait_for_goal(self, ns, client, base_frame, goal_xy, timeout):
        deadline = rospy.Time.now() + rospy.Duration(timeout)
        last_progress = rospy.Time.now()
        pose = self._get_robot_pose(base_frame)
        best_dist = (math.hypot(goal_xy[0] - pose[0], goal_xy[1] - pose[1])
                     if pose is not None else float('inf'))
        rate = rospy.Rate(5.0)
        while rospy.Time.now() < deadline and not rospy.is_shutdown() and not self._stop.is_set():
            # Interruption check: check if robot detected a flag or if game state changed
            has_flag, _ = self._get_fresh_flag_estimate(ns)
            if has_flag or self.game_state != 'EXPLORING':
                client.cancel_goal()
                return False

            state = client.get_state()
            if state in (GoalStatus.SUCCEEDED, GoalStatus.ABORTED,
                         GoalStatus.REJECTED, GoalStatus.PREEMPTED):
                return state == GoalStatus.SUCCEEDED

            pose = self._get_robot_pose(base_frame)
            if pose is not None:
                dist = math.hypot(goal_xy[0] - pose[0], goal_xy[1] - pose[1])
                if dist + self.min_progress_distance < best_dist:
                    best_dist = dist
                    last_progress = rospy.Time.now()

            if (rospy.Time.now() - last_progress).to_sec() > self.progress_timeout:
                rospy.logwarn('%s Goal stalled: no progress for %.1f s', ns, self.progress_timeout)
                client.cancel_goal()
                return False

            rate.sleep()
        client.cancel_goal()
        return False

    def _spin_to_scan(self, ns, duration=3.0):
        pub = self._cmd_pubs[ns]
        twist = Twist()
        twist.angular.z = 0.8
        end = rospy.Time.now() + rospy.Duration(duration)
        rate = rospy.Rate(10)
        while rospy.Time.now() < end and not rospy.is_shutdown() and not self._stop.is_set():
            if self.game_state != 'EXPLORING':
                break
            pub.publish(twist)
            rate.sleep()
        pub.publish(Twist())

    def _should_send_flag_goal(self, ns, fx, fy):
        now = rospy.Time.now().to_sec()
        last_goal = self._last_flag_goal.get(ns)
        last_time = self._last_flag_goal_time.get(ns, 0.0)

        moved = last_goal is None or math.hypot(fx - last_goal[0], fy - last_goal[1]) > 0.3
        stale = (now - last_time) > 1.0

        if moved or stale:
            self._last_flag_goal[ns] = (fx, fy)
            self._last_flag_goal_time[ns] = now
            return True
        return False

    def _get_approach_goal(self, robot_pose, flag_xy):
        rx, ry, _ = robot_pose
        fx, fy = flag_xy
        dx = fx - rx
        dy = fy - ry
        dist = math.hypot(dx, dy)
        yaw = math.atan2(dy, dx)

        if dist > self.flag_standoff_distance and dist > 1e-3:
            ux = dx / dist
            uy = dy / dist
            gx = fx - ux * self.flag_standoff_distance
            gy = fy - uy * self.flag_standoff_distance
        else:
            gx = rx
            gy = ry
        return gx, gy, yaw

    def _capture_flag(self, ns, flag_xy):
        rospy.loginfo(f'==================================================')
        rospy.loginfo(f'!!! FLAG CAPTURED BY {ns} at ({flag_xy[0]:.2f}, {flag_xy[1]:.2f}) !!!')
        rospy.loginfo(f'==================================================')

        # Cancel goals on both robots
        for client in self._clients.values():
            client.cancel_all_goals()

        # Stop exploration thread logic
        self.game_state = 'CAPTURED'
        self.carrier_ns = ns
        self.pursuer_ns = 'robot2' if ns == 'robot1' else 'robot1'
        self._flag_captured_pub.publish(Bool(data=True))

        # Pause for capture celebration
        rospy.sleep(self.capture_pause_sec)

        # Start Chase phase
        self.game_state = 'CHASE'

    def _robot_loop(self, cfg):
        ns = cfg['ns']
        client = self._clients[ns]
        rospy.loginfo('%s loop started', ns)
        consecutive_fails = 0

        while not self._stop.is_set() and not rospy.is_shutdown():
            if self.game_state == 'FINISHED':
                break

            if self.game_state == 'CHASE' or self.game_state == 'CAPTURED':
                # The main game loop handles Chase phase command logic
                rospy.sleep(0.5)
                continue

            # Check if this robot has a fresh flag estimate
            has_flag, flag_xy = self._get_fresh_flag_estimate(ns)
            if has_flag:
                pose = self._get_robot_pose(cfg['base_frame'])
                if pose is not None:
                    # Check if close enough to capture
                    d_flag = math.hypot(pose[0] - flag_xy[0], pose[1] - flag_xy[1])
                    if d_flag <= self.flag_capture_distance:
                        self._capture_flag(ns, flag_xy)
                        continue

                    # Drive to flag approach stance
                    if self._should_send_flag_goal(ns, flag_xy[0], flag_xy[1]):
                        gx, gy, gyaw = self._get_approach_goal(pose, flag_xy)
                        rospy.loginfo(f'[{ns}] Pursuing flag candidate at ({flag_xy[0]:.2f}, {flag_xy[1]:.2f}), distance: {d_flag:.2f} m')
                        client.send_goal(self._make_goal(gx, gy, gyaw))
                
                rospy.sleep(0.2)
                continue

            # Standard exploration logic
            grid = self._get_map()
            if grid is None:
                rospy.sleep(1.0)
                continue

            pose = self._get_robot_pose(cfg['base_frame'])
            if pose is None:
                rospy.sleep(1.0)
                continue

            frontiers = self._find_frontiers(grid)
            if not frontiers:
                rospy.loginfo_throttle(15.0, '%s: no frontiers visible', ns)
                self._spin_to_scan(ns, 4.0)
                continue

            all_poses = self._all_robot_poses()
            pick = self._pick_frontier(ns, pose, frontiers, all_poses)
            if pick is None:
                rospy.logwarn_throttle(10.0, '%s: no assignable frontier, spinning', ns)
                self._spin_to_scan(ns, 3.0)
                continue

            wx, wy, size = pick
            safe_goal = self._safe_goal_near_frontier(grid, wx, wy)
            if safe_goal is None:
                self._blacklist_goal(wx, wy, duration=30.0)
                rospy.sleep(0.2)
                continue

            wx, wy = safe_goal
            yaw = math.atan2(wy - pose[1], wx - pose[0])
            rospy.loginfo('%s -> frontier (%.2f, %.2f) size=%d', ns, wx, wy, size)
            client.send_goal(self._make_goal(wx, wy, yaw))
            ok = self._wait_for_goal(ns, client, cfg['base_frame'], (wx, wy), self.goal_timeout)

            if ok:
                consecutive_fails = 0
            else:
                consecutive_fails += 1
                self._blacklist_goal(wx, wy, duration=40.0)
                if consecutive_fails >= 3:
                    rospy.logwarn('%s: %d consecutive fails, spinning to recover',
                                 ns, consecutive_fails)
                    self._spin_to_scan(ns, 5.0)
                    consecutive_fails = 0

            rospy.sleep(0.2)

        client.cancel_all_goals()
        rospy.loginfo('%s loop stopped', ns)

    def _publish_markers(self):
        arr = MarkerArray()
        
        # 1. CTF Flag marker
        # Find the latest flag estimate to show sphere
        flag_est = None
        for ns in ('robot1', 'robot2'):
            has_flag, xy = self._get_fresh_flag_estimate(ns)
            if has_flag:
                flag_est = xy
                break

        if flag_est:
            m = Marker()
            m.header.frame_id = self.map_frame
            m.header.stamp = rospy.Time.now()
            m.ns = "ctf_flag"
            m.id = 0
            m.type = Marker.CYLINDER
            m.action = Marker.ADD
            m.pose.position.x = flag_est[0]
            m.pose.position.y = flag_est[1]
            m.pose.position.z = 0.4
            m.pose.orientation.w = 1.0
            m.scale.x = 0.25
            m.scale.y = 0.25
            m.scale.z = 0.8
            m.color.r = 1.0
            m.color.g = 0.1
            m.color.b = 0.1
            m.color.a = 0.9
            arr.markers.append(m)

        # 2. Robot Home bases
        for idx, (ns, coords) in enumerate(self.homes.items()):
            m = Marker()
            m.header.frame_id = self.map_frame
            m.header.stamp = rospy.Time.now()
            m.ns = "ctf_home"
            m.id = 1 + idx
            m.type = Marker.CYLINDER
            m.action = Marker.ADD
            m.pose.position.x = coords[0]
            m.pose.position.y = coords[1]
            m.pose.position.z = 0.015
            m.pose.orientation.w = 1.0
            m.scale.x = 0.9
            m.scale.y = 0.9
            m.scale.z = 0.03
            if ns == 'robot1':
                m.color.r = 0.1
                m.color.g = 0.5
                m.color.b = 1.0
            else:
                m.color.r = 1.0
                m.color.g = 0.6
                m.color.b = 0.1
            m.color.a = 0.8
            arr.markers.append(m)

        self._marker_pub.publish(arr)

    def run(self):
        rospy.loginfo('Exploration-based Capture the Flag simulation active')

        threads = []
        for cfg in self.robots:
            t = threading.Thread(target=self._robot_loop, args=(cfg,))
            t.daemon = True
            t.start()
            threads.append(t)

        rate = rospy.Rate(5.0)
        last_chase_update = 0.0

        while not rospy.is_shutdown() and self.game_state != 'FINISHED':
            # Publish markers and update telemetry
            self._publish_markers()

            grid = self._get_map()
            cov = self._coverage(grid)
            unknown = self._unknown_count(grid)
            
            if self.game_state == 'EXPLORING':
                rospy.loginfo_throttle(
                    10.0, f'Exploration: map coverage={cov*100.0:.1f}%, unknown cells={unknown}, game_state={self.game_state}'
                )

            elif self.game_state == 'CHASE':
                cpose = self._get_robot_pose(self.carrier_ns + '/base_footprint')
                ppose = self._get_robot_pose(self.pursuer_ns + '/base_footprint')

                if cpose and ppose:
                    # 1. Check if pursuer caught the carrier
                    dist = math.hypot(cpose[0] - ppose[0], cpose[1] - ppose[1])
                    rospy.loginfo_throttle(1.0, f'CHASE: Distance between carrier({self.carrier_ns}) and pursuer({self.pursuer_ns}) = {dist:.2f} m')
                    if dist <= self.catch_distance:
                        rospy.loginfo(f'==================================================')
                        rospy.loginfo(f'!!! GAME OVER: PURSUER {self.pursuer_ns} CAUGHT THE CARRIER {self.carrier_ns} (dist={dist:.2f}m) !!!')
                        rospy.loginfo(f'==================================================')
                        self.game_state = 'FINISHED'
                        break

                    # 2. Check if carrier reached home
                    home = self.homes[self.carrier_ns]
                    dist_home = math.hypot(cpose[0] - home[0], cpose[1] - home[1])
                    if dist_home <= 0.55 or self._clients[self.carrier_ns].get_state() == GoalStatus.SUCCEEDED:
                        rospy.loginfo(f'==================================================')
                        rospy.loginfo(f'!!! GAME OVER: CARRIER {self.carrier_ns} REACHED HOME BASE SAFELY !!!')
                        rospy.loginfo(f'==================================================')
                        self.game_state = 'FINISHED'
                        break

                    # Send home goal to carrier if not already running
                    carrier_state = self._clients[self.carrier_ns].get_state()
                    if carrier_state not in (GoalStatus.ACTIVE, GoalStatus.PENDING):
                        rospy.loginfo(f'CHASE: Sending carrier {self.carrier_ns} home to {home}')
                        self._clients[self.carrier_ns].send_goal(self._make_goal(home[0], home[1], home[2]))

                    # 3. Periodically update pursuer's goal to follow the carrier
                    if time.monotonic() - last_chase_update >= self.chase_goal_period:
                        self._clients[self.pursuer_ns].send_goal(self._make_goal(cpose[0], cpose[1], cpose[2]))
                        last_chase_update = time.monotonic()

                else:
                    rospy.logwarn_throttle(3.0, 'CHASE: waiting for TF frames of robots...')

            rate.sleep()

        # Stop both robots when finishing
        self._stop.set()
        for client in self._clients.values():
            client.cancel_all_goals()
        self._done_pub.publish(Bool(data=True))

        for t in threads:
            t.join(timeout=3.0)


def main():
    try:
        SlamFrontierExplorerCtf().run()
    except rospy.ROSInterruptException:
        pass


if __name__ == '__main__':
    main()
