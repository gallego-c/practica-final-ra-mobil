#!/usr/bin/env python3
"""
Multi-robot frontier exploration and Capture the Flag (CTF) game loop on the merged SLAM map.
Combines:
1. Voronoi-partitioned frontier exploration (SLAM map).
2. Vision-based flag detection, pursuit, and capture.
3. Return/chase phase (carrier heads home, pursuer chases carrier).
"""
from __future__ import division

import json
import math
import os
import threading
import time

import actionlib
import rospkg
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

# #region agent log
_DEBUG_LOG_PATH = os.path.join(
    rospkg.RosPack().get_path('ctf_navigation'), 'debug-36a89d.log')
def _agent_debug_log(location, message, hypothesis_id, data):
    try:
        payload = {
            'sessionId': '36a89d',
            'location': location,
            'message': message,
            'hypothesisId': hypothesis_id,
            'data': data,
            'timestamp': int(time.time() * 1000),
        }
        with open(_DEBUG_LOG_PATH, 'a') as log_file:
            log_file.write(json.dumps(payload) + '\n')
    except Exception:
        pass
# #endregion


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
        self.separation_weight = float(rospy.get_param('~separation_weight', 2.0))
        self.flag_bias_weight = float(rospy.get_param('~flag_bias_weight', 1.0))
        self.territory_penalty = float(rospy.get_param('~territory_penalty', 4.0))
        self.frontier_claim_sec = float(rospy.get_param('~frontier_claim_sec', 50.0))
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
        self.waypoint_reached_dist = float(rospy.get_param('~waypoint_reached_dist', 0.55))
        self.chase_flag_clearance = float(rospy.get_param('~chase_flag_clearance', 0.75))
        self.chase_clearance_reached_dist = float(
            rospy.get_param('~chase_clearance_reached_dist', 0.20))
        self.chase_clearance_min_travel = float(
            rospy.get_param('~chase_clearance_min_travel', 0.25))
        self.intercept_enabled = bool(rospy.get_param('~intercept_enabled', True))
        self.intercept_direct_chase_distance = float(
            rospy.get_param('~intercept_direct_chase_distance', 1.25))
        self.intercept_arrival_margin = float(
            rospy.get_param('~intercept_arrival_margin', 0.30))
        self.intercept_step = float(rospy.get_param('~intercept_step', 0.25))
        self.intercept_min_lead = float(rospy.get_param('~intercept_min_lead', 1.0))
        self.intercept_carrier_move_threshold = float(
            rospy.get_param('~intercept_carrier_move_threshold', 1.25))
        self.carrier_home_resend_cooldown = float(
            rospy.get_param('~carrier_home_resend_cooldown', 3.0))
        self.carrier_home_stuck_timeout = float(
            rospy.get_param('~carrier_home_stuck_timeout', 6.0))
        self.carrier_home_progress_threshold = float(
            rospy.get_param('~carrier_home_progress_threshold', 0.10))
        self.direct_chase_refresh_sec = float(
            rospy.get_param('~direct_chase_refresh_sec', 2.0))
        self.direct_chase_carrier_move = float(
            rospy.get_param('~direct_chase_carrier_move', 0.25))

        # Phase 2 direct-start support
        self.start_phase = rospy.get_param('~start_phase', 'EXPLORING')
        self.initial_carrier_ns = rospy.get_param('~initial_carrier_ns', 'robot1')
        self.chase_startup_delay = float(rospy.get_param('~chase_startup_delay', 8.0))

        # Velocity-aware intercept parameters
        self.pursuer_nav_speed = float(rospy.get_param('~pursuer_nav_speed', 0.20))
        self.carrier_speed_window = float(rospy.get_param('~carrier_speed_window', 4.0))
        self.carrier_min_speed = float(rospy.get_param('~carrier_min_speed', 0.05))
        self.intercept_margin_time = float(rospy.get_param('~intercept_margin_time', 1.5))
        self.intercept_look_back = float(rospy.get_param('~intercept_look_back', 0.5))
        self.goal_snap_clearance = float(rospy.get_param('~goal_snap_clearance', 0.30))
        self.goal_snap_search_radius = float(
            rospy.get_param('~goal_snap_search_radius', 2.0))
        self.flag_x = float(rospy.get_param('~flag_x', -3.2))
        self.flag_y = float(rospy.get_param('~flag_y', 3.2))

        # Game states: 'EXPLORING', 'CAPTURED', 'CHASE', 'FINISHED'
        self.game_state = 'EXPLORING'
        self.carrier_ns = None
        self.pursuer_ns = None
        self._capture_lock = threading.Lock()
        self._chase_initialized = False
        self._carrier_on_home_goal = False
        self._clearance_goal = None
        self._has_intercept_goal = False
        self._intercept_anchor = None
        self._pursuer_was_succeeded = False
        self._last_carrier_home_resend = 0.0
        self._last_direct_chase_sent = 0.0
        self._last_carrier_progress_time = 0.0
        self._last_carrier_progress_home_dist = -1.0

        # Carrier velocity tracking for time-based intercept
        self._carrier_vel_history = []  # list of (x, y, monotonic_time) tuples

        self.robots = rospy.get_param('~robots', [
            {'ns': 'robot1', 'base_frame': 'robot1/base_footprint'},
            {'ns': 'robot2', 'base_frame': 'robot2/base_footprint'},
        ])

        # Flag estimate and detection status
        self.flag_found = {'robot1': False, 'robot2': False}
        self.flag_estimate = {'robot1': None, 'robot2': None}
        self.flag_estimate_time = {'robot1': 0.0, 'robot2': 0.0}
        self.last_known_flag_xy = None
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
        self._frontier_claims = {}  # (wx, wy) -> (owner_ns, expiry monotonic time)
        self._frontier_claim_lock = threading.Lock()
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

        delay = (self.chase_startup_delay
                 if self.start_phase == 'CHASE'
                 else self.startup_delay)
        rospy.loginfo('Waiting %.0f s for navigation stack...', delay)
        rospy.sleep(delay)
        if not self._wait_for_all_move_base():
            raise rospy.ROSException('move_base not ready for all robots')

        if self.start_phase == 'CHASE':
            rospy.loginfo(
                'start_phase=CHASE: skipping exploration; %s is the carrier',
                self.initial_carrier_ns)
            self.carrier_ns = self.initial_carrier_ns
            self.pursuer_ns = (
                'robot2' if self.initial_carrier_ns == 'robot1' else 'robot1')
            self.game_state = 'CAPTURED'  # triggers _init_chase() on first run() tick
            self._chase_initialized = False
            self._flag_captured_pub.publish(Bool(data=True))
        else:
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
            self.last_known_flag_xy = (msg.pose.position.x, msg.pose.position.y)

    def _get_own_fresh_flag_estimate(self, ns):
        with self.flag_lock:
            if self.flag_estimate[ns] is None:
                return False, None
            age = rospy.Time.now().to_sec() - self.flag_estimate_time[ns]
            if age <= self.flag_memory_timeout:
                return True, self.flag_estimate[ns]
            return False, None

    def _get_fresh_flag_estimate(self, ns):
        """Any fresh estimate (own or teammate) — for markers / awareness only."""
        has_own, own_est = self._get_own_fresh_flag_estimate(ns)
        if has_own:
            return True, own_est

        with self.flag_lock:
            other_ns = 'robot2' if ns == 'robot1' else 'robot1'
            if self.flag_estimate[other_ns] is not None:
                age_other = rospy.Time.now().to_sec() - self.flag_estimate_time[other_ns]
                if age_other <= self.flag_memory_timeout:
                    return True, self.flag_estimate[other_ns]
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

    def _claim_frontier(self, ns, wx, wy):
        key = self._blacklist_key(wx, wy)
        with self._frontier_claim_lock:
            self._frontier_claims[key] = (ns, time.monotonic() + self.frontier_claim_sec)

    def _is_frontier_claimed_by_other(self, ns, wx, wy):
        key = self._blacklist_key(wx, wy)
        now = time.monotonic()
        with self._frontier_claim_lock:
            claim = self._frontier_claims.get(key)
            if claim is None:
                return False
            owner, expiry = claim
            if now >= expiry:
                del self._frontier_claims[key]
                return False
            return owner != ns

    def _recover_navigation(self, ns, spin_sec=2.5):
        self._clients[ns].cancel_all_goals()
        self._spin_to_scan(ns, spin_sec)
        self._last_flag_goal.pop(ns, None)
        self._last_flag_goal_time.pop(ns, None)

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

    def _cell_has_clearance(self, grid, mx, my, clearance_m=None):
        info = grid.info
        w, h = info.width, info.height
        data = grid.data
        if clearance_m is None:
            clearance_m = self.min_goal_clearance
        clear_cells = max(1, int(round(clearance_m / info.resolution)))

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
        search_cells = max(2, int(round(1.2 / info.resolution)))

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

    def _territory_penalty_for(self, ns, wx, wy):
        """Penalize frontiers on the teammate's side of the map (diagonal x+y=0)."""
        bisector = wx + wy
        if ns == 'robot1' and bisector > 0.0:
            return self.territory_penalty * bisector
        if ns == 'robot2' and bisector < 0.0:
            return self.territory_penalty * abs(bisector)
        return 0.0

    def _pick_frontier_by_separation(self, ns, robot_xy, frontiers, all_poses):
        best = None
        best_sep = -float('inf')
        for wx, wy, size in frontiers:
            if self._is_blacklisted(wx, wy) or self._is_frontier_claimed_by_other(ns, wx, wy):
                continue
            my_dist = math.hypot(wx - robot_xy[0], wy - robot_xy[1])
            other_dists = [
                math.hypot(wx - other_xy[0], wy - other_xy[1])
                for other_ns, other_xy in all_poses.items()
                if other_ns != ns
            ]
            if not other_dists:
                continue
            separation = my_dist - min(other_dists)
            if ns == 'robot2' and abs(separation) < 0.15:
                separation -= 0.25
            separation -= self._territory_penalty_for(ns, wx, wy) * 0.1
            if separation > best_sep:
                best_sep = separation
                best = (wx, wy, size)
        return best, best_sep

    def _pick_frontier(self, ns, robot_xy, frontiers, all_poses, strict_voronoi=True):
        best = None
        best_score = float('inf')
        used_fallback = False

        has_own_flag, own_flag = self._get_own_fresh_flag_estimate(ns)

        for wx, wy, size in frontiers:
            if self._is_blacklisted(wx, wy):
                continue
            if self._is_frontier_claimed_by_other(ns, wx, wy):
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
            score += self._territory_penalty_for(ns, wx, wy)

            for other_ns, other_xy in all_poses.items():
                if other_ns == ns:
                    continue
                other_dist = math.hypot(wx - other_xy[0], wy - other_xy[1])
                if other_dist < my_dist:
                    score += self.separation_weight * (my_dist - other_dist)

            if has_own_flag and own_flag is not None:
                dist_to_flag = math.hypot(wx - own_flag[0], wy - own_flag[1])
                score += self.flag_bias_weight * dist_to_flag

            if score < best_score:
                best_score = score
                best = (wx, wy, size)

        if best is None and strict_voronoi:
            used_fallback = True
            best, best_sep = self._pick_frontier_by_separation(ns, robot_xy, frontiers, all_poses)
        else:
            best_sep = None

        if best is not None:
            other_pose = None
            for other_ns, other_xy in all_poses.items():
                if other_ns != ns:
                    other_pose = other_xy
                    break
            inter_robot = None
            if other_pose is not None:
                inter_robot = math.hypot(robot_xy[0] - other_pose[0], robot_xy[1] - other_pose[1])
            # #region agent log
            _agent_debug_log(
                'slam_frontier_explorer_ctf.py:_pick_frontier',
                'frontier selected',
                'H',
                {
                    'robot': ns,
                    'goalX': best[0],
                    'goalY': best[1],
                    'robotX': robot_xy[0],
                    'robotY': robot_xy[1],
                    'strictVoronoi': strict_voronoi,
                    'usedFallback': used_fallback,
                    'separation': best_sep,
                    'interRobotDist': inter_robot,
                    'bisectorSum': best[0] + best[1],
                })
            # #endregion

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
            # Only interrupt exploration for this robot's own flag sighting
            has_flag, _ = self._get_own_fresh_flag_estimate(ns)
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

            # Reset progress timeout if robots are close to avoid canceling goals due to yielding/avoidance
            other_ns = 'robot2' if ns == 'robot1' else 'robot1'
            other_pose = self._get_robot_pose(other_ns + '/base_footprint')
            if pose is not None and other_pose is not None:
                d_robots = math.hypot(pose[0] - other_pose[0], pose[1] - other_pose[1])
                if d_robots < 2.2:
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

    def _goal_terminal(self, ns):
        state = self._clients[ns].get_state()
        return state in (GoalStatus.SUCCEEDED, GoalStatus.ABORTED,
                         GoalStatus.REJECTED, GoalStatus.PREEMPTED)

    def _goal_succeeded(self, ns):
        return self._clients[ns].get_state() == GoalStatus.SUCCEEDED

    def _snap_chase_goal(self, wx, wy):
        """Return nearest collision-free goal to (wx, wy) on the current map."""
        grid = self._get_map()
        if grid is None:
            return wx, wy

        info = grid.info
        mx, my = world_to_map(wx, wy, info)
        search_cells = max(1, int(round(self.goal_snap_search_radius / info.resolution)))

        clearance = self.goal_snap_clearance
        if self._cell_has_clearance(grid, mx, my, clearance):
            return map_to_world(mx, my, info)

        best = None
        best_dist = float('inf')
        for radius in range(1, search_cells + 1):
            for dy in range(-radius, radius + 1):
                for dx in range(-radius, radius + 1):
                    if abs(dx) != radius and abs(dy) != radius:
                        continue
                    cx = mx + dx
                    cy = my + dy
                    if not self._cell_has_clearance(grid, cx, cy, clearance):
                        continue
                    gx, gy = map_to_world(cx, cy, info)
                    d = math.hypot(gx - wx, gy - wy)
                    if d < best_dist:
                        best_dist = d
                        best = (gx, gy)
            if best is not None:
                if best_dist > 0.05:
                    rospy.loginfo_throttle(
                        2.0, 'CTF CHASE: snapped goal (%.2f, %.2f) -> (%.2f, %.2f)',
                        wx, wy, best[0], best[1])
                return best

        return wx, wy

    def _update_carrier_vel(self, x, y):
        t = time.monotonic()
        self._carrier_vel_history.append((x, y, t))
        cutoff = t - self.carrier_speed_window
        self._carrier_vel_history = [
            s for s in self._carrier_vel_history if s[2] >= cutoff
        ]

    def _estimate_carrier_speed(self):
        if len(self._carrier_vel_history) < 2:
            return 0.0
        total_dist = 0.0
        for i in range(1, len(self._carrier_vel_history)):
            x1, y1, _ = self._carrier_vel_history[i - 1]
            x2, y2, _ = self._carrier_vel_history[i]
            total_dist += math.hypot(x2 - x1, y2 - y1)
        dt = self._carrier_vel_history[-1][2] - self._carrier_vel_history[0][2]
        return total_dist / dt if dt > 1e-3 else 0.0

    def _resolve_flag_position(self, carrier_ns):
        has_flag, flag_xy = self._get_fresh_flag_estimate(carrier_ns)
        if has_flag and flag_xy is not None:
            return flag_xy
        return (self.flag_x, self.flag_y)

    def _make_chase_clearance_goal(self, carrier_pose, home, flag_xy=None):
        cx, cy, _ = carrier_pose
        hx, hy, hyaw = home
        home_dx = hx - cx
        home_dy = hy - cy
        home_dist = math.hypot(home_dx, home_dy)
        if home_dist < 1e-3:
            return hx, hy, hyaw

        step = min(self.chase_flag_clearance, max(0.0, home_dist - 0.10))
        if step < self.chase_clearance_min_travel:
            return hx, hy, hyaw

        if flag_xy is not None:
            fx, fy = flag_xy
            away_dx = cx - fx
            away_dy = cy - fy
            flag_dist = math.hypot(away_dx, away_dy)
            if flag_dist > 1e-3 and flag_dist <= self.flag_capture_distance * 2.5:
                return (cx + (away_dx / flag_dist) * step,
                        cy + (away_dy / flag_dist) * step,
                        math.atan2(away_dy, away_dx))

        return (cx + (home_dx / home_dist) * step,
                cy + (home_dy / home_dist) * step,
                math.atan2(home_dy, home_dx))

    # carrier_speed: estimated carrier speed (m/s); 0 triggers legacy fallback.
    def _make_interception_goal(self, carrier_pose, pursuer_pose, home, carrier_speed=0.0):
        cx, cy, _ = carrier_pose
        px, py, _ = pursuer_pose
        hx, hy, _ = home
        home_dx = hx - cx
        home_dy = hy - cy
        home_dist = math.hypot(home_dx, home_dy)
        if not self.intercept_enabled or home_dist < 1e-3:
            return cx, cy, math.atan2(cy - py, cx - px)

        robot_distance = math.hypot(cx - px, cy - py)
        if robot_distance <= self.intercept_direct_chase_distance:
            return cx, cy, math.atan2(cy - py, cx - px)

        ux = home_dx / home_dist
        uy = home_dy / home_dist
        step = max(0.10, self.intercept_step)

        # Use time-based model when carrier speed is reliably estimated.
        use_time_model = (carrier_speed >= self.carrier_min_speed)
        v_c = carrier_speed if use_time_model else 1.0
        v_p = self.pursuer_nav_speed if use_time_model else 1.0

        # Start from the pursuer's projection onto the carrier's path to home so
        # that we find the closest intercept point first, not the earliest.
        proj_s = (px - cx) * ux + (py - cy) * uy
        s_start = max(0.0, proj_s - self.intercept_look_back)

        chosen_s = -1.0
        best_deficit = float('inf')
        best_s = max(step, self.intercept_min_lead)

        s = s_start
        while s <= home_dist:
            ix = cx + ux * s
            iy = cy + uy * s
            pursuer_dist = math.hypot(px - ix, py - iy)

            t_carrier = s / v_c
            t_pursuer = pursuer_dist / v_p

            # Track least-bad point (minimum time deficit) for the fallback.
            deficit = t_pursuer - t_carrier
            if deficit < best_deficit:
                best_deficit = deficit
                best_s = s

            if use_time_model:
                if t_pursuer + self.intercept_margin_time <= t_carrier:
                    chosen_s = s
                    break
            else:
                # Legacy distance-ratio mode (original behaviour, equal-speed assumption).
                if pursuer_dist + self.intercept_arrival_margin <= s:
                    chosen_s = s
                    break
            s += step

        if chosen_s < 0.0:
            # Fallback: go to the point where we are closest to catching up.
            chosen_s = min(home_dist, best_s)

        gx = cx + ux * chosen_s
        gy = cy + uy * chosen_s
        return gx, gy, math.atan2(gy - py, gx - px)

    def _send_pursuer_intercept(self, carrier_pose, pursuer_pose):
        home = self.homes[self.carrier_ns]
        gx, gy, gyaw = self._make_interception_goal(
            carrier_pose, pursuer_pose, home, self._estimate_carrier_speed())
        gx, gy = self._snap_chase_goal(gx, gy)
        px, py, _ = pursuer_pose
        gyaw = math.atan2(gy - py, gx - px)
        self._clients[self.pursuer_ns].send_goal(self._make_goal(gx, gy, gyaw))
        self._has_intercept_goal = True
        self._intercept_anchor = (carrier_pose[0], carrier_pose[1])
        self._pursuer_was_succeeded = False

    def _send_pursuer_direct_chase(self, carrier_pose, pursuer_pose):
        cx, cy, _ = carrier_pose
        px, py, _ = pursuer_pose
        gx, gy = self._snap_chase_goal(cx, cy)
        gyaw = math.atan2(gy - py, gx - px)
        self._clients[self.pursuer_ns].send_goal(self._make_goal(gx, gy, gyaw))
        self._has_intercept_goal = True
        self._intercept_anchor = (cx, cy)
        self._pursuer_was_succeeded = False
        self._last_direct_chase_sent = time.monotonic()

    def _send_carrier_goal(self, wx, wy, yaw):
        gx, gy = self._snap_chase_goal(wx, wy)
        self._clients[self.carrier_ns].send_goal(self._make_goal(gx, gy, yaw))

    def _resend_carrier_goal(self):
        home = self.homes[self.carrier_ns]
        if self._carrier_on_home_goal:
            self._send_carrier_goal(home[0], home[1], home[2])
        elif self._clearance_goal is not None:
            gx, gy = self._clearance_goal
            gyaw = math.atan2(home[1] - gy, home[0] - gx)
            self._send_carrier_goal(gx, gy, gyaw)

    def _init_chase(self):
        for client in self._clients.values():
            client.cancel_all_goals()
        rospy.sleep(0.5)

        cpose = self._get_robot_pose(self.carrier_ns + '/base_footprint')
        ppose = self._get_robot_pose(self.pursuer_ns + '/base_footprint')
        if not cpose or not ppose:
            rospy.logwarn('CHASE: missing TF pose at start')
            self._chase_initialized = True
            return

        home = self.homes[self.carrier_ns]
        flag_xy = self._resolve_flag_position(self.carrier_ns)
        gx, gy, gyaw = self._make_chase_clearance_goal(cpose, home, flag_xy)
        clearance_dist = math.hypot(gx - cpose[0], gy - cpose[1])
        self._clearance_goal = (gx, gy)
        self._carrier_on_home_goal = clearance_dist < self.chase_clearance_min_travel

        if self._carrier_on_home_goal:
            self._send_carrier_goal(home[0], home[1], home[2])
            target = 'base'
        else:
            self._send_carrier_goal(gx, gy, gyaw)
            target = 'clearance'

        has_flag, flag_xy = self._get_fresh_flag_estimate(self.carrier_ns)
        flag_dist = (math.hypot(cpose[0] - flag_xy[0], cpose[1] - flag_xy[1])
                     if has_flag and flag_xy else -1.0)
        rospy.loginfo(
            'CTF CHASE START: %s -> %s (%.2f, %.2f); flag dist=%.2f m travel=%.2f m',
            self.carrier_ns, target,
            home[0] if self._carrier_on_home_goal else gx,
            home[1] if self._carrier_on_home_goal else gy,
            flag_dist, clearance_dist)

        self._last_carrier_progress_home_dist = math.hypot(
            cpose[0] - home[0], cpose[1] - home[1])
        self._last_carrier_progress_time = time.monotonic()
        self._send_pursuer_intercept(cpose, ppose)
        self._chase_initialized = True

    def _tick_chase(self):
        cpose = self._get_robot_pose(self.carrier_ns + '/base_footprint')
        ppose = self._get_robot_pose(self.pursuer_ns + '/base_footprint')
        if not cpose or not ppose:
            rospy.logwarn_throttle(3.0, 'CHASE: waiting for TF frames of robots...')
            return

        self._update_carrier_vel(cpose[0], cpose[1])

        home = self.homes[self.carrier_ns]
        d = math.hypot(cpose[0] - ppose[0], cpose[1] - ppose[1])
        carrier_home_dist = math.hypot(cpose[0] - home[0], cpose[1] - home[1])
        rospy.loginfo_throttle(
            1.0, 'CHASE: %s-%s dist=%.2f m; %s home dist=%.2f m',
            self.carrier_ns, self.pursuer_ns, d, self.carrier_ns, carrier_home_dist)

        if d <= self.catch_distance:
            rospy.loginfo('!!! GAME OVER: PURSUER %s CAUGHT CARRIER %s (dist=%.2fm) !!!',
                          self.pursuer_ns, self.carrier_ns, d)
            self.game_state = 'FINISHED'
            return

        if carrier_home_dist <= self.waypoint_reached_dist:
            rospy.loginfo('!!! GAME OVER: CARRIER %s REACHED HOME BASE SAFELY !!!',
                          self.carrier_ns)
            self.game_state = 'FINISHED'
            return

        if not self._carrier_on_home_goal and self._clearance_goal is not None:
            gx, gy = self._clearance_goal
            clearance_dist = math.hypot(cpose[0] - gx, cpose[1] - gy)
            if (clearance_dist <= self.chase_clearance_reached_dist or
                    self._goal_succeeded(self.carrier_ns)):
                self._send_carrier_goal(home[0], home[1], home[2])
                self._carrier_on_home_goal = True
                self._last_carrier_progress_time = time.monotonic()
                self._last_carrier_progress_home_dist = carrier_home_dist
                rospy.loginfo('CTF CHASE: %s reached clearance; continuing to base',
                              self.carrier_ns)

        now = time.monotonic()
        if self._goal_terminal(self.carrier_ns) and not self._goal_succeeded(self.carrier_ns):
            if now - self._last_carrier_home_resend >= self.carrier_home_resend_cooldown:
                self._resend_carrier_goal()
                self._last_carrier_home_resend = now
                self._last_carrier_progress_time = now
                self._last_carrier_progress_home_dist = carrier_home_dist
                rospy.logwarn('CTF CHASE: %s goal re-sent after navigation failure',
                              self.carrier_ns)
        elif not self._goal_terminal(self.carrier_ns):
            if (self._last_carrier_progress_home_dist < 0.0 or
                    self._last_carrier_progress_home_dist - carrier_home_dist >=
                    self.carrier_home_progress_threshold):
                self._last_carrier_progress_home_dist = carrier_home_dist
                self._last_carrier_progress_time = now

            stuck_for = now - self._last_carrier_progress_time
            if (stuck_for >= self.carrier_home_stuck_timeout and
                    now - self._last_carrier_home_resend >= self.carrier_home_resend_cooldown):
                if not self._carrier_on_home_goal:
                    self._send_carrier_goal(home[0], home[1], home[2])
                    self._carrier_on_home_goal = True
                    rospy.logwarn(
                        'CTF CHASE: %s stuck on clearance for %.1f s; escalating to home goal',
                        self.carrier_ns, stuck_for)
                else:
                    self._resend_carrier_goal()
                    rospy.logwarn(
                        'CTF CHASE: %s stuck ACTIVE for %.1f s at home dist %.2f m; re-sending goal',
                        self.carrier_ns, stuck_for, carrier_home_dist)
                self._last_carrier_home_resend = now
                self._last_carrier_progress_time = now
                self._last_carrier_progress_home_dist = carrier_home_dist

        direct_chase = d <= self.intercept_direct_chase_distance
        if direct_chase:
            carrier_move = 0.0
            if self._intercept_anchor is not None:
                carrier_move = math.hypot(
                    cpose[0] - self._intercept_anchor[0],
                    cpose[1] - self._intercept_anchor[1])
            pursuer_failed = (self._goal_terminal(self.pursuer_ns) and
                              not self._goal_succeeded(self.pursuer_ns))
            refresh_direct = (
                not self._has_intercept_goal or
                carrier_move >= self.direct_chase_carrier_move or
                pursuer_failed or
                (self._goal_succeeded(self.pursuer_ns) and not self._pursuer_was_succeeded) or
                (self._goal_succeeded(self.pursuer_ns) and
                 now - self._last_direct_chase_sent >= self.direct_chase_refresh_sec))
            if refresh_direct:
                self._send_pursuer_direct_chase(cpose, ppose)
            self._pursuer_was_succeeded = self._goal_succeeded(self.pursuer_ns)
        elif self._goal_succeeded(self.pursuer_ns):
            if not self._pursuer_was_succeeded:
                self._send_pursuer_intercept(cpose, ppose)
            self._pursuer_was_succeeded = True
        else:
            self._pursuer_was_succeeded = False
            if self._goal_terminal(self.pursuer_ns):
                self._send_pursuer_intercept(cpose, ppose)
            else:
                carrier_move = 0.0
                if self._intercept_anchor is not None:
                    carrier_move = math.hypot(
                        cpose[0] - self._intercept_anchor[0],
                        cpose[1] - self._intercept_anchor[1])
                if (not self._has_intercept_goal or
                        carrier_move >= self.intercept_carrier_move_threshold):
                    self._send_pursuer_intercept(cpose, ppose)

    def _try_capture(self, ns, pose, flag_xy):
        with self._capture_lock:
            if self.game_state != 'EXPLORING':
                return False

            d_flag = math.hypot(pose[0] - flag_xy[0], pose[1] - flag_xy[1])
            if d_flag > self.flag_capture_distance:
                return False

            if ns == 'robot2':
                r1_pose = self._get_robot_pose('robot1/base_footprint')
                has_r1, r1_flag = self._get_own_fresh_flag_estimate('robot1')
                if r1_pose is not None and has_r1 and r1_flag is not None:
                    d1 = math.hypot(r1_pose[0] - r1_flag[0], r1_pose[1] - r1_flag[1])
                    if d1 <= self.flag_capture_distance:
                        return False

            self._capture_flag(ns, flag_xy)
            return True

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
        self._chase_initialized = False

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

            # Only pursue the flag with this robot's own vision (not teammate estimate)
            has_flag, flag_xy = self._get_own_fresh_flag_estimate(ns)
            if has_flag:
                pose = self._get_robot_pose(cfg['base_frame'])
                if pose is not None:
                    if self._try_capture(ns, pose, flag_xy):
                        continue

                    mb_state = client.get_state()
                    if mb_state in (GoalStatus.ABORTED, GoalStatus.REJECTED, GoalStatus.PREEMPTED):
                        rospy.logwarn('[%s] Flag approach failed (%s); recovering', ns, mb_state)
                        self._recover_navigation(ns)
                        continue

                    if self._should_send_flag_goal(ns, flag_xy[0], flag_xy[1]):
                        gx, gy, gyaw = self._get_approach_goal(pose, flag_xy)
                        d_flag = math.hypot(pose[0] - flag_xy[0], pose[1] - flag_xy[1])
                        rospy.loginfo(
                            '[%s] Pursuing flag candidate at (%.2f, %.2f), distance: %.2f m',
                            ns, flag_xy[0], flag_xy[1], d_flag)
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
            self._claim_frontier(ns, wx, wy)
            rospy.loginfo('%s -> frontier (%.2f, %.2f) size=%d', ns, wx, wy, size)
            client.send_goal(self._make_goal(wx, wy, yaw))
            ok = self._wait_for_goal(ns, client, cfg['base_frame'], (wx, wy), self.goal_timeout)

            # Check if they were close to each other to avoid counting it as a failure
            were_close = False
            pose = self._get_robot_pose(cfg['base_frame'])
            other_ns = 'robot2' if ns == 'robot1' else 'robot1'
            other_pose = self._get_robot_pose(other_ns + '/base_footprint')
            if pose is not None and other_pose is not None:
                if math.hypot(pose[0] - other_pose[0], pose[1] - other_pose[1]) < 2.2:
                    were_close = True

            if ok:
                consecutive_fails = 0
            else:
                if not were_close:
                    consecutive_fails += 1
                    self._blacklist_goal(wx, wy, duration=40.0)
                    spin_sec = 4.0 if consecutive_fails >= 2 else 2.5
                    rospy.logwarn('%s: exploration goal failed (%d in a row); recovering',
                                  ns, consecutive_fails)
                    self._recover_navigation(ns, spin_sec=spin_sec)
                    if consecutive_fails >= 2:
                        consecutive_fails = 0
                else:
                    rospy.loginfo(
                        '%s: Goal cancelled or failed due to proximity/yielding. '
                        'Not counting as failure.', ns)

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

        while not rospy.is_shutdown() and self.game_state != 'FINISHED':
            # Publish markers and update telemetry
            self._publish_markers()

            grid = self._get_map()
            cov = self._coverage(grid)
            unknown = self._unknown_count(grid)

            if self.game_state == 'EXPLORING':
                rospy.loginfo_throttle(
                    10.0, 'Exploration: map coverage=%.1f%%, unknown cells=%d, game_state=%s',
                    cov * 100.0, unknown, self.game_state)

            elif self.game_state == 'CHASE':
                if not self._chase_initialized:
                    self._init_chase()
                else:
                    self._tick_chase()

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
