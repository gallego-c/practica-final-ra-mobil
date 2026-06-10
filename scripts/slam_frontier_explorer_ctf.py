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
        self.separation_weight = float(rospy.get_param('~separation_weight', 2.0))
        self.flag_bias_weight = float(rospy.get_param('~flag_bias_weight', 1.0))
        self.territory_penalty = float(rospy.get_param('~territory_penalty', 4.0))
        self.frontier_claim_sec = float(rospy.get_param('~frontier_claim_sec', 50.0))
        self.min_goal_clearance = float(rospy.get_param('~min_goal_clearance', 0.15))
        self.robot_radius = float(rospy.get_param('~robot_radius', 0.28))
        self.progress_timeout = float(rospy.get_param('~progress_timeout', 15.0))
        self.min_progress_distance = float(rospy.get_param('~min_progress_distance', 0.15))
        self.startup_delay = float(rospy.get_param('~startup_delay', 30.0))
        self.move_base_timeout = float(rospy.get_param('~move_base_timeout', 120.0))
        self.idle_frontier_cycles = int(rospy.get_param('~idle_frontier_cycles', 8))

        # CTF parameters
        self.flag_standoff_distance = float(rospy.get_param('~flag_standoff_distance', 0.20))
        self.flag_capture_margin = float(rospy.get_param('~flag_capture_margin', 0.10))
        self.flag_collision_radius = float(
            rospy.get_param('~flag_collision_radius', 0.12))
        self.flag_approach_stall_sec = float(
            rospy.get_param('~flag_approach_stall_sec', 5.0))
        self.flag_approach_nudge_sec = float(
            rospy.get_param('~flag_approach_nudge_sec', 2.5))
        self.flag_approach_max_distance = float(
            rospy.get_param('~flag_approach_max_distance', 0.85))
        self.flag_approach_flank_step = float(
            rospy.get_param('~flag_approach_flank_step', 0.50))
        self.flag_approach_step = float(
            rospy.get_param('~flag_approach_step', 1.20))
        self.flag_approach_backoff_max = float(
            rospy.get_param('~flag_approach_backoff_max', 1.10))
        self.flag_approach_detour_step = float(
            rospy.get_param('~flag_approach_detour_step', 1.0))
        self.flag_approach_detour_after = int(
            rospy.get_param('~flag_approach_detour_after', 3))
        self.flag_capture_min_pursuit_sec = float(
            rospy.get_param('~flag_capture_min_pursuit_sec', 4.0))
        self.flag_capture_min_progress = float(
            rospy.get_param('~flag_capture_min_progress', 0.20))
        self._min_reach_distance = (
            self.flag_collision_radius
            + self.robot_radius * 0.40
            + 0.05)
        # Capture range: standoff (security) + clearance (maneuverability) + margin
        if rospy.has_param('~flag_capture_distance'):
            self.flag_capture_distance = float(rospy.get_param('~flag_capture_distance'))
        else:
            self.flag_capture_distance = max(
                self._min_reach_distance + self.flag_capture_margin,
                self.flag_standoff_distance
                + self.min_goal_clearance
                + self.flag_capture_margin)
        self.catch_distance = float(rospy.get_param('~catch_distance', 0.65))
        self.chase_goal_period = float(rospy.get_param('~chase_goal_period', 1.0))
        self.capture_pause_sec = float(rospy.get_param('~capture_pause_sec', 3.0))
        self.flag_memory_timeout = float(rospy.get_param('~flag_memory_timeout', 5.0))
        self.flag_support_standoff = float(
            rospy.get_param('~flag_support_standoff', 1.20))
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

        # Game states: 'EXPLORING', 'CAPTURED', 'CHASE', 'FINISHED'
        self.game_state = 'EXPLORING'
        self.carrier_ns = None
        self.pursuer_ns = None
        self._capturer_ns = None
        self._capture_lock = threading.Lock()
        self._chase_initialized = False
        self._carrier_on_home_goal = False
        self._chase_started = 0.0
        self._clearance_goal = None
        self._has_intercept_goal = False
        self._intercept_anchor = None
        self._pursuer_was_succeeded = False
        self._last_carrier_home_resend = 0.0
        self._last_direct_chase_sent = 0.0
        self._last_carrier_progress_time = 0.0
        self._last_carrier_progress_home_dist = -1.0
        self._last_pursuer_progress_dist = -1.0
        self._last_pursuer_progress_time = 0.0

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

        # Per-robot search sub-state during EXPLORING phase:
        # EXPLORING | PURSUING_FLAG (own camera, can capture) | APPROACHING_SHARED (teammate estimate)
        self._search_state = {'robot1': 'EXPLORING', 'robot2': 'EXPLORING'}
        self._state_lock = threading.Lock()

        # Goal throttling
        self._last_flag_goal = {}
        self._last_flag_goal_time = {}
        self._pursuit_start_time = {}
        self._pursuit_closest_dist = {}
        self._pursuit_last_d_flag = {}
        self._pursuit_last_progress_time = {}
        self._pursuit_last_nudge_time = {}
        self._pursuit_stuck_count = {}
        self._pursuit_start_dist = {}
        self._flag_visible_since = {}

        # Robot homes
        self.homes = {
            'robot1': rospy.get_param('~robot1_home', [-3.0, -3.0, 0.0]),
            'robot2': rospy.get_param('~robot2_home', [3.0, 3.0, 3.1416]),
        }

        self._map = None
        self._map_lock = threading.Lock()
        self._robot_maps = {'robot1': None, 'robot2': None}
        self._robot_map_lock = threading.Lock()
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
        for cfg in self.robots:
            ns = cfg['ns']
            rospy.Subscriber(
                '/' + ns + '/map', OccupancyGrid, self._robot_map_cb,
                callback_args=ns, queue_size=1)
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
            self._cmd_pubs[ns] = rospy.Publisher('/' + ns + '/cmd_vel_raw', Twist, queue_size=1)
            
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
        rospy.loginfo(
            'SlamFrontierExplorerCtf: flag_capture_distance=%.2f m '
            '(standoff=%.2f + clearance=%.2f + margin=%.2f)',
            self.flag_capture_distance, self.flag_standoff_distance,
            self.min_goal_clearance, self.flag_capture_margin)
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
        now = time.monotonic()
        with self.flag_lock:
            if msg.data:
                if not self.flag_found.get(ns, False):
                    self._flag_visible_since[ns] = now
            else:
                self._flag_visible_since.pop(ns, None)
            self.flag_found[ns] = msg.data

    def _flag_visible_duration(self, ns):
        with self.flag_lock:
            if not self.flag_found.get(ns, False):
                return 0.0
            start = self._flag_visible_since.get(ns)
            if start is None:
                return 0.0
            return time.monotonic() - start

    def _flag_estimate_cb(self, msg, ns):
        with self.flag_lock:
            self.flag_estimate[ns] = (msg.pose.position.x, msg.pose.position.y)
            self.flag_estimate_time[ns] = rospy.Time.now().to_sec()
            self.last_known_flag_xy = self.flag_estimate[ns]

    def _has_fresh_flag_estimate(self, ns):
        """Fresh vision estimate for this robot (same logic as RobotAgent::hasFreshFlagEstimate)."""
        with self.flag_lock:
            if self.flag_estimate[ns] is None:
                return False, None
            age = rospy.Time.now().to_sec() - self.flag_estimate_time[ns]
            if age > self.flag_memory_timeout:
                return False, None
            return True, self.flag_estimate[ns]

    def _get_fresh_flag_estimate(self, ns):
        """Own estimate if fresh, otherwise teammate's (shared flag location)."""
        has_own, own_est = self._has_fresh_flag_estimate(ns)
        if has_own:
            return True, own_est

        with self.flag_lock:
            other_ns = 'robot2' if ns == 'robot1' else 'robot1'
            if self.flag_estimate[other_ns] is not None:
                age_other = rospy.Time.now().to_sec() - self.flag_estimate_time[other_ns]
                if age_other <= self.flag_memory_timeout:
                    return True, self.flag_estimate[other_ns]
            return False, None

    def _flag_estimate_source(self, ns):
        """Return 'own', 'teammate', or None."""
        has_own, _ = self._has_fresh_flag_estimate(ns)
        if has_own:
            return 'own'
        has_shared, _ = self._get_fresh_flag_estimate(ns)
        if has_shared:
            return 'teammate'
        return None

    def _clear_pursuit_tracking(self, ns):
        self._pursuit_start_time.pop(ns, None)
        self._pursuit_closest_dist.pop(ns, None)
        self._pursuit_last_d_flag.pop(ns, None)
        self._pursuit_last_progress_time.pop(ns, None)
        self._pursuit_last_nudge_time.pop(ns, None)
        self._pursuit_stuck_count.pop(ns, None)
        self._pursuit_start_dist.pop(ns, None)
        self._last_flag_goal.pop(ns, None)
        self._last_flag_goal_time.pop(ns, None)

    def _begin_flag_navigation(self, ns, flag_xy):
        self._pursuit_start_time[ns] = time.monotonic()
        self._pursuit_last_progress_time[ns] = time.monotonic()
        self._pursuit_last_nudge_time.pop(ns, None)
        self._pursuit_stuck_count[ns] = 0
        pose = self._get_robot_pose(ns + '/base_footprint')
        if pose is not None and flag_xy is not None:
            d0 = math.hypot(pose[0] - flag_xy[0], pose[1] - flag_xy[1])
            self._pursuit_start_dist[ns] = d0
            self._pursuit_closest_dist[ns] = d0
            self._pursuit_last_d_flag[ns] = d0

    def _update_search_flag_states(self):
        """Flag location is shared via _get_fresh_flag_estimate (teammate estimate).

        - PURSUING_FLAG: this robot sees the flag with its own camera → go capture.
        - APPROACHING_SHARED: teammate shared the location → navigate there to be
          close enough to intercept the carrier after capture.
        - EXPLORING: no flag knowledge yet.
        """
        with self._state_lock:
            for ns in ('robot1', 'robot2'):
                if self._search_state.get(ns) == 'IDLE':
                    continue
                has_own, own_xy = self._has_fresh_flag_estimate(ns)
                source = self._flag_estimate_source(ns)
                current = self._search_state[ns]

                if has_own:
                    if current != 'PURSUING_FLAG':
                        rospy.loginfo(
                            'CTF FLAG: %s pursuing flag (own camera)', ns)
                        self._search_state[ns] = 'PURSUING_FLAG'
                        self._begin_flag_navigation(ns, own_xy)
                        self._clients[ns].cancel_all_goals()
                elif source == 'teammate':
                    _, shared_xy = self._get_fresh_flag_estimate(ns)
                    if shared_xy is None:
                        continue
                    if current != 'APPROACHING_SHARED':
                        rospy.loginfo(
                            'CTF FLAG: %s approaching shared flag at (%.2f, %.2f)',
                            ns, shared_xy[0], shared_xy[1])
                        self._search_state[ns] = 'APPROACHING_SHARED'
                        self._begin_flag_navigation(ns, shared_xy)
                        self._clients[ns].cancel_all_goals()
                elif current != 'EXPLORING':
                    rospy.loginfo('%s: flag estimate lost; resuming exploration', ns)
                    self._search_state[ns] = 'EXPLORING'
                    self._clear_pursuit_tracking(ns)
                    self._clients[ns].cancel_all_goals()

    def _find_first_capture_candidate(self):
        """Capture only when close enough (map distance) AND camera sees the flag."""
        candidates = []
        for ns in ('robot1', 'robot2'):
            with self._state_lock:
                if self._search_state.get(ns) != 'PURSUING_FLAG':
                    continue

            has_own, flag_xy = self._has_fresh_flag_estimate(ns)
            if not has_own or flag_xy is None:
                continue

            with self.flag_lock:
                if not self.flag_found.get(ns, False):
                    continue
                age = rospy.Time.now().to_sec() - self.flag_estimate_time[ns]
            if age > 2.0:
                continue

            pose = self._get_robot_pose(ns + '/base_footprint')
            if pose is None:
                continue

            d_flag = math.hypot(pose[0] - flag_xy[0], pose[1] - flag_xy[1])
            if d_flag > self.flag_capture_distance:
                continue

            # Debounce: require flag visible briefly (not capture from one noisy frame)
            if self._flag_visible_duration(ns) < 0.25:
                continue

            candidates.append((ns, flag_xy, d_flag))

        if not candidates:
            return None
        candidates.sort(key=lambda c: c[2])
        return candidates[0][0], candidates[0][1]

    def _map_cb(self, msg):
        with self._map_lock:
            self._map = msg

    def _robot_map_cb(self, msg, ns):
        with self._robot_map_lock:
            self._robot_maps[ns] = msg

    def _get_map(self):
        with self._map_lock:
            return self._map

    def _get_robot_map(self, ns):
        with self._robot_map_lock:
            return self._robot_maps.get(ns)

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
            separation -= self._territory_penalty_for(ns, wx, wy) * 0.1
            if separation > best_sep:
                best_sep = separation
                best = (wx, wy, size)
        return best, best_sep

    def _pick_closest_frontier(self, ns, robot_xy, frontiers):
        """Last-resort frontier when Voronoi blocks everything (keeps idle robots moving)."""
        best = None
        best_dist = float('inf')
        for wx, wy, size in frontiers:
            if self._is_blacklisted(wx, wy):
                continue
            if self._is_frontier_claimed_by_other(ns, wx, wy):
                continue
            dist = math.hypot(wx - robot_xy[0], wy - robot_xy[1])
            if dist < best_dist:
                best_dist = dist
                best = (wx, wy, size)
        return best

    def _pick_frontier(self, ns, robot_xy, frontiers, all_poses, strict_voronoi=True):
        best = None
        best_score = float('inf')
        used_fallback = False

        has_flag_hint, flag_xy = self._get_fresh_flag_estimate(ns)
        flag_source = self._flag_estimate_source(ns)
        flag_bias = self.flag_bias_weight
        if flag_source == 'teammate':
            flag_bias *= 3.0

        for wx, wy, size in frontiers:
            if self._is_blacklisted(wx, wy):
                continue
            if self._is_frontier_claimed_by_other(ns, wx, wy):
                continue

            my_dist = math.hypot(wx - robot_xy[0], wy - robot_xy[1])

            if strict_voronoi and flag_source != 'teammate':
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

            if flag_source != 'teammate':
                score += self._territory_penalty_for(ns, wx, wy)

            for other_ns, other_xy in all_poses.items():
                if other_ns == ns:
                    continue
                other_dist = math.hypot(wx - other_xy[0], wy - other_xy[1])
                if other_dist < my_dist:
                    score += self.separation_weight * (my_dist - other_dist)

            if has_flag_hint and flag_xy is not None:
                dist_to_flag = math.hypot(wx - flag_xy[0], wy - flag_xy[1])
                score += flag_bias * dist_to_flag

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


        return best

    def _get_nav_grid(self, ns):
        robot_map = self._get_robot_map(ns)
        merged = self._get_map()
        if robot_map is None:
            return merged
        if merged is not None and robot_map.data:
            known = sum(1 for v in robot_map.data if v >= 0)
            coverage = float(known) / float(len(robot_map.data))
            if coverage < 0.20:
                return merged
        return robot_map or merged

    def _resolve_chase_goal(self, ns, robot_pose, target_xy, allow_partial=True):
        """Map-validated chase goal: known-free cell, stepping through mapped space."""
        grid = self._get_nav_grid(ns)
        if grid is None:
            return None

        tx, ty = target_xy[0], target_xy[1]
        travel = math.hypot(tx - robot_pose[0], ty - robot_pose[1])
        if travel < 0.20:
            return None

        snapped = self._snap_goal_to_free(grid, tx, ty)
        if snapped is not None:
            tx, ty = snapped
            mx, my = world_to_map(tx, ty, grid.info)
            if self._cell_has_clearance(grid, mx, my):
                yaw = math.atan2(ty - robot_pose[1], tx - robot_pose[0])
                return tx, ty, yaw

        if allow_partial:
            step = self._find_map_goal_toward(grid, robot_pose, target_xy)
            if step is not None:
                sx, sy = step
                yaw = math.atan2(sy - robot_pose[1], sx - robot_pose[0])
                return sx, sy, yaw
        return None

    def _send_carrier_goal(self, wx, wy, yaw, tag=''):
        if self.carrier_ns is None or self.game_state not in ('CAPTURED', 'CHASE'):
            return
        if self._capturer_ns is not None and self.carrier_ns != self._capturer_ns:
            rospy.logerr(
                'CHASE ROLE ERROR: carrier=%s but capturer=%s — fixing roles',
                self.carrier_ns, self._capturer_ns)
            self.carrier_ns = self._capturer_ns
            self.pursuer_ns = (
                'robot2' if self._capturer_ns == 'robot1' else 'robot1')
        rospy.loginfo(
            'CHASE carrier=%s [%s] -> (%.2f, %.2f) home=(%.2f, %.2f)',
            self.carrier_ns, tag, wx, wy,
            self.homes[self.carrier_ns][0], self.homes[self.carrier_ns][1])
        self._clients[self.carrier_ns].send_goal(
            self._make_goal(wx, wy, yaw, ns=self.carrier_ns))

    def _send_pursuer_goal(self, wx, wy, yaw, tag=''):
        if self.pursuer_ns is None or self.game_state != 'CHASE':
            return
        rospy.loginfo(
            'CHASE pursuer=%s [%s] -> (%.2f, %.2f) (target: carrier %s)',
            self.pursuer_ns, tag, wx, wy, self.carrier_ns)
        self._clients[self.pursuer_ns].send_goal(
            self._make_goal(wx, wy, yaw, ns=self.pursuer_ns))

    def _make_goal(self, wx, wy, yaw, ns=None):
        qx, qy, qz, qw = yaw_to_quaternion(yaw)
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = (
            ns + '/map' if ns else self.map_frame)
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
            if self.game_state != 'EXPLORING':
                client.cancel_goal()
                return False
            has_own, _ = self._has_fresh_flag_estimate(ns)
            if has_own:
                client.cancel_goal()
                return False
            with self._state_lock:
                mode = self._search_state.get(ns, 'EXPLORING')
            if mode in ('PURSUING_FLAG', 'APPROACHING_SHARED'):
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

    def _spin_to_scan(self, ns, duration=2.0):
        pub = self._cmd_pubs[ns]
        twist = Twist()
        twist.angular.z = 0.8
        end = rospy.Time.now() + rospy.Duration(min(duration, 2.0))
        rate = rospy.Rate(10)
        while rospy.Time.now() < end and not rospy.is_shutdown() and not self._stop.is_set():
            if self.game_state != 'EXPLORING':
                break
            pub.publish(twist)
            rate.sleep()
        pub.publish(Twist())

    def _should_send_flag_goal(self, ns, fx, fy, pursuing=False):
        now = rospy.Time.now().to_sec()
        last_goal = self._last_flag_goal.get(ns)
        last_time = self._last_flag_goal_time.get(ns, 0.0)

        moved = last_goal is None or math.hypot(fx - last_goal[0], fy - last_goal[1]) > 0.25
        stale_interval = 0.8 if pursuing else 1.0
        stale = (now - last_time) > stale_interval

        if moved or stale:
            self._last_flag_goal[ns] = (fx, fy)
            self._last_flag_goal_time[ns] = now
            return True
        return False

    def _snap_goal_to_free(self, grid, wx, wy):
        """Return nearest free map cell to (wx, wy), or None."""
        if grid is None:
            return wx, wy
        info = grid.info
        mx, my = world_to_map(wx, wy, info)
        if self._cell_has_clearance(grid, mx, my):
            return wx, wy

        search_cells = max(3, int(round(1.5 / info.resolution)))
        best = None
        best_dist = float('inf')
        for radius in range(1, search_cells + 1):
            for dy in range(-radius, radius + 1):
                for dx in range(-radius, radius + 1):
                    if abs(dx) != radius and abs(dy) != radius:
                        continue
                    cx, cy = mx + dx, my + dy
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

    def _find_map_goal_toward(self, grid, robot_xy, target_xy):
        """Known-free cell closer to target than the robot (avoids line-through-wall goals)."""
        info = grid.info
        data = grid.data
        w, h = info.width, info.height
        rx, ry = world_to_map(robot_xy[0], robot_xy[1], info)
        robot_dist = math.hypot(robot_xy[0] - target_xy[0], robot_xy[1] - target_xy[1])
        search_cells = max(4, int(round(2.5 / info.resolution)))

        best = None
        best_score = float('inf')
        for dy in range(-search_cells, search_cells + 1):
            for dx in range(-search_cells, search_cells + 1):
                cx, cy = rx + dx, ry + dy
                if not self._cell_has_clearance(grid, cx, cy):
                    continue
                wx, wy = map_to_world(cx, cy, info)
                to_flag = math.hypot(wx - target_xy[0], wy - target_xy[1])
                if to_flag >= robot_dist - 0.08:
                    continue
                step_cost = math.hypot(wx - robot_xy[0], wy - robot_xy[1])
                score = to_flag + 0.15 * step_cost
                if score < best_score:
                    best_score = score
                    best = (wx, wy)
        return best

    def _move_base_idle(self, client):
        state = client.get_state()
        return state not in (GoalStatus.ACTIVE, GoalStatus.PENDING, GoalStatus.RECALLING)

    def _approach_shared_flag(self, ns, client, cfg, pose, flag_xy):
        """Reach the flag area via frontiers / map — never a straight-line goal through walls."""
        if self.game_state != 'EXPLORING':
            return
        grid = self._get_map()
        if grid is None:
            return

        d_flag = math.hypot(pose[0] - flag_xy[0], pose[1] - flag_xy[1])
        if d_flag <= self.flag_support_standoff:
            return

        if not self._move_base_idle(client):
            return

        frontiers = self._find_frontiers(grid)
        all_poses = self._all_robot_poses()
        pick = (self._pick_frontier(ns, pose, frontiers, all_poses, strict_voronoi=False)
                if frontiers else None)

        wx = wy = None
        if pick is not None:
            fx, fy, _size = pick
            safe = self._safe_goal_near_frontier(grid, fx, fy)
            if safe is not None:
                wx, wy = safe

        if wx is None:
            step = self._find_map_goal_toward(grid, pose, flag_xy)
            if step is not None:
                wx, wy = step

        if wx is None:
            rospy.logwarn_throttle(8.0,
                                   '%s: no map-safe route toward shared flag yet', ns)
            return

        yaw = math.atan2(wy - pose[1], wx - pose[0])
        rospy.loginfo(
            '%s APPROACH_SHARED -> (%.2f, %.2f) toward flag (%.2f, %.2f) dist=%.2f',
            ns, wx, wy, flag_xy[0], flag_xy[1], d_flag)
        client.send_goal(self._make_goal(wx, wy, yaw, ns=ns))

    def _get_backoff_goal(self, robot_pose, grid, flank_sign=1):
        """Short move_base goal away from the current heading (map-validated)."""
        rx, ry, yaw = robot_pose
        lx, ly = -math.sin(yaw), math.cos(yaw)
        candidates = [
            (rx - math.cos(yaw) * 0.40, ry - math.sin(yaw) * 0.40),
            (rx + lx * 0.45 * flank_sign, ry + ly * 0.45 * flank_sign),
            (rx - math.cos(yaw) * 0.30 + lx * 0.35 * flank_sign,
             ry - math.sin(yaw) * 0.30 + ly * 0.35 * flank_sign),
        ]
        for gx, gy in candidates:
            snapped = self._snap_goal_to_free(grid, gx, gy)
            if snapped is not None:
                return snapped[0], snapped[1], math.atan2(snapped[1] - ry, snapped[0] - rx)
        return None

    def _flank_index_for_stuck(self, stuck_count):
        if stuck_count <= 0:
            return 0
        k = (stuck_count + 1) // 2
        return k if stuck_count % 2 == 1 else -k

    def _get_approach_goal(self, robot_pose, flag_xy, direct=False, flank_index=0):
        """Goal toward the flag: incremental steps when far, standoff when close."""
        rx, ry, _ = robot_pose
        fx, fy = flag_xy
        dx = fx - rx
        dy = fy - ry
        dist = math.hypot(dx, dy)
        if dist < 1e-3:
            return fx, fy, 0.0

        ux, uy = dx / dist, dy / dist
        px, py = -uy, ux
        lateral = flank_index * self.flag_approach_flank_step
        close = self.flag_approach_max_distance
        standoff = (
            self._min_reach_distance if (direct or dist <= close)
            else max(self.flag_standoff_distance, self._min_reach_distance))

        if dist > close:
            travel = min(self.flag_approach_step, max(0.55, dist - standoff - 0.12))
            gx = rx + ux * travel + px * lateral * 0.4
            gy = ry + uy * travel + py * lateral * 0.4
            gyaw = math.atan2(gy - ry, gx - rx)
            return gx, gy, gyaw

        gx = fx - ux * standoff + px * lateral
        gy = fy - uy * standoff + py * lateral
        gyaw = math.atan2(fy - gy, fx - gx)
        return gx, gy, gyaw

    def _get_detour_goal(self, robot_pose, flag_xy, flank_sign):
        """Side-step waypoint to leave a narrow gap and replan around obstacles."""
        rx, ry, _ = robot_pose
        fx, fy = flag_xy
        dx, dy = fx - rx, fy - ry
        dist = math.hypot(dx, dy)
        if dist < 1e-3:
            return rx, ry, 0.0

        ux, uy = dx / dist, dy / dist
        px, py = -uy, ux
        step = min(self.flag_approach_detour_step, max(0.40, dist * 0.35))
        gx = rx + ux * step + px * flank_sign * self.flag_approach_flank_step * 0.7
        gy = ry + uy * step + py * flank_sign * self.flag_approach_flank_step * 0.7
        return gx, gy, math.atan2(gy - ry, gx - rx)

    def _send_flag_approach(self, ns, client, pose, flag_xy, support_flank=0):
        if self.game_state != 'EXPLORING':
            return
        d_flag = math.hypot(pose[0] - flag_xy[0], pose[1] - flag_xy[1])
        mb_state = client.get_state()
        now = time.monotonic()
        stuck_count = self._pursuit_stuck_count.get(ns, 0)

        if d_flag < self._pursuit_closest_dist.get(ns, d_flag + 1.0) - 0.03:
            self._pursuit_closest_dist[ns] = d_flag
            self._pursuit_last_progress_time[ns] = now
            if stuck_count > 0:
                self._pursuit_stuck_count[ns] = stuck_count - 1
                stuck_count = self._pursuit_stuck_count[ns]

        last_d = self._pursuit_last_d_flag.get(ns, d_flag)
        no_progress = (
            now - self._pursuit_last_progress_time.get(ns, now) > self.flag_approach_nudge_sec
            and d_flag >= last_d - 0.03)
        terminal = mb_state in (
            GoalStatus.ABORTED, GoalStatus.REJECTED, GoalStatus.PREEMPTED)
        idle = mb_state not in (
            GoalStatus.ACTIVE, GoalStatus.PENDING, GoalStatus.RECALLING)
        oscillating = (
            mb_state in (GoalStatus.ACTIVE, GoalStatus.PENDING)
            and no_progress and d_flag > self._min_reach_distance + 0.15)

        use_backoff = False
        if d_flag <= self.flag_approach_backoff_max and (oscillating or (terminal and no_progress)):
            stuck_count += 1
            self._pursuit_stuck_count[ns] = stuck_count
            use_backoff = True
            client.cancel_all_goals()
            self._pursuit_last_progress_time[ns] = now
            self._last_flag_goal_time[ns] = 0.0
        elif oscillating or (terminal and no_progress):
            stuck_count += 1
            self._pursuit_stuck_count[ns] = stuck_count
            client.cancel_all_goals()
            self._pursuit_last_progress_time[ns] = now
            self._last_flag_goal_time[ns] = 0.0

        start_dist = self._pursuit_start_dist.get(ns, d_flag)
        closest = self._pursuit_closest_dist.get(ns, d_flag)
        made_approach_progress = closest <= start_dist - 0.35
        close_enough_to_refine = (
            d_flag <= self.flag_approach_max_distance
            and (start_dist <= self.flag_approach_max_distance + 0.30
                 or made_approach_progress))
        use_direct = close_enough_to_refine or terminal
        still_far = d_flag > self.flag_capture_distance + 0.15

        need_send = (
            idle
            or terminal
            or no_progress
            or oscillating
            or (mb_state == GoalStatus.SUCCEEDED and still_far)
            or self._should_send_flag_goal(ns, flag_xy[0], flag_xy[1], pursuing=True))

        if not need_send:
            return

        grid = self._get_robot_map(ns) or self._get_map()
        stuck_count = self._pursuit_stuck_count.get(ns, 0)
        flank_sign = 1 if stuck_count % 2 == 1 else -1
        mode = 'approach'
        gx = gy = gyaw = None

        if use_backoff and grid is not None:
            backoff = self._get_backoff_goal(pose, grid, flank_sign)
            if backoff is not None:
                gx, gy, gyaw = backoff
                mode = 'backoff'

        if gx is None:
            if stuck_count >= self.flag_approach_detour_after:
                gx, gy, gyaw = self._get_detour_goal(pose, flag_xy, flank_sign)
                mode = 'detour'
            else:
                flank = support_flank if support_flank else self._flank_index_for_stuck(stuck_count)
                gx, gy, gyaw = self._get_approach_goal(
                    pose, flag_xy, direct=use_direct, flank_index=flank)
                if not use_direct:
                    mode = 'step'
                elif flank:
                    mode = 'flank'
                elif use_direct:
                    mode = 'direct'
                else:
                    mode = 'approach'

        if grid is not None:
            if mode == 'step' or d_flag > self.flag_approach_max_distance:
                snapped = self._snap_goal_to_free(grid, gx, gy)
                if snapped is not None:
                    snap_shift = math.hypot(snapped[0] - gx, snapped[1] - gy)
                    if snap_shift <= 1.0:
                        gx, gy = snapped
            elif d_flag < 2.5:
                snapped = self._snap_goal_to_free(grid, gx, gy)
                if snapped is None:
                    rospy.logwarn('%s: no free cell near flag goal (%.2f, %.2f); detour',
                                  ns, gx, gy)
                    self._pursuit_stuck_count[ns] = stuck_count + 1
                    gx, gy, gyaw = self._get_detour_goal(pose, flag_xy, flank_sign)
                    mode = 'detour'
                else:
                    snap_shift = math.hypot(snapped[0] - gx, snapped[1] - gy)
                    if snap_shift <= 0.75:
                        gx, gy = snapped
                    else:
                        rospy.logwarn('%s: snap rejected (shift=%.2f m); keep raw goal',
                                      ns, snap_shift)

        rospy.loginfo(
            'CTF FLAG: %s approach goal=(%.2f, %.2f) flag=(%.2f, %.2f) '
            'dist=%.2f m mode=%s stuck=%d state=%d start=%.2f closest=%.2f',
            ns, gx, gy, flag_xy[0], flag_xy[1], d_flag, mode, stuck_count, mb_state,
            start_dist, closest)
        client.send_goal(self._make_goal(gx, gy, gyaw, ns=ns))
        self._pursuit_last_d_flag[ns] = d_flag

    def _goal_terminal(self, ns):
        state = self._clients[ns].get_state()
        return state in (GoalStatus.SUCCEEDED, GoalStatus.ABORTED,
                         GoalStatus.REJECTED, GoalStatus.PREEMPTED)

    def _goal_succeeded(self, ns):
        return self._clients[ns].get_state() == GoalStatus.SUCCEEDED

    def _make_chase_clearance_goal(self, carrier_pose, home):
        cx, cy, _ = carrier_pose
        hx, hy, hyaw = home
        dx = hx - cx
        dy = hy - cy
        dist = math.hypot(dx, dy)
        if dist < 1e-3:
            return hx, hy, hyaw

        step = min(self.chase_flag_clearance, max(0.0, dist - 0.10))
        if step < self.chase_clearance_min_travel:
            return hx, hy, hyaw

        return cx + (dx / dist) * step, cy + (dy / dist) * step, math.atan2(dy, dx)

    def _make_interception_goal(self, carrier_pose, pursuer_pose, home):
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
        chosen_s = -1.0
        s = step
        while s <= home_dist:
            ix = cx + ux * s
            iy = cy + uy * s
            pursuer_dist = math.hypot(px - ix, py - iy)
            if pursuer_dist + self.intercept_arrival_margin <= s:
                chosen_s = s
                break
            s += step

        if chosen_s < 0.0:
            chosen_s = min(home_dist, max(step, self.intercept_min_lead))

        gx = cx + ux * chosen_s
        gy = cy + uy * chosen_s
        return gx, gy, math.atan2(gy - py, gx - px)

    def _send_pursuer_intercept(self, carrier_pose, pursuer_pose):
        home = self.homes[self.carrier_ns]
        gx, gy, gyaw = self._make_interception_goal(carrier_pose, pursuer_pose, home)
        resolved = self._resolve_chase_goal(
            self.pursuer_ns, pursuer_pose, (gx, gy), allow_partial=True)
        if resolved is None:
            rospy.logwarn_throttle(
                4.0, 'CHASE: pursuer %s no safe intercept goal toward (%.2f, %.2f)',
                self.pursuer_ns, gx, gy)
            return
        gx, gy, gyaw = resolved
        self._send_pursuer_goal(gx, gy, gyaw, tag='intercept')
        self._has_intercept_goal = True
        self._intercept_anchor = (carrier_pose[0], carrier_pose[1])
        self._pursuer_was_succeeded = False
        self._last_pursuer_progress_dist = math.hypot(
            pursuer_pose[0] - gx, pursuer_pose[1] - gy)
        self._last_pursuer_progress_time = time.monotonic()

    def _send_pursuer_direct_chase(self, carrier_pose, pursuer_pose):
        cx, cy, _ = carrier_pose
        px, py, _ = pursuer_pose
        resolved = self._resolve_chase_goal(
            self.pursuer_ns, pursuer_pose, (cx, cy), allow_partial=True)
        if resolved is None:
            rospy.logwarn_throttle(
                4.0, 'CHASE: pursuer %s no safe chase step toward carrier yet',
                self.pursuer_ns)
            return
        gx, gy, gyaw = resolved
        self._send_pursuer_goal(gx, gy, gyaw, tag='direct-chase')
        self._has_intercept_goal = True
        self._intercept_anchor = (cx, cy)
        self._pursuer_was_succeeded = False
        self._last_direct_chase_sent = time.monotonic()
        self._last_pursuer_progress_dist = math.hypot(px - cx, py - cy)
        self._last_pursuer_progress_time = time.monotonic()

    def _resend_carrier_goal(self):
        home = self.homes[self.carrier_ns]
        cpose = self._get_robot_pose(self.carrier_ns + '/base_footprint')
        if cpose is None:
            return
        if self._carrier_on_home_goal:
            resolved = self._resolve_chase_goal(
                self.carrier_ns, cpose, (home[0], home[1]), allow_partial=True)
            if resolved is None:
                rospy.logwarn_throttle(5.0,
                                       'CTF CHASE: %s no safe plan to home yet',
                                       self.carrier_ns)
                return
            gx, gy, gyaw = resolved
            tag = 'home-resend'
        elif self._clearance_goal is not None:
            gx, gy = self._clearance_goal
            gyaw = math.atan2(home[1] - gy, home[0] - gx)
            resolved = self._resolve_chase_goal(
                self.carrier_ns, cpose, (gx, gy), allow_partial=True)
            if resolved is None:
                rospy.logwarn_throttle(
                    5.0, 'CTF CHASE: %s no safe clearance resend', self.carrier_ns)
                return
            gx, gy, gyaw = resolved
            tag = 'clearance-resend'
        else:
            return
        self._send_carrier_goal(gx, gy, gyaw, tag=tag)

    def _init_chase(self):
        for ns in self._clients:
            self._cancel_robot_goal(ns)
        for ns in self._clients:
            self._wait_move_base_idle(ns)
        rospy.sleep(0.3)

        cpose = self._get_robot_pose(self.carrier_ns + '/base_footprint')
        ppose = self._get_robot_pose(self.pursuer_ns + '/base_footprint')
        if not cpose or not ppose:
            rospy.logwarn('CHASE: missing TF pose at start')
            self._chase_initialized = True
            return

        home = self.homes[self.carrier_ns]
        gx, gy, gyaw = self._make_chase_clearance_goal(cpose, home)
        clearance_dist = math.hypot(gx - cpose[0], gy - cpose[1])
        self._clearance_goal = (gx, gy)
        self._carrier_on_home_goal = clearance_dist < self.chase_clearance_min_travel
        self._chase_started = time.monotonic()

        if self._carrier_on_home_goal:
            resolved = self._resolve_chase_goal(
                self.carrier_ns, cpose, (home[0], home[1]), allow_partial=True)
            if resolved is None:
                rospy.logwarn('CHASE: %s no safe direct home plan; trying clearance step',
                              self.carrier_ns)
                self._carrier_on_home_goal = False
                self._clearance_goal = (gx, gy)
                resolved = self._resolve_chase_goal(
                    self.carrier_ns, cpose, (gx, gy), allow_partial=True)
            if resolved is None:
                rospy.logwarn('CHASE: missing safe carrier goal at start')
                self._chase_initialized = True
                return
            tgx, tgy, tgyaw = resolved
            self._send_carrier_goal(tgx, tgy, tgyaw, tag='home-start')
            target = 'base'
        else:
            resolved = self._resolve_chase_goal(
                self.carrier_ns, cpose, (gx, gy), allow_partial=True)
            if resolved is None:
                rospy.logwarn('CHASE: missing safe clearance goal at start')
                self._chase_initialized = True
                return
            tgx, tgy, tgyaw = resolved
            self._send_carrier_goal(tgx, tgy, tgyaw, tag='clearance-start')
            target = 'clearance'

        rospy.loginfo(
            'CTF CHASE START: capturer=%s carrier=%s -> %s (%.2f, %.2f); pursuer=%s chases carrier',
            self._capturer_ns, self.carrier_ns, target,
            home[0] if self._carrier_on_home_goal else tgx,
            home[1] if self._carrier_on_home_goal else tgy,
            self.pursuer_ns)

        self._last_carrier_progress_home_dist = math.hypot(
            cpose[0] - home[0], cpose[1] - home[1])
        self._last_carrier_progress_time = time.monotonic()
        self._send_pursuer_direct_chase(cpose, ppose)
        self._chase_initialized = True

    def _tick_chase(self):
        cpose = self._get_robot_pose(self.carrier_ns + '/base_footprint')
        ppose = self._get_robot_pose(self.pursuer_ns + '/base_footprint')
        if not cpose or not ppose:
            rospy.logwarn_throttle(3.0, 'CHASE: waiting for TF frames of robots...')
            return

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
            clearance_ok = clearance_dist <= self.chase_clearance_reached_dist
            clearance_goal_done = (
                self._goal_succeeded(self.carrier_ns)
                and time.monotonic() - self._chase_started > 1.0)
            if clearance_ok or clearance_goal_done:
                resolved = self._resolve_chase_goal(
                    self.carrier_ns, cpose, (home[0], home[1]), allow_partial=True)
                if resolved is not None:
                    gx, gy, gyaw = resolved
                    self._send_carrier_goal(gx, gy, gyaw, tag='home-after-clearance')
                    self._carrier_on_home_goal = True
                    self._last_carrier_progress_time = time.monotonic()
                    self._last_carrier_progress_home_dist = carrier_home_dist
                    rospy.loginfo('CTF CHASE: %s reached clearance; continuing to base',
                                  self.carrier_ns)
                elif clearance_goal_done:
                    rospy.logwarn_throttle(
                        5.0, 'CTF CHASE: %s clearance done but no safe home plan yet',
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
                self._resend_carrier_goal()
                self._last_carrier_home_resend = now
                self._last_carrier_progress_time = now
                self._last_carrier_progress_home_dist = carrier_home_dist
                rospy.logwarn(
                    'CTF CHASE: %s stuck ACTIVE for %.1f s at home dist %.2f m; re-sending goal',
                    self.carrier_ns, stuck_for, carrier_home_dist)

        # Pursuer always chases the carrier — never receives a home goal.
        carrier_move = 0.0
        if self._intercept_anchor is not None:
            carrier_move = math.hypot(
                cpose[0] - self._intercept_anchor[0],
                cpose[1] - self._intercept_anchor[1])
        pursuer_failed = (self._goal_terminal(self.pursuer_ns) and
                          not self._goal_succeeded(self.pursuer_ns))
        pursuer_state = self._clients[self.pursuer_ns].get_state()
        pursuer_active = pursuer_state in (
            GoalStatus.ACTIVE, GoalStatus.PENDING, GoalStatus.RECALLING)
        pursuer_dist_carrier = math.hypot(ppose[0] - cpose[0], ppose[1] - cpose[1])
        if (self._last_pursuer_progress_dist < 0.0 or
                self._last_pursuer_progress_dist - pursuer_dist_carrier >= 0.12):
            self._last_pursuer_progress_dist = pursuer_dist_carrier
            self._last_pursuer_progress_time = now
        pursuer_stuck = (
            (now - self._last_pursuer_progress_time) >= 8.0 and pursuer_active)

        chase_ready = time.monotonic() - self._chase_started > 1.0
        refresh_pursuer = chase_ready and (
            not self._has_intercept_goal or
            pursuer_failed or
            pursuer_stuck or
            carrier_move >= self.direct_chase_carrier_move)
        if refresh_pursuer:
            if d <= self.intercept_direct_chase_distance or not self.intercept_enabled:
                self._send_pursuer_direct_chase(cpose, ppose)
            else:
                self._send_pursuer_intercept(cpose, ppose)

    def _cancel_robot_goal(self, ns):
        self._clients[ns].cancel_all_goals()

    def _wait_move_base_idle(self, ns, timeout=2.0):
        """Wait until move_base is not reporting a stale SUCCEEDED from a prior goal."""
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline and not rospy.is_shutdown():
            state = self._clients[ns].get_state()
            if state not in (GoalStatus.SUCCEEDED, GoalStatus.ABORTED,
                             GoalStatus.REJECTED, GoalStatus.PREEMPTED):
                return
            rospy.sleep(0.05)

    def _enter_post_capture_phase(self):
        """Stop exploration threads from sending goals; clear per-robot search state."""
        with self._state_lock:
            for ns in ('robot1', 'robot2'):
                self._search_state[ns] = 'IDLE'
                self._clear_pursuit_tracking(ns)
        for ns in self._clients:
            self._cancel_robot_goal(ns)
        self._has_intercept_goal = False
        self._intercept_anchor = None
        self._pursuer_was_succeeded = False
        self._carrier_on_home_goal = False
        self._clearance_goal = None

    def _capture_flag(self, ns, flag_xy):
        with self._capture_lock:
            if self.game_state != 'EXPLORING':
                return

            pose = self._get_robot_pose(ns + '/base_footprint')
            d_flag = (math.hypot(pose[0] - flag_xy[0], pose[1] - flag_xy[1])
                      if pose is not None else -1.0)
            rospy.loginfo('==================================================')
            rospy.loginfo(
                'CTF CAPTURE: %s reached the flag at (%.2f, %.2f), distance=%.2f m',
                ns, flag_xy[0], flag_xy[1], d_flag)
            rospy.loginfo('==================================================')

            self._capturer_ns = ns
            self.carrier_ns = ns
            self.pursuer_ns = 'robot2' if ns == 'robot1' else 'robot1'
            rospy.loginfo(
                'CTF ROLES: capturer=%s -> carrier (home at %.2f, %.2f); pursuer=%s (chases carrier)',
                self._capturer_ns,
                self.homes[self.carrier_ns][0], self.homes[self.carrier_ns][1],
                self.pursuer_ns)

            self.game_state = 'CAPTURED'
            self._flag_captured_pub.publish(Bool(data=True))

        self._enter_post_capture_phase()
        rospy.sleep(self.capture_pause_sec)

        with self._capture_lock:
            if self.game_state == 'CAPTURED':
                self.game_state = 'CHASE'
                self._chase_initialized = False

    def _robot_loop(self, cfg):
        ns = cfg['ns']
        client = self._clients[ns]
        rospy.loginfo('%s loop started', ns)
        consecutive_fails = 0
        idle_no_frontier = 0

        while not self._stop.is_set() and not rospy.is_shutdown():
            if self.game_state == 'FINISHED':
                break

            if self.game_state in ('CHASE', 'CAPTURED'):
                with self._state_lock:
                    if self._search_state.get(ns) != 'IDLE':
                        self._search_state[ns] = 'IDLE'
                        self._clear_pursuit_tracking(ns)
                rospy.sleep(0.2)
                continue

            self._update_search_flag_states()

            with self._state_lock:
                search_state = self._search_state[ns]
                if search_state == 'IDLE':
                    rospy.sleep(0.2)
                    continue

            if search_state == 'PURSUING_FLAG':
                has_own, own_xy = self._has_fresh_flag_estimate(ns)
                if has_own and own_xy is not None:
                    pose = self._get_robot_pose(cfg['base_frame'])
                    if pose is not None:
                        self._send_flag_approach(ns, client, pose, own_xy)

                rospy.sleep(0.2)
                continue

            if search_state == 'APPROACHING_SHARED':
                has_shared, shared_xy = self._get_fresh_flag_estimate(ns)
                if has_shared and shared_xy is not None:
                    pose = self._get_robot_pose(cfg['base_frame'])
                    if pose is not None:
                        self._approach_shared_flag(
                            ns, client, cfg, pose, shared_xy)
                rospy.sleep(0.2)
                continue

            # Standard exploration logic (only while EXPLORING sub-state)
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
                idle_no_frontier += 1
                rospy.loginfo_throttle(15.0, '%s: no frontiers visible (idle=%d)', ns, idle_no_frontier)
                self._spin_to_scan(ns, min(4.0, 1.5 + idle_no_frontier * 0.5))
                continue

            all_poses = self._all_robot_poses()
            # After repeated failures, relax Voronoi partitioning
            use_strict = idle_no_frontier < self.idle_frontier_cycles
            pick = self._pick_frontier(ns, pose, frontiers, all_poses, strict_voronoi=use_strict)
            if pick is None:
                pick = self._pick_closest_frontier(ns, pose, frontiers)
                if pick is not None:
                    rospy.logwarn_throttle(
                        10.0, '%s: voronoi blocked all frontiers; using closest', ns)
            if pick is None:
                idle_no_frontier += 1
                if not use_strict:
                    rospy.logwarn_throttle(10.0,
                        '%s: no frontier even with relaxed Voronoi (idle=%d), spinning', ns, idle_no_frontier)
                else:
                    rospy.logwarn_throttle(10.0,
                        '%s: no assignable frontier (idle=%d/%d until Voronoi relaxed), spinning',
                        ns, idle_no_frontier, self.idle_frontier_cycles)
                self._spin_to_scan(ns, min(3.0, 1.5 + idle_no_frontier * 0.3))
                continue

            idle_no_frontier = 0

            if self.game_state != 'EXPLORING':
                rospy.sleep(0.2)
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
            client.send_goal(self._make_goal(wx, wy, yaw, ns=ns))
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
                    self._blacklist_goal(wx, wy, duration=25.0)
                    rospy.logwarn('%s: exploration goal failed (%d in a row); recovering',
                                  ns, consecutive_fails)
                    self._recover_navigation(ns, spin_sec=1.5)
                    if consecutive_fails >= 3:
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
                self._update_search_flag_states()
                candidate = self._find_first_capture_candidate()
                if candidate is not None:
                    cap_ns, flag_xy = candidate
                    self._capture_flag(cap_ns, flag_xy)

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
