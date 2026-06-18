"""Shared frontier exploration logic for multi-robot SLAM explorers."""
from __future__ import division

import math
import time

import rospy
import tf2_ros
from actionlib_msgs.msg import GoalStatus

import ctf_scripts  # noqa: F401
from frontier_map_utils import (
    FREE_THRESH,
    NEIGHBORS_4,
    cell_index,
    coverage,
    is_free,
    map_to_world,
    quat_to_yaw,
    unknown_count,
    world_to_map,
    yaw_to_quaternion,
)


class FrontierExplorationMixin:
    """Mixin providing map, frontier, and navigation helpers."""

    safe_goal_search_meters = 0.8

    def _map_cb(self, msg):
        with self._map_lock:
            self._map = msg

    def _get_map(self):
        with self._map_lock:
            return self._map

    def _coverage(self, grid):
        return coverage(grid)

    def _unknown_count(self, grid):
        return unknown_count(grid)

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
            pose = self._get_robot_pose(cfg['base_frame'])
            if pose is not None:
                poses[cfg['ns']] = pose
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
        search_cells = max(2, int(round(self.safe_goal_search_meters / info.resolution)))

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

    def _make_move_base_goal(self, wx, wy, yaw, frame_id=None):
        qx, qy, qz, qw = yaw_to_quaternion(yaw)
        from move_base_msgs.msg import MoveBaseGoal
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = frame_id or self.map_frame
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = wx
        goal.target_pose.pose.position.y = wy
        goal.target_pose.pose.orientation.x = qx
        goal.target_pose.pose.orientation.y = qy
        goal.target_pose.pose.orientation.z = qz
        goal.target_pose.pose.orientation.w = qw
        return goal

    def _wait_for_goal_basic(self, client, base_frame, goal_xy, timeout):
        """Wait for move_base goal with progress timeout; no CTF preemption."""
        deadline = rospy.Time.now() + rospy.Duration(timeout)
        last_progress = rospy.Time.now()
        pose = self._get_robot_pose(base_frame)
        best_dist = (math.hypot(goal_xy[0] - pose[0], goal_xy[1] - pose[1])
                     if pose is not None else float('inf'))
        rate = rospy.Rate(5.0)
        while rospy.Time.now() < deadline and not rospy.is_shutdown() and not self._stop.is_set():
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
                rospy.logwarn('Goal stalled: no progress for %.1f s', self.progress_timeout)
                client.cancel_goal()
                return False

            rate.sleep()
        client.cancel_goal()
        return False

    def _spin_to_scan_basic(self, ns, duration=3.0, max_duration=None):
        from geometry_msgs.msg import Twist
        pub = self._cmd_pubs[ns]
        twist = Twist()
        twist.angular.z = 0.8
        spin_time = min(duration, max_duration) if max_duration is not None else duration
        end = rospy.Time.now() + rospy.Duration(spin_time)
        rate = rospy.Rate(10)
        while rospy.Time.now() < end and not rospy.is_shutdown() and not self._stop.is_set():
            pub.publish(twist)
            rate.sleep()
        pub.publish(Twist())


def load_robot_homes(defaults=None):
    """Load robot home poses from private params or /ctf_navigation/spawns."""
    defaults = defaults or {
        'robot1': [-3.0, -3.0, 0.0],
        'robot2': [3.0, 3.0, 3.1416],
    }
    homes = {}
    for ns in ('robot1', 'robot2'):
        private = rospy.get_param('~{}_home'.format(ns), None)
        if private is not None:
            homes[ns] = private
            continue
        spawn = rospy.get_param('/ctf_navigation/spawns/' + ns, None)
        if spawn is not None and len(spawn) >= 3:
            homes[ns] = list(spawn[:3])
        else:
            homes[ns] = defaults[ns]
    return homes
