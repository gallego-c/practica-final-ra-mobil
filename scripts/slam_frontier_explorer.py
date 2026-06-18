#!/usr/bin/env python3
"""
Multi-robot frontier exploration on the merged SLAM map.

Goal: maximize mapped area (unknown -> free/occupied), not reach the flag.
Each robot runs its own exploration loop and picks frontiers in a Voronoi-like
partition so they spread out instead of crowding the centre.
"""
from __future__ import division

import math
import threading

import actionlib
import rospy
import tf2_ros
from geometry_msgs.msg import Twist
from move_base_msgs.msg import MoveBaseAction
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Bool

import ctf_scripts  # noqa: F401
from frontier_exploration_mixin import FrontierExplorationMixin


class SlamFrontierExplorer(FrontierExplorationMixin):
    safe_goal_search_meters = 0.8

    def __init__(self):
        rospy.init_node('slam_frontier_explorer')

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

        self.robots = rospy.get_param('~robots', [
            {'ns': 'robot1', 'base_frame': 'robot1/base_footprint'},
            {'ns': 'robot2', 'base_frame': 'robot2/base_footprint'},
        ])

        self._map = None
        self._map_lock = threading.Lock()
        self._stop = threading.Event()
        self._blacklist = {}
        self._blacklist_lock = threading.Lock()
        self._stats_lock = threading.Lock()
        self._idle_cycles = 0
        self._last_coverage = 0.0

        rospy.Subscriber(self.map_topic, OccupancyGrid, self._map_cb, queue_size=1)
        self._done_pub = rospy.Publisher(
            '/slam_exploration/complete', Bool, queue_size=1, latch=True)

        self._tf_buffer = tf2_ros.Buffer()
        tf2_ros.TransformListener(self._tf_buffer)

        self._clients = {}
        self._cmd_pubs = {}
        for cfg in self.robots:
            ns = cfg['ns']
            self._clients[ns] = actionlib.SimpleActionClient(
                '/' + ns + '/move_base', MoveBaseAction)
            self._cmd_pubs[ns] = rospy.Publisher(
                '/' + ns + '/cmd_vel', Twist, queue_size=1)

        rospy.loginfo('Waiting %.0f s for navigation stack...', self.startup_delay)
        rospy.sleep(self.startup_delay)
        if not self._wait_for_all_move_base():
            raise rospy.ROSException('move_base not ready for all robots')

        rospy.loginfo('Frontier explorer ready on %s', self.map_topic)

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
        return self._make_move_base_goal(wx, wy, yaw)

    def _wait_for_goal(self, client, base_frame, goal_xy, timeout):
        return self._wait_for_goal_basic(client, base_frame, goal_xy, timeout)

    def _spin_to_scan(self, ns, duration=3.0):
        self._spin_to_scan_basic(ns, duration=duration)

    def _robot_loop(self, cfg):
        ns = cfg['ns']
        client = self._clients[ns]
        rospy.loginfo('%s exploration loop started', ns)
        consecutive_fails = 0

        while not self._stop.is_set() and not rospy.is_shutdown():
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
            ok = self._wait_for_goal(client, cfg['base_frame'], (wx, wy), self.goal_timeout)

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
        rospy.loginfo('%s exploration loop stopped', ns)

    def run(self):
        rospy.loginfo(
            'Max-coverage frontier exploration (timeout=%.0fs, target coverage=%.0f%%)',
            self.exploration_timeout, self.min_coverage * 100.0)

        threads = []
        for cfg in self.robots:
            t = threading.Thread(target=self._robot_loop, args=(cfg,))
            t.daemon = True
            t.start()
            threads.append(t)

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
        for t in threads:
            t.join(timeout=5.0)
        for client in self._clients.values():
            client.cancel_all_goals()

        self._done_pub.publish(Bool(data=True))
        rospy.loginfo('Frontier exploration finished (coverage=%.1f%%)',
                      self._last_coverage * 100.0)


def main():
    try:
        SlamFrontierExplorer().run()
    except rospy.ROSInterruptException:
        pass


if __name__ == '__main__':
    main()
