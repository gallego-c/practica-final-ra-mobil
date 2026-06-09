#!/usr/bin/env python3
"""
random_free_spawn.py
--------------------
Helper script that finds a random collision-free spawn position on the CTF map
for the pursuer robot.

Usage (static map, after map_server is running):
  rosrun ctf_navigation random_free_spawn.py

Usage (SLAM mode, after map_merge is publishing /merged_map):
  rosrun ctf_navigation random_free_spawn.py _map_topic:=/merged_map

The script will print a ready-to-use roslaunch argument string such as:
  pursuer_x:=1.52 pursuer_y:=-2.10

You can also source it inline:
  SPAWN=$(rosrun ctf_navigation random_free_spawn.py --raw)
  roslaunch ctf_navigation ctf_chase_test_static.launch $SPAWN
"""
from __future__ import print_function

import argparse
import math
import random
import sys

import rospy
from nav_msgs.msg import OccupancyGrid


# ── Constants ─────────────────────────────────────────────────────────────────

# Map cells with occupancy value below this threshold are considered free.
FREE_THRESH = 25
# Map cells with occupancy value above this threshold are considered occupied.
OCC_THRESH = 65
# Unknown cells have value -1.

# Minimum distance (metres) between the chosen spawn and any exclusion zone.
EXCLUSION_RADIUS = 1.5

# Obstacle inflation radius (metres): cells closer than this to an occupied
# cell are discarded.
INFLATION_RADIUS = 0.35

# Default exclusion zone centres: flag, carrier spawn, both home bases.
DEFAULT_EXCLUSIONS = [
    (-3.2,  3.2),   # flag
    (-3.0,  3.0),   # carrier spawn (near flag)
    (-3.0, -3.0),   # robot1 home base
    ( 3.0,  3.0),   # robot2 home base
]


# ── Map helpers ───────────────────────────────────────────────────────────────

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


def is_near_excluded(wx, wy, exclusions, radius):
    for ex, ey in exclusions:
        if math.hypot(wx - ex, wy - ey) < radius:
            return True
    return False


def find_free_cells(grid, inflation_cells):
    """Return list of (mx, my) map cells that are free and clear of obstacles."""
    info = grid.info
    w, h = info.width, info.height
    data = grid.data

    def occ(mx, my):
        if mx < 0 or my < 0 or mx >= w or my >= h:
            return 100
        return data[cell_index(mx, my, w)]

    free = []
    for my in range(inflation_cells, h - inflation_cells):
        for mx in range(inflation_cells, w - inflation_cells):
            v = occ(mx, my)
            if v < 0 or v >= FREE_THRESH:
                continue
            # Check inflation square: skip if any neighbour is occupied or unknown.
            clear = True
            for dy in range(-inflation_cells, inflation_cells + 1):
                for dx in range(-inflation_cells, inflation_cells + 1):
                    nv = occ(mx + dx, my + dy)
                    if nv < 0 or nv >= OCC_THRESH:
                        clear = False
                        break
                if not clear:
                    break
            if clear:
                free.append((mx, my))
    return free


def pick_random_spawn(grid, exclusions, inflation_radius, exclusion_radius,
                      max_attempts=5000, seed=None):
    """Return (wx, wy) of a valid random spawn, or None if not found."""
    info = grid.info
    inflation_cells = max(1, int(math.ceil(inflation_radius / info.resolution)))

    rospy.loginfo('Scanning map %dx%d (res=%.3f m) for free cells '
                  '(inflation=%.2f m = %d cells)...',
                  info.width, info.height, info.resolution,
                  inflation_radius, inflation_cells)

    free_cells = find_free_cells(grid, inflation_cells)
    rospy.loginfo('Found %d candidate free cells', len(free_cells))

    if not free_cells:
        rospy.logerr('No free cells found – check map and inflation_radius')
        return None

    if seed is not None:
        random.seed(seed)
    random.shuffle(free_cells)

    for mx, my in free_cells[:max_attempts]:
        wx, wy = map_to_world(mx, my, info)
        if not is_near_excluded(wx, wy, exclusions, exclusion_radius):
            return wx, wy

    rospy.logwarn('Could not find a cell outside all exclusion zones; '
                  'relaxing to nearest valid cell')
    for mx, my in free_cells[:max_attempts]:
        wx, wy = map_to_world(mx, my, info)
        return wx, wy

    return None


# ── Main ──────────────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(
        description='Pick a random collision-free spawn for the pursuer robot.',
        add_help=False)
    parser.add_argument('--raw', action='store_true',
                        help='Print only "pursuer_x:=X pursuer_y:=Y" (no extra text)')
    parser.add_argument('--seed', type=int, default=None,
                        help='Random seed for reproducibility')
    parser.add_argument('--help', action='store_true')
    # ROS private params are passed via _param:=value syntax; collect but ignore here.
    args, _ = parser.parse_known_args()

    if args.help:
        parser.print_help()
        sys.exit(0)

    rospy.init_node('random_free_spawn', anonymous=True)

    map_topic = rospy.get_param('~map_topic', '/map')
    inflation_radius = float(rospy.get_param('~inflation_radius', INFLATION_RADIUS))
    exclusion_radius = float(rospy.get_param('~exclusion_radius', EXCLUSION_RADIUS))
    seed = args.seed if args.seed is not None else rospy.get_param('~seed', -1)
    if seed < 0:
        seed = None

    # Extra exclusion centres can be added via ROS param (list of [x, y] pairs).
    extra_excl = rospy.get_param('~extra_exclusions', [])
    exclusions = list(DEFAULT_EXCLUSIONS)
    for pt in extra_excl:
        if len(pt) >= 2:
            exclusions.append((float(pt[0]), float(pt[1])))

    rospy.loginfo('Waiting for map on %s...', map_topic)
    try:
        grid = rospy.wait_for_message(map_topic, OccupancyGrid, timeout=30.0)
    except rospy.ROSException:
        rospy.logerr('Timed out waiting for map on %s. '
                     'Make sure map_server (or map_merge) is running.', map_topic)
        sys.exit(1)

    rospy.loginfo('Map received (%dx%d, res=%.3f m)',
                  grid.info.width, grid.info.height, grid.info.resolution)

    result = pick_random_spawn(grid, exclusions, inflation_radius, exclusion_radius,
                               seed=seed)
    if result is None:
        rospy.logerr('Failed to find a valid spawn position')
        sys.exit(1)

    wx, wy = result

    if args.raw:
        print('pursuer_x:={:.4f} pursuer_y:={:.4f}'.format(wx, wy))
    else:
        rospy.loginfo('Random pursuer spawn: (%.4f, %.4f)', wx, wy)
        print()
        print('  pursuer_x:={:.4f} pursuer_y:={:.4f}'.format(wx, wy))
        print()
        print('Use it like:')
        print('  roslaunch ctf_navigation ctf_chase_test_static.launch '
              'pursuer_x:={:.4f} pursuer_y:={:.4f}'.format(wx, wy))
        print('  roslaunch ctf_navigation ctf_chase_test_slam.launch '
              'pursuer_x:={:.4f} pursuer_y:={:.4f}'.format(wx, wy))


if __name__ == '__main__':
    main()
