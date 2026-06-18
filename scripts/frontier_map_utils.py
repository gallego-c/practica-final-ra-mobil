"""Occupancy-grid helpers shared by frontier exploration nodes."""
from __future__ import division

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


def coverage(grid):
    if grid is None or not grid.data:
        return 0.0
    known = sum(1 for v in grid.data if v >= 0)
    return float(known) / float(len(grid.data))


def unknown_count(grid):
    if grid is None:
        return 10 ** 9
    return sum(1 for v in grid.data if v < 0)
