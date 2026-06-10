#!/usr/bin/env python3
"""Validate predefined pursuer spawn points against ctf_map.pgm."""
from __future__ import print_function

import math
import sys
from pathlib import Path

import yaml

try:
    import rospkg
except ImportError:
    rospkg = None

RESOLUTION = 0.05
ORIGIN_X = -5.0
ORIGIN_Y = -5.0
FREE_THRESH = 205  # PGM: 254=free, 0=occupied
CLEARANCE_M = 0.75


def load_pgm(path):
    with open(path, 'r') as f:
        magic = f.readline().strip()
        if magic != 'P2':
            raise ValueError('expected P2 PGM')
        line = f.readline()
        while line.startswith('#'):
            line = f.readline()
        w, h = map(int, line.split())
        maxval = int(f.readline().strip())
        data = list(map(int, f.read().split()))
    return w, h, data


def world_to_map(wx, wy, width, height):
    mx = int((wx - ORIGIN_X) / RESOLUTION)
    my = int((wy - ORIGIN_Y) / RESOLUTION)
    my = height - 1 - my
    return mx, my


def map_to_world(mx, my, height):
    row_from_bottom = height - 1 - my
    wx = ORIGIN_X + (mx + 0.5) * RESOLUTION
    wy = ORIGIN_Y + (row_from_bottom + 0.5) * RESOLUTION
    return wx, wy


def cell_value(data, width, mx, my):
    if mx < 0 or my < 0 or mx >= width:
        return 0
    return data[my * width + mx]


def min_obstacle_distance(data, width, height, mx, my):
    """Euclidean distance (m) from map cell centre to nearest occupied cell."""
    wx, wy = map_to_world(mx, my, height)
    best = float('inf')
    radius_cells = int(math.ceil(CLEARANCE_M / RESOLUTION)) + 2
    for dy in range(-radius_cells, radius_cells + 1):
        for dx in range(-radius_cells, radius_cells + 1):
            nx, ny = mx + dx, my + dy
            if cell_value(data, width, nx, ny) < FREE_THRESH:
                nwx, nwy = map_to_world(nx, ny, height)
                d = math.hypot(wx - nwx, wy - nwy)
                if d < best:
                    best = d
    return best


def validate_spawn(data, width, height, spawn):
    wx, wy = spawn['x'], spawn['y']
    mx, my = world_to_map(wx, wy, width, height)
    v = cell_value(data, width, mx, my)
    clearance = min_obstacle_distance(data, width, height, mx, my)
    ok = v >= FREE_THRESH and clearance >= CLEARANCE_M
    return ok, clearance, v


def pkg_path():
    if rospkg is not None:
        try:
            return Path(rospkg.RosPack().get_path('ctf_navigation'))
        except Exception:
            pass
    return Path(__file__).resolve().parents[1]


def main():
    pkg = pkg_path()
    pgm_path = pkg / 'maps' / 'ctf_map.pgm'
    yaml_path = pkg / 'params' / 'pursuer_spawn_points.yaml'

    width, height, data = load_pgm(str(pgm_path))
    with open(yaml_path, 'r') as f:
        spawns = yaml.safe_load(f)['spawns']

    all_ok = True
    for s in spawns:
        ok, clearance, val = validate_spawn(data, width, height, s)
        status = 'OK' if ok else 'FAIL'
        print('[{status}] id={id} ({x:.2f},{y:.2f}) cell={cell} clearance={clr:.3f}m  {label}'.format(
            status=status, clr=clearance, cell=val, **s))
        if not ok:
            all_ok = False

    return 0 if all_ok else 1


if __name__ == '__main__':
    sys.exit(main())
