#!/usr/bin/env python3
"""
List or look up pursuer spawn points from params/pursuer_spawn_points.yaml.

Usage:
  # List all predefined spawns (slots 1-10)
  rosrun ctf_navigation pick_pursuer_spawn.py --list

  # Look up slot 4 (prints: x y yaw)
  rosrun ctf_navigation pick_pursuer_spawn.py --slot 4
"""
from __future__ import print_function

import argparse
import sys
from pathlib import Path

import yaml

try:
    import rospkg
except ImportError:
    rospkg = None


def pkg_path():
    if rospkg is not None:
        try:
            return Path(rospkg.RosPack().get_path('ctf_navigation'))
        except Exception:
            pass
    return Path(__file__).resolve().parents[1]


def load_spawns():
    yaml_path = pkg_path() / 'params' / 'pursuer_spawn_points.yaml'
    with open(yaml_path, 'r') as f:
        data = yaml.safe_load(f)
    return data['spawns']


def pick_spawn(slot_1_based):
    spawns = load_spawns()
    if slot_1_based < 1 or slot_1_based > len(spawns):
        raise ValueError('slot must be 1..%d' % len(spawns))
    return spawns[slot_1_based - 1]


def main():
    parser = argparse.ArgumentParser(
        description='List or look up pursuer spawn points (slots 1-10).')
    parser.add_argument('--slot', type=int, default=None,
                        help='Spawn slot 1-10 (prints x y yaw)')
    parser.add_argument('--list', action='store_true',
                        help='List all predefined spawn points')
    args, extras = parser.parse_known_args()

    if args.slot is None and extras:
        try:
            args.slot = int(extras[0])
        except ValueError:
            pass

    if args.list:
        for s in load_spawns():
            print('  [{n}] ({x:.2f}, {y:.2f}) yaw={yaw:.4f}  {label}'.format(
                n=s['id'] + 1, **s))
        return 0

    if args.slot is None:
        parser.print_help()
        return 1

    spawn = pick_spawn(args.slot)
    print('{x:.4f} {y:.4f} {yaw:.4f}'.format(**spawn))
    return 0


if __name__ == '__main__':
    sys.exit(main())
