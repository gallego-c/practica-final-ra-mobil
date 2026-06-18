#!/usr/bin/env python3
"""Merge per-robot SLAM maps using TF (same placement as RViz).

multirobot_map_merge caches init_pose only when a robot is first discovered.
On real multi-PC setups that often stays at (0,0,0). This node re-merges every
cycle from map -> robotN/map static TF, so /merged_map aligns with /robotN/map.
"""
import math

import numpy as np
import rospy
import tf2_ros
from nav_msgs.msg import OccupancyGrid


def _yaw_from_quat(q):
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny, cosy)


def _map_to_world(mx, my, tf):
    yaw = _yaw_from_quat(tf.rotation)
    c, s = math.cos(yaw), math.sin(yaw)
    tx = tf.translation.x
    ty = tf.translation.y
    return c * mx - s * my + tx, s * mx + c * my + ty


def _lookup_tf(buffer, world_frame, child_frame, stamp):
    try:
        ts = buffer.lookup_transform(world_frame, child_frame, stamp, rospy.Duration(0.5))
    except tf2_ros.TransformException:
        ts = buffer.lookup_transform(world_frame, child_frame, rospy.Time(0), rospy.Duration(0.5))
    return ts.transform


def _corners_world(grid, tf):
    ox = grid.info.origin.position.x
    oy = grid.info.origin.position.y
    res = grid.info.resolution
    w = grid.info.width * res
    h = grid.info.height * res
    corners = ((ox, oy), (ox + w, oy), (ox, oy + h), (ox + w, oy + h))
    return [_map_to_world(mx, my, tf) for mx, my in corners]


def _merge_grids(grids, transforms, world_frame, resolution):
    if not grids:
        return None

    min_x = min_y = float('inf')
    max_x = max_y = float('-inf')
    for grid, tf in zip(grids, transforms):
        for wx, wy in _corners_world(grid, tf):
            min_x = min(min_x, wx)
            max_x = max(max_x, wx)
            min_y = min(min_y, wy)
            max_y = max(max_y, wy)

    if not math.isfinite(min_x):
        return None

    pad = resolution
    min_x -= pad
    min_y -= pad
    max_x += pad
    max_y += pad

    width = int(math.ceil((max_x - min_x) / resolution))
    height = int(math.ceil((max_y - min_y) / resolution))
    if width <= 0 or height <= 0:
        return None

    merged = np.full((height, width), -1, dtype=np.int16)

    for grid, tf in zip(grids, transforms):
        ox = grid.info.origin.position.x
        oy = grid.info.origin.position.y
        res = grid.info.resolution
        data = np.asarray(grid.data, dtype=np.int16).reshape(grid.info.height, grid.info.width)

        ys, xs = np.nonzero(data >= 0)
        for gy, gx in zip(ys, xs):
            mx = ox + (gx + 0.5) * res
            my = oy + (gy + 0.5) * res
            wx, wy = _map_to_world(mx, my, tf)
            ix = int((wx - min_x) / resolution)
            iy = int((wy - min_y) / resolution)
            if 0 <= ix < width and 0 <= iy < height:
                val = int(data[gy, gx])
                if val > merged[iy, ix]:
                    merged[iy, ix] = val

    out = OccupancyGrid()
    out.header.frame_id = world_frame
    out.header.stamp = rospy.Time.now()
    out.info.resolution = resolution
    out.info.width = width
    out.info.height = height
    out.info.origin.position.x = min_x
    out.info.origin.position.y = min_y
    out.info.origin.position.z = 0.0
    out.info.origin.orientation.w = 1.0
    out.data = merged.flatten().tolist()
    return out


class TfMapMerge(object):
    def __init__(self):
        self.world_frame = rospy.get_param('~world_frame', 'map')
        self.merged_topic = rospy.get_param('~merged_map_topic', '/merged_map')
        self.map_topics = rospy.get_param('~robot_map_topics', ['/robot1/map', '/robot2/map'])
        self.rate_hz = float(rospy.get_param('~rate', 0.5))

        self._maps = {}
        self._tf_buffer = tf2_ros.Buffer()
        tf2_ros.TransformListener(self._tf_buffer)
        self._pub = rospy.Publisher(self.merged_topic, OccupancyGrid, queue_size=1, latch=True)

        for topic in self.map_topics:
            rospy.Subscriber(topic, OccupancyGrid, self._cb_map, callback_args=topic)

        rospy.loginfo('tf_map_merge: world=%s topics=%s', self.world_frame, self.map_topics)
        self._log_tf_offsets_once()

    def _cb_map(self, msg, topic):
        self._maps[topic] = msg

    def _log_tf_offsets_once(self):
        rospy.sleep(1.0)
        for topic in self.map_topics:
            grid = self._maps.get(topic)
            if grid is None:
                continue
            try:
                tf = _lookup_tf(self._tf_buffer, self.world_frame, grid.header.frame_id, rospy.Time(0))
            except tf2_ros.TransformException as ex:
                rospy.logwarn('tf_map_merge: TF %s -> %s: %s',
                              self.world_frame, grid.header.frame_id, ex)
                continue
            yaw = _yaw_from_quat(tf.rotation)
            rospy.loginfo('tf_map_merge: %s TF offset (%.2f, %.2f, yaw=%.2f)',
                          grid.header.frame_id, tf.translation.x, tf.translation.y, yaw)

    def spin(self):
        rate = rospy.Rate(self.rate_hz)
        while not rospy.is_shutdown():
            grids = []
            transforms = []
            stamp = rospy.Time(0)
            for topic in self.map_topics:
                grid = self._maps.get(topic)
                if grid is None or grid.info.width == 0 or grid.info.height == 0:
                    continue
                try:
                    tf = _lookup_tf(self._tf_buffer, self.world_frame, grid.header.frame_id, stamp)
                except tf2_ros.TransformException:
                    continue
                grids.append(grid)
                transforms.append(tf)

            if len(grids) < 1:
                rate.sleep()
                continue

            resolution = min(g.info.resolution for g in grids)
            merged = _merge_grids(grids, transforms, self.world_frame, resolution)
            if merged is not None:
                self._pub.publish(merged)
            rate.sleep()


def main():
    rospy.init_node('tf_map_merge')
    TfMapMerge().spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
