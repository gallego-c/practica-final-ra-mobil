#!/usr/bin/env python3
"""
Merge the two per-robot gmapping OccupancyGrid maps into a shared /map topic.

This is intentionally a simple starting point for the SLAM branch:
- it assumes both maps are axis-aligned and use the same resolution;
- it merges cells by priority: occupied > free > unknown;
- it does not solve map alignment. The frame bridges in shared_slam.launch are
  the current place to experiment with alignment between robot maps.
"""
import math
from typing import Optional, Tuple

import rospy
from geometry_msgs.msg import Pose
from nav_msgs.msg import MapMetaData, OccupancyGrid
import tf2_ros

UNKNOWN = -1
FREE_MAX = 25
OCCUPIED_MIN = 65


def _yaw_is_zero(pose_or_quaternion) -> bool:
    quaternion = getattr(pose_or_quaternion, 'orientation', pose_or_quaternion)
    return (
        abs(quaternion.x) < 1e-6
        and abs(quaternion.y) < 1e-6
        and abs(quaternion.z) < 1e-6
    )


def _map_bounds(
    msg: OccupancyGrid, origin_x: float, origin_y: float
) -> Tuple[float, float, float, float]:
    width_m = msg.info.width * msg.info.resolution
    height_m = msg.info.height * msg.info.resolution
    return origin_x, origin_y, origin_x + width_m, origin_y + height_m


def _cell_value(current: int, incoming: int) -> int:
    if incoming >= OCCUPIED_MIN:
        return incoming
    if current >= OCCUPIED_MIN:
        return current
    if incoming == UNKNOWN:
        return current
    if current == UNKNOWN:
        return incoming
    if incoming <= FREE_MAX and current <= FREE_MAX:
        return min(current, incoming)
    return max(current, incoming)


class SharedMapMerger:
    def __init__(self):
        self._robot1_map: Optional[OccupancyGrid] = None
        self._robot2_map: Optional[OccupancyGrid] = None

        self._robot1_topic = rospy.get_param('~robot1_map_topic', '/robot1/map')
        self._robot2_topic = rospy.get_param('~robot2_map_topic', '/robot2/map')
        self._merged_topic = rospy.get_param('~merged_map_topic', '/map')
        self._frame_id = rospy.get_param('~frame_id', 'map')
        self._publish_rate = float(rospy.get_param('~publish_rate', 1.0))

        self._tf_buffer = tf2_ros.Buffer()
        tf2_ros.TransformListener(self._tf_buffer)

        self._pub = rospy.Publisher(
            self._merged_topic, OccupancyGrid, queue_size=1, latch=True
        )
        rospy.Subscriber(self._robot1_topic, OccupancyGrid, self._robot1_cb, queue_size=1)
        rospy.Subscriber(self._robot2_topic, OccupancyGrid, self._robot2_cb, queue_size=1)

    def _robot1_cb(self, msg: OccupancyGrid):
        self._robot1_map = msg

    def _robot2_cb(self, msg: OccupancyGrid):
        self._robot2_map = msg

    def spin(self):
        rate = rospy.Rate(self._publish_rate)
        while not rospy.is_shutdown():
            merged = self._merge()
            if merged is not None:
                self._pub.publish(merged)
            rate.sleep()

    def _merge(self) -> Optional[OccupancyGrid]:
        maps = [m for m in (self._robot1_map, self._robot2_map) if m is not None]
        if not maps:
            return None

        resolution = maps[0].info.resolution
        if resolution <= 0.0:
            rospy.logwarn_throttle(5.0, 'shared_map_merger: invalid map resolution')
            return None

        for msg in maps[1:]:
            if abs(msg.info.resolution - resolution) > 1e-6:
                rospy.logwarn_throttle(
                    5.0,
                    'shared_map_merger: map resolutions differ (%.4f vs %.4f)',
                    resolution,
                    msg.info.resolution,
                )
                return None

        map_entries = []
        for msg in maps:
            origin = self._origin_in_shared_frame(msg)
            if origin is None:
                return None
            if not _yaw_is_zero(origin):
                rospy.logwarn_throttle(
                    5.0,
                    'shared_map_merger: rotated map origins are not supported yet',
                )
                return None
            map_entries.append((msg, origin.position.x, origin.position.y))

        min_x = min(_map_bounds(*entry)[0] for entry in map_entries)
        min_y = min(_map_bounds(*entry)[1] for entry in map_entries)
        max_x = max(_map_bounds(*entry)[2] for entry in map_entries)
        max_y = max(_map_bounds(*entry)[3] for entry in map_entries)

        width = int(math.ceil((max_x - min_x) / resolution))
        height = int(math.ceil((max_y - min_y) / resolution))
        if width <= 0 or height <= 0:
            return None

        data = [UNKNOWN] * (width * height)
        for msg, origin_x, origin_y in map_entries:
            self._copy_into_merged(
                msg, origin_x, origin_y, data, width, min_x, min_y, resolution
            )

        merged = OccupancyGrid()
        merged.header.stamp = rospy.Time.now()
        merged.header.frame_id = self._frame_id
        merged.info = MapMetaData()
        merged.info.map_load_time = rospy.Time.now()
        merged.info.resolution = resolution
        merged.info.width = width
        merged.info.height = height
        merged.info.origin.position.x = min_x
        merged.info.origin.position.y = min_y
        merged.info.origin.position.z = 0.0
        merged.info.origin.orientation.w = 1.0
        merged.data = data
        return merged

    def _origin_in_shared_frame(self, msg: OccupancyGrid) -> Optional[Pose]:
        origin = Pose()
        origin.position.x = msg.info.origin.position.x
        origin.position.y = msg.info.origin.position.y
        origin.position.z = msg.info.origin.position.z
        origin.orientation = msg.info.origin.orientation

        source_frame = msg.header.frame_id
        if not source_frame or source_frame == self._frame_id:
            return origin

        try:
            tf = self._tf_buffer.lookup_transform(
                self._frame_id, source_frame, rospy.Time(0), rospy.Duration(0.1)
            )
        except tf2_ros.TransformException as exc:
            rospy.logwarn_throttle(
                5.0,
                'shared_map_merger: missing TF %s -> %s: %s',
                self._frame_id,
                source_frame,
                str(exc),
            )
            return None

        if not _yaw_is_zero(tf.transform.rotation):
            rospy.logwarn_throttle(
                5.0,
                'shared_map_merger: rotated TF bridges are not supported yet',
            )
            return None

        origin.position.x += tf.transform.translation.x
        origin.position.y += tf.transform.translation.y
        origin.position.z += tf.transform.translation.z
        return origin

    def _copy_into_merged(
        self,
        msg: OccupancyGrid,
        origin_x: float,
        origin_y: float,
        merged_data,
        merged_width: int,
        merged_min_x: float,
        merged_min_y: float,
        resolution: float,
    ):
        x_offset = int(round((origin_x - merged_min_x) / resolution))
        y_offset = int(round((origin_y - merged_min_y) / resolution))

        for y in range(msg.info.height):
            src_row = y * msg.info.width
            dst_row = (y + y_offset) * merged_width
            for x in range(msg.info.width):
                dst_index = dst_row + x + x_offset
                src_index = src_row + x
                merged_data[dst_index] = _cell_value(
                    merged_data[dst_index], msg.data[src_index]
                )


def main():
    rospy.init_node('shared_map_merger')
    rospy.loginfo('shared_map_merger: publishing merged map')
    SharedMapMerger().spin()


if __name__ == '__main__':
    main()
