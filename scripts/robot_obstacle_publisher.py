#!/usr/bin/env python3
"""
Publishes each robot's footprint as a PointCloud2 so the other robot's local
costmap can avoid collisions. During SLAM exploration, obstacles are only
published when robots are close — avoids blocking each other across the map.
"""
import math
import struct

import rospy
import tf2_ros
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header

ROBOT_RADIUS = 0.28
NUM_ANGLES   = 20
NUM_RINGS    = 3
PUBLISH_HZ   = 10.0
MAX_AVOID_DISTANCE = 3.5


def _empty_cloud(frame_id: str) -> PointCloud2:
    cloud = PointCloud2()
    cloud.header = Header(stamp=rospy.Time.now(), frame_id=frame_id)
    cloud.height = 1
    cloud.width = 0
    cloud.fields = [
        PointField('x', 0, PointField.FLOAT32, 1),
        PointField('y', 4, PointField.FLOAT32, 1),
        PointField('z', 8, PointField.FLOAT32, 1),
    ]
    cloud.is_bigendian = False
    cloud.point_step = 12
    cloud.row_step = 0
    cloud.data = b''
    cloud.is_dense = True
    return cloud


def _make_cloud(frame_id: str, cx: float, cy: float, radius: float) -> PointCloud2:
    header = Header(stamp=rospy.Time.now(), frame_id=frame_id)
    fields = [
        PointField('x', 0, PointField.FLOAT32, 1),
        PointField('y', 4, PointField.FLOAT32, 1),
        PointField('z', 8, PointField.FLOAT32, 1),
    ]
    point_step = 12
    data = bytearray()
    points = [(cx, cy)]
    for ring in range(1, NUM_RINGS + 1):
        ring_radius = radius * float(ring) / float(NUM_RINGS)
        for i in range(NUM_ANGLES):
            angle = 2.0 * math.pi * i / NUM_ANGLES
            x = cx + ring_radius * math.cos(angle)
            y = cy + ring_radius * math.sin(angle)
            points.append((x, y))
    for x, y in points:
        data += struct.pack('fff', float(x), float(y), 0.0)

    cloud = PointCloud2()
    cloud.header = header
    cloud.height = 1
    cloud.width = len(points)
    cloud.fields = fields
    cloud.is_bigendian = False
    cloud.point_step = point_step
    cloud.row_step = point_step * len(points)
    cloud.data = bytes(data)
    cloud.is_dense = True
    return cloud


def main():
    rospy.init_node('robot_obstacle_publisher')
    target_frame = rospy.get_param('~target_frame', 'map')
    robot_radius = float(rospy.get_param('~robot_radius', ROBOT_RADIUS))
    max_dist = float(rospy.get_param('~max_avoid_distance', MAX_AVOID_DISTANCE))

    tf_buffer = tf2_ros.Buffer()
    tf2_ros.TransformListener(tf_buffer)

    pub1 = rospy.Publisher('/robot1/other_robot_cloud', PointCloud2, queue_size=1)
    pub2 = rospy.Publisher('/robot2/other_robot_cloud', PointCloud2, queue_size=1)
    rate = rospy.Rate(PUBLISH_HZ)

    pairs = [('robot2', pub1), ('robot1', pub2)]
    rospy.loginfo('robot_obstacle_publisher: radius=%.2f max_dist=%.1f',
                  robot_radius, max_dist)

    while not rospy.is_shutdown():
        poses = {}
        for source_ns, _ in pairs:
            try:
                tf = tf_buffer.lookup_transform(
                    target_frame,
                    source_ns + '/base_footprint',
                    rospy.Time(0),
                    rospy.Duration(0.15))
                poses[source_ns] = (
                    tf.transform.translation.x,
                    tf.transform.translation.y)
            except tf2_ros.TransformException:
                poses[source_ns] = None

        dist = None
        if all(poses.get(ns) for ns in ('robot1', 'robot2')):
            p1 = poses['robot1']
            p2 = poses['robot2']
            dist = math.hypot(p1[0] - p2[0], p1[1] - p2[1])

        for source_ns, pub in pairs:
            if dist is None or dist > max_dist:
                pub.publish(_empty_cloud(target_frame))
                continue
            pos = poses.get(source_ns)
            if pos is None:
                continue
            pub.publish(_make_cloud(target_frame, pos[0], pos[1], robot_radius))

        rate.sleep()


if __name__ == '__main__':
    main()
