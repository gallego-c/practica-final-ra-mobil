#!/usr/bin/env python3
"""
Publishes each robot's footprint as a PointCloud2 in the 'map' frame so that
the *other* robot's local costmap obstacle layer treats it as a dynamic obstacle.

Topics published:
  /robot1/other_robot_cloud  – contains robot2's footprint  (robot1 avoids robot2)
  /robot2/other_robot_cloud  – contains robot1's footprint  (robot2 avoids robot1)
"""
import math
import struct

import rospy
import tf2_ros
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header

ROBOT_RADIUS = 0.22   # inscribed radius of Waffle Pi footprint (metres)
NUM_POINTS   = 20     # points evenly spaced around the footprint circle
PUBLISH_HZ   = 5.0    # obstacle cloud update rate


def _make_cloud(frame_id: str, cx: float, cy: float) -> PointCloud2:
    """Build a flat PointCloud2 ring around (cx, cy) in *frame_id*."""
    header = Header(stamp=rospy.Time.now(), frame_id=frame_id)
    fields = [
        PointField('x', 0,  PointField.FLOAT32, 1),
        PointField('y', 4,  PointField.FLOAT32, 1),
        PointField('z', 8,  PointField.FLOAT32, 1),
    ]
    point_step = 12
    data = bytearray()
    for i in range(NUM_POINTS):
        angle = 2.0 * math.pi * i / NUM_POINTS
        x = cx + ROBOT_RADIUS * math.cos(angle)
        y = cy + ROBOT_RADIUS * math.sin(angle)
        data += struct.pack('fff', float(x), float(y), 0.0)

    cloud = PointCloud2()
    cloud.header       = header
    cloud.height       = 1
    cloud.width        = NUM_POINTS
    cloud.fields       = fields
    cloud.is_bigendian = False
    cloud.point_step   = point_step
    cloud.row_step     = point_step * NUM_POINTS
    cloud.data         = bytes(data)
    cloud.is_dense     = True
    return cloud


def main():
    rospy.init_node('robot_obstacle_publisher')
    target_frame = rospy.get_param('~target_frame', 'map')

    tf_buffer = tf2_ros.Buffer()
    tf2_ros.TransformListener(tf_buffer)

    # robot1's costmap subscribes to /robot1/other_robot_cloud → robot2 footprint
    # robot2's costmap subscribes to /robot2/other_robot_cloud → robot1 footprint
    pub1 = rospy.Publisher('/robot1/other_robot_cloud', PointCloud2, queue_size=1)
    pub2 = rospy.Publisher('/robot2/other_robot_cloud', PointCloud2, queue_size=1)

    rate = rospy.Rate(PUBLISH_HZ)

    # (source robot whose footprint we publish, publisher that receives it)
    pairs = [('robot2', pub1), ('robot1', pub2)]

    rospy.loginfo('robot_obstacle_publisher: started')

    while not rospy.is_shutdown():
        for source_ns, pub in pairs:
            try:
                tf = tf_buffer.lookup_transform(
                    target_frame,
                    source_ns + '/base_footprint',
                    rospy.Time(0),
                    rospy.Duration(0.15)
                )
                cx = tf.transform.translation.x
                cy = tf.transform.translation.y
                pub.publish(_make_cloud(target_frame, cx, cy))
            except tf2_ros.TransformException as exc:
                rospy.logwarn_throttle(
                    5.0, 'robot_obstacle_publisher: TF lookup failed: %s', str(exc)
                )

        rate.sleep()


if __name__ == '__main__':
    main()
