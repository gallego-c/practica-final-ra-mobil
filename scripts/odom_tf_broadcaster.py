#!/usr/bin/env python3
"""Publish robotN/odom -> robotN/base_footprint from nav_msgs/Odometry.

TurtleBot3 bringup in a namespace often publishes TF with relative frame ids
(odom, base_footprint) while SLAM/navigation expect robotN/odom and
robotN/base_footprint. This node bridges that gap using explicit frame names.
"""
import rospy
import tf2_ros
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry


def _make_cb(ns, br):
    odom_frame = ns + '/odom'
    base_frame = ns + '/base_footprint'

    def _odom_cb(msg):
        t = TransformStamped()
        t.header.stamp = rospy.Time.now()
        t.header.frame_id = odom_frame
        t.child_frame_id = base_frame
        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = msg.pose.pose.position.z
        t.transform.rotation = msg.pose.pose.orientation
        br.sendTransform(t)

    return _odom_cb


def main():
    rospy.init_node('odom_tf_broadcaster')
    robots = rospy.get_param('~robots', ['robot1', 'robot2'])
    br = tf2_ros.TransformBroadcaster()
    for ns in robots:
        ns = ns.strip('/')
        topic = '/' + ns + '/odom'
        rospy.Subscriber(topic, Odometry, _make_cb(ns, br), queue_size=1)
        rospy.loginfo(
            'odom_tf_broadcaster: %s -> TF %s/odom -> %s/base_footprint',
            topic, ns, ns)
    rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
