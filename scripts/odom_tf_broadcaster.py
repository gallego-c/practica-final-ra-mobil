#!/usr/bin/env python3
"""Publish robotN/odom -> robotN/base_footprint from nav_msgs/Odometry.

Use when TurtleBot3 bringup publishes /robotN/odom but not the wheel odometry TF
(common with multi-PC setups or partial bringup).
"""
import rospy
import tf2_ros
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry


def _odom_cb(msg, br):
    t = TransformStamped()
    t.header = msg.header
    t.child_frame_id = msg.child_frame_id
    t.transform.translation.x = msg.pose.pose.position.x
    t.transform.translation.y = msg.pose.pose.position.y
    t.transform.translation.z = msg.pose.pose.position.z
    t.transform.rotation = msg.pose.pose.orientation
    br.sendTransform(t)


def main():
    rospy.init_node('odom_tf_broadcaster')
    robots = rospy.get_param('~robots', ['robot1', 'robot2'])
    br = tf2_ros.TransformBroadcaster()
    for ns in robots:
        ns = ns.strip('/')
        topic = '/' + ns + '/odom'
        rospy.Subscriber(topic, Odometry, _odom_cb, callback_args=br, queue_size=1)
        rospy.loginfo('odom_tf_broadcaster: %s -> %s/odom TF', topic, ns)
    rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
