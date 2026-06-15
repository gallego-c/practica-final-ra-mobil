#!/usr/bin/env python3
"""Warn when sensor/odom message stamps drift from the roslaunch PC clock.

Large drift (>5 s) after NTP usually means ROS was not restarted after the
clock step. Kill all nodes on all machines and relaunch.
"""
import rospy
from nav_msgs.msg import Odometry


class ClockDriftCheck:
    def __init__(self):
        rospy.init_node('clock_drift_check')
        self.max_drift = float(rospy.get_param('~max_drift_sec', 5.0))
        robots = rospy.get_param('~robots', ['robot1', 'robot2'])
        for ns in robots:
            topic = '/' + ns.strip('/') + '/odom'
            rospy.Subscriber(topic, Odometry, self._odom_cb, callback_args=ns, queue_size=1)
            rospy.loginfo('clock_drift_check: monitoring %s (max %.1f s)', topic, self.max_drift)

    def _odom_cb(self, msg, ns):
        drift = abs((rospy.Time.now() - msg.header.stamp).to_sec())
        if drift > self.max_drift:
            rospy.logwarn_throttle(
                15.0,
                '[CLOCK] %s odom stamp %.0fs off vs this PC — '
                'stop ALL ROS on all machines, verify NTP, relaunch',
                ns, drift)


if __name__ == '__main__':
    try:
        ClockDriftCheck()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
