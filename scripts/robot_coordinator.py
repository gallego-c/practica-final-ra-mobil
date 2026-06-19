#!/usr/bin/env python3
"""
Robot Coordinator Node
Handles inter-robot collision avoidance (PointCloud2 footprints) and cmd_vel
multiplexing with priority yielding when robots are too close.
"""
import math

import rospy
import tf2_ros
from geometry_msgs.msg import Twist
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Bool

import os as _os, sys as _sys
_d = _os.path.dirname(_os.path.abspath(__file__))
if _d not in _sys.path:
    _sys.path.insert(0, _d)
del _os, _sys, _d

from robot_cloud_utils import empty_cloud, make_footprint_cloud

ROBOT_RADIUS = 0.28
PUBLISH_HZ = 10.0
MAX_AVOID_DISTANCE = 3.5
YIELD_DISTANCE = 1.5
RESUME_DISTANCE = 1.8
YIELD_TIMEOUT = 5.0
YIELD_COOLDOWN = 5.0


class RobotCoordinator:
    def __init__(self):
        rospy.init_node('robot_coordinator')
        self.target_frame = rospy.get_param('~target_frame', 'map')
        self.robot_radius = float(rospy.get_param('~robot_radius', ROBOT_RADIUS))
        self.max_avoid_dist = float(rospy.get_param('~max_avoid_distance', MAX_AVOID_DISTANCE))

        self.tf_buffer = tf2_ros.Buffer()
        tf2_ros.TransformListener(self.tf_buffer)

        self.cloud_pub1 = rospy.Publisher('/robot1/other_robot_cloud', PointCloud2, queue_size=1)
        self.cloud_pub2 = rospy.Publisher('/robot2/other_robot_cloud', PointCloud2, queue_size=1)
        self.cmd_pub1 = rospy.Publisher('/robot1/cmd_vel', Twist, queue_size=1)
        self.cmd_pub2 = rospy.Publisher('/robot2/cmd_vel', Twist, queue_size=1)

        self.yielding_robot = None
        self.yield_start_time = None
        self.yield_cooldown_until = None
        self.flag_captured = False
        self.in_capture_phase = False

        rospy.Subscriber('/robot1/cmd_vel_raw', Twist, self._cmd_cb1, queue_size=1)
        rospy.Subscriber('/robot2/cmd_vel_raw', Twist, self._cmd_cb2, queue_size=1)
        rospy.Subscriber('/ctf/flag_captured', Bool, self._flag_captured_cb, queue_size=1)
        rospy.Subscriber('/ctf/capture_phase', Bool, self._capture_phase_cb, queue_size=1)

        rospy.loginfo('RobotCoordinator: initialized. Priority: robot1 > robot2')

    def _flag_captured_cb(self, msg):
        self.flag_captured = msg.data
        if self.flag_captured:
            self.yielding_robot = None

    def _capture_phase_cb(self, msg):
        self.in_capture_phase = msg.data
        if self.in_capture_phase:
            self.yielding_robot = None

    def _yield_twist(self, ns):
        twist = Twist()
        twist.linear.x = -0.10
        twist.angular.z = -0.4 if ns == 'robot1' else 0.4
        return twist

    def _cmd_cb1(self, msg):
        if self.yielding_robot != 'robot1':
            self.cmd_pub1.publish(msg)
        else:
            self.cmd_pub1.publish(self._yield_twist('robot1'))

    def _cmd_cb2(self, msg):
        if self.yielding_robot != 'robot2':
            self.cmd_pub2.publish(msg)
        else:
            self.cmd_pub2.publish(self._yield_twist('robot2'))

    def run(self):
        rate = rospy.Rate(PUBLISH_HZ)
        pairs = [('robot2', self.cloud_pub1), ('robot1', self.cloud_pub2)]

        while not rospy.is_shutdown():
            poses = {}
            for source_ns, _ in pairs:
                try:
                    tf = self.tf_buffer.lookup_transform(
                        self.target_frame,
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

            if dist is not None and not self.flag_captured and not self.in_capture_phase:
                if self.yielding_robot is None:
                    in_cooldown = (
                        self.yield_cooldown_until
                        and rospy.Time.now() < self.yield_cooldown_until)
                    if dist < YIELD_DISTANCE and not in_cooldown:
                        self.yielding_robot = 'robot2'
                        self.yield_start_time = rospy.Time.now()
                        rospy.logwarn(
                            'Robots too close (%.2fm). robot2 YIELDS to robot1.', dist)
                else:
                    yield_elapsed = (
                        (rospy.Time.now() - self.yield_start_time).to_sec()
                        if self.yield_start_time else 0.0)
                    if dist > RESUME_DISTANCE or yield_elapsed > YIELD_TIMEOUT:
                        is_timeout = dist <= RESUME_DISTANCE
                        reason = 'timeout (%.1fs)' % yield_elapsed if is_timeout else 'separated'
                        rospy.loginfo('robot2 RESUMES (%s, dist=%.2fm).', reason, dist)
                        self.yielding_robot = None
                        self.yield_start_time = None
                        if is_timeout:
                            self.yield_cooldown_until = (
                                rospy.Time.now() + rospy.Duration(YIELD_COOLDOWN))
                            rospy.logwarn('Entering yield cooldown for %.0fs', YIELD_COOLDOWN)
            elif self.flag_captured or self.in_capture_phase:
                self.yielding_robot = None
                self.yield_start_time = None

            if self.yielding_robot == 'robot1':
                self.cmd_pub1.publish(self._yield_twist('robot1'))
            elif self.yielding_robot == 'robot2':
                self.cmd_pub2.publish(self._yield_twist('robot2'))

            for source_ns, pub in pairs:
                if dist is None or dist > self.max_avoid_dist or self.flag_captured:
                    pub.publish(empty_cloud(self.target_frame))
                    continue
                pos = poses.get(source_ns)
                if pos is None:
                    continue
                radius = 0.20 if self.in_capture_phase else self.robot_radius
                pub.publish(make_footprint_cloud(
                    self.target_frame, pos[0], pos[1], radius))

            rate.sleep()


if __name__ == '__main__':
    try:
        RobotCoordinator().run()
    except rospy.ROSInterruptException:
        pass
