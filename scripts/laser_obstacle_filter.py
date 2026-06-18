#!/usr/bin/env python3
"""
Laser Obstacle Filter Node
Subscribes to raw scan, filters out scan points that fall within the other robot's
bounding circle, and publishes the filtered scan for GMapping and move_base.

Uses spawn + wheel odometry for inter-robot geometry when enabled, so filtering
stays correct even if gmapping map->odom drifts.
"""
import math
import rospy
import tf2_ros
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan

# TurtleBot3 Waffle Pi: base_link -> base_scan (matches real_robot_tf.launch)
LIDAR_X = -0.064
LIDAR_Y = 0.0


def _yaw_from_quat(q):
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny, cosy)


def _spawn_odom_to_map(spawn, odom_msg):
    x_o = odom_msg.pose.pose.position.x
    y_o = odom_msg.pose.pose.position.y
    yaw_o = _yaw_from_quat(odom_msg.pose.pose.orientation)
    sx, sy, syaw = spawn
    cos_y = math.cos(syaw)
    sin_y = math.sin(syaw)
    mx = sx + cos_y * x_o - sin_y * y_o
    my = sy + sin_y * x_o + cos_y * y_o
    myaw = syaw + yaw_o
    return mx, my, myaw


def _other_in_scan_frame(self_spawn, self_odom, other_spawn, other_odom, lidar_yaw):
    """Other robot center in this robot's base_scan frame."""
    mx1, my1, yaw1 = _spawn_odom_to_map(self_spawn, self_odom)
    mx2, my2, _ = _spawn_odom_to_map(other_spawn, other_odom)
    dx = mx2 - mx1
    dy = my2 - my1
    cos_y = math.cos(-yaw1)
    sin_y = math.sin(-yaw1)
    ox = cos_y * dx - sin_y * dy
    oy = sin_y * dx + cos_y * dy
    # base_footprint -> base_link (z only) -> base_scan
    cos_l = math.cos(lidar_yaw)
    sin_l = math.sin(lidar_yaw)
    sx = cos_l * (ox - LIDAR_X) - sin_l * (oy - LIDAR_Y)
    sy = sin_l * (ox - LIDAR_X) + cos_l * (oy - LIDAR_Y)
    return sx, sy


class LaserObstacleFilter:
    def __init__(self):
        rospy.init_node('laser_obstacle_filter', anonymous=True)
        self.obstacle_frame = rospy.get_param('~obstacle_frame', '')
        self.robot_radius = float(rospy.get_param('~robot_radius', 0.32))
        self.use_odom_relative = bool(rospy.get_param('~use_odom_relative', True))
        self.lidar_yaw = float(rospy.get_param('~lidar_yaw', 0.0))

        ns = rospy.get_namespace().strip('/')
        self.self_ns = ns or 'robot1'
        self.other_ns = 'robot2' if self.self_ns == 'robot1' else 'robot1'
        self.spawn = {
            'robot1': rospy.get_param('~robot1_spawn', [-1.5, 0.0, 0.0]),
            'robot2': rospy.get_param('~robot2_spawn', [1.5, 0.0, 3.1416]),
        }
        self.odom = {'robot1': None, 'robot2': None}

        if not self.obstacle_frame:
            rospy.logerr('LaserObstacleFilter: obstacle_frame parameter is required!')
            return

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.pub = rospy.Publisher('scan_filtered', LaserScan, queue_size=1)
        self.sub = rospy.Subscriber('scan', LaserScan, self.scan_cb, queue_size=1)
        rospy.Subscriber('/robot1/odom', Odometry, self._odom1_cb, queue_size=5)
        rospy.Subscriber('/robot2/odom', Odometry, self._odom2_cb, queue_size=5)

        mode = 'spawn+wheel odom' if self.use_odom_relative else 'TF'
        rospy.loginfo(
            'LaserObstacleFilter: %s, other=%s, mode=%s',
            self.self_ns, self.obstacle_frame, mode)

    def _odom1_cb(self, msg):
        self.odom['robot1'] = msg

    def _odom2_cb(self, msg):
        self.odom['robot2'] = msg

    def _other_xy_in_scan(self):
        if self.use_odom_relative:
            self_odom = self.odom.get(self.self_ns)
            other_odom = self.odom.get(self.other_ns)
            if self_odom and other_odom:
                return _other_in_scan_frame(
                    self.spawn[self.self_ns], self_odom,
                    self.spawn[self.other_ns], other_odom,
                    self.lidar_yaw)
        try:
            tf = self.tf_buffer.lookup_transform(
                self.self_ns + '/base_scan',
                self.obstacle_frame,
                rospy.Time(0),
                rospy.Duration(0.2))
            return tf.transform.translation.x, tf.transform.translation.y
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException):
            return None

    def scan_cb(self, msg):
        ns = rospy.get_namespace().strip('/')
        if ns and not msg.header.frame_id.startswith(ns + '/'):
            clean_frame = msg.header.frame_id.lstrip('/')
            msg.header.frame_id = '%s/%s' % (ns, clean_frame)

        other_xy = self._other_xy_in_scan()
        if other_xy is None:
            self.pub.publish(msg)
            return

        tx, ty = other_xy
        dist = math.hypot(tx, ty)

        if dist > 4.0:
            self.pub.publish(msg)
            return

        angle_to_obstacle = math.atan2(ty, tx)
        if dist > self.robot_radius:
            half_angle = math.asin(min(1.0, self.robot_radius / dist))
        else:
            half_angle = math.pi

        new_ranges = list(msg.ranges)
        angle_min = msg.angle_min
        angle_inc = msg.angle_increment
        max_filter_dist = dist + self.robot_radius + 0.12

        for i in range(len(new_ranges)):
            beam_angle = angle_min + i * angle_inc
            diff = beam_angle - angle_to_obstacle
            diff = (diff + math.pi) % (2.0 * math.pi) - math.pi
            if abs(diff) <= half_angle and new_ranges[i] <= max_filter_dist:
                new_ranges[i] = float('inf')

        msg.ranges = new_ranges
        self.pub.publish(msg)


if __name__ == '__main__':
    try:
        LaserObstacleFilter()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
