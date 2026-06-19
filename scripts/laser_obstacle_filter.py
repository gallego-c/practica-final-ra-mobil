#!/usr/bin/env python3
"""
Laser Obstacle Filter Node
Subscribes to raw scan, filters out scan points that fall within the other robot's bounding circle,
and publishes the filtered scan to be used by GMapping and move_base.
"""
import math
import rospy
import tf2_ros
from sensor_msgs.msg import LaserScan

class LaserObstacleFilter:
    def __init__(self):
        rospy.init_node('laser_obstacle_filter', anonymous=True)
        self.obstacle_frame = rospy.get_param('~obstacle_frame', '')
        self.robot_radius = float(rospy.get_param('~robot_radius', 0.32))

        if not self.obstacle_frame:
            rospy.logerr("LaserObstacleFilter: obstacle_frame parameter is required!")
            return

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.pub = rospy.Publisher('scan_filtered', LaserScan, queue_size=1)
        self.sub = rospy.Subscriber('scan', LaserScan, self.scan_cb, queue_size=1)
        rospy.loginfo("LaserObstacleFilter initialized. Obstacle: %s, Radius: %.2f", 
                       self.obstacle_frame, self.robot_radius)

    def scan_cb(self, msg):
        try:
            tf = self.tf_buffer.lookup_transform(
                msg.header.frame_id,
                self.obstacle_frame,
                rospy.Time(0),
                rospy.Duration(0.05)
            )
            tx = tf.transform.translation.x
            ty = tf.transform.translation.y

            dist = math.hypot(tx, ty)

            if dist > 4.0:
                self.pub.publish(msg)
                return

            angle_to_obstacle = math.atan2(ty, tx)

            if dist > self.robot_radius:
                half_angle = math.asin(self.robot_radius / dist)
            else:
                half_angle = math.pi  # robot overlap: blank out full scan

            new_ranges = list(msg.ranges)
            angle_min = msg.angle_min
            angle_inc = msg.angle_increment
            max_filter_dist = dist + self.robot_radius + 0.10

            for i in range(len(new_ranges)):
                beam_angle = angle_min + i * angle_inc
                # normalize to [-pi, pi]
                diff = (beam_angle - angle_to_obstacle + math.pi) % (2.0 * math.pi) - math.pi
                if abs(diff) <= half_angle and new_ranges[i] <= max_filter_dist:
                    new_ranges[i] = float('inf')

            msg.ranges = new_ranges

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            pass  # TF not ready yet; publish original scan

        self.pub.publish(msg)

if __name__ == '__main__':
    try:
        LaserObstacleFilter()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
