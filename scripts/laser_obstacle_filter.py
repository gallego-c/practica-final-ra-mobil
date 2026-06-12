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
        # Asegurar que el frame_id del escaneo incluya el namespace del robot (ej. base_scan -> robot1/base_scan)
        # Esto corrige el problema si olvidaron pasar set_lidar_frame_id en el bringup del robot.
        ns = rospy.get_namespace().strip('/')
        if ns and not msg.header.frame_id.startswith(ns + '/'):
            clean_frame = msg.header.frame_id.lstrip('/')
            msg.header.frame_id = f"{ns}/{clean_frame}"

        try:
            # Lookup transform from scan frame to the other robot's footprint
            tf = self.tf_buffer.lookup_transform(
                msg.header.frame_id,
                self.obstacle_frame,
                rospy.Time(0),
                rospy.Duration(0.05)
            )
            tx = tf.transform.translation.x
            ty = tf.transform.translation.y

            dist = math.hypot(tx, ty)
            
            # If the other robot is too far, don't filter anything
            if dist > 4.0:
                self.pub.publish(msg)
                return

            angle_to_obstacle = math.atan2(ty, tx)

            # Calculate the angular width of the other robot's footprint
            if dist > self.robot_radius:
                half_angle = math.asin(self.robot_radius / dist)
            else:
                half_angle = math.pi  # Overlap, filter everything

            # Create a mutable copy of ranges
            new_ranges = list(msg.ranges)
            
            angle_min = msg.angle_min
            angle_inc = msg.angle_increment
            max_filter_dist = dist + self.robot_radius + 0.10

            for i in range(len(new_ranges)):
                beam_angle = angle_min + i * angle_inc
                
                # Normalize angle difference to [-pi, pi]
                diff = beam_angle - angle_to_obstacle
                diff = (diff + math.pi) % (2.0 * math.pi) - math.pi

                if abs(diff) <= half_angle:
                    # Filter out scan returns that are close to or hit the other robot
                    if new_ranges[i] <= max_filter_dist:
                        new_ranges[i] = float('inf')

            msg.ranges = new_ranges

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            # If TF is not available yet, publish original scan for safety
            pass

        self.pub.publish(msg)

if __name__ == '__main__':
    try:
        LaserObstacleFilter()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
