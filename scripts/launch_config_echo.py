#!/usr/bin/env python3
"""Print effective spawn/merge/TF config at startup (debug multi-robot real setup)."""
import math
import rospy
import tf2_ros


def _yaw(q):
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny, cosy)


def main():
    rospy.init_node('launch_config_echo')
    world = rospy.get_param('~world_frame', 'map')
    merge_mode = rospy.get_param('~map_merge_mode', 'unknown')
    robots = rospy.get_param('~robots', ['robot1', 'robot2'])
    rospy.logwarn('=== ctf_navigation real launch config ===')
    rospy.logwarn('map_merge_mode: %s', merge_mode)
    for ns in robots:
        ns = ns.strip('/')
        base = '/' + ns + '/map_merge'
        for key in ('init_pose_x', 'init_pose_y', 'init_pose_yaw'):
            p = base + '/' + key
            if rospy.has_param(p):
                rospy.logwarn('  %s = %s', p, rospy.get_param(p))
            else:
                rospy.logwarn('  %s MISSING', p)
    buffer = tf2_ros.Buffer()
    tf2_ros.TransformListener(buffer)
    rospy.sleep(2.0)
    for ns in robots:
        ns = ns.strip('/')
        try:
            tf = buffer.lookup_transform(world, ns + '/map', rospy.Time(0), rospy.Duration(1.0))
        except tf2_ros.TransformException as ex:
            rospy.logwarn('  TF %s -> %s/map: %s', world, ns, ex)
            continue
        rospy.logwarn('  TF %s -> %s/map: (%.2f, %.2f, yaw=%.2f)',
                      world, ns,
                      tf.transform.translation.x,
                      tf.transform.translation.y,
                      _yaw(tf.transform.rotation))
    rospy.logwarn('=========================================')
    rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
