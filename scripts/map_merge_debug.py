#!/usr/bin/env python3
"""Compare map_merge init_pose params vs static TF map->robotN/map."""
import math
import rospy
import tf2_ros


def _yaw_from_quat(q):
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny, cosy)


def _norm_angle(a):
    return math.atan2(math.sin(a), math.cos(a))


def main():
    rospy.init_node('map_merge_debug')
    world = rospy.get_param('~world_frame', 'map')
    robots = rospy.get_param('~robots', ['robot1', 'robot2'])
    period = float(rospy.get_param('~period', 8.0))
    buffer = tf2_ros.Buffer()
    tf2_ros.TransformListener(buffer)
    rate = rospy.Rate(1.0 / period)

    while not rospy.is_shutdown():
        for ns in robots:
            ns = ns.strip('/')
            base = '/' + ns + '/map_merge'
            missing = [k for k in ('init_pose_x', 'init_pose_y', 'init_pose_yaw')
                       if not rospy.has_param(base + '/' + k)]
            if missing:
                rospy.logwarn('[%s] missing params: %s', ns, missing)
                continue
            ix = float(rospy.get_param(base + '/init_pose_x'))
            iy = float(rospy.get_param(base + '/init_pose_y'))
            iyaw = float(rospy.get_param(base + '/init_pose_yaw'))
            try:
                tf = buffer.lookup_transform(
                    world, ns + '/map', rospy.Time(0), rospy.Duration(0.5))
            except tf2_ros.TransformException as ex:
                rospy.logwarn('[%s] init=(%.2f,%.2f,%.2f) TF failed: %s',
                              ns, ix, iy, iyaw, ex)
                continue
            tx = tf.transform.translation.x
            ty = tf.transform.translation.y
            tyaw = _yaw_from_quat(tf.transform.rotation)
            if (abs(ix - tx) > 0.05 or abs(iy - ty) > 0.05 or
                    abs(_norm_angle(iyaw - tyaw)) > 0.08):
                rospy.logwarn(
                    '[%s] MISMATCH init=(%.2f,%.2f,%.2f) TF=(%.2f,%.2f,%.2f)',
                    ns, ix, iy, iyaw, tx, ty, tyaw)
            else:
                rospy.loginfo_throttle(
                    30.0, '[%s] init_pose matches TF (%.2f, %.2f)', ns, ix, iy)
        rate.sleep()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
