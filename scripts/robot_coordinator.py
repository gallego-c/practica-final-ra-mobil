#!/usr/bin/env python3
"""
Robot Coordinator Node
Handles both:
1. Inter-robot collision avoidance via PointCloud2 footprint publishing.
2. Twist multiplexing and priority yielding when robots are too close.

Inter-robot positions use spawn pose + wheel odometry (not gmapping map->odom),
so costmaps stay stable when independent SLAM maps drift apart.
"""
import math
import struct
import rospy
import tf2_ros
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2, PointField
from geometry_msgs.msg import Twist
from std_msgs.msg import Header, Bool

ROBOT_RADIUS = 0.28
NUM_ANGLES   = 20
NUM_RINGS    = 3
PUBLISH_HZ   = 20.0
MAX_AVOID_DISTANCE = 3.5

YIELD_DISTANCE = 1.5
RESUME_DISTANCE = 1.8
YIELD_TIMEOUT = 5.0
YIELD_COOLDOWN = 5.0


def _yaw_from_quat(q):
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny, cosy)


def _empty_cloud(frame_id: str) -> PointCloud2:
    cloud = PointCloud2()
    cloud.header = Header(stamp=rospy.Time.now(), frame_id=frame_id)
    cloud.height = 1
    cloud.width = 0
    cloud.fields = [
        PointField('x', 0, PointField.FLOAT32, 1),
        PointField('y', 4, PointField.FLOAT32, 1),
        PointField('z', 8, PointField.FLOAT32, 1),
    ]
    cloud.is_bigendian = False
    cloud.point_step = 12
    cloud.row_step = 0
    cloud.data = b''
    cloud.is_dense = True
    return cloud


def _make_cloud(frame_id: str, cx: float, cy: float, radius: float) -> PointCloud2:
    header = Header(stamp=rospy.Time.now(), frame_id=frame_id)
    fields = [
        PointField('x', 0, PointField.FLOAT32, 1),
        PointField('y', 4, PointField.FLOAT32, 1),
        PointField('z', 8, PointField.FLOAT32, 1),
    ]
    point_step = 12
    data = bytearray()
    points = [(cx, cy)]
    for ring in range(1, NUM_RINGS + 1):
        ring_radius = radius * float(ring) / float(NUM_RINGS)
        for i in range(NUM_ANGLES):
            angle = 2.0 * math.pi * i / NUM_ANGLES
            x = cx + ring_radius * math.cos(angle)
            y = cy + ring_radius * math.sin(angle)
            points.append((x, y))
    for x, y in points:
        data += struct.pack('fff', float(x), float(y), 0.0)

    cloud = PointCloud2()
    cloud.header = header
    cloud.height = 1
    cloud.width = len(points)
    cloud.fields = fields
    cloud.is_bigendian = False
    cloud.point_step = point_step
    cloud.row_step = point_step * len(points)
    cloud.data = bytes(data)
    cloud.is_dense = True
    return cloud


def _spawn_odom_to_map(spawn, odom_msg):
    """Map pose from known spawn + wheel odom (ignores gmapping drift)."""
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


def _other_in_robot_odom(self_spawn, self_odom, other_spawn, other_odom):
    """Position of other robot base in this robot's odom frame."""
    mx1, my1, yaw1 = _spawn_odom_to_map(self_spawn, self_odom)
    mx2, my2, _ = _spawn_odom_to_map(other_spawn, other_odom)
    dx = mx2 - mx1
    dy = my2 - my1
    cos_y = math.cos(-yaw1)
    sin_y = math.sin(-yaw1)
    ox = cos_y * dx - sin_y * dy
    oy = sin_y * dx + cos_y * dy
    return ox, oy


class RobotCoordinator:
    def __init__(self):
        rospy.init_node('robot_coordinator')
        self.target_frame = rospy.get_param('~target_frame', 'map')
        self.robot_radius = float(rospy.get_param('~robot_radius', ROBOT_RADIUS))
        self.max_avoid_dist = float(rospy.get_param('~max_avoid_distance', MAX_AVOID_DISTANCE))
        self.use_odom_relative = bool(rospy.get_param('~use_odom_relative', True))

        self.spawn = {
            'robot1': rospy.get_param('~robot1_spawn', [-1.5, 0.0, 0.0]),
            'robot2': rospy.get_param('~robot2_spawn', [1.5, 0.0, 0.0]),
        }
        self.odom = {'robot1': None, 'robot2': None}

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

        self.raw_cmd1 = Twist()
        self.raw_cmd2 = Twist()

        rospy.Subscriber('/robot1/odom', Odometry, self._odom_cb1, queue_size=5)
        rospy.Subscriber('/robot2/odom', Odometry, self._odom_cb2, queue_size=5)
        self.cmd_sub1 = rospy.Subscriber('/robot1/cmd_vel_raw', Twist, self.cmd_cb1, queue_size=1)
        self.cmd_sub2 = rospy.Subscriber('/robot2/cmd_vel_raw', Twist, self.cmd_cb2, queue_size=1)
        self.flag_captured_sub = rospy.Subscriber('/ctf/flag_captured', Bool, self.flag_captured_cb, queue_size=1)
        self.capture_phase_sub = rospy.Subscriber('/ctf/capture_phase', Bool, self.capture_phase_cb, queue_size=1)

        mode = 'spawn+wheel odom' if self.use_odom_relative else 'TF map frame'
        rospy.loginfo('RobotCoordinator: inter-robot pose via %s', mode)

    def _odom_cb1(self, msg):
        self.odom['robot1'] = msg

    def _odom_cb2(self, msg):
        self.odom['robot2'] = msg

    def flag_captured_cb(self, msg):
        self.flag_captured = msg.data
        if self.flag_captured:
            self.yielding_robot = None

    def capture_phase_cb(self, msg):
        self.in_capture_phase = msg.data
        if self.in_capture_phase:
            self.yielding_robot = None

    def _yield_twist(self, _ns):
        twist = Twist()
        twist.linear.x = -0.10
        return twist

    def cmd_cb1(self, msg):
        self.raw_cmd1 = msg
        if self.yielding_robot != 'robot1':
            self.cmd_pub1.publish(msg)
        else:
            self.cmd_pub1.publish(self._yield_twist('robot1'))

    def cmd_cb2(self, msg):
        self.raw_cmd2 = msg
        if self.yielding_robot != 'robot2':
            self.cmd_pub2.publish(msg)
        else:
            self.cmd_pub2.publish(self._yield_twist('robot2'))

    def _poses_from_odom(self):
        if self.odom['robot1'] is None or self.odom['robot2'] is None:
            return None
        p1 = _spawn_odom_to_map(self.spawn['robot1'], self.odom['robot1'])
        p2 = _spawn_odom_to_map(self.spawn['robot2'], self.odom['robot2'])
        return {'robot1': p1, 'robot2': p2}

    def _poses_from_tf(self):
        poses = {}
        for ns in ('robot1', 'robot2'):
            try:
                tf = self.tf_buffer.lookup_transform(
                    self.target_frame,
                    ns + '/base_footprint',
                    rospy.Time(0),
                    rospy.Duration(0.15))
                poses[ns] = (
                    tf.transform.translation.x,
                    tf.transform.translation.y,
                    _yaw_from_quat(tf.transform.rotation))
            except tf2_ros.TransformException:
                return None
        return poses

    def _other_cloud_pose(self, observer_ns, observed_ns):
        """Return (frame_id, x, y) for other-robot cloud seen by observer."""
        if self.use_odom_relative and self.odom[observer_ns] and self.odom[observed_ns]:
            ox, oy = _other_in_robot_odom(
                self.spawn[observer_ns], self.odom[observer_ns],
                self.spawn[observed_ns], self.odom[observed_ns])
            return observer_ns + '/odom', ox, oy

        poses = self._poses_from_tf()
        if poses is None:
            return None
        obs = poses[observer_ns]
        ref = poses[observed_ns]
        dx = ref[0] - obs[0]
        dy = ref[1] - obs[1]
        cos_y = math.cos(-obs[2])
        sin_y = math.sin(-obs[2])
        ox = cos_y * dx - sin_y * dy
        oy = sin_y * dx + cos_y * dy
        return observer_ns + '/odom', ox, oy

    def run(self):
        rate = rospy.Rate(PUBLISH_HZ)
        cloud_pairs = [
            ('robot1', 'robot2', self.cloud_pub1),
            ('robot2', 'robot1', self.cloud_pub2),
        ]

        while not rospy.is_shutdown():
            poses_map = self._poses_from_odom() if self.use_odom_relative else self._poses_from_tf()
            if poses_map is None and self.use_odom_relative:
                poses_map = self._poses_from_tf()

            dist = None
            if poses_map is not None:
                p1 = poses_map['robot1']
                p2 = poses_map['robot2']
                dist = math.hypot(p1[0] - p2[0], p1[1] - p2[1])

            if dist is not None and not self.flag_captured and not self.in_capture_phase:
                if self.yielding_robot is None:
                    in_cooldown = self.yield_cooldown_until and rospy.Time.now() < self.yield_cooldown_until
                    if dist < YIELD_DISTANCE and not in_cooldown:
                        self.yielding_robot = 'robot2'
                        self.yield_start_time = rospy.Time.now()
                        rospy.logwarn('Robots too close (%.2fm). robot2 YIELDS to robot1.', dist)
                else:
                    yield_elapsed = (rospy.Time.now() - self.yield_start_time).to_sec() if self.yield_start_time else 0.0
                    if dist > RESUME_DISTANCE or yield_elapsed > YIELD_TIMEOUT:
                        is_timeout = dist <= RESUME_DISTANCE
                        reason = 'timeout (%.1fs)' % yield_elapsed if is_timeout else 'separated'
                        rospy.loginfo('robot2 RESUMES (%s, dist=%.2fm).', reason, dist)
                        self.yielding_robot = None
                        self.yield_start_time = None
                        if is_timeout:
                            self.yield_cooldown_until = rospy.Time.now() + rospy.Duration(YIELD_COOLDOWN)
                            rospy.logwarn('Entering yield cooldown for %.0fs', YIELD_COOLDOWN)
            elif self.flag_captured or self.in_capture_phase:
                self.yielding_robot = None
                self.yield_start_time = None

            if self.yielding_robot == 'robot1':
                self.cmd_pub1.publish(self._yield_twist('robot1'))
            elif self.yielding_robot == 'robot2':
                self.cmd_pub2.publish(self._yield_twist('robot2'))

            for observer_ns, observed_ns, pub in cloud_pairs:
                if dist is None or dist > self.max_avoid_dist or self.flag_captured:
                    pub.publish(_empty_cloud(observer_ns + '/odom'))
                    continue
                pose = self._other_cloud_pose(observer_ns, observed_ns)
                if pose is None:
                    pub.publish(_empty_cloud(observer_ns + '/odom'))
                    continue
                frame_id, cx, cy = pose
                radius = 0.20 if self.in_capture_phase else self.robot_radius
                pub.publish(_make_cloud(frame_id, cx, cy, radius))

            rate.sleep()


if __name__ == '__main__':
    try:
        coordinator = RobotCoordinator()
        coordinator.run()
    except rospy.ROSInterruptException:
        pass
