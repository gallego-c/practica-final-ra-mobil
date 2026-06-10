#!/usr/bin/env python3
"""
Robot Coordinator Node
Handles both:
1. Inter-robot collision avoidance via PointCloud2 footprint publishing.
2. Twist multiplexing and priority yielding when robots are too close.
"""
import math
import struct
import rospy
import tf2_ros
from sensor_msgs.msg import PointCloud2, PointField
from geometry_msgs.msg import Twist
from std_msgs.msg import Header, Bool

ROBOT_RADIUS = 0.28
NUM_ANGLES   = 20
NUM_RINGS    = 3
PUBLISH_HZ   = 10.0
MAX_AVOID_DISTANCE = 3.5

# Priority distance thresholds
YIELD_DISTANCE = 1.5      # Distance at which robot2 yields to robot1
RESUME_DISTANCE = 1.8     # Distance at which robot2 is allowed to resume
YIELD_TIMEOUT = 5.0       # Max seconds to yield before releasing (prevents deadlocks)
YIELD_COOLDOWN = 5.0      # Seconds to wait after a timeout before yielding again

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

class RobotCoordinator:
    def __init__(self):
        rospy.init_node('robot_coordinator')
        self.target_frame = rospy.get_param('~target_frame', 'map')
        self.robot_radius = float(rospy.get_param('~robot_radius', ROBOT_RADIUS))
        self.max_avoid_dist = float(rospy.get_param('~max_avoid_distance', MAX_AVOID_DISTANCE))

        self.tf_buffer = tf2_ros.Buffer()
        tf2_ros.TransformListener(self.tf_buffer)

        # Publishers for PointCloud2 footprints
        self.cloud_pub1 = rospy.Publisher('/robot1/other_robot_cloud', PointCloud2, queue_size=1)
        self.cloud_pub2 = rospy.Publisher('/robot2/other_robot_cloud', PointCloud2, queue_size=1)

        # Multiplexer publishers
        self.cmd_pub1 = rospy.Publisher('/robot1/cmd_vel', Twist, queue_size=1)
        self.cmd_pub2 = rospy.Publisher('/robot2/cmd_vel', Twist, queue_size=1)

        # State variable: None (both free), 'robot1' (robot1 yields), or 'robot2' (robot2 yields)
        self.yielding_robot = None
        self.yield_start_time = None
        self.yield_cooldown_until = None
        self.flag_captured = False
        self.in_capture_phase = False

        # Cached raw cmd_vel commands
        self.raw_cmd1 = Twist()
        self.raw_cmd2 = Twist()

        # Subscribers to cmd_vel_raw
        self.cmd_sub1 = rospy.Subscriber('/robot1/cmd_vel_raw', Twist, self.cmd_cb1, queue_size=1)
        self.cmd_sub2 = rospy.Subscriber('/robot2/cmd_vel_raw', Twist, self.cmd_cb2, queue_size=1)
        self.flag_captured_sub = rospy.Subscriber('/ctf/flag_captured', Bool, self.flag_captured_cb, queue_size=1)
        self.capture_phase_sub = rospy.Subscriber('/ctf/capture_phase', Bool, self.capture_phase_cb, queue_size=1)

        rospy.loginfo('RobotCoordinator: initialized. Priority: robot1 > robot2')

    def flag_captured_cb(self, msg):
        self.flag_captured = msg.data
        if self.flag_captured:
            self.yielding_robot = None

    def capture_phase_cb(self, msg):
        self.in_capture_phase = msg.data
        if self.in_capture_phase:
            self.yielding_robot = None

    def _yield_twist(self, ns):
        """Back away and turn slightly instead of just spinning in place."""
        twist = Twist()
        twist.linear.x = -0.10  # back up slowly
        twist.angular.z = -0.4 if ns == 'robot1' else 0.4
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

            # Calculate inter-robot distance
            dist = None
            if all(poses.get(ns) for ns in ('robot1', 'robot2')):
                p1 = poses['robot1']
                p2 = poses['robot2']
                dist = math.hypot(p1[0] - p2[0], p1[1] - p2[1])

            # Priority yielding state machine
            if dist is not None and not self.flag_captured and not self.in_capture_phase:
                if self.yielding_robot is None:
                    # Only yield if we are not in a cooldown period
                    in_cooldown = self.yield_cooldown_until and rospy.Time.now() < self.yield_cooldown_until
                    if dist < YIELD_DISTANCE and not in_cooldown:
                        self.yielding_robot = 'robot2' # robot2 yields to robot1
                        self.yield_start_time = rospy.Time.now()
                        rospy.logwarn(f'Robots too close ({dist:.2f}m). robot2 YIELDS to robot1.')
                else:
                    yield_elapsed = (rospy.Time.now() - self.yield_start_time).to_sec() if self.yield_start_time else 0.0
                    if dist > RESUME_DISTANCE or yield_elapsed > YIELD_TIMEOUT:
                        is_timeout = dist <= RESUME_DISTANCE
                        reason = f'timeout ({yield_elapsed:.1f}s)' if is_timeout else 'separated'
                        rospy.loginfo(f'robot2 RESUMES ({reason}, dist={dist:.2f}m).')
                        self.yielding_robot = None
                        self.yield_start_time = None
                        if is_timeout:
                            self.yield_cooldown_until = rospy.Time.now() + rospy.Duration(YIELD_COOLDOWN)
                            rospy.logwarn(f'Entering yield cooldown for {YIELD_COOLDOWN}s')
            elif self.flag_captured or self.in_capture_phase:
                self.yielding_robot = None
                self.yield_start_time = None

            # Yielding robot spins in place so it can clear narrow passages
            if self.yielding_robot == 'robot1':
                self.cmd_pub1.publish(self._yield_twist('robot1'))
            elif self.yielding_robot == 'robot2':
                self.cmd_pub2.publish(self._yield_twist('robot2'))

            # Publish PointCloud2 footprints for local costmaps
            for source_ns, pub in pairs:
                if dist is None or dist > self.max_avoid_dist or self.flag_captured:
                    pub.publish(_empty_cloud(self.target_frame))
                    continue
                pos = poses.get(source_ns)
                if pos is None:
                    continue
                radius = 0.20 if self.in_capture_phase else self.robot_radius
                pub.publish(_make_cloud(self.target_frame, pos[0], pos[1], radius))

            rate.sleep()

if __name__ == '__main__':
    try:
        coordinator = RobotCoordinator()
        coordinator.run()
    except rospy.ROSInterruptException:
        pass
