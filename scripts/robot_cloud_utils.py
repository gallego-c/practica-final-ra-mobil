"""PointCloud2 footprint helpers for inter-robot collision avoidance."""
import math
import struct

import rospy
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header

NUM_ANGLES = 20
NUM_RINGS = 3


def empty_cloud(frame_id):
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


def make_footprint_cloud(frame_id, cx, cy, radius):
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
