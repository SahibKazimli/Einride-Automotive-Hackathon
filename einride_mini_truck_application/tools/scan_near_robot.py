#!/usr/bin/env python3
"""Print the lidar points close to the robot, in base_footprint.

The collision monitor stops the robot when 3+ scan points fall inside
PolygonStop (|x| < 0.206, |y| < 0.13). This shows which points those are,
e.g. the lidar hitting the robot's own body.

    python3 tools/scan_near_robot.py            # one scan
    python3 tools/scan_near_robot.py --box 0.4  # wider box
"""

import argparse
import math

import rclpy
from rclpy.duration import Duration
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformListener
from tf_transformations import quaternion_matrix

STOP_X, STOP_Y = 0.206, 0.13


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument('--box', type=float, default=0.30, help='half-size of box to print, m')
    args = parser.parse_args()

    rclpy.init()
    node = rclpy.create_node('scan_near_robot')
    buffer = Buffer()
    TransformListener(buffer, node, spin_thread=True)
    scans: list[LaserScan] = []
    node.create_subscription(LaserScan, '/scan', scans.append, qos_profile_sensor_data)
    while rclpy.ok() and not scans:
        rclpy.spin_once(node, timeout_sec=0.5)
    scan = scans[0]

    tf = buffer.lookup_transform('base_footprint', scan.header.frame_id, Time(),
                                 timeout=Duration(seconds=3.0))
    t = tf.transform.translation
    q = tf.transform.rotation
    m = quaternion_matrix([q.x, q.y, q.z, q.w])
    print(f'scan frame {scan.header.frame_id} at ({t.x:.3f}, {t.y:.3f}, {t.z:.3f}), '
          f'range_min {scan.range_min:.3f}')

    inside = 0
    for i, r in enumerate(scan.ranges):
        if not math.isfinite(r) or r < scan.range_min or r > scan.range_max:
            continue
        a = scan.angle_min + i * scan.angle_increment
        lx, ly = r * math.cos(a), r * math.sin(a)
        x = m[0][0] * lx + m[0][1] * ly + t.x
        y = m[1][0] * lx + m[1][1] * ly + t.y
        if abs(x) < args.box and abs(y) < args.box:
            stop = abs(x) < STOP_X and abs(y) < STOP_Y
            inside += stop
            print(f'  x={x:+.3f} y={y:+.3f}  range={r:.3f}  {"<-- in PolygonStop" if stop else ""}')
    print(f'{inside} points inside PolygonStop (stops the robot at 3)')
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
