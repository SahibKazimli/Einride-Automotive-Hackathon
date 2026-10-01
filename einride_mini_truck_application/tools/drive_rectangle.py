#!/usr/bin/env python3
"""Drive a rectangle: forward, turn 90 deg left, forward, ... four times.

No build needed. On the robot:

    source /opt/ros/jazzy/setup.bash
    source /opt/einride_mini_truck/install/setup.bash
    set -a; . /etc/default/einride-mini-truck; set +a
    python3 drive_rectangle.py                 # 0.5 m x 0.5 m
    python3 drive_rectangle.py --side 1.0 --speed 0.2

Straights are timed (distance / speed). Turns read the chassis gyro (/imu) and
stop when it says 90 deg; without gyro data they fall back to timing.
Ctrl+C stops the robot. Nothing else may publish /cmd_vel while this runs.
"""

import argparse
import math
import time

from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu

RATE = 20.0  # Hz. The hardware stops if /cmd_vel is quiet for 0.5 s.


class Rectangle(Node):
    def __init__(self) -> None:
        super().__init__('drive_rectangle')
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        # /imu is best-effort: a default (reliable) subscription receives nothing.
        self.create_subscription(Imu, '/imu', self.on_imu, qos_profile_sensor_data)
        self.gyro_z = None
        self.yaw = 0.0          # integrated gyro, radians
        self.last_imu = None

    def on_imu(self, msg: Imu) -> None:
        now = time.monotonic()
        if self.last_imu is not None:
            self.yaw += msg.angular_velocity.z * (now - self.last_imu)
        self.last_imu = now
        self.gyro_z = msg.angular_velocity.z

    def send(self, v: float, w: float) -> None:
        msg = Twist()
        msg.linear.x = v
        msg.angular.z = w
        self.pub.publish(msg)

    def tick(self) -> None:
        """Wait one control period while still processing incoming /imu."""
        end = time.monotonic() + 1.0 / RATE
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=max(0.0, end - time.monotonic()))

    def drive(self, distance: float, speed: float) -> None:
        duration = distance / speed
        self.get_logger().info(f'Forward {distance:.2f} m ({duration:.1f} s)')
        end = time.monotonic() + duration
        while time.monotonic() < end:
            self.send(speed, 0.0)
            self.tick()
        self.stop(0.5)

    def turn(self, angle: float, rate: float) -> None:
        start = self.yaw
        timeout = time.monotonic() + 3.0 * abs(angle) / rate + 2.0
        use_gyro = self.gyro_z is not None
        if not use_gyro:
            self.get_logger().warn('No /imu data: timing the turn instead')
            timeout = time.monotonic() + abs(angle) / rate
        while time.monotonic() < timeout:
            if use_gyro and abs(self.yaw - start) >= abs(angle):
                break
            self.send(0.0, math.copysign(rate, angle))
            self.tick()
        self.stop(0.5)
        if use_gyro:
            self.get_logger().info(
                f'Turned {math.degrees(self.yaw - start):.1f} deg (gyro), wanted '
                f'{math.degrees(angle):.0f}')

    def stop(self, seconds: float = 0.0) -> None:
        end = time.monotonic() + seconds
        while True:
            self.send(0.0, 0.0)
            if time.monotonic() >= end:
                break
            self.tick()


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('--side', type=float, default=0.5, help='side length, m')
    parser.add_argument('--speed', type=float, default=0.15, help='forward speed, m/s')
    parser.add_argument('--turn-rate', type=float, default=0.8, help='turn speed, rad/s')
    parser.add_argument('--right', action='store_true', help='turn right instead of left')
    args = parser.parse_args()

    rclpy.init()
    node = Rectangle()
    try:
        # Let /imu messages arrive first.
        for _ in range(int(RATE)):
            node.tick()
        angle = math.radians(-90 if args.right else 90)
        for side in range(4):
            node.get_logger().info(f'--- side {side + 1}/4 ---')
            node.drive(args.side, args.speed)
            node.turn(angle, args.turn_rate)
        node.get_logger().info('Done. Measure how far the robot is from where it started.')
    except KeyboardInterrupt:
        pass
    finally:
        node.send(0.0, 0.0)
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
