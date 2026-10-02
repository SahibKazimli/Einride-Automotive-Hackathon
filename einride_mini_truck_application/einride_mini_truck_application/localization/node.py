"""Prepares wheel and gyro data for robot_localization's EKF.

Subscribes
    /wheel_encoders (sensor_msgs/JointState, best effort)
    /imu (sensor_msgs/Imu, best effort)
Publishes
    /wheel/odom (nav_msgs/Odometry): forward speed only (twist), for the EKF
    /imu/corrected (sensor_msgs/Imu): /imu with the gyro z bias removed

The EKF (config/localization/ekf.yaml) turns these into /odom and the TF
odom -> base_footprint.
"""

from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import Imu, JointState

from .wheel_odometry import GyroBias, WheelSpeed


def seconds(stamp) -> float:
    return Time.from_msg(stamp).nanoseconds * 1e-9


class WheelOdometryNode(Node):
    def __init__(self) -> None:
        super().__init__('wheel_odometry')
        p = self.declare_parameter
        p('wheel_radius', 0.040)
        # The motor board reports centimetres, but the hardware driver treats
        # them as metres, so /wheel_encoders is 100x too large. Measured
        # 2026-10-02: 1 m driven = ~100 steps of 25 "rad".
        p('encoder_scale', 0.01)
        p('left_joint', 'left_up_wheel_link_joint')
        p('right_joint', 'right_up_wheel_link_joint')
        p('base_frame', 'base_footprint')
        # Variances handed to the EKF. Skid steering slips, so speed is
        # trusted less than the gyro.
        p('speed_variance', 0.01)
        p('gyro_variance', 0.0001)
        g = lambda name: self.get_parameter(name).value  # noqa: E731

        self.speed = WheelSpeed(wheel_radius=g('wheel_radius'))
        self.bias = GyroBias()
        self.encoder_scale = g('encoder_scale')
        self.left_joint = g('left_joint')
        self.right_joint = g('right_joint')
        self.base_frame = g('base_frame')
        self.speed_variance = g('speed_variance')
        self.gyro_variance = g('gyro_variance')
        self.reported = False

        self.odom_pub = self.create_publisher(Odometry, '/wheel/odom', 10)
        self.imu_pub = self.create_publisher(Imu, '/imu/corrected', 10)
        self.create_subscription(JointState, '/wheel_encoders', self.on_encoders,
                                 qos_profile_sensor_data)
        self.create_subscription(Imu, '/imu', self.on_imu, qos_profile_sensor_data)
        self.get_logger().info('Keep the robot still for ~3 s to calibrate the gyro.')

    def on_encoders(self, msg: JointState) -> None:
        try:
            left = msg.position[msg.name.index(self.left_joint)]
            right = msg.position[msg.name.index(self.right_joint)]
        except (ValueError, IndexError):
            self.get_logger().warn(f'Encoder joints not in {list(msg.name)}',
                                   throttle_duration_sec=5.0)
            return
        k = self.encoder_scale
        v = self.speed.update(seconds(msg.header.stamp), left * k, right * k)
        if v is None:
            return
        out = Odometry()
        out.header.stamp = msg.header.stamp
        out.header.frame_id = 'odom'
        out.child_frame_id = self.base_frame
        out.twist.twist.linear.x = v   # linear.y stays 0: the robot cannot slide sideways
        cov = [0.0] * 36
        cov[0] = self.speed_variance   # vx
        cov[7] = 0.001                 # vy (the zero above)
        cov[35] = 1.0                  # vyaw: not provided, the gyro covers it
        out.twist.covariance = cov
        self.odom_pub.publish(out)

    def on_imu(self, msg: Imu) -> None:
        still = self.speed.is_still(seconds(msg.header.stamp))
        msg.angular_velocity.z = self.bias.update(msg.angular_velocity.z, still)
        cov = list(msg.angular_velocity_covariance)
        cov[8] = self.gyro_variance
        msg.angular_velocity_covariance = cov
        self.imu_pub.publish(msg)
        if self.bias.calibrated and not self.reported:
            self.reported = True
            self.get_logger().info(f'Gyro bias calibrated: {self.bias.bias:+.5f} rad/s')


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = WheelOdometryNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
