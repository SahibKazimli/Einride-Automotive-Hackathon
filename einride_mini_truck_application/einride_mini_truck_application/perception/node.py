"""Turns apriltag_ros detections into the dock pose Nav2's docking server wants.

apriltag_ros publishes a TF frame per seen tag (tag_<id>, set in
config/perception/apriltag.yaml). For the tag the mission is docking at, this
node looks that frame up in base_footprint, computes where the robot must stop
(dock_pose.py), and publishes it.

Subscribes
    /detections (apriltag_msgs/AprilTagDetectionArray): which tags were seen, when
    /mission/target_tag (std_msgs/Int32): which tag to report, -1 = none
Publishes
    /detected_dock_pose (geometry_msgs/PoseStamped) in base_footprint
"""

from apriltag_msgs.msg import AprilTagDetectionArray
from geometry_msgs.msg import PoseStamped
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from std_msgs.msg import Int32
from tf2_ros import Buffer, TransformException, TransformListener
from tf_transformations import quaternion_from_euler, quaternion_matrix

from .dock_pose import docked_pose


class DockPoseNode(Node):
    def __init__(self) -> None:
        super().__init__('dock_pose')
        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter('tag_frame_prefix', 'tag_')
        self.base_frame = self.get_parameter('base_frame').value
        self.prefix = self.get_parameter('tag_frame_prefix').value
        self.target = -1
        self.tf_buffer = Buffer()
        # Own thread: the tag's TF can arrive just after its /detections message,
        # and the lookup below must be able to wait for it.
        self.tf_listener = TransformListener(self.tf_buffer, self, spin_thread=True)
        self.pub = self.create_publisher(PoseStamped, '/detected_dock_pose', 10)
        self.create_subscription(Int32, '/mission/target_tag', self.on_target, 10)
        self.create_subscription(AprilTagDetectionArray, '/detections', self.on_detections, 10)

    def on_target(self, msg: Int32) -> None:
        if msg.data != self.target:
            self.get_logger().info(f'Reporting dock pose for tag {msg.data}')
        self.target = msg.data

    def on_detections(self, msg: AprilTagDetectionArray) -> None:
        if self.target < 0 or not any(d.id == self.target for d in msg.detections):
            return
        frame = f'{self.prefix}{self.target}'
        try:
            tf = self.tf_buffer.lookup_transform(
                self.base_frame, frame, Time.from_msg(msg.header.stamp),
                timeout=Duration(seconds=0.2))
        except TransformException as error:
            self.get_logger().warn(f'No TF {self.base_frame} <- {frame}: {error}',
                                   throttle_duration_sec=2.0)
            return
        t = tf.transform.translation
        q = tf.transform.rotation
        rotation = quaternion_matrix([q.x, q.y, q.z, q.w])[:3, :3].tolist()
        try:
            x, y, yaw = docked_pose([t.x, t.y, t.z], rotation)
        except ValueError:
            return
        out = PoseStamped()
        out.header.stamp = msg.header.stamp
        out.header.frame_id = self.base_frame
        out.pose.position.x = x
        out.pose.position.y = y
        qx, qy, qz, qw = quaternion_from_euler(0.0, 0.0, yaw)
        out.pose.orientation.x = qx
        out.pose.orientation.y = qy
        out.pose.orientation.z = qz
        out.pose.orientation.w = qw
        self.pub.publish(out)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = DockPoseNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
