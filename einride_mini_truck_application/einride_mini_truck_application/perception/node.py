"""Turns apriltag_ros detections into the dock pose Nav2's docking server wants.

apriltag_ros publishes a TF frame per seen tag (tag_<id>, set in
config/perception/apriltag.yaml). For the tag the mission is docking at, this
node looks that frame up in base_footprint, computes where the robot must stop
(dock_pose.py), and publishes it.

It also gates the camera (camera_gate.py): apriltag_ros reads /dock_camera/*,
which carries images only while the robot is near the target dock, so the
detector idles the rest of the time. Images pass through as raw bytes (never
decoded here), at most `max_image_rate` per second and one at a time
(timing.py). Published poses are moved onto this computer's clock if the
camera's stamps are off it (timing.py), since the docking server compares
them with its own clock.

Subscribes
    /detections (apriltag_msgs/AprilTagDetectionArray): which tags were seen, when
    /mission/target_tag (std_msgs/Int32): which tag to report, -1 = none
    /oak/rgb/image_raw, /oak/rgb/camera_info: only while the gate is open
Publishes
    /detected_dock_pose (geometry_msgs/PoseStamped) in base_footprint
    /dock_camera/image_raw, /dock_camera/camera_info: input for apriltag_ros
"""

from apriltag_msgs.msg import AprilTagDetectionArray
from geometry_msgs.msg import PoseStamped
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, qos_profile_sensor_data, QoSProfile
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Int32
from tf2_ros import Buffer, TransformException, TransformListener
from tf_transformations import quaternion_from_euler, quaternion_matrix

from .camera_gate import CameraGate, load_dock_positions
from .dock_pose import docked_pose
from .timing import ClockSkew, ImageThrottle


def seconds(stamp) -> float:
    return Time.from_msg(stamp).nanoseconds * 1e-9


class DockPoseNode(Node):
    def __init__(self) -> None:
        super().__init__('dock_pose')
        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter('tag_frame_prefix', 'tag_')
        self.base_frame = self.get_parameter('base_frame').value
        self.prefix = self.get_parameter('tag_frame_prefix').value
        # Seconds a tag TF may lag its detection and still count as that sighting.
        self.max_age = self.declare_parameter('max_tf_age', 2.0).value
        self.target = -1
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.pub = self.create_publisher(PoseStamped, '/detected_dock_pose', 10)
        # Latched like the mission's publisher, so a restarted dock_pose still
        # learns the current target (test by hand with
        # `ros2 topic pub --qos-durability transient_local ...`).
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(Int32, '/mission/target_tag', self.on_target, latched)
        self.create_subscription(AprilTagDetectionArray, '/detections', self.on_detections, 10)

        # Camera gate.
        p = self.declare_parameter
        database = p('dock_database', '').value
        self.docks = load_dock_positions(database) if database else {}
        self.dock_frame = p('dock_frame', 'arena').value
        self.gate = CameraGate(p('gate_on_distance', 1.6).value,
                               p('gate_off_distance', 1.9).value)
        self.throttle = ImageThrottle(p('max_image_rate', 10.0).value,
                                      p('detector_timeout', 0.5).value)
        self.skew = ClockSkew(p('max_camera_delay', 2.0).value)
        out = QoSProfile(depth=2)
        self.image_pub = self.create_publisher(Image, '/dock_camera/image_raw', out)
        self.info_pub = self.create_publisher(CameraInfo, '/dock_camera/camera_info', out)
        self.camera_subs = []
        self.create_timer(0.2, self.update_gate)
        if not self.docks:
            self.get_logger().warn('No dock_database: camera gate opens whenever a target is set')

    def on_target(self, msg: Int32) -> None:
        if msg.data != self.target:
            self.get_logger().info(f'Reporting dock pose for tag {msg.data}')
        self.target = msg.data

    def update_gate(self) -> None:
        if self.target < 0:
            dock = None
        elif self.docks:
            dock = self.docks.get(self.target)
        else:
            dock = (0.0, 0.0)   # no layout known: treat every target as near
        robot = None
        if dock is not None and self.docks:
            try:
                tf = self.tf_buffer.lookup_transform(self.dock_frame, self.base_frame, Time())
                robot = (tf.transform.translation.x, tf.transform.translation.y)
            except TransformException:
                pass
        was_open = self.gate.open
        is_open = self.gate.update(robot, dock) if self.docks else dock is not None
        self.gate.open = is_open
        if is_open == was_open:
            return
        if is_open:
            # raw=True: the image callback gets the serialized bytes, forwarded
            # undecoded. camera_info is small and decoded for its stamp.
            self.skew.clear()
            self.camera_subs = [
                self.create_subscription(Image, '/oak/rgb/image_raw', self.on_image,
                                         qos_profile_sensor_data, raw=True),
                self.create_subscription(CameraInfo, '/oak/rgb/camera_info', self.on_info,
                                         qos_profile_sensor_data)]
            self.get_logger().info(f'Near dock of tag {self.target}: tag detection on')
        else:
            for sub in self.camera_subs:
                self.destroy_subscription(sub)
            self.camera_subs = []
            self.get_logger().info('Tag detection off')

    def clock_seconds(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def on_image(self, data: bytes) -> None:
        if self.throttle.offer(self.clock_seconds()):
            self.image_pub.publish(data)

    def on_info(self, msg: CameraInfo) -> None:
        self.skew.add(self.clock_seconds(), seconds(msg.header.stamp))
        self.info_pub.publish(msg)
        correction = self.skew.correction()
        if correction:
            side = 'behind' if correction > 0 else 'ahead of'
            self.get_logger().error(
                f'Camera clock is {abs(correction):.2f} s {side} this computer\'s (system '
                'clock stepped after the camera driver started?). Shifting tag poses by that; '
                'restart the camera driver to fix it at the source.',
                throttle_duration_sec=10.0)

    def on_detections(self, msg: AprilTagDetectionArray) -> None:
        self.throttle.answered()
        if self.target < 0:
            return
        seen = [d.id for d in msg.detections]
        if self.target not in seen:
            self.get_logger().info(f'Tag {self.target} not in the image (seen: {seen})',
                                   throttle_duration_sec=2.0)
            return
        frame = f'{self.prefix}{self.target}'
        try:
            # Newest available, not the image time: the tag's TF can be handled
            # after /detections. The pose is stamped with the TF's own time
            # below, so it stays exact.
            tf = self.tf_buffer.lookup_transform(self.base_frame, frame, Time())
        except TransformException as error:
            self.get_logger().warn(f'No TF {self.base_frame} <- {frame}: {error}',
                                   throttle_duration_sec=2.0)
            return
        age = Time.from_msg(msg.header.stamp) - Time.from_msg(tf.header.stamp)
        if age > Duration(seconds=self.max_age):
            self.get_logger().warn(
                f'Dropped tag {self.target}: newest {frame} TF is '
                f'{age.nanoseconds * 1e-9:.2f} s older than the detection',
                throttle_duration_sec=2.0)
            return
        t = tf.transform.translation
        q = tf.transform.rotation
        rotation = quaternion_matrix([q.x, q.y, q.z, q.w])[:3, :3].tolist()
        try:
            x, y, yaw = docked_pose([t.x, t.y, t.z], rotation)
        except ValueError as error:
            self.get_logger().warn(f'Dropped tag {self.target}: {error}',
                                   throttle_duration_sec=2.0)
            return
        stamp = Time.from_msg(tf.header.stamp) + Duration(seconds=self.skew.correction())
        # The docking server rejects poses older than external_detection_timeout.
        age = self.clock_seconds() - stamp.nanoseconds * 1e-9
        delay = self.skew.delay()
        delay_text = 'unknown' if delay is None else f'{delay:.2f} s'
        self.get_logger().info(
            f'Dock pose for tag {self.target}: ({x:.2f}, {y:.2f}) m, {age:.2f} s old '
            f'(camera -> here {delay_text})',
            throttle_duration_sec=2.0)
        out = PoseStamped()
        out.header.stamp = stamp.to_msg()
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
