"""Collect all detected AprilTags as named landmarks in the SLAM map frame."""

import math
import os
import statistics
import time

from apriltag_msgs.msg import AprilTagDetectionArray
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformException, TransformListener
from tf_transformations import euler_from_quaternion
import yaml


class TagSurveyNode(Node):
    """Record repeated map-frame observations and save them on request."""

    def __init__(self) -> None:
        super().__init__('tag_survey')
        self.map_frame = self.declare_parameter('map_frame', 'map').value
        self.tag_prefix = self.declare_parameter('tag_frame_prefix', 'tag_').value
        self.output_file = os.path.expanduser(
            self.declare_parameter('output_file', '~/.ros/arena_tags.yaml').value)
        self.min_observations = int(self.declare_parameter('min_observations', 5).value)
        self.sample_period = float(self.declare_parameter('sample_period', 0.5).value)
        self.max_tf_age = float(self.declare_parameter('max_tf_age', 2.0).value)
        self.max_position_error = float(
            self.declare_parameter('max_position_error', 0.30).value)
        self.max_yaw_error = float(self.declare_parameter('max_yaw_error', 0.50).value)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.samples: dict[int, list[tuple[float, float, float]]] = {}
        self.last_sample: dict[int, float] = {}
        self.create_subscription(AprilTagDetectionArray, '/detections', self.on_detections, 10)
        self.create_service(Trigger, '/tag_survey/save', self.save_survey)
        self.get_logger().info(
            f'Surveying tag IDs 0-7 in {self.map_frame}; save with /tag_survey/save')

    def on_detections(self, msg: AprilTagDetectionArray) -> None:
        now = time.monotonic()
        for detection in msg.detections:
            tag_id = int(detection.id)
            if not 0 <= tag_id <= 7 or now - self.last_sample.get(tag_id, -math.inf) \
                    < self.sample_period:
                continue
            frame = f'{self.tag_prefix}{tag_id}'
            try:
                tf = self.tf_buffer.lookup_transform(self.map_frame, frame, Time())
            except TransformException as error:
                self.get_logger().warn(
                    f'No map pose for tag {tag_id}: {error}', throttle_duration_sec=2.0)
                continue

            age = Time.from_msg(msg.header.stamp) - Time.from_msg(tf.header.stamp)
            if age > Duration(seconds=self.max_tf_age):
                continue

            t = tf.transform.translation
            q = tf.transform.rotation
            yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])[2]
            if not all(math.isfinite(value) for value in (t.x, t.y, yaw)):
                continue

            samples = self.samples.setdefault(tag_id, [])
            samples.append((float(t.x), float(t.y), float(yaw)))
            self.last_sample[tag_id] = now
            if len(samples) == 1 or len(samples) % 10 == 0:
                self.get_logger().info(
                    f'Tag {chr(ord("A") + tag_id)} (ID {tag_id}): '
                    f'{len(samples)} observations in {self.map_frame}')

    @staticmethod
    def yaw_difference(a: float, b: float) -> float:
        return math.atan2(math.sin(a - b), math.cos(a - b))

    def summarize(self, samples: list[tuple[float, float, float]]) \
            -> tuple[float, float, float, int] | None:
        if len(samples) < self.min_observations:
            return None
        center_x = statistics.median(sample[0] for sample in samples)
        center_y = statistics.median(sample[1] for sample in samples)
        center_yaw = math.atan2(sum(math.sin(s[2]) for s in samples),
                                sum(math.cos(s[2]) for s in samples))
        accepted = [sample for sample in samples
                    if math.hypot(sample[0] - center_x, sample[1] - center_y)
                    <= self.max_position_error
                    and abs(self.yaw_difference(sample[2], center_yaw))
                    <= self.max_yaw_error]
        if len(accepted) < self.min_observations:
            return None
        x = sum(sample[0] for sample in accepted) / len(accepted)
        y = sum(sample[1] for sample in accepted) / len(accepted)
        yaw = math.atan2(sum(math.sin(s[2]) for s in accepted),
                         sum(math.cos(s[2]) for s in accepted))
        return x, y, yaw, len(accepted)

    def save_survey(self, request: Trigger.Request,
                    response: Trigger.Response) -> Trigger.Response:
        tags: dict[int, dict[str, object]] = {}
        missing: list[str] = []
        for tag_id in range(8):
            pose = self.summarize(self.samples.get(tag_id, []))
            if pose is None:
                missing.append(chr(ord('A') + tag_id))
                continue
            x, y, yaw, count = pose
            tags[tag_id] = {
                'name': chr(ord('A') + tag_id),
                'pose': [round(x, 4), round(y, 4), round(yaw, 4)],
                'observations': count,
            }

        document = {
            'frame': self.map_frame,
            'tag_family': 'tag36h11',
            'tag_size_m': 0.1,
            'tags': tags,
        }
        try:
            os.makedirs(os.path.dirname(self.output_file) or '.', exist_ok=True)
            with open(self.output_file, 'w', encoding='utf-8') as output:
                yaml.safe_dump(document, output, sort_keys=True)
        except OSError as error:
            response.success = False
            response.message = f'Could not write {self.output_file}: {error}'
            return response

        response.success = bool(tags)
        saved = ', '.join(value['name'] for value in tags.values()) or 'none'
        response.message = f'Saved tags {saved} to {self.output_file}.'
        if missing:
            response.message += (' Need at least '
                                 f'{self.min_observations} consistent observations for: '
                                 f'{", ".join(missing)}.')
        return response


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = TagSurveyNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
