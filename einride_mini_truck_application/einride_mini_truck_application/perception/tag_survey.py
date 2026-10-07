"""AprilTag survey: record where to dock at each tag, on the SLAM map.

For every sighting of tag 0-7 this node computes the docked pose in the robot's
frame (dock_pose.py, as live docking does), and places it on the map with the
robot's map pose at the time of that image. tag_catalog.py averages the
sightings; /tag_survey/save writes them as a dock database (frame `map`),
usable as a layout (layout:=<file name> with slam:=true).

Subscribes
    /detections (apriltag_msgs/AprilTagDetectionArray)
Services
    /tag_survey/save (std_srvs/Trigger): write `output_file`
"""

import os
import time
from typing import Optional

from apriltag_msgs.msg import AprilTagDetectionArray
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformException, TransformListener
from tf_transformations import euler_from_quaternion, quaternion_matrix

from .dock_pose import docked_pose
from .tag_catalog import compose, NAMES, Pose, summarize, write_dock_database


def planar(transform) -> Pose:
    t, q = transform.translation, transform.rotation
    return (t.x, t.y, euler_from_quaternion([q.x, q.y, q.z, q.w])[2])


class TagSurveyNode(Node):
    """Collect docked-pose sightings per tag and save them on request."""

    def __init__(self) -> None:
        super().__init__('tag_survey')
        p = self.declare_parameter
        self.map_frame = p('map_frame', 'map').value
        self.base_frame = p('base_frame', 'base_footprint').value
        self.tag_prefix = p('tag_frame_prefix', 'tag_').value
        self.output_file = os.path.expanduser(
            p('output_file', '~/.ros/arena_tag_survey.yaml').value)
        self.min_observations = int(p('min_observations', 5).value)
        self.sample_period = float(p('sample_period', 0.5).value)
        self.max_tf_age = float(p('max_tf_age', 2.0).value)
        self.max_position_error = float(p('max_position_error', 0.30).value)
        self.max_yaw_error = float(p('max_yaw_error', 0.50).value)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.samples: dict[int, list[Pose]] = {}
        self.last_sample: dict[int, float] = {}
        self.create_subscription(AprilTagDetectionArray, '/detections', self.on_detections, 10)
        self.create_service(Trigger, '/tag_survey/save', self.save_survey)
        self.get_logger().info(
            f'Surveying tags 0-7 on {self.map_frame}; save with /tag_survey/save')

    def docked_on_map(self, tag_id: int, stamp) -> Optional[Pose]:
        """Docked pose for this sighting in the map frame, None if TF is missing."""
        frame = f'{self.tag_prefix}{tag_id}'
        try:
            # Newest tag TF, as in perception/node.py: on the busy Jetson it can
            # arrive over a second after /detections.
            tag = self.tf_buffer.lookup_transform(self.base_frame, frame, Time())
            if Time.from_msg(stamp) - Time.from_msg(tag.header.stamp) \
                    > Duration(seconds=self.max_tf_age):
                return None   # an older sighting, not this detection
            # Where the robot was when that image was taken.
            robot = self.tf_buffer.lookup_transform(
                self.map_frame, self.base_frame, Time.from_msg(tag.header.stamp))
        except TransformException as error:
            self.get_logger().warn(f'No map pose for tag {tag_id}: {error}',
                                   throttle_duration_sec=2.0)
            return None
        t, q = tag.transform.translation, tag.transform.rotation
        rotation = quaternion_matrix([q.x, q.y, q.z, q.w])[:3, :3].tolist()
        try:
            local = docked_pose([t.x, t.y, t.z], rotation)
        except ValueError:
            return None
        return compose(planar(robot.transform), local)

    def on_detections(self, msg: AprilTagDetectionArray) -> None:
        now = time.monotonic()
        for detection in msg.detections:
            tag_id = int(detection.id)
            if not 0 <= tag_id < len(NAMES) or \
                    now - self.last_sample.get(tag_id, -1e9) < self.sample_period:
                continue
            pose = self.docked_on_map(tag_id, msg.header.stamp)
            if pose is None:
                continue
            samples = self.samples.setdefault(tag_id, [])
            samples.append(pose)
            self.last_sample[tag_id] = now
            if len(samples) == 1 or len(samples) % 10 == 0:
                self.get_logger().info(
                    f'Tag {NAMES[tag_id]} (ID {tag_id}): {len(samples)} observations, '
                    f'dock at ({pose[0]:.2f}, {pose[1]:.2f}) on {self.map_frame}')

    def save_survey(self, request: Trigger.Request,
                    response: Trigger.Response) -> Trigger.Response:
        docks: dict[int, Pose] = {}
        counts: dict[int, int] = {}
        seen_too_little = []
        for tag_id, samples in sorted(self.samples.items()):
            result = summarize(samples, self.min_observations,
                               self.max_position_error, self.max_yaw_error)
            if result is None:
                seen_too_little.append(NAMES[tag_id])
            else:
                docks[tag_id], counts[tag_id] = result
        try:
            os.makedirs(os.path.dirname(self.output_file) or '.', exist_ok=True)
            with open(self.output_file, 'w', encoding='utf-8') as out:
                write_dock_database(out, docks, counts, self.map_frame)
        except OSError as error:
            response.success = False
            response.message = f'Could not write {self.output_file}: {error}'
            return response

        response.success = bool(docks)
        saved = ', '.join(NAMES[tag] for tag in docks) or 'none'
        response.message = f'Saved docks {saved} to {self.output_file}.'
        if seen_too_little:
            response.message += (f' Need at least {self.min_observations} consistent '
                                 f'observations for: {", ".join(seen_too_little)}.')
        return response


def main(args: Optional[list] = None) -> None:
    rclpy.init(args=args)
    node = TagSurveyNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
