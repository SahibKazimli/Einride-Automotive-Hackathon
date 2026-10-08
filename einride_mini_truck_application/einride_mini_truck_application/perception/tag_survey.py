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
import math
import hashlib
from typing import Optional
import yaml

from apriltag_msgs.msg import AprilTagDetectionArray
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformException, TransformListener
from tf_transformations import euler_from_quaternion, quaternion_matrix
from visualization_msgs.msg import Marker, MarkerArray

from .dock_pose import DOCK_GAP, FRONT_OFFSET, docked_pose
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
        self.map_file = os.path.expanduser(p('map_file', '').value)
        self.map_identity = self.compute_map_identity()
        self.min_observations = int(p('min_observations', 5).value)
        self.sample_period = float(p('sample_period', 0.5).value)
        self.max_tf_age = float(p('max_tf_age', 2.0).value)
        self.max_position_error = float(p('max_position_error', 0.30).value)
        self.max_yaw_error = float(p('max_yaw_error', 0.50).value)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.dock_samples: dict[int, list[Pose]] = {}
        self.tag_samples: dict[int, list[Pose]] = {}
        self.last_sample: dict[int, float] = {}
        self.saved_docks: dict[int, Pose] = {}
        self.saved_tags: dict[int, Pose] = {}
        self.saved_counts: dict[int, int] = {}
        self.catalog_error: Optional[str] = None
        self.load_catalog()
        self.marker_pub = self.create_publisher(MarkerArray, '/tag_survey/saved_tags', 10)
        self.create_timer(1.0, self.publish_saved_tags)
        self.create_subscription(AprilTagDetectionArray, '/detections', self.on_detections, 10)
        self.create_service(Trigger, '/tag_survey/save', self.save_survey)
        self.get_logger().info(
            f'Surveying tags 0-7 on {self.map_frame}; map identity '
            f'{self.map_identity[:12] if self.map_identity else "unavailable"}; '
            'save with /tag_survey/save')

    def compute_map_identity(self) -> Optional[str]:
        """Fingerprint the SLAM pose graph and data so catalogs stay map-specific."""
        if not self.map_file:
            return None
        digest = hashlib.sha256()
        map_paths = [self.map_file + suffix for suffix in ('.posegraph', '.data')]
        missing = [path for path in map_paths if not os.path.isfile(path)]
        if missing:
            self.get_logger().error(
                f'Cannot identify SLAM map; missing map files: {", ".join(missing)}')
            return None
        for path in map_paths:
            suffix = os.path.splitext(path)[1]
            digest.update(suffix.encode('utf-8'))
            try:
                with open(path, 'rb') as source:
                    for chunk in iter(lambda: source.read(1024 * 1024), b''):
                        digest.update(chunk)
            except OSError as error:
                self.get_logger().error(f'Cannot read SLAM map file {path}: {error}')
                return None
        return digest.hexdigest()

    def tag_and_dock_on_map(self, tag_id: int, stamp) -> Optional[tuple[Pose, Pose]]:
        """Return physical tag and dock poses for this sighting in the map frame."""
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
            local_dock = docked_pose([t.x, t.y, t.z], rotation)
        except ValueError:
            return None
        robot_on_map = planar(robot.transform)
        return compose(robot_on_map, planar(tag.transform)), compose(robot_on_map, local_dock)

    def load_catalog(self) -> None:
        """Load prior saves so partial surveys retain existing tags and markers."""
        if not os.path.isfile(self.output_file):
            return
        try:
            with open(self.output_file, encoding='utf-8') as source:
                catalog = yaml.safe_load(source) or {}
            frame = catalog.get('frame', self.map_frame)
            if frame != self.map_frame:
                raise ValueError(f'catalog frame is {frame!r}, expected {self.map_frame!r}')
            stored_identity = catalog.get('map_identity')
            if not stored_identity or stored_identity != self.map_identity:
                raise ValueError(
                    'catalog has no matching SLAM map identity; use a new output_file '
                    'for this map to avoid mixing surveys')

            def get_tag_id(key, item):
                raw_id = item.get('id', key.rsplit('_', 1)[-1])
                if isinstance(raw_id, str) and raw_id.upper() in NAMES:
                    return NAMES.index(raw_id.upper())
                return int(raw_id)

            for key, item in (catalog.get('docks') or {}).items():
                tag_id = get_tag_id(key, item)
                pose = item.get('pose')
                if 0 <= tag_id < len(NAMES) and pose and len(pose) >= 3:
                    self.saved_docks[tag_id] = tuple(float(v) for v in pose[:3])
            for key, item in (catalog.get('tags') or {}).items():
                tag_id = get_tag_id(key, item)
                pose = item.get('pose')
                if 0 <= tag_id < len(NAMES) and pose and len(pose) >= 3:
                    self.saved_tags[tag_id] = tuple(float(v) for v in pose[:3])
                    self.saved_counts[tag_id] = int(item.get('observations', 0))
            # Older dock-only files have no physical tag pose; infer it from the
            # documented 0.326 m standoff so those saved tags remain viewable.
            for tag_id, dock in self.saved_docks.items():
                if tag_id not in self.saved_tags:
                    yaw = dock[2]
                    standoff = DOCK_GAP + FRONT_OFFSET
                    self.saved_tags[tag_id] = (
                        dock[0] + standoff * math.cos(yaw),
                        dock[1] + standoff * math.sin(yaw),
                        yaw + math.pi)
        except (OSError, ValueError, TypeError, AttributeError, yaml.YAMLError) as error:
            self.catalog_error = str(error)
            self.get_logger().error(f'Cannot load existing survey catalog: {error}')

    def publish_saved_tags(self) -> None:
        """Publish persistent markers for the saved physical AprilTag poses."""
        markers = MarkerArray()
        now = self.get_clock().now().to_msg()
        if not self.map_identity or self.catalog_error:
            self.marker_pub.publish(markers)
            return
        for tag_id, pose in sorted(self.saved_tags.items()):
            x, y, yaw = pose
            for marker_id, marker_type in ((tag_id * 3, Marker.SPHERE),
                                           (tag_id * 3 + 1, Marker.ARROW),
                                           (tag_id * 3 + 2, Marker.TEXT_VIEW_FACING)):
                marker = Marker()
                marker.header.frame_id = self.map_frame
                marker.header.stamp = now
                marker.ns = f'tag_{NAMES[tag_id]}'
                marker.id = marker_id
                marker.type = marker_type
                marker.action = Marker.ADD
                marker.pose.position.x = x
                marker.pose.position.y = y
                marker.pose.position.z = 0.08 if marker_type != Marker.TEXT_VIEW_FACING else 0.28
                marker.pose.orientation.z = math.sin(yaw / 2)
                marker.pose.orientation.w = math.cos(yaw / 2)
                marker.scale.x = 0.12 if marker_type != Marker.TEXT_VIEW_FACING else 0.0
                marker.scale.y = 0.12 if marker_type != Marker.TEXT_VIEW_FACING else 0.0
                marker.scale.z = 0.12 if marker_type != Marker.TEXT_VIEW_FACING else 0.16
                marker.color.r, marker.color.g, marker.color.b, marker.color.a = (
                    1.0, 0.55, 0.05, 1.0)
                if marker_type == Marker.ARROW:
                    marker.scale.x, marker.scale.y, marker.scale.z = (0.22, 0.035, 0.035)
                if marker_type == Marker.TEXT_VIEW_FACING:
                    marker.text = NAMES[tag_id]
                markers.markers.append(marker)
        self.marker_pub.publish(markers)

    def on_detections(self, msg: AprilTagDetectionArray) -> None:
        now = time.monotonic()
        for detection in msg.detections:
            tag_id = int(detection.id)
            if not 0 <= tag_id < len(NAMES) or \
                    now - self.last_sample.get(tag_id, -1e9) < self.sample_period:
                continue
            poses = self.tag_and_dock_on_map(tag_id, msg.header.stamp)
            if poses is None:
                continue
            tag_pose, dock_pose = poses
            tag_samples = self.tag_samples.setdefault(tag_id, [])
            tag_samples.append(tag_pose)
            samples = self.dock_samples.setdefault(tag_id, [])
            samples.append(dock_pose)
            self.last_sample[tag_id] = now
            if len(samples) == 1 or len(samples) % 10 == 0:
                self.get_logger().info(
                    f'Tag {NAMES[tag_id]} (ID {tag_id}): {len(samples)} observations, '
                    f'tag at ({tag_pose[0]:.2f}, {tag_pose[1]:.2f}) on {self.map_frame}')

    def save_survey(self, request: Trigger.Request,
                    response: Trigger.Response) -> Trigger.Response:
        if not self.map_identity:
            response.success = False
            response.message = (
                f'Cannot save tags because the SLAM map identity is unavailable for '
                f'{self.map_file!r}; check map_file and the .posegraph/.data files.')
            return response
        if self.catalog_error:
            response.success = False
            response.message = (f'Existing catalog could not be loaded safely: '
                                f'{self.catalog_error}. Fix or move {self.output_file} first.')
            return response
        docks = dict(self.saved_docks)
        tags = dict(self.saved_tags)
        counts = dict(self.saved_counts)
        newly_saved = []
        for tag_id in range(len(NAMES)):
            dock_result = summarize(self.dock_samples.get(tag_id, []), self.min_observations,
                                     self.max_position_error, self.max_yaw_error)
            tag_result = summarize(self.tag_samples.get(tag_id, []), self.min_observations,
                                   self.max_position_error, self.max_yaw_error)
            if dock_result is not None and tag_result is not None:
                docks[tag_id], counts[tag_id] = dock_result
                tags[tag_id] = tag_result[0]
                newly_saved.append(tag_id)
        try:
            os.makedirs(os.path.dirname(self.output_file) or '.', exist_ok=True)
            with open(self.output_file, 'w', encoding='utf-8') as out:
                write_dock_database(out, docks, counts, self.map_frame, tags,
                                    self.map_identity)
        except OSError as error:
            response.success = False
            response.message = f'Could not write {self.output_file}: {error}'
            return response

        self.saved_docks, self.saved_tags, self.saved_counts = docks, tags, counts
        self.publish_saved_tags()
        missing = [NAMES[tag] for tag in range(len(NAMES)) if tag not in docks]
        saved_now = ', '.join(NAMES[tag] for tag in newly_saved) or 'none this call'
        response.success = True
        response.message = f'Saved new survey data for {saved_now}; catalog is {self.output_file}.'
        response.message += (' Missing tags/docks: ' + ', '.join(missing) + '.'
                             if missing else ' All tags A-H have saved docks.')
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
