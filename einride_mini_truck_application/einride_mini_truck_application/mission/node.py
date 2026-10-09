"""Mission: Saga's next dock -> Nav2.

Docking runs in two steps so obstacles near the dock do not block it forever:
  1. pick a staging pose in front of the dock that is free on the live
     costmap (staging.py), and let Nav2 drive there around obstacles
     (NavigateToPose);
  2. DockRobot without the docking server's own fixed staging pose: it
     approaches from where we are, steered by the AprilTag.
If no staging pose is free (something parked in the bay), the mission waits
and tries again with a fresh costmap.

Subscribes
    /saga/next_tag (std_msgs/Int32): tag id to go to, -1 = stay still
    /global_costmap/costmap (nav_msgs/OccupancyGrid): to find a free staging pose
Publishes
    /mission/target_tag (std_msgs/Int32): tells perception which tag to track
    /mission/state (std_msgs/String): for watching in Foxglove
Uses
    Nav2 NavigateToPose / DockRobot / UndockRobot via nav2_simple_commander.
    Dock instances are named dock_<tag id> in config/docks/*.yaml.
"""

import math

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import DockRobot
from nav2_simple_commander.robot_navigator import BasicNavigator
from nav_msgs.msg import OccupancyGrid
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from std_msgs.msg import Int32, String
from tf2_ros import Buffer, TransformException, TransformListener

from .mission import Mission
from .staging import (choose_staging, Grid, load_dock_frame, load_dock_poses, Pose,
                      realign_goal, turn_to_tag)

DOCK_TYPE = 'competition_dock'
# Smaller turns are not worth a Spin: the tag is then already near image centre.
MIN_FACE_TURN = math.radians(10.0)
# A camera dock pose older than this (s) is not trusted for re-staging; with the
# camera clock behind the system clock every pose looks old, so this is skipped.
MAX_SEEN_DOCK_AGE = 1.0


class MissionIO(Node):
    """Topics for the mission; Nav2 is driven through a separate BasicNavigator."""

    def __init__(self) -> None:
        super().__init__('mission')
        self.declare_parameter('tick_period', 0.2)
        database = self.declare_parameter('dock_database', '').value
        # The dock file names its frame (arena, or map for a tag survey).
        default_frame = self.declare_parameter('dock_frame', 'arena').value
        self.dock_frame = (load_dock_frame(database) if database else None) or default_frame
        self.docks = load_dock_poses(database) if database else {}
        if not self.docks:
            self.get_logger().warn('No dock_database: using the docking server staging pose')
        self.requested: int | None = None
        self.grid: Grid | None = None
        self.grid_frame = 'odom'
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(Int32, '/saga/next_tag', self.on_next_tag, latched)
        costmap_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                                 reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(OccupancyGrid, '/global_costmap/costmap',
                                 self.on_costmap, costmap_qos)
        # Docked pose from the live tag (perception/node.py), in base_footprint.
        self.seen: PoseStamped | None = None
        self.create_subscription(PoseStamped, '/detected_dock_pose', self.on_seen_dock, 10)
        self.target_pub = self.create_publisher(Int32, '/mission/target_tag', latched)
        self.state_pub = self.create_publisher(String, '/mission/state', 10)

    def on_next_tag(self, msg: Int32) -> None:
        self.requested = None if msg.data < 0 else msg.data

    def on_seen_dock(self, msg: PoseStamped) -> None:
        self.seen = msg

    def seen_dock(self) -> Pose | None:
        """The camera's docked pose in the robot frame, None if missing or stale."""
        msg = self.seen
        if msg is None or msg.header.frame_id != 'base_footprint':
            return None
        age = (self.get_clock().now() - Time.from_msg(msg.header.stamp)).nanoseconds * 1e-9
        if not 0.0 <= age <= MAX_SEEN_DOCK_AGE:
            return None
        q = msg.pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        return (msg.pose.position.x, msg.pose.position.y, yaw)

    def on_costmap(self, msg: OccupancyGrid) -> None:
        info = msg.info
        self.grid_frame = msg.header.frame_id
        self.grid = Grid(info.origin.position.x, info.origin.position.y,
                         info.resolution, info.width, info.height, msg.data)

    def dock_in_grid_frame(self, tag: int) -> Pose | None:
        """The docked pose of `tag` in the costmap's frame, None if unknown."""
        if tag not in self.docks:
            return None
        x, y, yaw = self.docks[tag]
        try:
            tf = self.tf_buffer.lookup_transform(self.grid_frame, self.dock_frame, Time())
        except TransformException as error:
            self.get_logger().warn(f'No TF {self.grid_frame} <- {self.dock_frame}: {error}')
            return None
        t, q = tf.transform.translation, tf.transform.rotation
        rot = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        c, s = math.cos(rot), math.sin(rot)
        return (t.x + c * x - s * y, t.y + s * x + c * y, yaw + rot)

    def robot_pose(self) -> Pose | None:
        """base_footprint in the costmap's frame, None if TF is not there yet."""
        try:
            tf = self.tf_buffer.lookup_transform(self.grid_frame, 'base_footprint', Time())
        except TransformException:
            return None
        t, q = tf.transform.translation, tf.transform.rotation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        return (t.x, t.y, yaw)

    def pose_msg(self, pose: Pose) -> PoseStamped:
        msg = PoseStamped()
        msg.header.frame_id = self.grid_frame
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.position.x, msg.pose.position.y = pose[0], pose[1]
        msg.pose.orientation.z = math.sin(pose[2] / 2)
        msg.pose.orientation.w = math.cos(pose[2] / 2)
        return msg


def lost_tag(nav: BasicNavigator) -> bool:
    """True if the finished DockRobot failed because the tag was not seen."""
    try:
        return nav.result_future.result().result.error_code == \
            DockRobot.Result.FAILED_TO_DETECT_DOCK
    except AttributeError:   # no result (cancelled) or an older nav2_msgs
        return False


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    io = MissionIO()
    nav = BasicNavigator('mission_navigator')
    log = io.get_logger()
    mission = Mission()
    period = io.get_parameter('tick_period').value
    # None, 'staging' / 'align' (NavigateToPose), 'face' (Spin), 'dock' or 'undock'
    task = None
    target = None      # tag of the running dock attempt
    rejected = False   # Nav2 refused / no staging pose: report as a failed task
    staging = None     # staging pose of the running dock attempt
    failed: list[Pose] = []   # staging poses where docking at `target` failed
    retried = False    # the running dock attempt was already retried in place
    aligned = False    # this staging pose was already corrected from the live tag
    last_state = None

    def start_dock(tag: int) -> str | None:
        """Step 1: drive to a free staging pose. Returns the running task."""
        nonlocal staging, aligned
        staging = None
        aligned = False
        if tag not in io.docks:   # no layout known: let the docking server stage itself
            return 'dock' if nav.dockRobotByID(f'dock_{tag}', nav_to_dock=True) else None
        dock = io.dock_in_grid_frame(tag)
        if dock is None:   # TF not received yet (right after startup): retry shortly
            return None
        if io.grid is None:   # picking blind would give the nominal pose, maybe in an obstacle
            log.info('No global costmap yet; waiting before choosing a staging pose')
            return None
        staging = choose_staging(io.grid, dock, failed)
        if staging is None and failed:   # tried every free pose: start over
            log.warn(f'Docking at tag {tag} failed from every staging pose; trying them again')
            failed.clear()
            staging = choose_staging(io.grid, dock)
        if staging is None:
            log.warn(f'Dock of tag {tag} is blocked (no free staging pose); waiting')
            return None
        log.info(f'Staging for tag {tag} at ({staging[0]:.2f}, {staging[1]:.2f}, '
                 f'{math.degrees(staging[2]):.0f} deg)')
        return 'staging' if nav.goToPose(io.pose_msg(staging)) else None

    try:
        log.info('Waiting for Nav2 to come up...')
        # robot_localization replaces AMCL, so do not wait for an initial pose.
        nav.waitUntilNav2Active(localizer='robot_localization')
        log.info('Nav2 is up. Waiting for Saga.')

        while rclpy.ok():
            rclpy.spin_once(io, timeout_sec=period)
            now = io.get_clock().now().nanoseconds * 1e-9

            done = ok = False
            if rejected:
                rejected = False
                done = True
            elif task is not None and nav.isTaskComplete():
                succeeded = nav.status == GoalStatus.STATUS_SUCCEEDED
                if task == 'staging' and succeeded:
                    # Step 2: if the camera shows the robot off the real tag's
                    # centre line (skewed survey), first re-stage straight in
                    # front of the tag it sees; then the tag-guided approach.
                    robot, seen = io.robot_pose(), io.seen_dock()
                    goal = realign_goal(robot, seen) if robot and seen and not aligned \
                        else None
                    if goal is not None and nav.goToPose(io.pose_msg(goal)):
                        aligned = True
                        log.info(f'Off the centre line of tag {target}; re-staging straight '
                                 f'at ({goal[0]:.2f}, {goal[1]:.2f}, '
                                 f'{math.degrees(goal[2]):.0f} deg)')
                        task = 'align'
                    else:
                        task = 'dock' if nav.dockRobotByID(f'dock_{target}',
                                                           nav_to_dock=False) else None
                        rejected = task is None
                    retried = False
                elif task == 'align':   # straightened up (or Nav2 could not): dock
                    task = 'dock' if nav.dockRobotByID(f'dock_{target}', nav_to_dock=False) \
                        else None
                    rejected = task is None
                elif task == 'dock' and not succeeded and not retried and lost_tag(nav):
                    # The tag dropped out mid-approach. The robot is near the dock,
                    # so try again from here before driving off to another staging
                    # pose (that turn-away-and-back looked erratic, run_2137). The
                    # approach curve can leave the tag outside the camera's view
                    # (run_2226, 2026-10-03), so first turn to face where it is.
                    retried = True
                    robot, dock = io.robot_pose(), io.dock_in_grid_frame(target)
                    turn = turn_to_tag(robot, dock) if robot and dock else 0.0
                    log.warn(f'Lost tag {target} while docking; turning '
                             f'{math.degrees(turn):.0f} deg to it and retrying from here')
                    if abs(turn) >= MIN_FACE_TURN and nav.spin(spin_dist=turn):
                        task = 'face'
                    else:
                        task = 'dock' if nav.dockRobotByID(f'dock_{target}',
                                                           nav_to_dock=False) else None
                        rejected = task is None
                elif task == 'face':   # turned (or could not): dock from here
                    task = 'dock' if nav.dockRobotByID(f'dock_{target}', nav_to_dock=False) \
                        else None
                    rejected = task is None
                else:
                    if task in ('staging', 'dock') and staging is not None:
                        if succeeded:
                            failed.clear()
                        else:   # e.g. tag not seen from there: look from elsewhere next
                            failed.append(staging)
                    task = None
                    done = True
                    ok = succeeded

            action = mission.step(now, io.requested, done, ok)
            if action is not None:
                if action.kind == 'dock':
                    if action.tag != target:
                        failed.clear()
                    target = action.tag
                    io.target_pub.publish(Int32(data=action.tag))
                    task = start_dock(action.tag)
                    rejected = task is None
                elif action.kind == 'undock':
                    task = 'undock' if nav.undockRobot(DOCK_TYPE) else None
                    rejected = task is None
                elif action.kind == 'cancel':
                    nav.cancelTask()
                    task = None

            if mission.state != last_state:
                last_state = mission.state
                log.info(f'Mission: {mission.state} (target {mission.target})')
            io.state_pub.publish(String(data=f'{mission.state} target={mission.target}'))
    except KeyboardInterrupt:
        pass
    finally:
        if task is not None:
            nav.cancelTask()
        nav.destroy_node()
        io.destroy_node()
        rclpy.try_shutdown()
