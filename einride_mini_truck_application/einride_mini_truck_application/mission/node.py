"""Mission: Saga's next dock -> Nav2.

Docking runs in two steps so obstacles near the dock do not block it forever:
  1. pick a staging pose in front of the dock that is free on the live
     costmap (staging.py), and let Nav2 drive there around obstacles
     (NavigateToPose, with the short-recovery tree in staging_bt.py);
  2. DockRobot without the docking server's own fixed staging pose: it
     approaches from where we are, steered by the AprilTag.
If either step fails, the next attempt uses another staging pose. If no
staging pose is free (something parked in the bay), the mission waits and
tries again with a fresh costmap.

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
import os
import tempfile

from action_msgs.msg import GoalStatus
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator
from nav_msgs.msg import OccupancyGrid
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from std_msgs.msg import Int32, String
from tf2_ros import Buffer, TransformException, TransformListener

from .mission import Mission
from .staging import (choose_staging, Grid, load_dock_poses, load_limits, Pose,
                      staging_clearance)
from .staging_bt import write_tree

DOCK_TYPE = 'competition_dock'
CONFIG = os.path.join(get_package_share_directory('einride_mini_truck_application'), 'config')


class MissionIO(Node):
    """Topics for the mission; Nav2 is driven through a separate BasicNavigator."""

    def __init__(self) -> None:
        super().__init__('mission')
        self.declare_parameter('tick_period', 0.2)
        database = self.declare_parameter('dock_database', '').value
        self.dock_frame = self.declare_parameter('dock_frame', 'arena').value
        self.docks = load_dock_poses(database) if database else {}
        if not self.docks:
            self.get_logger().warn('No dock_database: using the docking server staging pose')
        # The same files Nav2 is launched with (launch/navigation.launch.py).
        nav2 = self.declare_parameter(
            'nav2_params', os.path.join(CONFIG, 'navigation', 'nav2.yaml')).value
        monitor = self.declare_parameter(
            'collision_monitor_params', os.path.join(CONFIG, 'safety', 'collision_monitor.yaml')
        ).value
        self.limits = load_limits(nav2, monitor)
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
        self.target_pub = self.create_publisher(Int32, '/mission/target_tag', latched)
        self.state_pub = self.create_publisher(String, '/mission/state', 10)

    def on_next_tag(self, msg: Int32) -> None:
        self.requested = None if msg.data < 0 else msg.data

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

    def pose_msg(self, pose: Pose) -> PoseStamped:
        msg = PoseStamped()
        msg.header.frame_id = self.grid_frame
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.position.x, msg.pose.position.y = pose[0], pose[1]
        msg.pose.orientation.z = math.sin(pose[2] / 2)
        msg.pose.orientation.w = math.cos(pose[2] / 2)
        return msg


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    io = MissionIO()
    nav = BasicNavigator('mission_navigator')
    log = io.get_logger()
    mission = Mission()
    period = io.get_parameter('tick_period').value
    task = None        # None, 'staging' (NavigateToPose), 'dock' or 'undock'
    target = None      # tag of the running dock attempt
    rejected = False   # Nav2 refused / no staging pose: report as a failed task
    staging = None     # staging pose of the running dock attempt
    failed: list[Pose] = []   # staging poses where docking at `target` failed
    last_state = None
    staging_tree = write_tree(tempfile.gettempdir())

    def go_to_staging(pose: Pose) -> bool:
        if nav.goToPose(io.pose_msg(pose), behavior_tree=staging_tree):
            return True
        log.warn(f'bt_navigator rejected {staging_tree}; using its default tree')
        return nav.goToPose(io.pose_msg(pose))

    def start_dock(tag: int) -> str | None:
        """Step 1: drive to a free staging pose. Returns the running task."""
        nonlocal staging
        staging = None
        if tag not in io.docks:   # no layout known: let the docking server stage itself
            return 'dock' if nav.dockRobotByID(f'dock_{tag}', nav_to_dock=True) else None
        dock = io.dock_in_grid_frame(tag)
        if dock is None:   # TF not received yet (right after startup): retry shortly
            return None
        if io.grid is None:   # picking blind would give the nominal pose, maybe in an obstacle
            log.info('No global costmap yet; waiting before choosing a staging pose')
            return None
        staging = choose_staging(io.grid, dock, io.limits, failed)
        if staging is None and failed:   # tried every free pose: start over
            log.warn(f'Docking at tag {tag} failed from every staging pose; trying them again')
            failed.clear()
            staging = choose_staging(io.grid, dock, io.limits)
        if staging is None:
            log.warn(f'Dock of tag {tag} is blocked (no free staging pose); waiting')
            return None
        room = staging_clearance(io.grid, staging, io.limits)
        log.info(f'Staging for tag {tag} at ({staging[0]:.2f}, {staging[1]:.2f}, '
                 f'{math.degrees(staging[2]):.0f} deg), clearance {room:.2f} m '
                 f'(needs {io.limits.turn_radius:.2f} to turn)')
        return 'staging' if go_to_staging(staging) else None

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
                    # Step 2: tag-guided approach from here.
                    task = 'dock' if nav.dockRobotByID(f'dock_{target}', nav_to_dock=False) \
                        else None
                    rejected = task is None
                else:
                    if task in ('staging', 'dock') and staging is not None:
                        if succeeded:
                            failed.clear()
                        else:   # Nav2 could not get there, or the tag was not seen from there
                            log.warn(f'Task {task} for staging pose ({staging[0]:.2f}, '
                                     f'{staging[1]:.2f}) failed (status {nav.status}); '
                                     'next try from another pose')
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
