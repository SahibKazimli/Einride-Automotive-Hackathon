"""Mission: Saga's next dock -> Nav2 docking server.

Subscribes
    /saga/next_tag (std_msgs/Int32): tag id to go to, -1 = stay still
Publishes
    /mission/target_tag (std_msgs/Int32): tells perception which tag to track
    /mission/state (std_msgs/String): for watching in Foxglove
Uses
    Nav2 DockRobot / UndockRobot actions via nav2_simple_commander. Dock
    instances are named dock_<tag id> in config/docks/*.yaml.
"""

from action_msgs.msg import GoalStatus
from nav2_simple_commander.robot_navigator import BasicNavigator
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import Int32, String

from .mission import Mission

DOCK_TYPE = 'competition_dock'


class MissionIO(Node):
    """Topics for the mission; Nav2 is driven through a separate BasicNavigator."""

    def __init__(self) -> None:
        super().__init__('mission')
        self.declare_parameter('tick_period', 0.2)
        self.requested: int | None = None
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(Int32, '/saga/next_tag', self.on_next_tag, latched)
        self.target_pub = self.create_publisher(Int32, '/mission/target_tag', latched)
        self.state_pub = self.create_publisher(String, '/mission/state', 10)

    def on_next_tag(self, msg: Int32) -> None:
        self.requested = None if msg.data < 0 else msg.data


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    io = MissionIO()
    nav = BasicNavigator('mission_navigator')
    log = io.get_logger()
    mission = Mission()
    period = io.get_parameter('tick_period').value
    task_running = False
    rejected = False   # Nav2 refused the last request: report it as a failed task
    last_state = None

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
            elif task_running and nav.isTaskComplete():
                task_running = False
                done = True
                ok = nav.status == GoalStatus.STATUS_SUCCEEDED

            action = mission.step(now, io.requested, done, ok)
            if action is not None:
                if action.kind == 'dock':
                    io.target_pub.publish(Int32(data=action.tag))
                    task_running = nav.dockRobotByID(f'dock_{action.tag}', nav_to_dock=True)
                    rejected = not task_running
                elif action.kind == 'undock':
                    task_running = nav.undockRobot(DOCK_TYPE)
                    rejected = not task_running
                elif action.kind == 'cancel':
                    nav.cancelTask()
                    task_running = False

            if mission.state != last_state:
                last_state = mission.state
                log.info(f'Mission: {mission.state} (target {mission.target})')
            io.state_pub.publish(String(data=f'{mission.state} target={mission.target}'))
    except KeyboardInterrupt:
        pass
    finally:
        if task_running:
            nav.cancelTask()
        nav.destroy_node()
        io.destroy_node()
        rclpy.try_shutdown()
