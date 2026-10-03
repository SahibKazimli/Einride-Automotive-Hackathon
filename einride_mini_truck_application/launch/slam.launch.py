"""SLAM (slam_toolbox): build a LiDAR map of the arena, or localize against one.

    # 1. build a map while you drive the robot around:
    ros2 launch einride_mini_truck_application slam.launch.py mode:=mapping use_sim_time:=true
    #    ...then save it (writes maps/arena.posegraph + maps/arena.data):
    ros2 service call /slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph \
        "{filename: '/einride_mini_truck_ws/src/einride_mini_truck_application/maps/arena'}"

    # 2. localize against the saved map (architecture "version 4"):
    ros2 launch einride_mini_truck_application slam.launch.py mode:=localization use_sim_time:=true

slam_toolbox publishes map -> odom. odom -> base_footprint still comes from
Gazebo's DiffDrive (sim) or the EKF (hardware), so SLAM only ADDS the global
correction - it does not replace the odometry underneath it.

slam_toolbox's nodes are LIFECYCLE nodes: launching them is not enough, they
only create their /scan subscription and /map publisher once configured AND
activated. This launch does both transitions automatically.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler
from launch.events import matches_action
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import LifecycleNode
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from launch_ros.parameter_descriptions import ParameterValue
from lifecycle_msgs.msg import Transition


def generate_launch_description() -> LaunchDescription:
    share = get_package_share_directory('einride_mini_truck_application')
    cfg = os.path.join(share, 'config', 'localization')
    mapping_yaml = os.path.join(cfg, 'slam_mapping.yaml')
    localisation_yaml = os.path.join(cfg, 'slam_localisation.yaml')

    mode = LaunchConfiguration('mode')
    # mapping -> async_slam_toolbox_node + slam_mapping.yaml
    # localization -> localization_slam_toolbox_node + slam_localisation.yaml
    executable = PythonExpression(
        ["'async_slam_toolbox_node' if '", mode, "' == 'mapping' "
         "else 'localization_slam_toolbox_node'"])
    config = PythonExpression(
        ["'", mapping_yaml, "' if '", mode, "' == 'mapping' else '",
         localisation_yaml, "'"])
    use_sim_time = {'use_sim_time': ParameterValue(
        LaunchConfiguration('use_sim_time'), value_type=bool)}

    slam = LifecycleNode(
        package='slam_toolbox', executable=executable, name='slam_toolbox',
        namespace='', output='screen', parameters=[config, use_sim_time])

    # Drive the lifecycle automatically: configure now, and activate as soon as
    # configuring succeeds (reaches the 'inactive' state).
    configure = EmitEvent(event=ChangeState(
        lifecycle_node_matcher=matches_action(slam),
        transition_id=Transition.TRANSITION_CONFIGURE))
    activate = RegisterEventHandler(OnStateTransition(
        target_lifecycle_node=slam, goal_state='inactive',
        entities=[EmitEvent(event=ChangeState(
            lifecycle_node_matcher=matches_action(slam),
            transition_id=Transition.TRANSITION_ACTIVATE))]))

    return LaunchDescription([
        DeclareLaunchArgument('mode', default_value='mapping',
                              description='mapping | localization'),
        DeclareLaunchArgument('use_sim_time', default_value='false',
                              description='true against the simulation.'),
        slam,
        configure,
        activate,
    ])
