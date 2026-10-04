"""Navigation: Nav2 (map-free, in odom) + docking server + collision monitor.

Started node by node instead of through nav2_bringup so that:
* only what we use runs (no map server, AMCL, route server, waypoint follower),
* the docking server's commands also go through the velocity smoother and the
  collision monitor (nav2_bringup sends them straight to /cmd_vel).

Arguments
    layout     home | arena: which config/docks/<layout>.yaml to use
    start_x, start_y, start_yaw: robot start pose in that layout's frame
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.conditions import UnlessCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node

LIFECYCLE_NODES = [
    'controller_server',
    'planner_server',
    'behavior_server',
    'velocity_smoother',
    'collision_monitor',
    'bt_navigator',
    'docking_server',
]


def generate_launch_description() -> LaunchDescription:
    share = get_package_share_directory('einride_mini_truck_application')
    nav2 = os.path.join(share, 'config', 'navigation', 'nav2.yaml')
    safety = os.path.join(share, 'config', 'safety', 'collision_monitor.yaml')
    scan_filter = os.path.join(share, 'config', 'safety', 'scan_filter.yaml')
    docks = PathJoinSubstitution(
        [share, 'config', 'docks', [LaunchConfiguration('layout'), '.yaml']])
    to_nav = [('cmd_vel', 'cmd_vel_nav')]

    return LaunchDescription([
        DeclareLaunchArgument('layout', default_value='home'),
        DeclareLaunchArgument('start_x', default_value='0.0'),
        DeclareLaunchArgument('start_y', default_value='0.0'),
        DeclareLaunchArgument('start_yaw', default_value='0.0'),
        DeclareLaunchArgument('use_slam_map', default_value='false',
                              description='SLAM publishes map->odom; omit arena->odom'),

        # Dock poses are written in the "arena" frame; odom starts where the
        # robot was switched on, so arena -> odom is the start pose. When SLAM
        # is active it publishes map -> odom instead; do not give odom two
        # competing world parents.
        Node(package='tf2_ros', executable='static_transform_publisher',
             name='arena_to_odom',
             arguments=['--x', LaunchConfiguration('start_x'),
                        '--y', LaunchConfiguration('start_y'),
                        '--yaw', LaunchConfiguration('start_yaw'),
                        '--frame-id', 'arena', '--child-frame-id', 'odom'],
             condition=UnlessCondition(LaunchConfiguration('use_slam_map'))),

        # /scan without the points that hit the robot itself -> /scan_filtered.
        Node(package='laser_filters', executable='scan_to_scan_filter_chain',
             name='scan_filter', parameters=[scan_filter],
             remappings=[('scan', '/scan'), ('scan_filtered', '/scan_filtered')]),

        Node(package='nav2_controller', executable='controller_server', output='screen',
             parameters=[nav2], remappings=to_nav),
        Node(package='nav2_planner', executable='planner_server', name='planner_server',
             output='screen', parameters=[nav2]),
        Node(package='nav2_behaviors', executable='behavior_server', name='behavior_server',
             output='screen', parameters=[nav2], remappings=to_nav),
        Node(package='nav2_bt_navigator', executable='bt_navigator', name='bt_navigator',
             output='screen', parameters=[nav2]),
        Node(package='nav2_velocity_smoother', executable='velocity_smoother',
             name='velocity_smoother', output='screen', parameters=[nav2],
             remappings=to_nav),   # in: cmd_vel_nav, out: cmd_vel_smoothed
        Node(package='nav2_collision_monitor', executable='collision_monitor',
             name='collision_monitor', output='screen', parameters=[safety]),
        Node(package='opennav_docking', executable='opennav_docking', name='docking_server',
             output='screen', parameters=[nav2, {'dock_database': docks}],
             remappings=to_nav),
        # Started late: on the busy Jetson, bt_navigator takes several seconds
        # to load its plugins, and a lifecycle manager that asks too early
        # gives up and aborts the whole bring-up.
        TimerAction(period=5.0, actions=[
            Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
                 name='lifecycle_manager_navigation', output='screen',
                 parameters=[{'autostart': True, 'node_names': LIFECYCLE_NODES,
                              'bond_timeout': 10.0}]),
        ]),
    ])
