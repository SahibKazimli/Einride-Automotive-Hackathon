"""The whole application. Runs next to the einride-mini-truck service (drivers).

    ros2 launch einride_mini_truck_application app.launch.py
    ros2 launch einride_mini_truck_application app.launch.py layout:=arena start_x:=0.0 start_y:=0.0
    ros2 launch einride_mini_truck_application app.launch.py mission:=false   # no Saga/mission
    ros2 launch einride_mini_truck_application app.launch.py mission:=false \
        tag_survey:=true use_slam_map:=true  # precompetition tag survey

Saga settings come from config/saga/saga.secret.yaml if it exists, else
saga.example.yaml.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    share = get_package_share_directory('einride_mini_truck_application')

    def include(name: str, **arguments) -> IncludeLaunchDescription:
        return IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(share, 'launch', name)),
            launch_arguments=arguments.items())

    saga_dir = os.path.join(share, 'config', 'saga')
    saga_params = os.path.join(saga_dir, 'saga.secret.yaml')
    if not os.path.exists(saga_params):
        saga_params = os.path.join(saga_dir, 'saga.example.yaml')

    mission = IfCondition(LaunchConfiguration('mission'))
    docks = PathJoinSubstitution(
        [share, 'config', 'docks', [LaunchConfiguration('layout'), '.yaml']])
    return LaunchDescription([
        DeclareLaunchArgument('layout', default_value='home',
                              description='config/docks/<layout>.yaml'),
        DeclareLaunchArgument('start_x', default_value='0.0'),
        DeclareLaunchArgument('start_y', default_value='0.0'),
        DeclareLaunchArgument('start_yaw', default_value='0.0'),
        DeclareLaunchArgument('mission', default_value='true',
                              description='start the Saga client and mission'),
        DeclareLaunchArgument('rectify', default_value='false',
                              description='undistort the image before tag detection'),
        DeclareLaunchArgument('tag_survey', default_value='false',
                              description='detect and survey all AprilTags in map frame'),
        DeclareLaunchArgument(
            'tag_survey_output',
            default_value=os.path.expanduser(
                '~/ws/src/Einride-Automotive-Hackathon/'
                'einride_mini_truck_application/config/docks/arena_tag_survey.yaml'),
            description='file written by the AprilTag survey save service'),
        DeclareLaunchArgument('use_slam_map', default_value='false',
                              description='SLAM publishes map->odom; omit arena->odom'),

        include('localization.launch.py'),
        include('perception.launch.py', rectify=LaunchConfiguration('rectify'),
                layout=LaunchConfiguration('layout'),
                survey_mode=LaunchConfiguration('tag_survey'),
                survey_output=LaunchConfiguration('tag_survey_output')),
        include('navigation.launch.py',
                layout=LaunchConfiguration('layout'),
                start_x=LaunchConfiguration('start_x'),
                start_y=LaunchConfiguration('start_y'),
                start_yaw=LaunchConfiguration('start_yaw'),
                use_slam_map=LaunchConfiguration('use_slam_map')),

        Node(package='einride_mini_truck_application', executable='saga', name='saga',
             output='screen', parameters=[saga_params], condition=mission),
        Node(package='einride_mini_truck_application', executable='mission',
             output='screen', parameters=[{'dock_database': docks}], condition=mission),
    ])
