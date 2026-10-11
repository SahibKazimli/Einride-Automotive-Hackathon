"""Perception: colour image -> apriltag_ros -> dock pose for docking.

The dock_pose node gates the camera: apriltag_ros reads /dock_camera/*, which
only carries images near the target dock (config/docks/<layout>.yaml), so the
detector does not burn a CPU core while driving between docks.

Arguments
    layout   home (default) | arena. Dock positions for the camera gate.
    rectify  false (default) | true. Undistort the image before detection.
             More accurate away from the image centre, but costs a full
             1280x720 copy per frame on a CPU the camera driver already
             loads heavily. While docking, the tag is near the centre, where
             the OAK-D lens distortion is small.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    share = get_package_share_directory('einride_mini_truck_application')
    apriltag_params = os.path.join(share, 'config', 'perception', 'apriltag.yaml')
    rectify = LaunchConfiguration('rectify')
    survey_mode = LaunchConfiguration('survey_mode')
    survey_output = LaunchConfiguration('survey_output')
    survey_map_file = LaunchConfiguration('survey_map_file')
    docks = PathJoinSubstitution(
        [share, 'config', 'docks', [LaunchConfiguration('layout'), '.yaml']])

    def apriltag(image_topic: str, condition) -> Node:
        # camera_info is found next to the image topic automatically.
        return Node(package='apriltag_ros', executable='apriltag_node', name='apriltag',
                    output='screen', parameters=[apriltag_params], condition=condition,
                    remappings=[('image_rect', image_topic),
                                ('camera_info', '/dock_camera/camera_info')])

    return LaunchDescription([
        DeclareLaunchArgument('rectify', default_value='false'),
        DeclareLaunchArgument('layout', default_value='home'),
        DeclareLaunchArgument('survey_mode', default_value='false'),
        DeclareLaunchArgument('survey_output', default_value='~/.ros/arena_tag_survey.yaml'),
        DeclareLaunchArgument('survey_map_file', default_value=''),
        Node(package='image_proc', executable='rectify_node', name='rectify_color',
             output='screen', condition=IfCondition(rectify),
             remappings=[('image', '/dock_camera/image_raw'),
                         ('image_rect', '/dock_camera/image_rect')]),
        apriltag('/dock_camera/image_rect', IfCondition(rectify)),
        apriltag('/dock_camera/image_raw', UnlessCondition(rectify)),
        Node(package='einride_mini_truck_application', executable='dock_pose',
             output='screen', parameters=[
                 {'dock_database': docks,
                  'survey_mode': ParameterValue(survey_mode, value_type=bool)}]),
        Node(package='einride_mini_truck_application', executable='tag_survey',
             output='screen', condition=IfCondition(survey_mode),
             parameters=[{'output_file': survey_output, 'map_file': survey_map_file}]),
    ])
