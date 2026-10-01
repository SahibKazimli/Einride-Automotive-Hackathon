"""Perception: colour image -> apriltag_ros -> dock pose for docking.

Arguments
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
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    share = get_package_share_directory('einride_mini_truck_application')
    apriltag_params = os.path.join(share, 'config', 'perception', 'apriltag.yaml')
    rectify = LaunchConfiguration('rectify')

    def apriltag(image_topic: str, condition) -> Node:
        # camera_info is found next to the image topic automatically.
        return Node(package='apriltag_ros', executable='apriltag_node', name='apriltag',
                    output='screen', parameters=[apriltag_params], condition=condition,
                    remappings=[('image_rect', image_topic),
                                ('camera_info', '/oak/rgb/camera_info')])

    return LaunchDescription([
        DeclareLaunchArgument('rectify', default_value='false'),
        Node(package='image_proc', executable='rectify_node', name='rectify_color',
             output='screen', condition=IfCondition(rectify),
             remappings=[('image', '/oak/rgb/image_raw'),
                         ('image_rect', '/oak/rgb/image_rect')]),
        apriltag('/oak/rgb/image_rect', IfCondition(rectify)),
        apriltag('/oak/rgb/image_raw', UnlessCondition(rectify)),
        Node(package='einride_mini_truck_application', executable='dock_pose',
             output='screen'),
    ])
