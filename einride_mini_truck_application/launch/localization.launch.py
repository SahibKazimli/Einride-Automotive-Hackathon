"""Localization: wheel speed + bias-free gyro -> robot_localization EKF -> /odom + TF."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    share = get_package_share_directory('einride_mini_truck_application')
    return LaunchDescription([
        Node(package='einride_mini_truck_application', executable='wheel_odometry',
             output='screen'),
        Node(package='robot_localization', executable='ekf_node', name='ekf_filter_node',
             output='screen',
             parameters=[os.path.join(share, 'config', 'localization', 'ekf.yaml')],
             remappings=[('odometry/filtered', '/odom')]),
    ])
