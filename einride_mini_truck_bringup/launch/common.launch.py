# Copyright 2022 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Nodes that are identical in simulation and on hardware.

Included by both simulation.launch.py and hardware.launch.py, which differ only
in the value of use_sim_time. Anything that runs in exactly one of the two modes
belongs in that file, not here - this is the single definition of the shared
set, and the reason the two entry points cannot quietly drift apart.

The EKF joins this file once the odometry phase lands and /odom exists on
hardware; the joint-state throttle joins it once /joint_states_raw does.
"""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_project_bringup = get_package_share_directory('einride_mini_truck_bringup')
    pkg_project_description = get_package_share_directory('einride_mini_truck_description')

    use_sim_time = ParameterValue(
        LaunchConfiguration('use_sim_time'), value_type=bool)
    use_rviz = LaunchConfiguration('rviz')

    # robot_state_publisher parses the SDF directly via sdformat_urdf. The same
    # file describes the simulated and the real robot, so the TF tree, the RViz
    # RobotModel and every frame name are identical in both modes.
    sdf_file = os.path.join(pkg_project_description, 'models',
                            'einride_mini_truck', 'model.sdf')
    with open(sdf_file, 'r') as infp:
        robot_desc = infp.read()

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='both',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'robot_description': robot_desc},
        ]
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', os.path.join(pkg_project_bringup, 'config',
                                      'einride_mini_truck.rviz')],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(use_rviz)
    )

    return LaunchDescription([
        # No default: each top-level file must state which mode it is, because
        # getting this wrong is the single most likely cause of "works in sim,
        # silently stalls on hardware". A node with use_sim_time true and no
        # /clock publisher sits at time zero forever.
        DeclareLaunchArgument('use_sim_time',
                              description='Take time from /clock. True in '
                                          'simulation, false on hardware.'),
        DeclareLaunchArgument('rviz', default_value='true',
                              description='Open RViz.'),
        robot_state_publisher,
        rviz,
    ])
