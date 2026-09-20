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

    # model.urdf, not model.sdf. robot_state_publisher reads either - it loads
    # sdformat_urdf as a urdf_parser plugin - and the two produce the same TF
    # tree, because model.urdf is generated from model.sdf by that very
    # converter at build time. What differs is who else can read the result:
    # RViz resolves `robot_description` through the same pluggable parser and so
    # accepts SDF, but Foxglove's 3D panel has its own URDF reader, understands
    # only <robot>, and given an <sdf> document renders nothing and reports
    # nothing. Publishing URDF is what makes the robot visible in both.
    #
    # model.sdf remains the single description; see einride_mini_truck_description
    # /tools/sdf_to_urdf.cpp. Gazebo is unaffected either way - it loads the SDF
    # itself through model://, never through this topic.
    urdf_file = os.path.join(pkg_project_description, 'models',
                             'einride_mini_truck', 'model.urdf')
    with open(urdf_file, 'r') as infp:
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

    # -f overrides the Fixed Frame baked into the .rviz file, which is what keeps
    # one config serving both modes instead of two that drift apart. It has to be
    # overridden, because the right frame genuinely differs: the file says odom,
    # and on hardware there is no odom frame at all until the deferred odometry
    # phase publishes one. RViz does not report that as an error - every display
    # that needs a transform to the fixed frame simply queues its messages until
    # the queue fills, and then logs "Message Filter dropping message" forever.
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', os.path.join(pkg_project_bringup, 'config',
                                      'einride_mini_truck.rviz'),
                   '-f', LaunchConfiguration('fixed_frame')],
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
        # No default, for the same reason use_sim_time has none: the correct
        # value is a property of the mode, and the failure when it is wrong is
        # silent. A third entry point gets an error here rather than an RViz
        # that drops every scan.
        DeclareLaunchArgument('fixed_frame',
                              description='RViz fixed frame. odom in '
                                          'simulation; base_footprint on '
                                          'hardware, which has no odom frame '
                                          'until wheel odometry lands.'),
        robot_state_publisher,
        rviz,
    ])
