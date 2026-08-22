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

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression

from launch_ros.actions import Node


def generate_launch_description():
    pkg_project_bringup = get_package_share_directory('einride_mini_truck_bringup')
    pkg_project_gazebo = get_package_share_directory('einride_mini_truck_gazebo')
    pkg_project_description = get_package_share_directory('einride_mini_truck_description')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    world = LaunchConfiguration('world')
    headless = LaunchConfiguration('headless')
    use_rviz = LaunchConfiguration('rviz')
    joint_state_rate = LaunchConfiguration('joint_state_rate')

    # robot_state_publisher parses the SDF directly via sdformat_urdf.
    sdf_file = os.path.join(pkg_project_description, 'models',
                            'einride_mini_truck', 'model.sdf')
    with open(sdf_file, 'r') as infp:
        robot_desc = infp.read()

    # Project GUI config: frames the camera on the robot rather than gz-sim's
    # 6 m default standoff. It must be a full copy of the default config, because
    # a partial <gui> replaces the default plugin set instead of merging with it.
    gui_config = os.path.join(pkg_project_bringup, 'config', 'gui.config')

    # '-r' starts the simulation running rather than paused, which is what you
    # almost always want from a launch file; '-s' drops the GUI when headless.
    gz_args = [
        PythonExpression(["'-s ' if '", headless, "'.lower() == 'true' else ''"]),
        PythonExpression(["'' if '", headless, "'.lower() == 'true' "
                          "else '--gui-config ", gui_config, " '"]),
        '-r -v 3 ', world, '.sdf',
    ]

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': gz_args}.items(),
    )

    # gz-sim's JointStatePublisher has no rate parameter and publishes every
    # physics step (~1 kHz). Throttle the bridged stream to something sane before
    # robot_state_publisher turns each message into a TF broadcast.
    joint_state_throttle = Node(
        package='topic_tools',
        executable='throttle',
        name='joint_state_throttle',
        arguments=['messages', '/joint_states_raw', joint_state_rate, '/joint_states'],
        output='both',
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='both',
        parameters=[
            {'use_sim_time': True},
            {'robot_description': robot_desc},
        ]
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{
            'config_file': os.path.join(pkg_project_bringup, 'config',
                                        'einride_mini_truck_bridge.yaml'),
            'qos_overrides./tf_static.publisher.durability': 'transient_local',
            'use_sim_time': True,
        }],
        output='screen'
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', os.path.join(pkg_project_bringup, 'config',
                                      'einride_mini_truck.rviz')],
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(use_rviz)
    )

    return LaunchDescription([
        DeclareLaunchArgument('world', default_value='einride_mini_truck',
                              description='World file basename (without .sdf).'),
        DeclareLaunchArgument('headless', default_value='false',
                              description='Run the Gazebo server without the GUI.'),
        DeclareLaunchArgument('rviz', default_value='true',
                              description='Open RViz.'),
        DeclareLaunchArgument('joint_state_rate', default_value='50',
                              description='Hz to throttle /joint_states_raw down to.'),
        gz_sim,
        bridge,
        joint_state_throttle,
        robot_state_publisher,
        rviz,
    ])
