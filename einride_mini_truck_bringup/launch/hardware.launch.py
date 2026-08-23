# Copyright 2025 Einride AB
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

"""Run the robot on real hardware. The simulation counterpart is simulation.launch.py.

This file owns only what is specific to hardware - the three device drivers.
Everything shared with simulation lives in common.launch.py, which is included
with use_sim_time false because there is no /clock on a real robot.

Not started here, and deliberately: the EKF and the joint-state throttle. Both
need topics that only the deferred odometry phase will provide, so starting them
now would give a filter with no input rather than an honest gap.
"""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_project_bringup = get_package_share_directory('einride_mini_truck_bringup')
    # Lazy, unlike the line above: the driver packages and the hardware package
    # are only needed on the robot, so nothing here may look them up until the
    # corresponding launch argument says they are wanted.
    pkg_project_hardware = FindPackageShare('einride_mini_truck_hardware')

    use_rviz = LaunchConfiguration('rviz')
    use_lidar = LaunchConfiguration('lidar')
    use_camera = LaunchConfiguration('camera')

    # The serial hardware abstraction layer: /cmd_vel in, /imu, /mag,
    # /wheel_encoders and /voltage out, reproducing what ros_gz_bridge provides
    # in simulation. Its parameters all live in hardware.yaml.
    ugv02_serial = Node(
        package='einride_mini_truck_hardware',
        executable='ugv02_serial',
        name='ugv02_serial_node',
        parameters=[
            LaunchConfiguration('hardware_params'),
            {'use_sim_time': False},
        ],
        output='both',
    )

    # LD19P 360-degree lidar. The stock ld19.launch.py already publishes on
    # /scan with frame_id base_lidar_link, which is exactly what the simulation
    # bridge publishes, so it is included unmodified.
    lidar = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('ldlidar'), 'launch', 'ld19.launch.py'])),
        condition=IfCondition(use_lidar),
    )

    # Simulation also publishes /scan/points, because gz's lidar sensor emits a
    # cloud alongside the scan. The LD19 driver has no equivalent, so derive one
    # from the scan to keep the contract whole. It is the same data either way -
    # a 2D scan lifted into the lidar frame - so nothing downstream can tell the
    # two modes apart.
    scan_to_points = Node(
        package='pointcloud_to_laserscan',
        executable='laserscan_to_pointcloud_node',
        name='scan_to_points',
        remappings=[('scan_in', '/scan'), ('cloud', '/scan/points')],
        parameters=[{'use_sim_time': False}],
        condition=IfCondition(use_lidar),
        output='both',
    )

    # OAK-D Lite. The resolution parameters in oak_d_lite.yaml are what make the
    # real camera's field of view match the simulated one - see "Matching the
    # real camera" in the README.
    camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('depthai_ros_driver'), 'launch', 'camera.launch.py'])),
        launch_arguments={
            'name': 'oak',
            # Attach the driver's internal frames under the link the model
            # declares, so the camera tree hangs off the robot rather than
            # floating in its own root.
            'parent_frame': 'oak_d_lite_link',
            'params_file': os.path.join(
                pkg_project_bringup, 'config', 'oak_d_lite.yaml'),
            'use_rviz': 'False',
        }.items(),
        condition=IfCondition(use_camera),
    )

    common = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_project_bringup, 'launch', 'common.launch.py')),
        launch_arguments={
            # No /clock exists on hardware. A node left at use_sim_time true
            # here waits at time zero forever and looks like a hang.
            'use_sim_time': 'false',
            'rviz': use_rviz,
        }.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'rviz', default_value='true',
            description='Open RViz. Pass false on a headless Jetson.'),
        DeclareLaunchArgument(
            'lidar', default_value='true',
            description='Start the LD19P driver and its /scan/points converter.'),
        DeclareLaunchArgument(
            'camera', default_value='true',
            description='Start the OAK-D Lite driver.'),
        DeclareLaunchArgument(
            'hardware_params',
            default_value=PathJoinSubstitution(
                [pkg_project_hardware, 'config', 'hardware.yaml']),
            description='Parameter file for ugv02_serial_node.'),
        ugv02_serial,
        lidar,
        scan_to_points,
        camera,
        common,
    ])
