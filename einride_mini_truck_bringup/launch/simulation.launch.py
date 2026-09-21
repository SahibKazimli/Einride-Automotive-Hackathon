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

"""Run the robot in Gazebo. The hardware counterpart is hardware.launch.py.

This file owns only what is specific to simulation - the Gazebo server, the
ros_gz bridge, the joint-state throttle, and ld19_scan_model, which finishes the
bridged scan into what the LD19 driver would have published. Everything shared
with hardware lives in common.launch.py.

scan_to_points is the one node that is not simulation-specific: hardware.launch.py
runs the same converter on the same topic, because /scan/points has to be built
the same way in both modes to be the same topic.
"""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_project_bringup = get_package_share_directory('einride_mini_truck_bringup')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    world = LaunchConfiguration('world')
    headless = LaunchConfiguration('headless')
    use_rviz = LaunchConfiguration('rviz')
    joint_state_rate = LaunchConfiguration('joint_state_rate')
    scan_intensity = LaunchConfiguration('scan_intensity')

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
    #
    # This stays here rather than in common.launch.py because only the bridge
    # produces /joint_states_raw. It moves across once the deferred odometry node
    # publishes that topic on hardware too.
    joint_state_throttle = Node(
        package='topic_tools',
        executable='throttle',
        name='joint_state_throttle',
        arguments=['messages', '/joint_states_raw', joint_state_rate, '/joint_states'],
        # Without this the throttle paces itself on wall time while every other
        # node here runs on /clock, so the rate it delivers is scaled by the
        # real-time factor - i.e. it depends on machine load. Measured against
        # message stamps at RTF 0.96, asking for 50: wall time gives 46.0 per
        # simulated second, sim time gives 49.5. The gap is small at RTF 0.96
        # and grows without bound as RTF falls, in the wrong direction - a
        # slower machine gets a *higher* rate, and so more TF traffic exactly
        # when it has least to spare.
        parameters=[{'use_sim_time': True}],
        output='both',
    )

    # gz's gpu_lidar produces the LD19's scan geometry (455 bins over 0..2*pi,
    # 0.02-12 m - see the sensor block in model.sdf) but not the LD19 driver's
    # message semantics. This node supplies the rest: NaN rather than +inf for a
    # bin that returned nothing, a measured scan_time and time_increment, and a
    # finite intensity on the bins that did return. The bridge hands it
    # /scan/raw and it publishes the /scan everything else subscribes to.
    ld19_scan_model = Node(
        package='einride_mini_truck_gazebo',
        executable='ld19_scan_model',
        name='ld19_scan_model',
        # Without value_type the substitution arrives as a string and the node
        # rejects it: it declared 'intensity' as a double.
        parameters=[{
            'use_sim_time': True,
            'intensity': ParameterValue(scan_intensity, value_type=float),
        }],
        output='both',
    )

    # The same converter hardware.launch.py runs, on the same input, so
    # /scan/points is produced identically in both modes rather than being gz's
    # separate ray-cast cloud. It has to come after ld19_scan_model, not off
    # /scan/raw, or the cloud would still carry the +inf bins.
    scan_to_points = Node(
        package='pointcloud_to_laserscan',
        executable='laserscan_to_pointcloud_node',
        name='scan_to_points',
        remappings=[('scan_in', '/scan'), ('cloud', '/scan/points')],
        parameters=[{'use_sim_time': True}],
        output='both',
    )

    # The frames inside the OAK-D Lite, which model.sdf deliberately does not
    # declare: on hardware depthai_ros_driver publishes them from the camera's own
    # EEPROM, because a description cannot know which physical camera is fitted.
    # This replays the same arithmetic over a checked-in dump of that EEPROM, so
    # both modes carry the same tree rather than two hand-synced copies of it.
    #
    # It publishes the six camera frames only. oak_imu_frame comes from model.sdf
    # in BOTH modes - the driver's version of that one transform is 120 degrees
    # wrong, so hardware suppresses it too. See the node's docstring.
    oak_calibration_tf = Node(
        package='einride_mini_truck_gazebo',
        executable='oak_calibration_tf',
        name='oak_calibration_tf',
        parameters=[{'use_sim_time': True}],
        output='both',
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

    common = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_project_bringup, 'launch', 'common.launch.py')),
        launch_arguments={
            'use_sim_time': 'true',
            'rviz': use_rviz,
            # gz's DiffDrive publishes odom -> base_footprint, so the world can
            # stay still and the robot drive through it.
            'fixed_frame': 'odom',
        }.items(),
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
        DeclareLaunchArgument(
            'scan_intensity', default_value='200.0',
            description=('Intensity for a /scan bin that returned something. '
                         'Gazebo does not model return strength, so this is a '
                         "placeholder inside the real device's observed 7-255; "
                         '0.0 restores raw Gazebo behaviour.')),
        gz_sim,
        bridge,
        ld19_scan_model,
        scan_to_points,
        joint_state_throttle,
        oak_calibration_tf,
        common,
    ])
