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

scan_to_points is the exception: it is not hardware-specific, and
simulation.launch.py starts the same node on the same topic. /scan/points has to
be built the same way in both modes to be the same topic.

Not started here, and deliberately: the EKF. It needs /odom, which only the
deferred odometry phase will provide, so starting it now would give a filter
with no input rather than an honest gap.

joint_state_relay is started here, not deferred with the EKF: it only carries
the two wheels ugv02_serial_node actually instruments, so it is a partial fix
rather than nothing - see the node's own comment below.
"""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression

from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode
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

    # LD19P 360-degree lidar, driven by ldlidar_component from
    # github.com/Myzhar/ldrobot-lidar-ros2.
    #
    # The component is loaded into a container here rather than through that
    # package's own ldlidar_bringup.launch.py, which cannot be used as-is for
    # two independent reasons:
    #
    #  * it starts a second robot_state_publisher, with the bare lidar's own
    #    URDF, on /robot_description and /tf. common.launch.py already publishes
    #    the whole robot on those topics, so the two would fight over the frames
    #    and the TF tree would flicker between them. Exactly the collision the
    #    camera comment below describes, and avoided the same way: take the node,
    #    not the launch file.
    #  * it hardcodes its own params/ldlidar.yaml, so the port, the frame and the
    #    range could not be set. config/ldlidar.yaml explains what has to change
    #    and why.
    #
    # ldlidar_component ships no standalone executable - the whole repo's only
    # main() functions belong to the vendor SDK's demo programs - so a container
    # is not a design choice here, it is the only way to run the node at all.
    #
    # Container and component are declared together rather than as a Node plus a
    # LoadComposableNodes, which would have to name its target as the literal
    # string '/ldlidar_container'. A rename or an added namespace does not make
    # that lookup fail loudly; it makes it wait. One silent failure mode around
    # this driver is enough - see the remap note below.
    #
    # use_intra_process_comms buys nothing while this is the only component in
    # the container, since intra-process only short-circuits nodes sharing a
    # process. It is here so that a second one - the camera also ships composable
    # nodes - gets zero-copy by joining the list, rather than needing this block
    # restructured first.
    lidar = ComposableNodeContainer(
        name='ldlidar_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_isolated',
        # The container is a node in its own right, so it needs this as much as
        # anything else here - the component's own copy below does not cover it.
        # False is already the ROS default, so this only bites when something
        # sets use_sim_time globally with a /**: wildcard, which is exactly the
        # case the explicit value exists to survive.
        parameters=[{'use_sim_time': False}],
        condition=IfCondition(use_lidar),
        output='both',
        composable_node_descriptions=[
            ComposableNode(
                package='ldlidar_component',
                plugin='ldlidar::LdLidarComponent',
                name='ldlidar_node',
                parameters=[
                    LaunchConfiguration('lidar_params'),
                    {'use_sim_time': False},
                ],
                # Deliberately NOT remapped to /scan, however much it looks
                # like it should be. The component publishes on the private
                # topic ~/scan, and its read loop gates on
                # count_subscribers("~/scan") - the unresolved string - so it
                # only talks to the device while something is subscribed. A
                # publisher remap does not move that check: remap ~/scan to
                # /scan and the publisher appears on /scan while the gate keeps
                # watching /ldlidar_node/scan, which now has no publisher and so
                # never has a subscriber. The node then reports 'active',
                # advertises /scan, logs nothing at all, and never reads the
                # lidar. Verified on the device: /scan was silent until a
                # subscriber was attached to /ldlidar_node/scan, at which point
                # it immediately ran at 9.89 Hz. scan_relay below is what
                # bridges the two names instead.
                extra_arguments=[{'use_intra_process_comms': True}],
            ),
        ],
    )

    # ldlidar_component is a lifecycle node: on its own it reaches 'unconfigured'
    # and stops there, having opened no serial port and advertised no /scan. It
    # does not fail, it simply sits - which looks exactly like a dead lidar. The
    # manager walks it through configure and activate, and the bond it keeps
    # afterwards means a driver that dies is noticed rather than silently absent.
    lidar_lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lidar_lifecycle_manager',
        parameters=[{
            'use_sim_time': False,
            'autostart': True,
            'node_names': ['ldlidar_node'],
        }],
        condition=IfCondition(use_lidar),
        output='both',
    )

    # Moves the driver's private ~/scan onto /scan, which is the name simulation
    # publishes and every consumer in this project subscribes. Its subscription
    # is also what keeps the driver's lazy read loop running - see the comment
    # above - so this node is load-bearing twice over and is not an optional
    # convenience. Not lazy itself: topic_tools relay defaults to lazy:=false and
    # must stay that way, or the two lazy gates wait on each other and nothing
    # ever starts.
    scan_relay = Node(
        package='topic_tools',
        executable='relay',
        name='scan_relay',
        arguments=['/ldlidar_node/scan', '/scan'],
        parameters=[{'use_sim_time': False}],
        condition=IfCondition(use_lidar),
        output='both',
    )

    # /scan/points, derived from /scan by the same converter simulation runs on
    # the same topic - see simulation.launch.py. Neither the LD19 driver nor
    # anything else here emits a cloud of its own, and building it the same way
    # in both modes is what makes it the same topic rather than two topics that
    # share a name.
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
    #
    # camera_config PICKS THE PARAMS FILE, NOT JUST A RESOLUTION
    # ------------------------------------------------------------
    # The device's XLink link cannot carry RGB + left + right + depth + IMU all
    # at once - the full set starves the IMU's queue indefinitely, every time.
    # See OAK_IMU_INVESTIGATION.md, "Root cause: IMU starved by the full RGBD
    # pipeline". camera_config therefore selects between two mutually exclusive
    # params files, not a parameter within one:
    #   rgbd (default)  -> oak_d_lite.yaml            RGB + depth, no left/right raw
    #   rgbstereo       -> oak_d_lite_rgbstereo.yaml   RGB + left/right raw, no depth
    # The IMU streams in both. Pass camera_params directly to override with a
    # third file entirely; camera_config only changes camera_params' default.
    #
    # camera_as_part_of_a_robot.launch.py, not camera.launch.py: the latter brings
    # its own robot_state_publisher, which would fight the one common.launch.py
    # already runs.
    #
    # WHO OWNS THE CAMERA'S FRAMES
    # ----------------------------
    # Split, deliberately. model.sdf owns where the camera BODY sits on the robot
    # (oak_d_lite_link) and it owns oak_imu_frame. The device's own EEPROM
    # calibration owns everything between its sensors - oak_{rgb,left,right}_camera_frame
    # and their _camera_optical_frame children - published by the driver because
    # oak_d_lite.yaml sets camera.i_publish_tf_from_calibration. A description
    # cannot know which physical camera is bolted on; this one measures 74.75 mm
    # between the mono pair where the nominal figure is 75.
    #
    # The two must not overlap. model.sdf therefore no longer declares
    # oak_rgb_camera_optical_frame: while it did, and with this driver setting on,
    # /tf_static carried that frame twice with different parents. tf2's static
    # cache is keyed by child and simply overwrites, and the topic is latched, so
    # which one a subscriber believes depends on arrival order - it does not warn
    # and it is not deterministic.
    #
    # The graft is exact rather than approximate. The driver bolts the root of its
    # socket chain to i_tf_base_frame with an identity transform, and on this
    # device that root is the colour camera, which model.sdf already places at
    # oak_d_lite_link's origin. Verified on the robot.
    #
    # i_tf_base_frame names a link called 'oak' rather than oak_d_lite_link, and
    # that spelling is forced: the driver reuses the same parameter as the prefix
    # for every image frame_id while naming published frames after the node, so
    # anything but the node's own name splits the two apart. model.sdf carries an
    # identity link for exactly this. See its comment, and oak_d_lite.yaml.
    #
    # Expect one puzzling log line: "Published URDF". The driver runs xacro and
    # hands the result to a node called oak_state_publisher, which nothing here
    # starts, so it goes nowhere. Harmless, and not worth chasing.
    camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('depthai_ros_driver'), 'launch',
                'camera_as_part_of_a_robot.launch.py'])),
        launch_arguments={
            'name': 'oak',
            'params_file': LaunchConfiguration('camera_params'),
            'rectify_rgb': 'False',
            # NOT publish_tf_from_calibration:=true, even though that is what this
            # robot wants. That path hardcodes i_tf_base_frame to the camera name
            # and appends its own dict AFTER params_file, so it would silently
            # override the oak_d_lite_link the YAML asks for. Configuring through
            # the params file and leaving this include alone is what makes it stick.
        }.items(),
        condition=IfCondition(use_camera),
    )

    # robot_state_publisher never broadcasts TF for a joint it has no JointState
    # for, and ugv02_serial_node publishes the two encoded wheels on
    # /wheel_encoders, not /joint_states - so without this, every one of the six
    # wheel frames sits at its zero-pose URDF default (Foxglove reports this as
    # "the default URDF transform will be used"; RViz shows the same thing but
    # without a warning). Relaying, not renaming the publisher: simulation keeps
    # its own /wheel_encoders too (see einride_mini_truck_bridge.yaml), and
    # renaming here would make the two modes diverge for no reason.
    #
    # Only the front axle moves - the other four wheels have no encoder and
    # hold their last/zero position - but a partial fix beats the current
    # all-six-wrong state. This goes away, not just moves, once the deferred
    # odometry phase gives every wheel joint (see model.sdf's comment on why TF
    # needs all six) a real publisher.
    #
    # Reliability is forced to reliable: /wheel_encoders is sensor-data QoS
    # (best effort), but robot_state_publisher's joint_states subscription is
    # plain QoS(10) (reliable, volatile). A best-effort publisher cannot connect
    # to a reliable subscriber, so without this override the relay would run
    # and log nothing wrong while carrying the encoder data nowhere.
    joint_state_relay = Node(
        package='topic_tools',
        executable='relay',
        name='joint_state_relay',
        arguments=['/wheel_encoders', '/joint_states'],
        parameters=[{
            'use_sim_time': False,
            'qos_overrides./joint_states.publisher.reliability': 'reliable',
        }],
        output='both',
    )

    common = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_project_bringup, 'launch', 'common.launch.py')),
        launch_arguments={
            # No /clock exists on hardware. A node left at use_sim_time true
            # here waits at time zero forever and looks like a hang.
            'use_sim_time': 'false',
            'rviz': use_rviz,
            # Not odom: nothing publishes that frame on hardware until the
            # deferred wheel-odometry phase does, so RViz would have no
            # transform for any sensor and would drop every message it was
            # given. base_footprint is the root of what robot_state_publisher
            # actually puts on /tf_static here. Change this to odom in the same
            # commit that starts publishing it.
            'fixed_frame': 'base_footprint',
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
            'lidar_params',
            default_value=PathJoinSubstitution(
                [pkg_project_bringup, 'config', 'ldlidar.yaml']),
            description='Parameter file for ldlidar_node.'),
        DeclareLaunchArgument(
            'camera', default_value='true',
            description='Start the OAK-D Lite driver.'),
        DeclareLaunchArgument(
            'camera_config', default_value='rgbd',
            description="Which camera_params default to use: 'rgbd' (RGB + "
                        "depth, no left/right raw; default) or 'rgbstereo' "
                        '(RGB + left/right raw, no depth). The IMU works in '
                        'both - see OAK_IMU_INVESTIGATION.md for why they '
                        'cannot both be true at once. Ignored if camera_params '
                        'is also passed explicitly.'),
        DeclareLaunchArgument(
            'camera_params',
            default_value=PathJoinSubstitution([
                pkg_project_bringup, 'config',
                PythonExpression([
                    "'oak_d_lite_rgbstereo.yaml' if '",
                    LaunchConfiguration('camera_config'),
                    "' == 'rgbstereo' else 'oak_d_lite.yaml'",
                ]),
            ]),
            description='Parameter file for the OAK-D Lite driver. Overrides '
                        'camera_config when set explicitly.'),
        DeclareLaunchArgument(
            'hardware_params',
            default_value=PathJoinSubstitution(
                [pkg_project_hardware, 'config', 'hardware.yaml']),
            description='Parameter file for ugv02_serial_node.'),
        ugv02_serial,
        lidar,
        lidar_lifecycle_manager,
        scan_relay,
        scan_to_points,
        camera,
        joint_state_relay,
        common,
    ])
