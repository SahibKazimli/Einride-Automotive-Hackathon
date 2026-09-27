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

"""Compares what simulation.launch.py and hardware.launch.py actually publish.

The whole point of the hardware abstraction layer is that everything above it
runs unchanged. The contract is the set of topics, types and QoS profiles the
robot presents; simulation's job is to reproduce it. Two separate launch files
can drift apart silently, so this launches each in turn, enumerates its graph,
and compares them - and when they disagree, it is normally the simulation that
has to move.

Scope, deliberately (topic, type, QoS) only:

* frame_id and units are checked where the messages actually exist, in
  einride_mini_truck_hardware's pty replay tests. There is no serial device
  here, so ugv02_serial_node advertises its topics but publishes nothing, and
  no message content can be compared from this side.
* the lidar and camera drivers are switched off - the lidar is not installed
  off the robot and the camera needs one on the USB bus - so their topics are
  listed as environment-missing rather than silently ignored.

Everything the deferred odometry phase will provide is listed explicitly below,
so this test documents the gap instead of failing opaquely when it finds one.
"""

import os
import signal
import subprocess
import tempfile
import time

import pytest
from rcl_interfaces.msg import ParameterType
from rcl_interfaces.srv import GetParameters
import rclpy
from rclpy.qos import DurabilityPolicy, ReliabilityPolicy

# Topics ugv02_serial_node is responsible for reproducing this phase. These must
# match exactly - name, type and QoS - or a subscriber written against the
# simulation silently fails to connect on the robot.
HAL_TOPICS = {
    '/cmd_vel': 'geometry_msgs/msg/Twist',
    '/imu': 'sensor_msgs/msg/Imu',
    '/mag': 'sensor_msgs/msg/MagneticField',
    '/wheel_encoders': 'sensor_msgs/msg/JointState',
}

# What each published topic must offer, in both modes. The sensor streams are
# best-effort (qos_profile_sensor_data in ugv02_serial_node.py, qos_profile:
# SENSOR_DATA in the bridge config); /voltage and the rest stay reliable. Getting this wrong
# does not degrade a subscriber, it disconnects it, so it is worth asserting
# per topic rather than in bulk.
SENSOR_STREAM_QOS = (ReliabilityPolicy.BEST_EFFORT, DurabilityPolicy.VOLATILE)
RELIABLE_QOS = (ReliabilityPolicy.RELIABLE, DurabilityPolicy.VOLATILE)
EXPECTED_QOS = {
    '/imu': SENSOR_STREAM_QOS,
    '/mag': SENSOR_STREAM_QOS,
    '/wheel_encoders': SENSOR_STREAM_QOS,
    '/voltage': RELIABLE_QOS,
    '/robot_description': (ReliabilityPolicy.RELIABLE,
                           DurabilityPolicy.TRANSIENT_LOCAL),
    '/joint_states': RELIABLE_QOS,
}

# Provided by robot_state_publisher, which is in common.launch.py and therefore
# byte-identical in both modes.
SHARED_TOPICS = {
    '/robot_description': 'std_msgs/msg/String',
    '/tf_static': 'tf2_msgs/msg/TFMessage',
}

# Present in both modes with the same type and QoS, but not byte-identical
# machinery the way SHARED_TOPICS is: simulation's joint_state_throttle
# downsamples the bridged gz JointStatePublisher feed (all six wheels),
# hardware's joint_state_relay forwards ugv02_serial_node's /wheel_encoders
# (front axle only - see hardware.launch.py). Content therefore differs; only
# name, type and QoS are asserted here, which is this file's whole scope.
JOINT_STATES_TOPIC = {'/joint_states': 'sensor_msgs/msg/JointState'}

# Simulation-only. /clock is unfixable - a real robot has no simulated clock, and
# every node's use_sim_time flips because of this one topic.
#
# /scan/raw is simulation-only by construction: it is the bridge's raw gz scan,
# and ld19_scan_model consumes it and publishes the /scan that both modes share.
# The robot's driver produces a finished scan directly, so it has no equivalent
# and should not grow one. See "Simulation matches the real LiDAR" in
# docs/lidar.md.
SIM_ONLY_TOPICS = {'/clock', '/scan/raw'}

# Hardware-only. Harmless; a battery model in the simulation would close it.
HARDWARE_ONLY_TOPICS = {'/voltage'}

# Provided on the robot by ldlidar_component and depthai_ros_driver. Both are
# switched off below - ldlidar_component because it is built from source and is
# usually not installed off the robot, depthai because it needs a camera on the
# USB bus and this test must pass on a machine with none. Listed so that "not
# compared here" is a stated fact rather than an omission.
#
# /scan's own content is compared against the real device in
# test_lidar_scan_contract.py, and against Gazebo in einride_mini_truck_gazebo's
# test_ld19_scan_model.py. Neither is reachable from here.
#
# The four names after /scan and /scan/points are what the lidar drags in with
# it on hardware: its private ~/scan, which scan_relay reads and republishes as
# /scan, its lifecycle transition_event, and the bond and diagnostics its
# lifecycle manager keeps. /clock belongs to that set too - nav2's lifecycle
# manager subscribes to it whatever use_sim_time says - which is why it is a
# subscription there and never a publication, and why test_clock_is_simulation_only
# below checks publishers rather than topics.
DRIVER_TOPICS = {
    '/scan', '/scan/points',
    '/ldlidar_node/scan', '/ldlidar_node/transition_event',
    '/bond', '/diagnostics',
    '/oak/rgb/image_raw', '/oak/rgb/camera_info',
    '/oak/stereo/image_raw', '/oak/stereo/camera_info',
    # The raw OV7251 mono pair - the stereo input, not a view of the depth
    # output. Simulation publishes these through the bridge, so they would
    # otherwise trip test_no_unexpected_topic_in_either_mode on the sim side
    # while the hardware side has the camera switched off.
    '/oak/left/image_raw', '/oak/left/camera_info',
    '/oak/right/image_raw', '/oak/right/camera_info',
    '/oak/points', '/oak/imu/data',
}

SIM_STARTUP_TIMEOUT = 120.0
HARDWARE_STARTUP_TIMEOUT = 30.0

# Anything else on this machine - a developer's own simulation, another test -
# would otherwise show up in the graph snapshot and fail the comparison for
# reasons that have nothing to do with the launch files. A per-process domain
# with discovery confined to this host keeps the run reproducible.
# Kept inside 0-101: above that the port arithmetic lands in the ephemeral
# range and can collide with a port the OS has already handed out.
ISOLATED_ENVIRONMENT = {
    'ROS_DOMAIN_ID': str(os.getpid() % 100 + 1),
    'ROS_AUTOMATIC_DISCOVERY_RANGE': 'LOCALHOST',
}


class Graph:
    """A snapshot of one launch file's ROS graph.

    Publications and subscriptions are kept apart on purpose.
    ``get_topic_names_and_types`` reports a topic that only has a subscriber,
    so treating its output as "what this mode provides" would have counted
    /joint_states as present on hardware purely because robot_state_publisher
    is waiting for it.
    """

    def __init__(self, publishers, subscriptions, endpoints, sim_time):
        self.publishers = publishers        # topic -> type, >= 1 publisher
        self.subscriptions = subscriptions  # topic -> type, >= 1 subscriber
        self.endpoints = endpoints          # topic -> (reliability, durability)
        self.sim_time = sim_time            # node name -> use_sim_time

    def qos(self, topic):
        return self.endpoints.get(topic)

    @property
    def topics(self):
        return set(self.publishers) | set(self.subscriptions)


def _snapshot(node, required, timeout):
    """Wait for the required topics to appear, then record the whole graph.

    ``required`` must cover every topic the assertions touch, not just the ones
    that identify the launch as started: ugv02_serial_node advertises in well
    under a second while robot_state_publisher first parses the model SDF
    through sdformat_urdf, so waiting on the HAL topics alone would snapshot the
    graph before the shared nodes are in it.

    Returns the graph and whatever of ``required`` never turned up, so the
    caller can report a launch that failed to start rather than letting every
    downstream comparison fail separately.
    """
    deadline = time.monotonic() + timeout
    publishers, subscriptions, endpoints = {}, {}, {}
    while True:
        publishers, subscriptions, endpoints = {}, {}, {}
        for topic, kinds in node.get_topic_names_and_types():
            infos = node.get_publishers_info_by_topic(topic)
            if infos:
                publishers[topic] = kinds[0]
                profile = infos[0].qos_profile
                endpoints[topic] = (profile.reliability, profile.durability)
            if node.get_subscriptions_info_by_topic(topic):
                subscriptions[topic] = kinds[0]
        missing = required - (set(publishers) | set(subscriptions))
        if not missing or time.monotonic() >= deadline:
            break
        time.sleep(0.5)

    sim_time = {}
    for name in node.get_node_names():
        if name == node.get_name():
            continue
        value = _get_use_sim_time(node, name)
        if value is not None:
            sim_time[name] = value
    return Graph(publishers, subscriptions, endpoints, sim_time), missing


def _get_use_sim_time(node, node_name):
    """Read one node's use_sim_time over its own parameter service.

    Asked from the probe node rather than by shelling out to `ros2 param get`:
    that spawns a process which has to rediscover the graph from scratch, and
    may route through the ros2 daemon, so it intermittently came back empty
    while the probe here was already connected to every node in the launch.

    Returns None if the node does not answer or does not have the parameter.
    """
    client = node.create_client(GetParameters, '/{}/get_parameters'.format(node_name))
    try:
        if not client.wait_for_service(timeout_sec=10.0):
            return None
        future = client.call_async(GetParameters.Request(names=['use_sim_time']))
        rclpy.spin_until_future_complete(node, future, timeout_sec=10.0)
        response = future.result()
        if response is None or not response.values:
            return None
        value = response.values[0]
        if value.type != ParameterType.PARAMETER_BOOL:
            return None
        return value.bool_value
    finally:
        node.destroy_client(client)


def _launch(arguments, required, timeout):
    """Run one launch file to the point where its topics exist, then snapshot."""
    command = ['ros2', 'launch', 'einride_mini_truck_bringup'] + arguments
    log = tempfile.NamedTemporaryFile(
        mode='w+', suffix='.log', prefix='conformance-', delete=False)
    # Its own session, so the whole tree - gz sim, the bridge, every node - can
    # be signalled together rather than orphaning children.
    process = subprocess.Popen(
        command, start_new_session=True, stdout=log, stderr=subprocess.STDOUT,
        env=dict(os.environ, **ISOLATED_ENVIRONMENT))
    node = rclpy.create_node('conformance_probe')
    try:
        graph, missing = _snapshot(node, required, timeout)
    finally:
        node.destroy_node()
        _terminate(process)
        log.close()

    if missing:
        # One clear failure beats sixteen opaque ones: if the launch itself did
        # not come up, every comparison below would fail for the same reason.
        with open(log.name) as handle:
            tail = ''.join(handle.readlines()[-40:])
        os.unlink(log.name)
        pytest.fail(
            '{} did not publish or subscribe {} within {:.0f}s.\n'
            'Last 40 lines of its output:\n{}'.format(
                ' '.join(arguments), sorted(missing), timeout, tail))
    os.unlink(log.name)
    return graph


def _terminate(process):
    """SIGINT the process group, as Ctrl-C would, then insist."""
    try:
        group = os.getpgid(process.pid)
    except ProcessLookupError:
        return          # already gone, e.g. the launch file itself errored out
    os.killpg(group, signal.SIGINT)
    try:
        process.wait(timeout=20)
    except subprocess.TimeoutExpired:
        os.killpg(group, signal.SIGKILL)
        process.wait(timeout=10)
    time.sleep(2.0)      # let discovery forget the endpoints before the next run


@pytest.fixture(scope='module')
def graphs():
    os.environ.update(ISOLATED_ENVIRONMENT)     # the probe joins the same domain
    rclpy.init()
    try:
        simulation = _launch(
            ['simulation.launch.py', 'headless:=true', 'rviz:=false'],
            (set(HAL_TOPICS) | set(SHARED_TOPICS) | set(JOINT_STATES_TOPIC)
             | SIM_ONLY_TOPICS),
            SIM_STARTUP_TIMEOUT)
        hardware = _launch(
            # Both drivers are switched off - see DRIVER_TOPICS. The serial port
            # does not exist either, but the node advertises regardless, which
            # is exactly what this test needs to see.
            ['hardware.launch.py', 'rviz:=false', 'lidar:=false', 'camera:=false'],
            (set(HAL_TOPICS) | set(SHARED_TOPICS) | set(JOINT_STATES_TOPIC)
             | HARDWARE_ONLY_TOPICS),
            HARDWARE_STARTUP_TIMEOUT)
        yield simulation, hardware
    finally:
        rclpy.shutdown()


# ------------------------------------------------------------------ contract

@pytest.mark.parametrize('topic', sorted(set(HAL_TOPICS) - {'/cmd_vel'}))
def test_hal_topic_is_published_in_both_modes(graphs, topic):
    simulation, hardware = graphs
    assert simulation.publishers.get(topic) == HAL_TOPICS[topic]
    assert hardware.publishers.get(topic) == HAL_TOPICS[topic]


def test_cmd_vel_is_subscribed_in_both_modes(graphs):
    """/cmd_vel runs the other way: the bridge and the HAL both consume it."""
    simulation, hardware = graphs
    assert simulation.subscriptions.get('/cmd_vel') == HAL_TOPICS['/cmd_vel']
    assert hardware.subscriptions.get('/cmd_vel') == HAL_TOPICS['/cmd_vel']


@pytest.mark.parametrize('topic', sorted(set(HAL_TOPICS) - {'/cmd_vel'}))
def test_hal_topic_qos_matches(graphs, topic):
    """The two modes must offer the same profile, and the documented one.

    A mismatch does not degrade a subscriber, it disconnects it. The robot picks
    the profile - qos_profile_sensor_data for the sensor streams - and the
    bridge is configured to reproduce it with a per-topic `qos_profile:` key.
    If these ever disagree, the bridge config is what moves. /cmd_vel is
    excluded because both modes only subscribe to it here; nothing in either
    launch publishes it.
    """
    simulation, hardware = graphs
    assert simulation.qos(topic) is not None, topic
    assert simulation.qos(topic) == hardware.qos(topic)
    assert simulation.qos(topic) == EXPECTED_QOS[topic]


def test_sensor_streams_are_best_effort(graphs):
    """Stated separately so the intent survives a careless edit to the map."""
    _, hardware = graphs
    for topic in ('/imu', '/mag', '/wheel_encoders'):
        assert hardware.qos(topic) == SENSOR_STREAM_QOS, topic
    # Not everything: /voltage is 1 Hz telemetry, where a lost sample is a whole
    # second of nothing rather than a 12 ms gap.
    assert hardware.qos('/voltage') == RELIABLE_QOS


@pytest.mark.parametrize('topic', sorted(SHARED_TOPICS))
def test_shared_topics_are_identical(graphs, topic):
    """These come from common.launch.py, so any difference is a wiring bug."""
    simulation, hardware = graphs
    assert simulation.publishers.get(topic) == SHARED_TOPICS[topic]
    assert hardware.publishers.get(topic) == SHARED_TOPICS[topic]
    assert simulation.qos(topic) == hardware.qos(topic)


def test_tf_static_is_transient_local(graphs):
    """A latched /tf_static is what lets a late subscriber find the frames."""
    simulation, hardware = graphs
    for graph in (simulation, hardware):
        assert graph.qos('/tf_static') == (
            ReliabilityPolicy.RELIABLE, DurabilityPolicy.TRANSIENT_LOCAL)


# ---------------------------------------------------------- documented gaps

def test_clock_is_simulation_only(graphs):
    """The single most likely cause of "works in sim, stalls on hardware".

    Publishers, not topics: with lidar:=true the robot does carry a /clock
    *subscription*, because nav2's lifecycle manager creates one whatever
    use_sim_time says. Nothing publishes it there, which is the thing that
    matters.
    """
    simulation, hardware = graphs
    assert '/clock' in simulation.publishers
    assert '/clock' not in hardware.publishers


def test_raw_scan_does_not_leak_out_of_simulation(graphs):
    """/scan is the contract; /scan/raw is simulation's own plumbing.

    If this ever fails on the hardware side, something started bridging or
    renaming the robot's scan instead of letting the driver publish it.
    """
    simulation, hardware = graphs
    assert simulation.publishers.get('/scan/raw') == 'sensor_msgs/msg/LaserScan'
    assert '/scan/raw' not in hardware.topics


def test_voltage_is_hardware_only(graphs):
    simulation, hardware = graphs
    assert hardware.publishers.get('/voltage') == 'std_msgs/msg/Float32'
    assert hardware.qos('/voltage') == RELIABLE_QOS
    assert '/voltage' not in simulation.topics


def test_driver_topics_are_absent_here(graphs):
    """States the limit of this test rather than pretending to cover it.

    Both drivers were switched off: ldlidar is not installed off the robot, and
    depthai needs a camera on the USB bus, which cannot be assumed here. If one
    of these ever shows up on the hardware side, the launch stopped honouring
    its own lidar:= / camera:= arguments.
    """
    _, hardware = graphs
    assert DRIVER_TOPICS.isdisjoint(hardware.topics)


# -------------------------------------------------------------- use_sim_time

def test_use_sim_time_is_set_consistently(graphs):
    """Every node in a mode must agree, or TF and filters silently stall."""
    simulation, hardware = graphs
    assert simulation.sim_time, 'no nodes reported use_sim_time in simulation'
    assert hardware.sim_time, 'no nodes reported use_sim_time on hardware'
    assert all(simulation.sim_time.values()), simulation.sim_time
    assert not any(hardware.sim_time.values()), hardware.sim_time
