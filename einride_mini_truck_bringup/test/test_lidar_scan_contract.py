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

"""Checks /scan against a real LD19, and against what simulation publishes.

test_interface_conformance.py compares the two launch files at the level of
topic, type and QoS, and switches both drivers off to do it. That is as far as
it can go with no hardware attached, and it leaves the lidar's actual contract -
how many bins, which way round, what a bin with no return contains - untested in
both modes.

This file closes that gap from the hardware side. It needs the lidar plugged in,
so it skips itself when there is none; the same numbers are asserted against
Gazebo by einride_mini_truck_gazebo's test_ld19_scan_model.py, which needs no
hardware. Between them, both modes are pinned to the constants below.

The one that matters most is the plain fact that messages arrive. The driver's
read loop only talks to the device while something is subscribed to its private
~/scan, and it checks that by name, so remapping the publisher onto /scan
silently stops it reading - active node, advertised topic, no data, no error.
hardware.launch.py uses a relay to avoid that, and "did any message arrive"
is the assertion that keeps it that way.
"""

import math
import os
import signal
import subprocess
import tempfile
import time

import pytest
import rclpy
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy)
from sensor_msgs.msg import LaserScan

# The device as ldlidar's own udev rule names it (rules/ldlidar.rules in
# ldrobot-lidar-ros2). config/ldlidar.yaml points the driver at this path, so if
# it does not exist there is nothing to test.
LIDAR_DEVICE = '/dev/ldlidar'

# The scan contract, measured on the device rather than taken from the
# datasheet. model.sdf is configured to reproduce exactly these, so the same
# numbers describe simulation - that is the point of asserting them here.
FRAME_ID = 'base_lidar_link'
BINS = 455                       # 4500 Hz sampling / ~9.9 Hz spin
ANGLE_MIN = 0.0                  # ldlidar hardcodes 0 .. 2*pi, not -pi .. +pi
ANGLE_MAX = 2.0 * math.pi
ANGLE_INCREMENT = (ANGLE_MAX - ANGLE_MIN) / (BINS - 1)
RANGE_MIN = 0.02                 # LD19 datasheet v1.0, 70% target reflectivity
RANGE_MAX = 12.0
NOMINAL_RATE = 10.0              # datasheet typical; 5-13 Hz is in spec

SCANS_WANTED = 10
STARTUP_TIMEOUT = 60.0

# Same isolation as test_interface_conformance.py: a developer's own simulation
# on this machine would otherwise publish /scan into the same graph.
ISOLATED_ENVIRONMENT = {
    'ROS_DOMAIN_ID': str(os.getpid() % 100 + 1),
    'ROS_AUTOMATIC_DISCOVERY_RANGE': 'LOCALHOST',
}

# ldlidar_component publishes with rclcpp::QoS(10), which is reliable.
DRIVER_QOS = QoSProfile(
    depth=10, history=HistoryPolicy.KEEP_LAST,
    reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.VOLATILE)


def _driver_is_installed():
    """ldlidar_component is built from source, so it is often simply absent."""
    from ament_index_python.packages import PackageNotFoundError
    from ament_index_python.packages import get_package_share_directory
    try:
        get_package_share_directory('ldlidar_component')
    except PackageNotFoundError:
        return False
    return True


requires_lidar = pytest.mark.skipif(
    not os.path.exists(LIDAR_DEVICE) or not _driver_is_installed(),
    reason='needs an LD19 on {} and the ldlidar_component package'.format(
        LIDAR_DEVICE))


@pytest.fixture(scope='module')
def scans():
    """Launch the hardware stack and collect a handful of real scans."""
    os.environ.update(ISOLATED_ENVIRONMENT)
    command = [
        'ros2', 'launch', 'einride_mini_truck_bringup', 'hardware.launch.py',
        'rviz:=false', 'camera:=false', 'lidar:=true',
    ]
    log = tempfile.NamedTemporaryFile(
        mode='w+', suffix='.log', prefix='lidar-contract-', delete=False)
    process = subprocess.Popen(
        command, start_new_session=True, stdout=log, stderr=subprocess.STDOUT,
        env=dict(os.environ, **ISOLATED_ENVIRONMENT))

    rclpy.init()
    node = rclpy.create_node('lidar_contract_probe')
    collected = []
    node.create_subscription(
        LaserScan, '/scan', collected.append, DRIVER_QOS)
    try:
        deadline = time.monotonic() + STARTUP_TIMEOUT
        while len(collected) < SCANS_WANTED and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.5)
    finally:
        node.destroy_node()
        rclpy.shutdown()
        _terminate(process)
        log.seek(0)
        tail = ''.join(log.readlines()[-40:])
        log.close()

    if len(collected) < SCANS_WANTED:
        os.unlink(log.name)
        pytest.fail(
            'only {} scans in {:.0f}s. An active driver that publishes nothing '
            'is the signature of the ~/scan subscriber-count trap - see the '
            'module docstring and hardware.launch.py.\n{}'.format(
                len(collected), STARTUP_TIMEOUT, tail))
    os.unlink(log.name)
    return collected


def _terminate(process):
    try:
        group = os.getpgid(process.pid)
    except ProcessLookupError:
        return
    os.killpg(group, signal.SIGINT)
    try:
        process.wait(timeout=20)
    except subprocess.TimeoutExpired:
        os.killpg(group, signal.SIGKILL)
        process.wait(timeout=10)
    time.sleep(2.0)


@requires_lidar
def test_scan_is_published_at_all(scans):
    """The regression guard for a driver that goes active and stays silent."""
    assert len(scans) >= SCANS_WANTED


@requires_lidar
def test_frame_is_the_robots_own(scans):
    """Not ldlidar's default ldlidar_link, which no TF tree here contains."""
    assert all(scan.header.frame_id == FRAME_ID for scan in scans)


@requires_lidar
def test_sweep_runs_from_zero_to_two_pi(scans):
    """Simulation matches this; -pi..+pi would rotate every bearing by 180."""
    for scan in scans:
        assert scan.angle_min == pytest.approx(ANGLE_MIN, abs=1e-6)
        assert scan.angle_max == pytest.approx(ANGLE_MAX, abs=1e-5)


@requires_lidar
def test_bin_count_and_increment_match_simulation(scans):
    for scan in scans:
        assert len(scan.ranges) == BINS
        assert len(scan.intensities) == BINS
        assert scan.angle_increment == pytest.approx(ANGLE_INCREMENT, rel=1e-5)


@requires_lidar
def test_range_limits_are_the_datasheets_not_the_drivers_defaults(scans):
    """The driver ships 0.03-15.0, claiming 3 m the LD19 does not have."""
    for scan in scans:
        assert scan.range_min == pytest.approx(RANGE_MIN, abs=1e-6)
        assert scan.range_max == pytest.approx(RANGE_MAX, abs=1e-6)


@requires_lidar
def test_a_bin_with_no_return_is_nan_never_inf(scans):
    """Simulation's ld19_scan_model exists to reproduce exactly this."""
    for scan in scans:
        assert not any(math.isinf(value) for value in scan.ranges)
    # A scan with no NaN at all would mean every bearing returned something,
    # which does not happen indoors - and would make the check above vacuous.
    assert any(
        any(math.isnan(value) for value in scan.ranges) for scan in scans)


@requires_lidar
def test_intensity_is_blank_exactly_where_the_range_is(scans):
    """The pairing ld19_scan_model reproduces; a consumer may key off either."""
    for scan in scans:
        blank_range = [math.isnan(value) for value in scan.ranges]
        blank_intensity = [math.isnan(value) for value in scan.intensities]
        # The driver writes both from one device sample, but a bin that two
        # samples landed in keeps the nearer range and the later intensity, so
        # they can differ in a couple of bins per scan.
        disagreements = sum(
            1 for a, b in zip(blank_range, blank_intensity) if a != b)
        assert disagreements <= 8, disagreements


@requires_lidar
def test_scan_time_is_measured_not_left_at_zero(scans):
    """Gazebo leaves this 0; anything deskewing a scan divides by it."""
    for scan in scans[1:]:
        assert scan.scan_time > 0.0
        assert scan.scan_time == pytest.approx(1.0 / NOMINAL_RATE, rel=0.35)
        assert scan.time_increment == pytest.approx(
            scan.scan_time / (BINS - 1), rel=1e-3)
