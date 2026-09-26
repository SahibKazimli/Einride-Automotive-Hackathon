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

"""Checks the gz-scan-to-LD19 transform against measurements from the device.

The numbers asserted here were read off the real LD19 on /ldlidar_node/scan,
not taken from the datasheet, so this test fails if the transform stops
reproducing what the hardware actually sends. Measuring the live device is
described under "Simulation matches the real LiDAR" in docs/lidar.md.
"""

import math

from einride_mini_truck_gazebo.ld19_scan_model import model_ld19
from sensor_msgs.msg import LaserScan


# One scan as the real unit reported it: 455 bins over a full turn at 9.93 Hz.
# Both numbers come from the same message, so they are consistent with each
# other - 0.100730 / 454 is exactly the time_increment below. Taking scan_time
# from one sample and time_increment from another puts them 0.2% apart and the
# comparison below stops meaning anything.
REAL_BINS = 455
REAL_SCAN_TIME = 0.100730
REAL_TIME_INCREMENT = 2.2187e-4


def _scan(ranges):
    scan = LaserScan()
    scan.angle_min = 0.0
    scan.angle_max = 2.0 * math.pi
    scan.angle_increment = (2.0 * math.pi) / (len(ranges) - 1)
    scan.range_min = 0.02
    scan.range_max = 12.0
    scan.ranges = list(ranges)
    scan.intensities = [0.0] * len(ranges)   # what gz supplies
    return scan


def test_no_return_becomes_nan_not_inf():
    """The difference that silently inverts every range validity check."""
    out = model_ld19(_scan([1.0, math.inf, 2.0]), 0.0, 0.1, 200.0)
    assert math.isnan(out.ranges[1])
    assert not any(math.isinf(value) for value in out.ranges)
    assert out.ranges[0] == 1.0 and out.ranges[2] == 2.0


def test_intensity_is_nan_exactly_where_the_range_is():
    """The driver blanks both together; a consumer may key off either."""
    out = model_ld19(_scan([1.0, math.inf, 2.0]), 0.0, 0.1, 200.0)
    assert [math.isnan(value) for value in out.intensities] == \
           [math.isnan(value) for value in out.ranges]
    assert out.intensities[0] == 200.0


def test_valid_bins_do_not_keep_gz_zero_intensity():
    """All-zero intensities make any intensity threshold reject the world."""
    out = model_ld19(_scan([1.0, 2.0]), 0.0, 0.1, 200.0)
    assert all(value == 200.0 for value in out.intensities)


def test_intensity_zero_restores_raw_gazebo_behaviour():
    out = model_ld19(_scan([1.0, 2.0]), 0.0, 0.1, 0.0)
    assert all(value == 0.0 for value in out.intensities)


def test_scan_time_matches_the_measured_device():
    """455 bins at the real 9.89 Hz reproduces the device's own numbers.

    Compared with a tolerance rather than exactly: scan_time and time_increment
    are float32 on the wire, so the value that comes back out has already been
    rounded once.
    """
    out = model_ld19(_scan([1.0] * REAL_BINS), 10.0, 10.0 + REAL_SCAN_TIME, 200.0)
    assert math.isclose(out.scan_time, REAL_SCAN_TIME, rel_tol=1e-6)
    assert math.isclose(
        out.time_increment, REAL_SCAN_TIME / (REAL_BINS - 1), rel_tol=1e-6)
    assert math.isclose(out.time_increment, REAL_TIME_INCREMENT, rel_tol=1e-3)


def test_first_scan_has_no_interval_to_measure():
    """No previous stamp means no honest scan_time; it must stay zero."""
    out = model_ld19(_scan([1.0, 2.0]), None, 10.0, 200.0)
    assert out.scan_time == 0.0
    assert out.time_increment == 0.0


def test_a_stalled_clock_does_not_produce_a_zero_interval():
    """Two scans on the same stamp would otherwise divide by zero downstream."""
    out = model_ld19(_scan([1.0, 2.0]), 10.0, 10.0, 200.0)
    assert out.scan_time == 0.0


def test_geometry_is_left_alone():
    """Only the driver's own fields are rewritten; the sweep comes from the SDF."""
    out = model_ld19(_scan([1.0] * REAL_BINS), 0.0, 0.1, 200.0)
    assert out.angle_min == 0.0
    assert math.isclose(out.angle_max, 2.0 * math.pi)
    assert len(out.ranges) == REAL_BINS
    assert out.range_min == 0.02 and out.range_max == 12.0
