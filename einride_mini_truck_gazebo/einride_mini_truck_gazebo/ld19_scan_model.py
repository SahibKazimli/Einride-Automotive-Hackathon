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

"""Make Gazebo's lidar output look like ldlidar_component's.

Simulation's counterpart to the LD19 driver. gz-sim's gpu_lidar produces the
right *geometry* once model.sdf is set up for it - 455 bins over 0..2*pi, 0.02
to 12 m - but it does not produce the right *message*, and the three remaining
differences are all of the silent kind:

* no return. gz writes +inf, ldlidar writes NaN. Both are legal in a
  sensor_msgs/LaserScan and nothing warns, so `r < max_range` admits every
  empty bearing in simulation and rejects every one on the robot, while
  `numpy.isnan` does the reverse. On a bench scan with the device on a desk,
  17% of bins came back NaN, so this is not a corner case.
* scan_time and time_increment. gz leaves both at 0. Anything that deskews a
  scan against motion divides by them, and a zero is either a silent no-op or
  a ZeroDivisionError depending on who wrote the consumer.
* intensities. gz reports 0.0 for every bin; the real device reported 7..255.
  An intensity threshold tuned in simulation therefore rejects everything.

What this node does NOT do is pretend to model return strength - see the
`intensity` parameter. Nor does it invent dropouts: the real device's empty
bins come from surfaces that returned nothing, and simulation reproduces that
honestly whenever a ray reaches max range.

The transform is a pure function, `model_ld19`, so the interesting part is
unit-testable without a Gazebo in the loop.
"""

import math
from typing import List, Optional

from rcl_interfaces.msg import ParameterDescriptor
import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan

# ldlidar_component publishes with rclcpp::QoS(10) - reliable, keep-last 10 -
# and so does the ros_gz bridge for /scan, having no qos_profile: key. Matching
# it here keeps the profile identical in both modes, which the bringup
# conformance test asserts.
DRIVER_QOS = QoSProfile(
    depth=10, history=HistoryPolicy.KEEP_LAST, reliability=ReliabilityPolicy.RELIABLE)

# Mid-scale, inside the 7..255 the real unit produced on the bench. Not
# physical - see the `intensity` parameter's description.
DEFAULT_INTENSITY = 200.0

INTENSITY_DESCRIPTOR = ParameterDescriptor(
    description=(
        'Intensity reported for a bin that returned something. Gazebo does not '
        'simulate return strength, so this is a constant placeholder: the '
        'valid/NaN structure matches the robot, the magnitude does not mean '
        'anything. 0.0 restores gz raw behaviour.'))


def model_ld19(scan: LaserScan, previous_stamp_s: Optional[float],
               stamp_s: float, intensity: float) -> LaserScan:
    """Rewrite one gz scan into what ldlidar_component would have published.

    `previous_stamp_s` is the stamp of the last scan that came through, or None
    for the very first one, in which case scan_time and time_increment are left
    at zero because there is nothing to measure an interval against. The real
    driver has the same blind spot and resolves it by dropping its first scan
    entirely (`_firstScan`), which is what the node below does too.

    Ranges and intensities are rewritten in step: a bin with no return carries
    NaN in both, which is what the driver does when it sees distance == 0 and
    intensity == 0 from the device.
    """
    ranges: List[float] = []
    intensities: List[float] = []
    for value in scan.ranges:
        if math.isfinite(value):
            ranges.append(value)
            intensities.append(intensity)
        else:
            # gz's +inf for "this ray hit nothing".
            ranges.append(math.nan)
            intensities.append(math.nan)

    scan.ranges = ranges
    scan.intensities = intensities

    if previous_stamp_s is not None:
        scan_time = stamp_s - previous_stamp_s
        if scan_time > 0.0:
            scan.scan_time = scan_time
            # The driver spreads the interval over the bins the same way. With
            # 455 bins at ~10 Hz this is the 2.2e-4 s the real device reports.
            if len(scan.ranges) > 1:
                scan.time_increment = scan_time / (len(scan.ranges) - 1)
    return scan


class Ld19ScanModel(Node):
    """Subscribes the bridged gz scan and republishes it as the driver would."""

    def __init__(self) -> None:
        super().__init__('ld19_scan_model')
        self.declare_parameter(
            'intensity', DEFAULT_INTENSITY,
            # Gazebo's gpu_lidar does not model return strength; every bin comes
            # back 0.0 regardless of what it hit. So this is a placeholder, and
            # only the *structure* around it is real - a bin that returned
            # something carries a finite intensity, a bin that did not carries
            # NaN, exactly as on the robot. The magnitude is not a reflectivity
            # and must not be treated as one. Set it to 0.0 to get gz's raw
            # behaviour back.
            descriptor=INTENSITY_DESCRIPTOR)
        self._intensity = float(
            self.get_parameter('intensity').get_parameter_value().double_value)

        self._previous_stamp_s: Optional[float] = None

        self._publisher = self.create_publisher(LaserScan, 'scan', DRIVER_QOS)
        self._subscription = self.create_subscription(
            LaserScan, 'scan/raw', self._on_scan, DRIVER_QOS)

    def _on_scan(self, scan: LaserScan) -> None:
        stamp_s = rclpy.time.Time.from_msg(scan.header.stamp).nanoseconds * 1e-9
        previous, self._previous_stamp_s = self._previous_stamp_s, stamp_s
        if previous is None:
            # Matches the driver's _firstScan: with no interval yet there is no
            # honest scan_time to publish, so it publishes nothing.
            return
        self._publisher.publish(
            model_ld19(scan, previous, stamp_s, self._intensity))


def main(args=None) -> None:
    """Spin the node until interrupted."""
    rclpy.init(args=args)
    node = Ld19ScanModel()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
