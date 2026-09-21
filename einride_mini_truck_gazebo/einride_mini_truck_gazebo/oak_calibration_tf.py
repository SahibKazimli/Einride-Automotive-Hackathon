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

"""Publish the OAK-D Lite's calibrated frames in simulation, as the driver does on hardware.

Simulation's counterpart to depthai_ros_driver's TFPublisher, the same way
ld19_scan_model is simulation's counterpart to the LD19 driver.

On the robot, the frames inside the camera - where the two mono sensors sit
relative to the colour one - come from the device's own EEPROM, because a robot
description cannot know which physical camera is bolted on. This unit's mono pair
measures 74.75 mm apart where the nominal figure is 75, and the pair is not even
symmetric about the colour camera. model.sdf therefore declares none of those
frames, and something has to publish them here instead.

Rather than copy the numbers, this runs the SAME ALGORITHM over the SAME EEPROM
dump the device would have handed the driver. Identical input, identical
arithmetic, so the two modes agree by construction rather than by a human keeping
two lists in step. test_oak_calibration_tf.py holds it to that: it checks this
output against a /tf_static snapshot recorded from the real camera.

SIX TRANSFORMS, NOT SEVEN. The driver also publishes oak_imu_frame, and this
deliberately does not, because that one transform of the driver's is wrong.
TFPublisher::quatFromRotM computes q_rot2rdf * q_extr * q_rot2rdf^-1, a similarity
transform: correct between two optical camera frames, which both need re-basing,
and wrong between a camera and the IMU, whose frame is not optical and where only
the left factor belongs. It lands exactly one q_rot2rdf out - 2*acos(0.5), or 120
degrees. Hardware suppresses it with camera.i_tf_imu_from_descr and takes
oak_imu_frame from model.sdf; simulation gets it from model.sdf too, via
robot_state_publisher. Publishing it here as well would both duplicate a frame and
reintroduce the error.

The transform is a pure function, `calibration_to_transforms`, so the interesting
part is unit-testable without a Gazebo or a camera in the loop.
"""

import json
import math
import os
from typing import Dict, List, Sequence, Tuple

from ament_index_python.packages import get_package_share_directory

from geometry_msgs.msg import TransformStamped

import rclpy
from rclpy.node import Node

from tf2_ros import StaticTransformBroadcaster

# dai::CameraBoardSocket -> the name depthai builds frame ids out of. TFPublisher
# prefixes these with the driver's NODE name, while the sensor nodes prefix their
# image frame_ids with i_tf_base_frame instead - so the two agree only because
# that parameter is set to 'oak', the node's own name. See oak_d_lite.yaml.
SOCKET_NAMES: Dict[int, str] = {
    0: 'rgb',           # CAM_A
    1: 'left',          # CAM_B
    2: 'right',         # CAM_C
    3: 'left_back',     # CAM_D
    4: 'right_back',    # CAM_E
}

# What toCameraSocket holds on the socket that ends the chain. That one is bolted
# to the base frame with an exact identity rather than to another camera.
NO_SOCKET = -1

# FLU -> optical, as (x, y, z, w): x right, y down, z along the view axis. The
# driver hardcodes this same quaternion rather than deriving it per camera, and it
# is exactly the -pi/2, 0, -pi/2 that model.sdf used to spell out.
OPTICAL_ROTATION: Tuple[float, float, float, float] = (-0.5, 0.5, -0.5, 0.5)

Quaternion = Tuple[float, float, float, float]

DEFAULT_CALIBRATION = os.path.join(
    get_package_share_directory('einride_mini_truck_description'),
    'calibration', 'oak_d_lite_194430107146157E00.json')


def _quaternion_multiply(lhs: Quaternion, rhs: Quaternion) -> Quaternion:
    """Return the Hamilton product of two (x, y, z, w) quaternions."""
    ax, ay, az, aw = lhs
    bx, by, bz, bw = rhs
    return (
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    )


def _quaternion_from_matrix(matrix: Sequence[Sequence[float]]) -> Quaternion:
    """Convert a 3x3 rotation matrix to an (x, y, z, w) quaternion.

    Branches on the largest diagonal term, as tf2::Matrix3x3::getRotation does,
    so that the square root is never taken of something near zero.
    """
    trace = matrix[0][0] + matrix[1][1] + matrix[2][2]
    if trace > 0.0:
        s = math.sqrt(trace + 1.0) * 2.0
        return ((matrix[2][1] - matrix[1][2]) / s,
                (matrix[0][2] - matrix[2][0]) / s,
                (matrix[1][0] - matrix[0][1]) / s,
                0.25 * s)
    if matrix[0][0] > matrix[1][1] and matrix[0][0] > matrix[2][2]:
        s = math.sqrt(1.0 + matrix[0][0] - matrix[1][1] - matrix[2][2]) * 2.0
        return (0.25 * s,
                (matrix[0][1] + matrix[1][0]) / s,
                (matrix[0][2] + matrix[2][0]) / s,
                (matrix[2][1] - matrix[1][2]) / s)
    if matrix[1][1] > matrix[2][2]:
        s = math.sqrt(1.0 + matrix[1][1] - matrix[0][0] - matrix[2][2]) * 2.0
        return ((matrix[0][1] + matrix[1][0]) / s,
                0.25 * s,
                (matrix[1][2] + matrix[2][1]) / s,
                (matrix[0][2] - matrix[2][0]) / s)
    s = math.sqrt(1.0 + matrix[2][2] - matrix[0][0] - matrix[1][1]) * 2.0
    return ((matrix[0][2] + matrix[2][0]) / s,
            (matrix[1][2] + matrix[2][1]) / s,
            0.25 * s,
            (matrix[1][0] - matrix[0][1]) / s)


def rotation_to_flu(matrix: Sequence[Sequence[float]]) -> Quaternion:
    """Re-express an EEPROM rotation, which is in optical axes, in FLU axes.

    Both the parent and the child of a camera-to-camera transform are optical
    frames, so both sides need rebasing and the answer is a similarity transform:
    q_rot2rdf * q * q_rot2rdf^-1. This is depthai_bridge's quatFromRotM with its
    q_flu left at identity, which is what the driver always passes.
    """
    extrinsic = _quaternion_from_matrix(matrix)
    inverse = (-OPTICAL_ROTATION[0], -OPTICAL_ROTATION[1],
               -OPTICAL_ROTATION[2], OPTICAL_ROTATION[3])
    return _quaternion_multiply(
        _quaternion_multiply(OPTICAL_ROTATION, extrinsic), inverse)


def translation_to_flu(translation: Dict[str, float]) -> Tuple[float, float, float]:
    """Convert an EEPROM translation, in optical-axis centimetres, to FLU metres."""
    return (translation['z'] / 100.0,
            -translation['x'] / 100.0,
            -translation['y'] / 100.0)


def calibration_to_transforms(
        calibration: dict,
        base_frame: str = 'oak',
        name: str = 'oak') -> List[TransformStamped]:
    """Build the camera frames the depthai driver would publish from this calibration.

    Two transforms per camera socket: the FLU frame itself, and its optical child.
    The sockets form a chain rather than a star - each is parented to the socket
    its extrinsics point at - and the one socket whose extrinsics point nowhere is
    parented to `base_frame` with an exact identity.

    `base_frame` defaults to 'oak', which model.sdf declares as a token link
    identity to oak_d_lite_link. It has to be spelled exactly like the driver's
    node, because on hardware that same string also prefixes every image
    frame_id - see the comment on i_tf_base_frame in oak_d_lite.yaml.

    Stamps are left at zero; the caller fills them in. No IMU frame - see the
    module docstring.
    """
    transforms: List[TransformStamped] = []
    for socket, camera in calibration['cameraData']:
        extrinsics = camera['extrinsics']
        to_socket = extrinsics['toCameraSocket']
        child = '{}_{}_camera_frame'.format(name, SOCKET_NAMES[socket])

        chain = TransformStamped()
        chain.child_frame_id = child
        if to_socket == NO_SOCKET:
            chain.header.frame_id = base_frame
            chain.transform.rotation.w = 1.0
        else:
            chain.header.frame_id = '{}_{}_camera_frame'.format(
                name, SOCKET_NAMES[to_socket])
            x, y, z = translation_to_flu(extrinsics['translation'])
            chain.transform.translation.x = x
            chain.transform.translation.y = y
            chain.transform.translation.z = z
            (chain.transform.rotation.x, chain.transform.rotation.y,
             chain.transform.rotation.z, chain.transform.rotation.w) = (
                rotation_to_flu(extrinsics['rotationMatrix']))

        optical = TransformStamped()
        optical.header.frame_id = child
        optical.child_frame_id = '{}_{}_camera_optical_frame'.format(
            name, SOCKET_NAMES[socket])
        (optical.transform.rotation.x, optical.transform.rotation.y,
         optical.transform.rotation.z, optical.transform.rotation.w) = OPTICAL_ROTATION

        transforms.append(chain)
        transforms.append(optical)
    return transforms


class OakCalibrationTf(Node):
    """Publish the calibrated camera frames once, latched, and then sit there."""

    def __init__(self) -> None:
        super().__init__('oak_calibration_tf')
        self.declare_parameter('calibration_file', DEFAULT_CALIBRATION)
        self.declare_parameter('base_frame', 'oak')
        self.declare_parameter('name', 'oak')

        path = self.get_parameter('calibration_file').get_parameter_value().string_value
        base_frame = self.get_parameter('base_frame').get_parameter_value().string_value
        name = self.get_parameter('name').get_parameter_value().string_value

        with open(path) as handle:
            calibration = json.load(handle)
        transforms = calibration_to_transforms(calibration, base_frame, name)

        # Static transforms are looked up regardless of time, so this stamp is
        # only ever cosmetic - which is just as well, because under use_sim_time
        # the clock may still read zero when this runs.
        stamp = self.get_clock().now().to_msg()
        for transform in transforms:
            transform.header.stamp = stamp

        # Held as an attribute because the latched message lives with the
        # broadcaster; let it fall out of scope and the frames go with it.
        self._broadcaster = StaticTransformBroadcaster(self)
        self._broadcaster.sendTransform(transforms)
        self.get_logger().info(
            'published {} camera frames from {}'.format(len(transforms), path))


def main(args=None) -> None:
    """Spin the node until interrupted."""
    rclpy.init(args=args)
    node = OakCalibrationTf()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
