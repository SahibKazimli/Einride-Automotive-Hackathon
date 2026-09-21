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

"""Holds the simulated mono cameras to the real camera's calibration.

model.sdf has to state where the two mono sensors sit, because Gazebo aims a
camera from a pose and cannot read an EEPROM. Those poses therefore duplicate
numbers that also live in the calibration dump, and duplicated numbers rot: TF
would keep tracking the device while the rendered images quietly did not.

So they are not trusted, they are recomputed here from the dump and compared.
Getting this wrong is otherwise close to undetectable - the images still arrive,
they are just taken from 0.75 mm off, which is exactly the error a nominal
plus-or-minus-half-baseline would introduce.

Lives in this package rather than in einride_mini_truck_description because the
arithmetic belongs to oak_calibration_tf and description cannot depend on gazebo
without a cycle. No Gazebo and no camera on the USB bus.
"""

import json
import math
import os
import xml.etree.ElementTree as ElementTree

from ament_index_python.packages import get_package_share_directory

from einride_mini_truck_gazebo.oak_calibration_tf import (
    calibration_to_transforms,
    DEFAULT_CALIBRATION,
)

import pytest

MODEL_SDF = os.path.join(
    get_package_share_directory('einride_mini_truck_description'),
    'models', 'einride_mini_truck', 'model.sdf')

# The generated URDF, where every joint origin is already resolved. Cheaper to
# compose than chasing model.sdf's relative_to chain, and it is what
# robot_state_publisher actually broadcasts.
MODEL_URDF = os.path.join(
    get_package_share_directory('einride_mini_truck_description'),
    'models', 'einride_mini_truck', 'model.urdf')

# The camera sits on top of the chassis, about 0.10 m up. Purely a sanity floor:
# anything near zero means the camera has ended up at the robot's ground-projected
# origin rather than on the robot.
MINIMUM_CAMERA_HEIGHT = 0.05

# Measured on the device, from the published camera_info: fx 452.6538 (left) and
# 453.8520 (right) over 640 px. Per camera, because the two genuinely differ.
EXPECTED_FX = {'oak_left': 452.6538, 'oak_right': 453.8520}

# A tenth of what the left/right asymmetry would cost if it were ignored.
POSITION_TOLERANCE = 1e-6


def _sensor(name):
    model = ElementTree.parse(MODEL_SDF).getroot().find('model')
    for link in model.findall('link'):
        for sensor in link.findall('sensor'):
            if sensor.get('name') == name:
                return sensor
    raise AssertionError('{} is not in model.sdf'.format(name))


def _pose(sensor):
    return [float(v) for v in sensor.find('pose').text.split()]


def _rotate(rotation, vector):
    """Rotate a vector by a geometry_msgs Quaternion, via the cross-product form."""
    vx, vy, vz = vector
    x, y, z, w = rotation.x, rotation.y, rotation.z, rotation.w
    tx = 2.0 * (y * vz - z * vy)
    ty = 2.0 * (z * vx - x * vz)
    tz = 2.0 * (x * vy - y * vx)
    return (vx + w * tx + y * tz - z * ty,
            vy + w * ty + z * tx - x * tz,
            vz + w * tz + x * ty - y * tx)


@pytest.fixture(scope='module')
def calibrated():
    """Locate each camera frame relative to oak_d_lite_link, per the dump."""
    with open(DEFAULT_CALIBRATION) as handle:
        calibration = json.load(handle)
    transforms = {t.child_frame_id: t for t in calibration_to_transforms(calibration)}
    # The chain is left -> right -> rgb -> oak_d_lite_link, and the sensors are
    # posed in the link, so it has to be walked rather than read off.
    #
    # The rotations cannot be skipped even though none exceeds 0.7 degrees: the
    # left camera is a whole baseline out along the chain, and half a degree
    # across 75 mm is 0.36 mm - a third of a millimetre of error, which is more
    # than the asymmetry this test exists to protect.
    positions = {}
    for socket in ('left', 'right'):
        position = (0.0, 0.0, 0.0)
        frame = 'oak_{}_camera_frame'.format(socket)
        while frame in transforms:
            step = transforms[frame].transform
            rotated = _rotate(step.rotation, position)
            position = (rotated[0] + step.translation.x,
                        rotated[1] + step.translation.y,
                        rotated[2] + step.translation.z)
            frame = transforms[frame].header.frame_id
        positions[socket] = position
    return positions


@pytest.mark.parametrize('name', ['oak_left', 'oak_right'])
def test_sensor_pose_matches_the_calibration(name, calibrated):
    """The rendered image must come from where the device says the camera is."""
    socket = name.split('_')[1]
    pose = _pose(_sensor(name))
    for axis, (got, want) in enumerate(zip(pose[:3], calibrated[socket])):
        assert abs(got - want) < POSITION_TOLERANCE, 'axis {}'.format(axis)


def test_the_pair_is_not_symmetric_about_the_colour_camera(calibrated):
    """Stated positively, because the obvious wrong answer is the symmetric one.

    Right sits 37.00 mm out and left 37.75 mm. If this ever becomes symmetric,
    someone has replaced measured extrinsics with a nominal half-baseline.
    """
    assert abs(abs(calibrated['left'][1]) - abs(calibrated['right'][1])) > 5e-4


@pytest.mark.parametrize('name', ['oak_left', 'oak_right'])
def test_field_of_view_reproduces_the_measured_focal_length(name):
    """horizontal_fov is how Gazebo is told the intrinsics; check what it implies.

    A typo here produces a perfectly plausible image at the wrong scale, which
    nothing downstream would flag.
    """
    camera = _sensor(name).find('camera')
    width = int(camera.find('image/width').text)
    hfov = float(camera.find('horizontal_fov').text)
    assert abs(width / 2.0 / math.tan(hfov / 2.0) - EXPECTED_FX[name]) < 0.01


@pytest.mark.parametrize('name', ['oak_left', 'oak_right'])
def test_stream_matches_the_hardware_topic(name):
    """Resolution, encoding, rate and frame_id are the parity contract."""
    sensor = _sensor(name)
    camera = sensor.find('camera')
    side = name.split('_')[1]
    assert sensor.get('type') == 'camera'
    assert sensor.findtext('topic') == 'oak/{}/image_raw'.format(side)
    assert float(sensor.findtext('update_rate')) == 30.0
    assert camera.findtext('image/width') == '640'
    assert camera.findtext('image/height') == '480'
    # L8 is what the bridge turns into mono8; R8G8B8 would silently give rgb8.
    assert camera.findtext('image/format') == 'L8'
    assert camera.findtext('optical_frame_id') == \
        'oak_{}_camera_optical_frame'.format(side)


def test_the_description_does_not_declare_the_calibrated_frames():
    """These come from the device now, and two publishers for one frame is a race.

    tf2's static cache is keyed by child frame and simply overwrites, and
    /tf_static is latched, so a duplicate does not warn - it just makes the tree
    depend on message arrival order, differently for each subscriber.
    """
    model = ElementTree.parse(MODEL_SDF).getroot().find('model')
    links = {link.get('name') for link in model.findall('link')}
    for frame in ('oak_rgb_camera_frame', 'oak_rgb_camera_optical_frame',
                  'oak_left_camera_frame', 'oak_left_camera_optical_frame',
                  'oak_right_camera_frame', 'oak_right_camera_optical_frame'):
        assert frame not in links, '{} must come from the calibration'.format(frame)
    # The exception, and the only one: the driver's IMU transform is 120 degrees
    # wrong, so this frame stays with the description in both modes.
    assert 'oak_imu_frame' in links
    # And the attachment point the driver's chain needs, which must be spelled
    # like the driver's node because it doubles as the image frame_id prefix.
    assert 'oak' in links


def _urdf_chain_to(link):
    """Compose translations from base_footprint down to `link` in the URDF.

    Rotations are ignored: every joint on this path is axis-aligned, and the
    question here is only where the camera ended up, not how it is oriented.
    """
    urdf = ElementTree.parse(MODEL_URDF).getroot()
    parents, origins = {}, {}
    for joint in urdf.findall('joint'):
        child = joint.find('child').get('link')
        parents[child] = joint.find('parent').get('link')
        xyz = joint.find('origin')
        origins[child] = [float(v) for v in
                          (xyz.get('xyz') if xyz is not None else '0 0 0').split()]
    total = [0.0, 0.0, 0.0]
    while link in parents:
        total = [a + b for a, b in zip(total, origins[link])]
        link = parents[link]
    return total


def test_the_camera_is_on_the_robot_not_at_its_origin():
    """Where the camera actually ends up, measured from base_footprint.

    Every other check in this file and in test_oak_calibration_tf.py is relative
    - chain links against each other, simulation against hardware - and a common
    offset is invisible to all of them. This one is absolute, and it exists
    because exactly that happened: `oak` was declared with no <pose>, an SDF link
    with no pose defaults to the MODEL frame rather than to its joint parent, and
    the whole camera subtree silently moved 66 mm back and 102 mm down onto the
    floor. Simulation and hardware agreed with each other perfectly the whole
    time, because both read the same broken description.
    """
    body = _urdf_chain_to('oak_d_lite_link')
    attachment = _urdf_chain_to('oak')
    assert body[2] > MINIMUM_CAMERA_HEIGHT, \
        'camera body is at z={:.4f}, i.e. on the floor'.format(body[2])
    for axis, (got, want) in enumerate(zip(attachment, body)):
        assert abs(got - want) < POSITION_TOLERANCE, (
            "'oak' must be coincident with oak_d_lite_link; axis {} differs by "
            '{:.4f} m. Did the link lose its <pose relative_to>?'
            .format(axis, got - want))
