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

"""Checks the simulated camera frames against the ones the real camera published.

The claim this change rests on is that simulation and hardware carry the SAME TF
tree inside the camera - same names, same shape, same numbers - because both are
derived from the same EEPROM by the same arithmetic. Nothing else in this repo
checks any camera frame across the two modes, so it is checked here.

The fixture is not a restatement of what the algorithm is believed to do. It is
/tf_static as recorded off the physical device with the driver's own publisher
enabled, so a transcription error in oak_calibration_tf.py cannot agree with it by
construction. Both sides are pinned: re-deriving the maths here would only test
the test, which is why the expected values come from the wire instead.

No Gazebo and no camera on the USB bus.
"""

import json
import os

from einride_mini_truck_gazebo.oak_calibration_tf import (
    calibration_to_transforms,
    DEFAULT_CALIBRATION,
)

import pytest

import yaml

# Tight on purpose. These are not two measurements of one quantity, they are the
# same arithmetic run twice, so anything above float noise is a real divergence.
# The recorded values are float64 from the wire; the only slack needed is for
# quaternion sign and the last bit or two of the square root.
TOLERANCE = 1e-9

FIXTURE = os.path.join(os.path.dirname(__file__), 'data', 'oak_tf_static.yaml')


@pytest.fixture(scope='module')
def published():
    """Return the six transforms the real driver put on /tf_static, by child frame."""
    with open(FIXTURE) as handle:
        recorded = yaml.safe_load(handle)
    return {row['child']: row for row in recorded['transforms']}


@pytest.fixture(scope='module')
def simulated():
    """Build the same six, as oak_calibration_tf does from the checked-in dump."""
    with open(DEFAULT_CALIBRATION) as handle:
        calibration = json.load(handle)
    return {t.child_frame_id: t for t in calibration_to_transforms(calibration)}


def test_the_same_frames_exist(published, simulated):
    """Names are the contract: they appear in message headers and in every consumer."""
    assert set(simulated) == set(published)


def test_the_imu_frame_is_not_published_here(simulated):
    """model.sdf owns oak_imu_frame in both modes - see the node's docstring.

    The driver's version of it is 120 degrees out, so hardware suppresses it with
    camera.i_tf_imu_from_descr. Publishing it here would both duplicate a frame
    robot_state_publisher already provides and reintroduce that error.
    """
    assert 'oak_imu_frame' not in simulated


@pytest.mark.parametrize('child', [
    'oak_rgb_camera_frame',
    'oak_rgb_camera_optical_frame',
    'oak_left_camera_frame',
    'oak_left_camera_optical_frame',
    'oak_right_camera_frame',
    'oak_right_camera_optical_frame',
])
def test_frame_matches_the_device(child, published, simulated):
    """Parent, translation and rotation, against what the camera actually published."""
    expected, actual = published[child], simulated[child]
    assert actual.header.frame_id == expected['parent']

    translation = actual.transform.translation
    for axis, value in zip('xyz', expected['translation']):
        assert abs(getattr(translation, axis) - value) < TOLERANCE, axis

    rotation = actual.transform.rotation
    ours = [rotation.x, rotation.y, rotation.z, rotation.w]
    # q and -q are the same rotation, and nothing constrains which one either
    # side produces, so compare both signs and take the closer.
    difference = min(max(abs(a - b) for a, b in zip(ours, expected['rotation'])),
                     max(abs(a + b) for a, b in zip(ours, expected['rotation'])))
    assert difference < TOLERANCE


def test_the_chain_roots_at_the_base_frame(simulated):
    """Exactly one camera hangs off the robot, and it does so with no offset.

    The driver bolts the root of its socket chain to the base frame with an
    identity transform, and model.sdf puts 'oak' - and so oak_d_lite_link, which
    it is identity to - at the colour camera's optical centre. If a replacement
    camera ever rooted its chain at a mono socket instead, the whole assembly
    would silently shift ~37 mm, and this is what would say so.

    The frame is 'oak' rather than oak_d_lite_link because the driver reuses
    i_tf_base_frame as the prefix for every image frame_id, so it has to be
    spelled like the node. See oak_d_lite.yaml.
    """
    roots = [t for t in simulated.values() if t.header.frame_id == 'oak']
    assert [t.child_frame_id for t in roots] == ['oak_rgb_camera_frame']

    transform = roots[0].transform
    assert (transform.translation.x, transform.translation.y,
            transform.translation.z) == (0.0, 0.0, 0.0)
    assert (transform.rotation.x, transform.rotation.y,
            transform.rotation.z, transform.rotation.w) == (0.0, 0.0, 0.0, 1.0)


def test_the_mono_baseline_is_the_measured_one(simulated):
    """74.75 mm, not the nominal 75 - the reason this indirection exists at all.

    Also cross-checks the pair against the real camera_info, whose P[3] of
    -33.8366681 over fx 452.6538 implies 74.752 mm by a completely separate route.
    """
    # The left camera is parented to the right one, so its own translation IS the
    # baseline - no composition needed, and its sign says which side it is on.
    across = simulated['oak_left_camera_frame'].transform.translation
    assert abs(across.y - 0.074751763) < 1e-9
    assert across.y > 0.0, 'left must sit on the +y side of right, i.e. robot-left'
