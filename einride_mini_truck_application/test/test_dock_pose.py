"""Docked pose computed from a tag pose in the robot frame."""

import math

from einride_mini_truck_application.perception.dock_pose import DOCK_GAP, docked_pose, FRONT_OFFSET
import pytest


def rotation_facing(yaw: float, z_into_wall: bool = False) -> list[list[float]]:
    """Tag rotation whose z axis (face normal) points along `yaw` in the floor plane.

    x = tag's right, y = up, z = normal; flipped if the detector's z points into the wall.
    """
    c, s = math.cos(yaw), math.sin(yaw)
    z = [c, s, 0.0]
    if z_into_wall:
        z = [-c, -s, 0.0]
    y = [0.0, 0.0, 1.0]
    x = [y[1] * z[2] - y[2] * z[1], y[2] * z[0] - y[0] * z[2], y[0] * z[1] - y[1] * z[0]]
    return [[x[i], y[i], z[i]] for i in range(3)]


@pytest.mark.parametrize('z_into_wall', [False, True])
def test_tag_straight_ahead(z_into_wall: bool) -> None:
    # Tag 1 m ahead, facing the robot (normal points back along -x).
    x, y, yaw = docked_pose([1.0, 0.0, 0.13], rotation_facing(math.pi, z_into_wall))
    assert x == pytest.approx(1.0 - DOCK_GAP - FRONT_OFFSET)
    assert y == pytest.approx(0.0)
    assert yaw == pytest.approx(0.0)


def test_front_gap_is_dock_gap() -> None:
    x, _, _ = docked_pose([0.8, 0.0, 0.13], rotation_facing(math.pi))
    assert 0.8 - (x + FRONT_OFFSET) == pytest.approx(DOCK_GAP)


def test_angled_tag_off_to_the_left() -> None:
    # Tag 1 m ahead and 0.5 m left, its face turned 30 deg toward the robot's side.
    normal_yaw = math.pi - math.radians(30)
    x, y, yaw = docked_pose([1.0, 0.5, 0.13], rotation_facing(normal_yaw))
    standoff = DOCK_GAP + FRONT_OFFSET
    assert x == pytest.approx(1.0 + standoff * math.cos(normal_yaw))
    assert y == pytest.approx(0.5 + standoff * math.sin(normal_yaw))
    # The docked robot looks straight at the tag, against its normal.
    assert yaw == pytest.approx(-math.radians(30))
