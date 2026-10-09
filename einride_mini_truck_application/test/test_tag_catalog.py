"""AprilTag survey: sightings -> docked poses on the map -> dock database."""

import math
import os

from einride_mini_truck_application.mission.staging import load_dock_frame, load_dock_poses
from einride_mini_truck_application.perception.camera_gate import load_dock_positions
from einride_mini_truck_application.perception.dock_pose import DOCK_GAP, docked_pose, FRONT_OFFSET
from einride_mini_truck_application.perception.tag_catalog import (
    compose, mean_yaw, summarize, write_dock_database)
import pytest

HOME = os.path.join(os.path.dirname(__file__), '..', 'config', 'docks', 'home.yaml')
STANDOFF = DOCK_GAP + FRONT_OFFSET


def sighting(robot, tag_xy, tag_facing):
    """Tag pose in the robot's frame, for a robot at `robot` (x, y, yaw) on the map
    and a tag at `tag_xy` whose face points along `tag_facing` (map yaw)."""
    rx, ry, ryaw = robot
    c, s = math.cos(-ryaw), math.sin(-ryaw)
    dx, dy = tag_xy[0] - rx, tag_xy[1] - ry
    position = [c * dx - s * dy, s * dx + c * dy, 0.1]
    a = tag_facing - ryaw
    z = [math.cos(a), math.sin(a), 0.0]   # face normal
    y = [0.0, 0.0, 1.0]
    x = [y[1] * z[2] - y[2] * z[1], y[2] * z[0] - y[0] * z[2], y[0] * z[1] - y[1] * z[0]]
    return position, [[x[i], y[i], z[i]] for i in range(3)]


def survey_one(robot, tag_xy, tag_facing):
    """What tag_survey records for one sighting."""
    return compose(robot, docked_pose(*sighting(robot, tag_xy, tag_facing)))


def test_compose() -> None:
    assert compose((1.0, 2.0, math.pi / 2), (1.0, 0.0, 0.0)) == pytest.approx(
        (1.0, 3.0, math.pi / 2))


@pytest.mark.parametrize('robot', [(0.0, 0.0, 0.0), (1.0, 0.5, 0.3), (2.0, -0.4, -0.6),
                                   (1.5, 1.5, 2.5)])
def test_same_dock_from_anywhere(robot) -> None:
    # Tag on a wall at (3, 1) facing back along -x: dock STANDOFF in front, facing +x.
    assert survey_one(robot, (3.0, 1.0), math.pi) == pytest.approx((3.0 - STANDOFF, 1.0, 0.0))


def test_dock_faces_the_tag_on_a_side_wall() -> None:
    # Tag at (0.5, 2) facing -y (wall on the left): dock in front, facing +y.
    got = survey_one((0.3, 0.2, 0.4), (0.5, 2.0), -math.pi / 2)
    assert got == pytest.approx((0.5, 2.0 - STANDOFF, math.pi / 2))


def test_mean_yaw_across_pi() -> None:
    assert abs(mean_yaw([math.pi - 0.1, -math.pi + 0.1])) == pytest.approx(math.pi)


def test_summarize_averages() -> None:
    samples = [(1.0 + d, 2.0 - d, 0.1 + d) for d in (-0.02, -0.01, 0.0, 0.01, 0.02)]
    (x, y, yaw), count = summarize(samples)
    assert (x, y, yaw) == pytest.approx((1.0, 2.0, 0.1), abs=1e-6)
    assert count == 5


def test_summarize_drops_outliers() -> None:
    samples = [(1.0, 2.0, 0.0)] * 6 + [(1.8, 2.0, 0.0), (1.0, 2.0, 1.5)]
    pose, count = summarize(samples)
    assert pose == pytest.approx((1.0, 2.0, 0.0))
    assert count == 6


def test_summarize_needs_enough_agreeing() -> None:
    assert summarize([(1.0, 2.0, 0.0)] * 4) is None
    assert summarize([(0.0, 0.0, 0.0), (1.0, 0.0, 0.0), (2.0, 0.0, 0.0),
                      (3.0, 0.0, 0.0), (4.0, 0.0, 0.0)]) is None


def test_written_database_loads_as_a_layout(tmp_path) -> None:
    docks = {0: (2.674, 1.0, 0.0), 2: (0.5, 1.674, math.pi / 2)}
    path = tmp_path / 'room.yaml'
    with open(path, 'w') as out:
        write_dock_database(out, docks, {0: 12, 2: 7}, 'map')
    assert load_dock_frame(str(path)) == 'map'
    poses = load_dock_poses(str(path))
    positions = load_dock_positions(str(path))
    assert sorted(poses) == sorted(positions) == [0, 2]
    for tag, pose in docks.items():
        assert poses[tag] == pytest.approx(pose, abs=1e-4)
        assert positions[tag] == pytest.approx(pose[:2], abs=1e-4)
    assert '#   dock_0 (A): 12 observations' in path.read_text()


def test_hand_written_layout_is_in_arena() -> None:
    assert load_dock_frame(HOME) == 'arena'
