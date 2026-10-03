"""Choosing a staging pose around obstacles."""

import math
import os

from einride_mini_truck_application.mission.staging import (
    candidates, choose_staging, footprint_free, Grid, lane_free, load_dock_poses)
import pytest

HOME = os.path.join(os.path.dirname(__file__), '..', 'config', 'docks', 'home.yaml')
DOCK_G = (1.674, 0.0, 0.0)   # tag 2.0 m straight ahead of the start


def grid_with(obstacles: list[tuple[float, float, float]]) -> Grid:
    """4 x 4 m grid around the start, with round obstacles (x, y, radius)."""
    res, size = 0.05, 80
    ox = oy = -2.0
    data = []
    for iy in range(size):
        for ix in range(size):
            cx, cy = ox + (ix + 0.5) * res, oy + (iy + 0.5) * res
            hit = any(math.hypot(cx - x, cy - y) <= r for x, y, r in obstacles)
            data.append(100 if hit else 0)
    return Grid(ox, oy, res, size, size, data)


def test_nominal_when_free() -> None:
    assert choose_staging(grid_with([]), DOCK_G) == pytest.approx((0.974, 0.0, 0.0))


def test_no_costmap_gives_nominal() -> None:
    assert choose_staging(None, DOCK_G) == pytest.approx((0.974, 0.0, 0.0))


def test_bucket_on_nominal_spot_moves_staging() -> None:
    """2026-10-03: a bucket ~1 m ahead sat on the staging pose; docking failed forever."""
    grid = grid_with([(1.1, 0.0, 0.15)])
    pose = choose_staging(grid, DOCK_G)
    assert pose is not None
    assert pose[:2] != pytest.approx((0.974, 0.0))
    assert footprint_free(grid, pose) and lane_free(grid, pose, DOCK_G)
    # Still facing the dock.
    assert pose[2] == pytest.approx(math.atan2(-pose[1], DOCK_G[0] - pose[0]))


def test_lidar_shadow_behind_bucket_is_not_staging() -> None:
    """2026-10-03: lidar saw only the bucket's front face; the spot behind it
    looked free and was chosen. Unseen cells (-1) must not hold the robot."""
    grid = grid_with([(0.95, 0.0, 0.05)])           # the visible front face
    for i, (iy, ix) in enumerate((iy, ix) for iy in range(grid.height)
                                 for ix in range(grid.width)):
        x = grid.origin_x + (ix + 0.5) * grid.resolution
        y = grid.origin_y + (iy + 0.5) * grid.resolution
        if 1.0 < x < 1.4 and abs(y) < 0.1 and grid.data[i] == 0:
            grid.data[i] = -1                       # shadow behind it
    pose = choose_staging(grid, DOCK_G)
    assert pose is not None
    assert footprint_free(grid, pose, unknown_blocks=True)
    assert abs(pose[1]) >= 0.3                      # beside the shadow, not in it


def test_dock_fully_blocked() -> None:
    """Something parked right in the bay: no staging pose, wait instead."""
    assert choose_staging(grid_with([(1.45, 0.0, 0.5)]), DOCK_G) is None


def test_retry_skips_failed_staging_pose() -> None:
    """2026-10-03: tag not visible from the chosen pose; every retry went back there."""
    grid = grid_with([(1.1, 0.0, 0.15)])
    first = choose_staging(grid, DOCK_G)
    second = choose_staging(grid, DOCK_G, skip=[first])
    assert second is not None and second[:2] != pytest.approx(first[:2])
    assert footprint_free(grid, second) and lane_free(grid, second, DOCK_G)


def test_all_skipped_gives_none() -> None:
    assert choose_staging(None, DOCK_G, skip=candidates(DOCK_G)) is None


def test_objects_at_the_dock_itself_are_ignored() -> None:
    """The tag holder / dock walls sit within the last 0.4 m."""
    grid = grid_with([(2.0, 0.0, 0.1)])   # the screen showing the tag
    assert choose_staging(grid, DOCK_G) == pytest.approx((0.974, 0.0, 0.0))


def test_candidates_face_the_dock_for_rotated_dock() -> None:
    dock = (0.42, 0.674, math.pi / 2)    # spot 2: approached from -y
    first = candidates(dock)[0]
    assert first == pytest.approx((0.42, -0.026, math.pi / 2))


def test_home_layout_loads_yaw() -> None:
    poses = load_dock_poses(HOME)
    assert poses[1] == pytest.approx((0.42, 0.674, 1.5708))


def test_all_unseen_still_gives_a_pose() -> None:
    """Never wait forever just because the lidar has not seen the bay yet."""
    grid = grid_with([])
    grid.data[:] = [-1] * len(grid.data)
    assert choose_staging(grid, DOCK_G) == pytest.approx((0.974, 0.0, 0.0))
