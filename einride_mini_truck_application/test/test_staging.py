"""Choosing a staging pose around obstacles."""

import math
import os
import xml.etree.ElementTree as ET

from einride_mini_truck_application.mission.staging import (
    candidates, choose_staging, Grid, load_dock_poses, load_limits, staging_clearance)
from einride_mini_truck_application.mission.staging_bt import write_tree, XML
import pytest

CONFIG = os.path.join(os.path.dirname(__file__), '..', 'config')
HOME = os.path.join(CONFIG, 'docks', 'home.yaml')
LIMITS = load_limits(os.path.join(CONFIG, 'navigation', 'nav2.yaml'),
                     os.path.join(CONFIG, 'safety', 'collision_monitor.yaml'))
DOCK_G = (1.674, 0.0, 0.0)   # tag 2.0 m straight ahead of the start
SCREEN = (2.03, 0.0, 0.03)   # the laptop showing the tag


def grid_with(obstacles: list[tuple[float, float, float]]) -> Grid:
    """4 x 4 m grid around the start, with round obstacles (x, y, radius), all seen."""
    res, size = 0.05, 80
    ox = oy = -2.0
    data = []
    for iy in range(size):
        for ix in range(size):
            cx, cy = ox + (ix + 0.5) * res, oy + (iy + 0.5) * res
            hit = any(math.hypot(cx - x, cy - y) <= r for x, y, r in obstacles)
            data.append(100 if hit else 0)
    return Grid(ox, oy, res, size, size, data)


def lidar_grid(circles: list[tuple[float, float, float]],
               viewpoints: list[tuple[float, float]]) -> Grid:
    """What Nav2's global costmap shows of round obstacles seen by the LD19P.

    From each viewpoint, 455 beams clear the cells up to their hit and mark the
    hit cell (ObstacleLayer, track_unknown_space). The far side of an obstacle
    stays unknown. Then inflation as in nav2.yaml: unknown cells only take it
    from inscribed up, and values are published as OccupancyGrid.
    """
    res, (w, h), (ox, oy) = 0.05, (90, 80), (-1.0, -2.0)
    cost = [255] * (w * h)

    def index(x: float, y: float) -> int | None:
        ix, iy = math.floor((x - ox) / res), math.floor((y - oy) / res)
        return iy * w + ix if 0 <= ix < w and 0 <= iy < h else None

    for vx, vy in viewpoints:
        hits = []
        for k in range(455):
            ux, uy = math.cos(2 * math.pi * k / 455), math.sin(2 * math.pi * k / 455)
            end, hit = 5.0, None
            for cx, cy, r in circles:
                t = (cx - vx) * ux + (cy - vy) * uy
                d2 = (cx - vx) ** 2 + (cy - vy) ** 2 - t * t
                if t > 0 and d2 <= r * r and t - math.sqrt(r * r - d2) < end:
                    end = t - math.sqrt(r * r - d2)
                    hit = index(vx + ux * end, vy + uy * end)
            for n in range(int(end / (res / 4))):
                i = index(vx + ux * n * res / 4, vy + uy * n * res / 4)
                if i is not None and i != hit:
                    cost[i] = 0
            hits.append(hit)
        for i in hits:
            if i is not None:
                cost[i] = 254
    lethal = [(i % w, i // w) for i, c in enumerate(cost) if c == 254]
    data = []
    for i, c in enumerate(cost):
        d = min((math.hypot(lx - i % w, ly - i // w) * res for lx, ly in lethal), default=9.0)
        if c == 254:
            data.append(100)
        elif d <= 0.12:                       # inscribed radius of the footprint
            data.append(99)
        elif c == 255:
            data.append(-1)
        elif d <= 0.45:                       # inflation_radius, cost_scaling_factor 3
            data.append(1 + 97 * (int(252 * math.exp(-3 * (d - 0.12))) - 1) // 251)
        else:
            data.append(0)
    return Grid(ox, oy, res, w, h, data)


def room(pose: tuple[float, float, float], circle: tuple[float, float, float]) -> float:
    """True distance from the robot centre to the obstacle's surface."""
    return math.hypot(pose[0] - circle[0], pose[1] - circle[1]) - circle[2]


def test_limits_come_from_the_nav2_config() -> None:
    assert LIMITS.staging_distance == pytest.approx(0.7)      # staging_x_offset
    assert LIMITS.dock_clearance == pytest.approx(0.4)        # dock_collision_threshold
    assert LIMITS.box == pytest.approx((0.206, 0.206, 0.13))  # PolygonStop
    assert LIMITS.turn_radius == pytest.approx(math.hypot(0.206, 0.13))
    assert LIMITS.hidden_depth == pytest.approx(2 * math.hypot(0.13, 0.12))


def test_candidate_distances_follow_staging_offset() -> None:
    distances = sorted({round(DOCK_G[0] - p[0], 3) for p in candidates(DOCK_G, LIMITS)
                        if p[1] == 0.0})
    assert distances == pytest.approx([0.4, 0.5, 0.6, 0.7])


def test_nominal_when_free() -> None:
    assert choose_staging(grid_with([]), DOCK_G, LIMITS) == pytest.approx((0.974, 0.0, 0.0))


def test_no_costmap_gives_nominal() -> None:
    assert choose_staging(None, DOCK_G, LIMITS) == pytest.approx((0.974, 0.0, 0.0))


def test_bucket_on_nominal_spot_moves_staging() -> None:
    """2026-10-03: a bucket ~1 m ahead sat on the staging pose; docking failed forever."""
    bucket = (1.1, 0.0, 0.15)
    pose = choose_staging(grid_with([bucket]), DOCK_G, LIMITS)
    assert pose is not None
    assert room(pose, bucket) >= LIMITS.turn_radius
    # Still facing the dock.
    assert pose[2] == pytest.approx(math.atan2(-pose[1], DOCK_G[0] - pose[0]))


def test_no_staging_beside_a_bucket_where_nav2_cannot_turn() -> None:
    """2026-10-03, second attempt: staging at (1.27, 0.30) left ~3 cm between the
    outline and the bucket. Arriving, the robot has to turn to face the dock,
    and the collision monitor's stop zone sweeps 0.24 m, so Nav2 never got
    there and its recoveries spun the robot away from the tag. The old check
    (outline + 5 cm) chose exactly that pose on this lidar view."""
    bucket = (1.30, -0.05, 0.15)
    grid = lidar_grid([bucket, SCREEN], [(0.0, 0.0)])
    cramped = (1.274, 0.30, math.atan2(-0.30, 0.40))
    assert staging_clearance(grid, cramped, LIMITS) < LIMITS.turn_radius
    pose = choose_staging(grid, DOCK_G, LIMITS)
    assert pose is not None and pose[:2] != pytest.approx(cramped[:2])
    assert room(pose, bucket) >= LIMITS.turn_radius


def test_unseen_back_of_a_bucket_counts_as_bucket() -> None:
    """The lidar sees about half of a round bucket; the old check put the robot
    over the unseen half here (outline 2 cm inside the real bucket)."""
    bucket = (0.95, 0.0, 0.12)
    first = choose_staging(lidar_grid([bucket, SCREEN], [(0.0, 0.0)]), DOCK_G, LIMITS)
    assert first is not None and room(first, bucket) >= LIMITS.turn_radius
    grid = lidar_grid([bucket, SCREEN], [(0.0, 0.0), first[:2]])
    second = choose_staging(grid, DOCK_G, LIMITS, skip=[first])
    assert second is not None and room(second, bucket) >= LIMITS.turn_radius


@pytest.mark.parametrize('bucket', [
    (0.95, -0.1, 0.15), (1.0, 0.05, 0.12), (1.05, -0.05, 0.15), (1.1, 0.0, 0.12),
    (1.15, -0.2, 0.15), (1.2, 0.1, 0.15), (1.25, 0.0, 0.12), (1.3, 0.05, 0.15)])
def test_every_attempt_has_room_to_turn(bucket: tuple[float, float, float]) -> None:
    """First attempt seen from the start, the retry also from the first staging
    pose: both keep the stop zone off the real bucket in any heading."""
    first = choose_staging(lidar_grid([bucket, SCREEN], [(0.0, 0.0)]), DOCK_G, LIMITS)
    grid = lidar_grid([bucket, SCREEN], [(0.0, 0.0), first[:2]])
    second = choose_staging(grid, DOCK_G, LIMITS, skip=[first])
    assert room(first, bucket) >= LIMITS.turn_radius
    assert room(second, bucket) >= LIMITS.turn_radius


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
    pose = choose_staging(grid, DOCK_G, LIMITS)
    assert pose is not None
    assert abs(pose[1]) >= 0.3                      # beside the shadow, not in it


def test_dock_fully_blocked() -> None:
    """Something parked right in the bay: no staging pose, wait instead."""
    assert choose_staging(grid_with([(1.45, 0.0, 0.5)]), DOCK_G, LIMITS) is None


def test_no_room_to_turn_anywhere_still_gives_the_roomiest_pose() -> None:
    """Posts all around the bay: never wait just because turning is tight."""
    posts = [(DOCK_G[0] - d, y, 0.03) for d in (0.25, 0.55, 0.85, 1.15)
             for y in (-0.8, -0.5, -0.2, 0.2, 0.5, 0.8)]
    grid = grid_with(posts)
    pose = choose_staging(grid, DOCK_G, LIMITS)
    assert pose is not None
    assert 0 < staging_clearance(grid, pose, LIMITS) < LIMITS.turn_radius


def test_retry_skips_failed_staging_pose() -> None:
    """2026-10-03: tag not visible from the chosen pose; every retry went back there."""
    bucket = (1.1, 0.0, 0.15)
    grid = grid_with([bucket])
    first = choose_staging(grid, DOCK_G, LIMITS)
    second = choose_staging(grid, DOCK_G, LIMITS, skip=[first])
    assert second is not None and second[:2] != pytest.approx(first[:2])
    assert room(second, bucket) >= LIMITS.turn_radius


def test_all_skipped_gives_none() -> None:
    assert choose_staging(None, DOCK_G, LIMITS, skip=candidates(DOCK_G, LIMITS)) is None


def test_objects_at_the_dock_itself_are_ignored() -> None:
    """The tag holder / dock walls sit within the last 0.4 m."""
    grid = grid_with([(2.0, 0.0, 0.1)])   # the screen showing the tag
    assert choose_staging(grid, DOCK_G, LIMITS) == pytest.approx((0.974, 0.0, 0.0))


def test_candidates_face_the_dock_for_rotated_dock() -> None:
    dock = (0.42, 0.674, math.pi / 2)    # spot 2: approached from -y
    first = candidates(dock, LIMITS)[0]
    assert first == pytest.approx((0.42, -0.026, math.pi / 2))


def test_home_layout_loads_yaw() -> None:
    poses = load_dock_poses(HOME)
    assert poses[1] == pytest.approx((0.42, 0.674, 1.5708))


def test_all_unseen_still_gives_a_pose() -> None:
    """Never wait forever just because the lidar has not seen the bay yet."""
    grid = grid_with([])
    grid.data[:] = [-1] * len(grid.data)
    assert choose_staging(grid, DOCK_G, LIMITS) == pytest.approx((0.974, 0.0, 0.0))


def test_staging_tree_has_no_spin_and_gives_up_quickly(tmp_path) -> None:
    """Recoveries must not turn the robot away from the tag, and Nav2 should hand
    a failed staging pose back to the mission instead of looping."""
    path = write_tree(str(tmp_path))
    root = ET.parse(path).getroot()
    assert root.get('BTCPP_format') == '4'
    names = {node.tag for node in root.iter()}
    assert {'ComputePathToPose', 'FollowPath', 'BackUp'} <= names
    assert not names & {'Spin', 'Wait'}
    top = root.find('BehaviorTree/RecoveryNode')
    assert int(top.get('number_of_retries')) <= 2
    assert open(path).read() == XML
