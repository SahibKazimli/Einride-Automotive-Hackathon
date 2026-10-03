"""Where to stand before docking, chosen from the live costmap. No ROS imports.

Nav2's docking server drives to one fixed staging pose 0.7 m straight in front
of the dock, then servos onto the tag. If anything sits on that pose (a bucket,
another robot) it can never get there and docking fails forever. Instead we try
a set of candidate poses around the dock, all facing it, and pick the first
one where the robot fits AND the lane from there into the dock is clear. Nav2
then routes there around obstacles, and docking runs without its own staging.

Grid values follow nav_msgs/OccupancyGrid as published by Nav2's costmap:
100 = lethal (an obstacle cell), 99 = inscribed, lower = inflation or free,
-1 = unknown (needs track_unknown_space: true in the global costmap).

A 2D lidar only sees the near face of an obstacle; the space behind it stays
unknown. The staging pose itself must therefore be on cells the lidar has
actually seen free, or it can land "behind" (in reality inside) a bucket
(2026-10-03). The lane to the dock only avoids obstacle cells: near the dock
much is unseen, and the docking server checks the approach itself.
"""

import math
from dataclasses import dataclass
from typing import Optional, Sequence

import yaml

Pose = tuple[float, float, float]   # x, y, yaw

# Robot outline from config/navigation/nav2.yaml (base_footprint centred).
HALF_LENGTH = 0.13
HALF_WIDTH = 0.12
# Keep this much clear around the outline. 0.05 put staging 3 cm from a bucket
# and the docking controller's curved approach hit it (2026-10-03).
MARGIN = 0.15
# Behind the robot less is needed: docking drives forward, away from it. This
# lets the robot stand in the gap between a bucket and the dock.
REAR_MARGIN = 0.05
# Last resort when nothing else fits: the old margin, tight but drivable.
TIGHT_MARGIN = 0.05
LETHAL = 100

# Tried in this order: straight in front first, nominal distance first.
DISTANCES = (0.7, 0.6, 0.5, 0.4, 0.3, 0.25, 0.2)   # <= 0.3: gap between obstacle and dock
LATERALS = (0.0, 0.15, -0.15, 0.3, -0.3, 0.45, -0.45, 0.6, -0.6)   # 0.6: lane past a bucket
# The last part of the lane next to the dock is ignored: the dock's own walls
# and the tag holder are there (matches docking_server dock_collision_threshold).
DOCK_CLEARANCE = 0.4
# The tag stands this far beyond the docked pose (perception/dock_pose.py:
# 0.126 m robot front + 0.200 m gap).
TAG_DISTANCE = 0.326
# Staging must see the tag within this angle off its face. ~30 deg is known to
# detect; (1.27, 0.6) beside a bucket saw it at ~40 deg and was borderline.
MAX_VIEW_ANGLE = math.radians(35.0)


def load_dock_poses(path: str) -> dict[int, Pose]:
    """Tag id -> (x, y, yaw) of its docked pose, from a Nav2 dock database file."""
    with open(path) as f:
        data = yaml.safe_load(f) or {}
    return {int(d['id']): (float(d['pose'][0]), float(d['pose'][1]), float(d['pose'][2]))
            for d in (data.get('docks') or {}).values()}


@dataclass
class Grid:
    """An axis-aligned occupancy grid (Nav2 costmaps are never rotated)."""
    origin_x: float
    origin_y: float
    resolution: float
    width: int
    height: int
    data: Sequence[int]

    def cost(self, x: float, y: float) -> int:
        ix = math.floor((x - self.origin_x) / self.resolution)
        iy = math.floor((y - self.origin_y) / self.resolution)
        if not (0 <= ix < self.width and 0 <= iy < self.height):
            return -1   # outside the rolling window: unknown
        return self.data[iy * self.width + ix]


def footprint_free(grid: Grid, pose: Pose, margin: float = MARGIN,
                   unknown_blocks: bool = False) -> bool:
    """True if no lethal cell lies under the robot outline (+ margin) at `pose`.

    The rear gets at most REAR_MARGIN. `unknown_blocks`: also refuse cells
    never seen by the lidar (-1).
    """
    x, y, yaw = pose
    c, s = math.cos(yaw), math.sin(yaw)
    front, back = HALF_LENGTH + margin, HALF_LENGTH + min(margin, REAR_MARGIN)
    hw = HALF_WIDTH + margin
    step = grid.resolution / 2
    nx, ny = int((front + back) / step) + 1, int(2 * hw / step) + 1
    for i in range(nx + 1):
        for j in range(ny + 1):
            lx = -back + (front + back) * i / nx
            ly = -hw + 2 * hw * j / ny
            cost = grid.cost(x + c * lx - s * ly, y + s * lx + c * ly)
            if cost >= LETHAL or (unknown_blocks and cost < 0):
                return False
    return True


def lane_free(grid: Grid, start: Pose, dock: Pose,
              clearance: float = DOCK_CLEARANCE, margin: float = MARGIN) -> bool:
    """True if the robot fits at every point on the straight line start -> dock,
    except the last `clearance` metres next to the dock."""
    dx, dy = dock[0] - start[0], dock[1] - start[1]
    length = math.hypot(dx, dy)
    if length <= clearance:
        return True
    yaw = math.atan2(dy, dx)
    step = grid.resolution
    n = int((length - clearance) / step)
    for k in range(n + 1):
        f = k * step / length
        if not footprint_free(grid, (start[0] + f * dx, start[1] + f * dy, yaw), margin):
            return False
    return True


def view_angle(pose: Pose, dock: Pose) -> float:
    """Angle between the tag's face normal and the line from the tag to `pose`."""
    x, y, yaw = dock
    tx, ty = x + TAG_DISTANCE * math.cos(yaw), y + TAG_DISTANCE * math.sin(yaw)
    seen_from = math.atan2(pose[1] - ty, pose[0] - tx)
    return abs(math.remainder(seen_from - (yaw + math.pi), math.tau))


def candidates(dock: Pose, distances: Sequence[float] = DISTANCES,
               laterals: Sequence[float] = LATERALS) -> list[Pose]:
    """Staging poses around `dock`, each facing the docked position.

    Most head-on first (then farthest first), so a robot blocked straight out
    tries the gap closer to the dock before swinging out to the side: a short
    straight approach is easier for the docking controller than a curve.
    """
    x, y, yaw = dock
    c, s = math.cos(yaw), math.sin(yaw)
    out = []
    for lat in laterals:
        for d in distances:
            px = x - c * d - s * lat
            py = y - s * d + c * lat
            out.append((px, py, math.atan2(y - py, x - px)))
    # Stable sort; rounded so float noise does not reorder equal angles.
    return sorted(out, key=lambda p: round(view_angle(p, dock), 6))


def choose_staging(grid: Optional[Grid], dock: Pose,
                   skip: Sequence[Pose] = ()) -> Optional[Pose]:
    """First candidate where the robot fits and can drive into the dock.

    `skip`: poses where docking already failed (e.g. tag not seen from there),
    so a retry looks from somewhere else instead of repeating the same failure.
    No grid yet: the first candidate (Nav2 will find out if it is blocked).
    The robot must keep trying, so when nothing passes the strict checks it
    relaxes them step by step; None only if every candidate is skipped or the
    robot does not even fit at any of them.
    """
    poses = [p for p in candidates(dock)
             if not any(math.dist(p[:2], s[:2]) < 0.01 for s in skip)]
    if grid is None:
        return poses[0] if poses else None
    # (margin, refuse unseen ground, require tag in view and a clear lane)
    passes = ((MARGIN, True, True),        # seen free, comfortable clearance
              (MARGIN, False, True),       # accept ground the lidar has not seen
              (TIGHT_MARGIN, False, True),  # squeeze past
              (TIGHT_MARGIN, False, False))  # least bad: somewhere to look from
    for margin, unknown_blocks, strict in passes:
        for pose in poses:
            if not footprint_free(grid, pose, margin, unknown_blocks):
                continue
            if strict and (view_angle(pose, dock) > MAX_VIEW_ANGLE
                           or not lane_free(grid, pose, dock, margin=margin)):
                continue
            return pose
    return None
