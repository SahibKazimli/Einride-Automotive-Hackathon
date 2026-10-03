"""Where to stand before docking, chosen from the live costmap. No ROS imports.

Nav2's docking server drives to one fixed staging pose 0.7 m straight in front
of the dock, then servos onto the tag. If anything sits on that pose (a bucket,
another robot) it can never get there and docking fails forever. Instead we try
a set of candidate poses around the dock, all facing it, and pick the first
one where the robot fits AND the lane from there into the dock is clear. Nav2
then routes there around obstacles, and docking runs without its own staging.

Grid values follow nav_msgs/OccupancyGrid as published by Nav2's costmap:
100 = lethal (an obstacle cell), 99 = inscribed, lower = inflation or free,
-1 = unknown (treated as free; the arena has no map).
"""

import math
from dataclasses import dataclass
from typing import Optional, Sequence

import yaml

Pose = tuple[float, float, float]   # x, y, yaw

# Robot outline from config/navigation/nav2.yaml (base_footprint centred).
HALF_LENGTH = 0.13
HALF_WIDTH = 0.12
MARGIN = 0.05         # keep this much clear around the outline
LETHAL = 100

# Tried in this order: straight in front first, nominal distance first.
DISTANCES = (0.7, 0.6, 0.5, 0.4)
LATERALS = (0.0, 0.15, -0.15, 0.3, -0.3)
# The last part of the lane next to the dock is ignored: the dock's own walls
# and the tag holder are there (matches docking_server dock_collision_threshold).
DOCK_CLEARANCE = 0.4


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


def footprint_free(grid: Grid, pose: Pose, margin: float = MARGIN) -> bool:
    """True if no lethal cell lies under the robot outline (+ margin) at `pose`."""
    x, y, yaw = pose
    c, s = math.cos(yaw), math.sin(yaw)
    hl, hw = HALF_LENGTH + margin, HALF_WIDTH + margin
    step = grid.resolution / 2
    nx, ny = int(2 * hl / step) + 1, int(2 * hw / step) + 1
    for i in range(nx + 1):
        for j in range(ny + 1):
            lx = -hl + 2 * hl * i / nx
            ly = -hw + 2 * hw * j / ny
            if grid.cost(x + c * lx - s * ly, y + s * lx + c * ly) >= LETHAL:
                return False
    return True


def lane_free(grid: Grid, start: Pose, dock: Pose,
              clearance: float = DOCK_CLEARANCE) -> bool:
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
        if not footprint_free(grid, (start[0] + f * dx, start[1] + f * dy, yaw)):
            return False
    return True


def candidates(dock: Pose, distances: Sequence[float] = DISTANCES,
               laterals: Sequence[float] = LATERALS) -> list[Pose]:
    """Staging poses around `dock`, each facing the docked position."""
    x, y, yaw = dock
    c, s = math.cos(yaw), math.sin(yaw)
    out = []
    for lat in laterals:
        for d in distances:
            px = x - c * d - s * lat
            py = y - s * d + c * lat
            out.append((px, py, math.atan2(y - py, x - px)))
    return out


def choose_staging(grid: Optional[Grid], dock: Pose) -> Optional[Pose]:
    """First candidate where the robot fits and can drive into the dock.

    No grid yet: the nominal pose (Nav2 will find out if it is blocked).
    Every candidate blocked: None, meaning the dock is blocked; wait.
    """
    poses = candidates(dock)
    if grid is None:
        return poses[0]
    for pose in poses:
        if footprint_free(grid, pose) and lane_free(grid, pose, dock):
            return pose
    return None
