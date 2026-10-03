"""Where to stand before docking, chosen from the live costmap. No ROS imports.

Nav2's docking server drives to one fixed staging pose straight in front of
the dock, then servos onto the tag. If anything sits on that pose (a bucket,
another robot) it can never get there and docking fails forever. Instead we try
a set of candidate poses around the dock, all facing it, and pick one where
Nav2 can actually park the robot AND the lane from there into the dock is clear.
Nav2 then routes there around obstacles, and docking runs without its own staging.

Grid values follow nav_msgs/OccupancyGrid as published by Nav2's costmap:
100 = lethal (an obstacle cell), 99 = inscribed, lower = inflation or free,
-1 = unknown (needs track_unknown_space: true in the global costmap).

"The outline fits" is not enough (2026-10-03: a pose a few cm beside a bucket
fitted, Nav2 could not get there, and its recoveries spun the robot away from
the tag). On arrival the robot turns in place to the goal heading, and the
collision monitor stops every motion while lidar points are inside its stop
zone. So nothing may be within reach of the stop zone in any heading
(`Limits.turn_radius`), and the stop zone must stay clear along the lane into
the dock. Both come from the config Nav2 itself runs with (`load_limits`).

A 2D lidar only sees the near face of an obstacle; the space behind it stays
unknown. Unknown cells close to a seen obstacle may be its hidden back, so they
count as obstacle (`Limits.hidden_depth`), as do inscribed cells, and ground
the lidar has seen free is preferred over unseen ground. Nav2's own
PyCostmap2D/FootprintCollisionChecker do not fit here: they need ROS messages,
and on a published OccupancyGrid (lethal = 100) they never report LETHAL (254).
"""

import math
from dataclasses import dataclass
from typing import Optional, Sequence

import yaml

Pose = tuple[float, float, float]   # x, y, yaw
Point = tuple[float, float]

LETHAL = 100
INSCRIBED = 99
STEP = 0.1   # between candidate distances from the dock
LATERALS = (0.0, 0.15, -0.15, 0.3, -0.3, 0.45, -0.45, 0.6, -0.6)

# How unknown cells are treated in a check.
BLOCK = 'block'     # all unknown blocks: only ground the lidar has seen free
HIDDEN = 'hidden'   # unknown next to an obstacle blocks (its unseen back)
FREE = 'free'       # unknown never blocks


@dataclass(frozen=True)
class Limits:
    """Robot outline and docking geometry, as configured for Nav2."""
    footprint: tuple[Point, ...]   # costmap footprint (base_footprint frame)
    stop_zone: tuple[Point, ...]   # collision monitor polygons that stop the robot
    staging_distance: float        # docking server staging_x_offset, metres in front
    dock_clearance: float          # docking server ignores collisions this close to the dock

    @property
    def turn_radius(self) -> float:
        """How far the stop zone and outline reach while turning in place."""
        return max(math.hypot(x, y) for x, y in self.footprint + self.stop_zone)

    @property
    def hidden_depth(self) -> float:
        """How far an obstacle may reach behind its seen face: the other
        robots are our size, the bucket is smaller."""
        return 2 * max(math.hypot(x, y) for x, y in self.footprint)

    @property
    def box(self) -> tuple[float, float, float]:
        """Front, back and half width of the stop zone (which holds the outline)."""
        points = self.footprint + self.stop_zone
        return (max(x for x, _ in points), -min(x for x, _ in points),
                max(abs(y) for _, y in points))


def _polygon(text: str) -> tuple[Point, ...]:
    """'[[x, y], ...]' as Nav2 takes footprints and polygons."""
    return tuple((float(x), float(y)) for x, y in yaml.safe_load(text))


def load_limits(nav2_path: str, collision_monitor_path: str) -> Limits:
    """Limits from config/navigation/nav2.yaml and config/safety/collision_monitor.yaml."""
    with open(nav2_path) as f:
        nav2 = yaml.safe_load(f)
    with open(collision_monitor_path) as f:
        monitor = yaml.safe_load(f)['collision_monitor']['ros__parameters']
    costmap = nav2['global_costmap']['global_costmap']['ros__parameters']
    docking = nav2['docking_server']['ros__parameters']
    dock = docking[docking['dock_plugins'][0]]
    stop = [monitor[name] for name in monitor['polygons']
            if monitor[name].get('action_type') == 'stop'
            and monitor[name].get('enabled', True) and 'points' in monitor[name]]
    return Limits(footprint=_polygon(costmap['footprint']),
                  stop_zone=sum((_polygon(p['points']) for p in stop), ()),
                  staging_distance=abs(float(dock['staging_x_offset'])),
                  dock_clearance=float(docking['controller']['dock_collision_threshold']))


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

    def index(self, x: float, y: float) -> tuple[int, int]:
        return (math.floor((x - self.origin_x) / self.resolution),
                math.floor((y - self.origin_y) / self.resolution))

    def value(self, ix: int, iy: int) -> int:
        if not (0 <= ix < self.width and 0 <= iy < self.height):
            return -1   # outside the rolling window: unknown
        return self.data[iy * self.width + ix]

    def cost(self, x: float, y: float) -> int:
        return self.value(*self.index(x, y))


class Cells:
    """Answers "is anything in the way" around one dock of a grid."""

    def __init__(self, grid: Grid, around: Point, radius: float, hidden_depth: float) -> None:
        """Obstacles within `radius` of `around` are considered for hidden backs."""
        self.grid = grid
        res = grid.resolution
        cx, cy = grid.index(*around)
        n = math.ceil(radius / res)
        k = math.ceil(hidden_depth / res)
        reach = [(i, j) for i in range(-k, k + 1) for j in range(-k, k + 1)
                 if math.hypot(i, j) * res <= hidden_depth]
        self.near_obstacle = {(ix + i, iy + j)
                              for ix in range(cx - n, cx + n + 1)
                              for iy in range(cy - n, cy + n + 1)
                              if grid.value(ix, iy) >= LETHAL
                              for i, j in reach}

    def blocked(self, ix: int, iy: int, unknown: str) -> bool:
        value = self.grid.value(ix, iy)
        if value >= LETHAL:
            return True
        if unknown == FREE:
            return False
        # Nav2 inflates into unknown cells near an obstacle, so a hidden back
        # within the inscribed radius is published as INSCRIBED, not unknown.
        if value == INSCRIBED:
            return True
        if value < 0:
            return unknown == BLOCK or (ix, iy) in self.near_obstacle
        return False

    def clearance(self, x: float, y: float, limit: float, unknown: str = HIDDEN) -> float:
        """Distance from (x, y) to the nearest blocked cell's edge, at most `limit`."""
        g = self.grid
        res = g.resolution
        (x0, y0), (x1, y1) = g.index(x - limit, y - limit), g.index(x + limit, y + limit)
        best = limit
        for ix in range(x0, x1 + 1):
            left = g.origin_x + ix * res
            dx = max(left - x, 0.0, x - left - res)
            if dx >= best:
                continue
            for iy in range(y0, y1 + 1):
                bottom = g.origin_y + iy * res
                d = math.hypot(dx, max(bottom - y, 0.0, y - bottom - res))
                if d < best and self.blocked(ix, iy, unknown):
                    best = d
        return best

    def box_clear(self, pose: Pose, box: tuple[float, float, float], unknown: str) -> bool:
        """True if no blocked cell overlaps the robot-frame box (front, back, half width)."""
        g = self.grid
        res = g.resolution
        x, y, yaw = pose
        front, back, half = box
        c, s = math.cos(yaw), math.sin(yaw)
        h = res / 2 * (abs(c) + abs(s))   # a cell's half extent along the robot axes
        r = math.hypot(max(front, back), half)
        (x0, y0), (x1, y1) = g.index(x - r, y - r), g.index(x + r, y + r)
        for ix in range(x0, x1 + 1):
            for iy in range(y0, y1 + 1):
                dx = g.origin_x + (ix + 0.5) * res - x
                dy = g.origin_y + (iy + 0.5) * res - y
                lx, ly = c * dx + s * dy, -s * dx + c * dy
                if (lx - h < front and lx + h > -back and abs(ly) - h < half
                        and self.blocked(ix, iy, unknown)):
                    return False
        return True

    def lane_clear(self, start: Pose, dock: Pose, limits: Limits, unknown: str) -> bool:
        """True if the stop zone stays clear driving straight from `start` towards
        the dock, except the last `limits.dock_clearance` (the dock's own walls)."""
        dx, dy = dock[0] - start[0], dock[1] - start[1]
        length = math.hypot(dx, dy)
        if length <= limits.dock_clearance:
            return True
        yaw = math.atan2(dy, dx)
        step = self.grid.resolution
        n = int((length - limits.dock_clearance) / step)
        return all(self.box_clear((start[0] + k * step / length * dx,
                                   start[1] + k * step / length * dy, yaw), limits.box, unknown)
                   for k in range(n + 1))


def candidates(dock: Pose, limits: Limits, laterals: Sequence[float] = LATERALS) -> list[Pose]:
    """Staging poses around `dock`, each facing the docked position.

    Straight in front first; at each lateral offset, the docking server's own
    staging distance first, then closer in steps down to `dock_clearance`.
    """
    near = min(limits.dock_clearance, limits.staging_distance)
    steps = int(round((limits.staging_distance - near) / STEP))
    distances = [limits.staging_distance - k * STEP for k in range(steps + 1)]
    x, y, yaw = dock
    c, s = math.cos(yaw), math.sin(yaw)
    out = []
    for lat in laterals:
        for d in distances:
            px = x - c * d - s * lat
            py = y - s * d + c * lat
            out.append((px, py, math.atan2(y - py, x - px)))
    return out


def cells_for(grid: Grid, dock: Pose, poses: Sequence[Pose], limits: Limits) -> Cells:
    reach = max((math.dist(p[:2], dock[:2]) for p in poses), default=0.0)
    radius = reach + limits.turn_radius + limits.hidden_depth + grid.resolution
    return Cells(grid, dock[:2], radius, limits.hidden_depth)


def choose_staging(grid: Optional[Grid], dock: Pose, limits: Limits,
                   skip: Sequence[Pose] = ()) -> Optional[Pose]:
    """A candidate where Nav2 can park and turn, with a clear lane into the dock.

    `skip`: poses where docking already failed (e.g. tag not seen from there),
    so a retry looks from somewhere else instead of repeating the same failure.
    No grid yet: the first candidate (Nav2 will find out if it is blocked).
    Every candidate blocked or skipped: None; the caller waits or clears `skip`.
    """
    poses = [p for p in candidates(dock, limits)
             if not any(math.dist(p[:2], s[:2]) < 0.01 for s in skip)]
    if grid is None:
        return poses[0] if poses else None
    cells = cells_for(grid, dock, poses, limits)
    turn = limits.turn_radius
    # Most cautious first. Near the dock much is unseen, so the lane never
    # demands seen ground; in the last round it ignores hidden backs as well.
    for at_pose, on_lane in ((BLOCK, HIDDEN), (HIDDEN, HIDDEN), (HIDDEN, FREE)):
        for pose in poses:
            if (cells.clearance(pose[0], pose[1], turn, at_pose) >= turn
                    and cells.lane_clear(pose, dock, limits, on_lane)):
                return pose
    # No room to turn anywhere: rather than wait, the roomiest pose where the
    # robot at least stands outside the stop zone, facing the dock.
    fits = [p for p in poses if cells.box_clear(p, limits.box, HIDDEN)
            and cells.lane_clear(p, dock, limits, FREE)]
    return max(fits, key=lambda p: cells.clearance(p[0], p[1], turn), default=None)


def staging_clearance(grid: Grid, pose: Pose, limits: Limits) -> float:
    """Room around `pose` (to obstacle or hidden cells), capped at twice the turn radius."""
    limit = 2 * limits.turn_radius
    cells = Cells(grid, pose[:2], limit + limits.hidden_depth + grid.resolution,
                  limits.hidden_depth)
    return cells.clearance(pose[0], pose[1], limit)
