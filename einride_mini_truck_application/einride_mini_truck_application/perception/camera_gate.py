"""When to feed camera images to the tag detector. No ROS imports.

apriltag_ros searches every image it gets for tags, which costs more than a
full CPU core at 1280x720 (measured 2026-10-02: ~135 %). Tags only matter in
the last metre of docking, so images are passed on only near the target dock;
the rest of the time the detector gets nothing and idles.
"""

import math
from typing import Optional

import yaml

Point = tuple[float, float]


def load_dock_positions(path: str) -> dict[int, Point]:
    """Tag id -> (x, y) of its docked pose, from a Nav2 dock database file."""
    with open(path) as f:
        data = yaml.safe_load(f) or {}
    return {int(dock['id']): (float(dock['pose'][0]), float(dock['pose'][1]))
            for dock in (data.get('docks') or {}).values()}


class CameraGate:
    """Open within `on_distance` of the target dock, closed beyond `off_distance`.

    The gap between the two keeps it from flickering at the boundary.
    """

    def __init__(self, on_distance: float = 1.6, off_distance: float = 1.9) -> None:
        self.on_distance = on_distance
        self.off_distance = off_distance
        self.open = False

    def update(self, robot: Optional[Point], dock: Optional[Point]) -> bool:
        """`robot`/`dock`: positions in one frame, None if unknown. Returns open."""
        if dock is None:
            self.open = False           # no target dock: nothing to look for
        elif robot is None:
            self.open = True            # lost our position: look rather than miss the tag
        else:
            distance = math.dist(robot, dock)
            if distance < self.on_distance:
                self.open = True
            elif distance > self.off_distance:
                self.open = False
        return self.open
