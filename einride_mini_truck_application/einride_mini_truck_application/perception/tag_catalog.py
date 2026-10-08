"""AprilTag survey maths: sightings -> docked poses on the map. No ROS imports.

Each sighting gives the docked pose in the robot's frame (dock_pose.py, the
same computation docking uses live) and the robot's pose on the map at that
moment. Composing the two puts the docked pose on the map. Many sightings per
tag are averaged, outliers dropped, and the result is written as a Nav2 dock
database, the same format as config/docks/*.yaml, so the docking server, the
mission and the camera gate read it as a layout.
"""

import math
import statistics
from typing import Optional, Sequence, TextIO

import yaml

Pose = tuple[float, float, float]   # x, y, yaw
DOCK_TYPE = 'competition_dock'
NAMES = 'ABCDEFGH'   # Saga's dock letters, tag ids 0-7


def wrap(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def compose(origin: Pose, local: Pose) -> Pose:
    """`local`, given in the frame whose pose is `origin`, in origin's parent frame."""
    ox, oy, oyaw = origin
    x, y, yaw = local
    c, s = math.cos(oyaw), math.sin(oyaw)
    return (ox + c * x - s * y, oy + s * x + c * y, wrap(oyaw + yaw))


def mean_yaw(yaws: Sequence[float]) -> float:
    """Average of angles, correct across +-pi."""
    return math.atan2(sum(math.sin(a) for a in yaws), sum(math.cos(a) for a in yaws))


def summarize(samples: Sequence[Pose], min_observations: int = 5,
              max_position_error: float = 0.30,
              max_yaw_error: float = 0.50) -> Optional[tuple[Pose, int]]:
    """Average pose and number of samples used, or None if too few agree.

    Samples further than the limits from the median position / mean yaw are
    dropped as outliers (a bad detection, a localization jump).
    """
    if len(samples) < min_observations:
        return None
    cx = statistics.median(s[0] for s in samples)
    cy = statistics.median(s[1] for s in samples)
    cyaw = mean_yaw([s[2] for s in samples])
    kept = [s for s in samples
            if math.hypot(s[0] - cx, s[1] - cy) <= max_position_error
            and abs(wrap(s[2] - cyaw)) <= max_yaw_error]
    if len(kept) < min_observations:
        return None
    x = sum(s[0] for s in kept) / len(kept)
    y = sum(s[1] for s in kept) / len(kept)
    return (x, y, mean_yaw([s[2] for s in kept])), len(kept)


def dock_database(docks: dict[int, Pose], frame: str,
                  tag_poses: Optional[dict[int, Pose]] = None,
                  counts: Optional[dict[int, int]] = None,
                  map_identity: Optional[str] = None) -> dict:
    """Nav2 dock layout plus physical AprilTag poses for visualization."""
    database = {
        'frame': frame,
        'docks': {
            f'dock_{tag}': {'type': DOCK_TYPE, 'frame': frame,
                            'pose': [round(v, 4) for v in pose], 'id': str(tag)}
            for tag, pose in sorted(docks.items())},
    }
    if tag_poses:
        database['tags'] = {
            f'tag_{tag}': {
                'id': tag,
                'name': NAMES[tag],
                'pose': [round(v, 4) for v in pose],
                'observations': (counts or {}).get(tag, 0),
            }
            for tag, pose in sorted(tag_poses.items())}
    if map_identity:
        database['map_identity'] = map_identity
    return database


def write_dock_database(out: TextIO, docks: dict[int, Pose], counts: dict[int, int],
                        frame: str, tag_poses: Optional[dict[int, Pose]] = None,
                        map_identity: Optional[str] = None) -> None:
    """Write dock targets and the actual observed tag poses."""
    out.write('# Dock database from the AprilTag survey (tag_survey). `pose` is the\n'
              '# docked base_footprint pose, 0.326 m in front of the tag, facing it.\n'
              '# `tags.*.pose` is now the physical AprilTag pose, not the docking pose.\n'
              '# Only valid with the SLAM map it was surveyed on (slam:=true).\n')
    for tag in sorted(docks):
        out.write(f'#   dock_{tag} ({NAMES[tag]}): {counts.get(tag, 0)} observations\n')
    yaml.safe_dump(dock_database(docks, frame, tag_poses, counts, map_identity),
                   out, sort_keys=False)
