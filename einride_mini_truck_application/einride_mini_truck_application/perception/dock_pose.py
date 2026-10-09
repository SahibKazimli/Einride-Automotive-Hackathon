"""Where the robot must stop in front of a seen AprilTag. No ROS imports.

Competition dock geometry (einride_mini_truck_gazebo/dock_template/model.sdf.in):
the tag is on the back wall, and a docked robot faces it with its front about
160 mm from the wall (what the organisers told us on 2026-10-09; earlier notes
said 150-250 mm, yaw error < 6 deg). The robot's front is 0.126 m ahead of
base_footprint, so base_footprint stops FRONT_OFFSET + DOCK_GAP (0.246 m) out
from the tag along the tag's normal.

Nav2's docking server (SimpleNonChargingDock with zero detection offsets)
drives base_footprint to exactly the pose computed here.
"""

import math
from typing import Sequence

#: base_footprint to the robot's front-most point (front wheel edge), m.
FRONT_OFFSET = 0.126
#: Nominal gap between the robot's front and the tag/back wall, m. The
#: organisers said ~0.16; 0.12 aims a little closer for the human judges
#: (2026-10-09). Not below ~0.1: the collision monitor's stop zone reaches
#: 0.08 m past the front, and with 0.08 the wall sat on its edge and stopped
#: every final approach.
DOCK_GAP = 0.120

Vector = Sequence[float]
Matrix = Sequence[Sequence[float]]


def tag_normal(position: Vector, rotation: Matrix) -> tuple[float, float]:
    """Unit floor-plane direction the tag faces, pointing toward the viewer.

    The tag's z axis is perpendicular to its face, but detectors differ on
    whether it points out of or into the wall, so the sign is chosen to point
    back toward the robot (the origin of the frame `position` is in).
    """
    nx, ny = rotation[0][2], rotation[1][2]
    if nx * position[0] + ny * position[1] > 0.0:   # points away from the robot
        nx, ny = -nx, -ny
    norm = math.hypot(nx, ny)
    if norm < 1e-6:
        raise ValueError('tag is lying flat; no usable normal')
    return nx / norm, ny / norm


def docked_pose(position: Vector, rotation: Matrix,
                gap: float = DOCK_GAP) -> tuple[float, float, float]:
    """Return (x, y, yaw) of base_footprint when docked at this tag.

    `position`/`rotation` are the tag pose in the robot's frame (x forward,
    y left, z up); the result is in that same frame. The docked robot faces
    the tag.
    """
    nx, ny = tag_normal(position, rotation)
    standoff = gap + FRONT_OFFSET
    return (position[0] + standoff * nx,
            position[1] + standoff * ny,
            math.atan2(-ny, -nx))
