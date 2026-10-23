"""Upright ER3 Pro cell frame. This is the geometric reference for arm motion.

World frame, standing at the robot and looking toward the table:

  +X forward, toward the table
  +Y to the left
  +Z up

The upright base uses those same axis directions. Rokae's manual defines base
+Z as the upward normal of the mounting face and base -X as the cable-exit
direction; with the cable leaving away from the table, base +X points forward
and the base rotation relative to this world frame is identity.

Heights are measured from the floor:

  robot base origin: 0.83 m
  table top:         0.825 m

In the base frame the table is therefore 5 mm below the origin.
"""

from __future__ import annotations

import numpy as np

BASE_HEIGHT_M = 0.83
TABLE_HEIGHT_M = 0.825
# base_z = world_z - BASE_HEIGHT_M, with both origins sharing X/Y directions.
TABLE_Z_IN_BASE_M = TABLE_HEIGHT_M - BASE_HEIGHT_M

# Open Inspire fingertips, measured from the ER3 Pro flange along flange +Z.
FLANGE_TO_FINGERTIP_M = 0.18
# Minimum distance from those fingertips to the table top.
FINGERTIP_CLEARANCE_M = 0.10

# Flange orientation for table work with J7 at 0.
# Tool +Z points down. Tool +X points back toward the robot (base -X),
# which is the J7=0 wrist roll of the elbow-up home. Columns are the flange
# axes expressed in the base frame.
TOOL_DOWN_ROTATION = np.array(
    [
        [-1.0, 0.0, 0.0],
        [0.0, 1.0, 0.0],
        [0.0, 0.0, -1.0],
    ],
    dtype=np.float64,
)


def fingertip_position(flange_xyz: np.ndarray) -> np.ndarray:
    """Fingertip point for a flange pose whose +Z axis points straight down."""
    point = np.asarray(flange_xyz, dtype=np.float64)
    return point + TOOL_DOWN_ROTATION[:, 2] * FLANGE_TO_FINGERTIP_M


def table_clearance(flange_xyz: np.ndarray) -> float:
    """Vertical gap from the open fingertips to the table, in metres."""
    return float(fingertip_position(flange_xyz)[2] - TABLE_Z_IN_BASE_M)
