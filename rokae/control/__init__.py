"""Host-side motion helpers for the ROKAE xMate ER Pro.

The cell frame is defined in ``cell``: from the robot looking at the table,
+X is forward, +Y is left, +Z is up. The base origin is 0.83 m above the
floor and the table top is 0.825 m above the floor.

``cartesian`` realtime control stays in the xCore callback. The other route is
Pinocchio differential IK on the host, then a 1 kHz joint stream.
"""

from .cell import (
    BASE_HEIGHT_M,
    FINGERTIP_CLEARANCE_M,
    FLANGE_TO_FINGERTIP_M,
    TABLE_HEIGHT_M,
    TABLE_Z_IN_BASE_M,
    TOOL_DOWN_ROTATION,
)
from .differential_ik import DifferentialIK
from .interpolate import sample_cartesian, sample_joint_quintic
from .model import XmateErPro7
from .smoothing import JointStream, JointStreamConfig

__all__ = [
    "BASE_HEIGHT_M",
    "DifferentialIK",
    "FINGERTIP_CLEARANCE_M",
    "FLANGE_TO_FINGERTIP_M",
    "JointStream",
    "JointStreamConfig",
    "TABLE_HEIGHT_M",
    "TABLE_Z_IN_BASE_M",
    "TOOL_DOWN_ROTATION",
    "XmateErPro7",
    "sample_cartesian",
    "sample_joint_quintic",
]
