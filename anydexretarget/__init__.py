"""AnyDexRetarget - Hand Pose Retargeting Module.

Provides hand pose retargeting from MediaPipe format to dexterous robot hand joint angles.

Main classes:
- Retargeter: High-level unified interface (recommended)
- BaseOptimizer: Low-level optimizer access

Example:
    from anydexretarget import Retargeter

    retargeter = Retargeter.from_yaml("config/mediapipe/mediapipe_shadow_hand.yaml", hand_side="right")
    qpos = retargeter.retarget(raw_keypoints)  # (21, 3) -> (22,)
"""

from __future__ import annotations

from typing import TYPE_CHECKING, Any

if TYPE_CHECKING:
    from .mediapipe import apply_mediapipe_transformations
    from .optimizer import BaseOptimizer, LPFilter
    from .retarget import Retargeter

__all__ = [
    "Retargeter",
    "BaseOptimizer",
    "LPFilter",
    "apply_mediapipe_transformations",
]


def __getattr__(name: str) -> Any:
    """Load Pinocchio-dependent retargeting code only when it is requested.

    Lightweight consumers such as the ROKAE state tools only need
    ``anydexretarget.teleop``. Eagerly importing ``Retargeter`` here used to
    load Pinocchio and its C++ runtime even for a joint-state read.
    """
    if name == "Retargeter":
        from .retarget import Retargeter

        return Retargeter
    if name in {"BaseOptimizer", "LPFilter"}:
        from .optimizer import BaseOptimizer, LPFilter

        return {"BaseOptimizer": BaseOptimizer, "LPFilter": LPFilter}[name]
    if name == "apply_mediapipe_transformations":
        from .mediapipe import apply_mediapipe_transformations

        return apply_mediapipe_transformations
    raise AttributeError(f"module {__name__!r} has no attribute {name!r}")


def __dir__() -> list[str]:
    return sorted(set(globals()) | set(__all__))
