"""Print the resolved ROKAE controller without connecting to the robot.

    python rokae/control/examples/print_controller.py
    python rokae/control/examples/print_controller.py --controller impedance --task cup
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[3]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from rokae.control.controllers import resolve_controller
from rokae.examples.common import DEFAULT_CONFIG, load_config


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    parser.add_argument("--task", choices=["cup", "box"], default=None)
    parser.add_argument(
        "--controller",
        choices=["position", "impedance", "pinocchio", "cartesian"],
        default=None,
    )
    args = parser.parse_args()
    config = load_config(args.config)
    spec = resolve_controller(config, cli=args.controller, task=args.task)
    print(f"kind: {spec.kind}")
    print(f"route: {spec.route}")
    print(f"position backend: {spec.position_backend}")
    print(f"impedance backend: {spec.impedance_backend}")
    print("stiffness N/m, Nm/rad:", spec.stiffness.tolist())
    print("desired wrench N, Nm:", spec.desired_wrench.tolist())
    print(
        "equilibrium limit: "
        f"{spec.max_equilibrium_translation:.3f} m, "
        f"{spec.max_equilibrium_rotation:.3f} rad"
    )
    print("SDK damping: not exposed by xCoreSDK 0.3.4; controller-internal")
    print("mock damping is offline only:", spec.mock_damping.tolist())


if __name__ == "__main__":
    main()
