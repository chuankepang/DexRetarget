"""CLI parsing and task resolution. --help does not import devices or storage."""
from __future__ import annotations
import argparse
import sys
from pathlib import Path

def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Quest 3 -> Inspire hand + ROKAE relative Cartesian teleoperation",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
        allow_abbrev=False,
    )
    parser.add_argument(
        "--enable-hand",
        action="store_true",
        help="Enable Inspire output; if neither output flag is given, enable both",
    )
    parser.add_argument(
        "--enable-arm",
        action="store_true",
        help="Enable ROKAE output; if neither output flag is given, enable both",
    )
    parser.add_argument("--robot", choices=["rokae"], default="rokae")
    parser.add_argument("--hand", choices=["left", "right"], default="right")
    parser.add_argument(
        "--optimizer", choices=["adaptive", "vector"], default="adaptive"
    )
    parser.add_argument("--hand-config", default=None)
    parser.add_argument("--quest3-port", type=int, default=9000)
    parser.add_argument("--quest3-protocol", choices=["udp", "tcp"], default="udp")
    parser.add_argument('--config', type=Path, default=None,
                        help='Quest communication YAML: config/quest/udp.yaml or usb_tcp.yaml')
    parser.add_argument('--quest3-source', choices=['auto', 'direct', 'relay'], default='auto',
                        help='auto prefers a running local Quest service for USB; relay requires it')
    parser.add_argument('--quest3-relay-socket', type=Path, default=None,
                        help='Override the Quest communication service Unix socket path')
    parser.add_argument(
        "--quest3-usb", action=argparse.BooleanOptionalAction, default=False,
        help="Set up ADB reverse, open HTS and wait for tracking before robot initialization",
    )
    parser.add_argument("--inspire-port", default="/dev/ttyUSB0")
    parser.add_argument("--inspire-baudrate", type=int, default=115200)
    parser.add_argument("--inspire-hand-id", type=int, default=1)

    parser.add_argument("--arm-config", default="config/robots/rokae.yaml")
    parser.add_argument(
        "--arm-control",
        choices=["pinocchio", "cartesian", "impedance"],
        default=None,
        help=(
            "Low-level arm route. pinocchio is host IK plus jointPosition. "
            "cartesian is xCore cartesianPosition. impedance is xCore "
            "cartesianImpedance. Omit this to use the YAML controller."
        ),
    )
    parser.add_argument("--print-arm-config", action="store_true", help="Print resolved arm configuration without hardware connection or motion")
    parser.add_argument("--mock-rokae", action="store_true")
    parser.add_argument("--arm-start-enabled", action="store_true")
    parser.add_argument("--rokae-robot-ip", default=None)
    parser.add_argument("--rokae-local-ip", default=None)
    parser.add_argument(
        "--rokae-robot-type",
        choices=["xmate-6", "xmate-er-pro-7", "standard-6"],
        default=None,
    )
    parser.add_argument(
        "--rokae-sdk-path",
        type=Path,
        default=None,
        help="Override the directory containing the built _rokae_cpp module",
    )
    parser.add_argument(
        "--rokae-move-to-init",
        action="store_true",
        help="Execute configured initial_joint_move before realtime mode",
    )
    parser.add_argument(
        "--skip-init",
        action="store_true",
        help="Skip initial_joint_move even when enabled in YAML (debug only)",
    )
    parser.add_argument("--rokae-rt-network-tolerance", type=int, default=None)
    parser.add_argument(
        "--rokae-power-on",
        action=argparse.BooleanOptionalAction,
        default=True,
        help=(
            "Request motor power-on through xCoreSDK (default); use "
            "--no-rokae-power-on to require external power-on; ignored by mock"
        ),
    )
    parser.add_argument(
        "--robot-initial-pose",
        type=float,
        nargs=7,
        metavar=("X", "Y", "Z", "QX", "QY", "QZ", "QW"),
        default=None,
        help="Mock-only base_T_tcp pose; real mode always reads the robot",
    )
    parser.add_argument(
        "--world-base-pose",
        type=float,
        nargs=7,
        metavar=("X", "Y", "Z", "QX", "QY", "QZ", "QW"),
        default=None,
        help="T_world_base as XYZ + quaternion XYZW",
    )
    parser.add_argument(
        "--quest-to-robot-rotation",
        type=float,
        nargs=9,
        metavar=("R00", "R01", "R02", "R10", "R11", "R12", "R20", "R21", "R22"),
        default=None,
    )
    parser.add_argument("--arm-translation-scale", type=float, default=None)
    parser.add_argument("--arm-rotation-scale", type=float, default=None)
    parser.add_argument("--arm-max-translation-speed", type=float, default=None)
    parser.add_argument("--arm-max-angular-speed", type=float, default=None)
    parser.add_argument("--arm-translation-cutoff-hz", type=float, default=None)
    parser.add_argument("--arm-rotation-cutoff-hz", type=float, default=None)
    parser.add_argument("--arm-translation-deadband", type=float, default=None)
    parser.add_argument("--arm-rotation-deadband", type=float, default=None)
    parser.add_argument("--arm-max-translation-acceleration", type=float, default=None)
    parser.add_argument("--arm-max-angular-acceleration", type=float, default=None)
    parser.add_argument("--arm-max-translation-jerk", type=float, default=None)
    parser.add_argument("--arm-max-angular-jerk", type=float, default=None)
    parser.add_argument(
        "--arm-max-target-translation-delta", type=float, default=None
    )
    parser.add_argument("--arm-max-target-rotation-delta", type=float, default=None)
    parser.add_argument("--arm-workspace-min", type=float, nargs=3, default=None)
    parser.add_argument("--arm-workspace-max", type=float, nargs=3, default=None)
    parser.add_argument("--arm-tracking-timeout", type=float, default=None)
    parser.add_argument("--arm-stop-timeout", type=float, default=None)
    parser.add_argument("--quest-reference-timeout", type=float, default=15.0)
    parser.add_argument("--arm-print-hz", type=float, default=10.0)
    parser.add_argument("--arm-record", type=Path, default=None, metavar="JSONL")
    parser.add_argument("--task-config", type=Path, help="Task YAML: instruction, teleop, collection reference and home")
    parser.add_argument("--capture", action=argparse.BooleanOptionalAction, default=None, help="Enable explicit multi-episode synchronized dataset capture")
    parser.add_argument("--task", default="", help="Language instruction overriding the task profile")
    parser.add_argument("--collection-config", type=Path, default=None)
    return parser


def main() -> None:
    parser = build_parser()
    args = parser.parse_args()
    from .task import apply_task
    explicit = {parser._option_string_actions[token.split("=", 1)[0]].dest
                for token in sys.argv[1:] if token.split("=", 1)[0] in parser._option_string_actions}
    try:
        apply_task(args, explicit)
    except (ValueError, KeyError, TypeError, OSError) as exc:
        parser.error(str(exc))
    if args.arm_print_hz < 0.0:
        parser.error("--arm-print-hz cannot be negative")
    if args.rokae_rt_network_tolerance is not None and not (
        0 <= args.rokae_rt_network_tolerance <= 100
    ):
        parser.error("--rokae-rt-network-tolerance must be in [0, 100]")
    if args.print_arm_config:
        import json
        from rokae.control.configuration import describe_arm_config
        try:
            print(json.dumps(describe_arm_config(args), ensure_ascii=False, indent=2, allow_nan=False))
        except (ValueError, KeyError, TypeError, OSError) as exc:
            parser.error(str(exc))
        return
    from .runtime import run_teleop
    run_teleop(args)



def entrypoint():
    from dexretarget.bootstrap import _ensure_conda_cpp_runtime
    _ensure_conda_cpp_runtime()
    main()
