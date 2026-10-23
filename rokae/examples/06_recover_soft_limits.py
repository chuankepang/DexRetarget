#!/usr/bin/env python3
"""Inspect/recover already-disabled controller soft limits, independently of init."""
from __future__ import annotations

import argparse
from typing import Sequence

import numpy as np

from common import add_connection_args, build_driver, format_vector, load_config, print_controller_logs


def main(argv: Sequence[str] | None = None) -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    add_connection_args(parser)
    parser.add_argument("--execute", action="store_true", help="Enable the explicit maintenance confirmation")
    parser.add_argument("--clearance-deg", type=float, default=0.5,
                        help="Target clearance inside original limit, 0.5..10 degrees (default: 0.5)")
    parser.add_argument("--manual-power-source", choices=("operator", "sdk"), default="operator",
                        help="operator: wait for physical manual enable (default); sdk: only without external enable/pendant")
    parser.add_argument("--jog-rate", type=float, default=None,
                        help="Recovery Jog rate fraction, 0.01..0.10; defaults to config recovery_jog_rate or 0.05")
    args = parser.parse_args(argv)
    config = load_config(args.config)
    jog_rate = (float(config["robot"]["initial_joint_move"].get("recovery_jog_rate", 0.05))
                if args.jog_rate is None else args.jog_rate)
    if not 0.01 <= jog_rate <= 0.10:
        parser.error("--jog-rate / recovery_jog_rate must be finite and in 0.01..0.10")
    driver = build_driver(args, config)
    try:
        driver.connect()
        state = driver.get_state()
        current = np.asarray(driver.get_joint_positions(), dtype=np.float64)
        enabled, limits = driver.get_soft_limits()
        print(f"controller: {state['controller_version']}; robot: {state['robot_type']}")
        print(f"power: {state['power_state']}; mode: {state['operate_mode']}; operation: {state['operation_state']}")
        print(f"controller soft limits enabled: {enabled}")
        print("current joints rad:", format_vector(current))
        for index, ((lower, upper), value) in enumerate(zip(limits, current), 1):
            print(f"  J{index}: [{lower:+.8f}, {upper:+.8f}], q={value:+.8f} rad")
        if enabled:
            raise SystemExit("Soft limits are already enabled; no maintenance action required by this tool.")
        if state["operation_state"] != "idle":
            raise RuntimeError("Controller must be idle; this tool will not interrupt another controller.")
        # Use exactly the same native validator as execution; no duplicate planner.
        from rokae.python import _rokae_cpp

        if not hasattr(_rokae_cpp, "plan_disabled_soft_limit_recovery"):
            raise RuntimeError("Rebuild the active Python bridge: bash rokae/build.sh")
        target = np.asarray(_rokae_cpp.plan_disabled_soft_limit_recovery(
            current.tolist(), limits.tolist(), args.clearance_deg))
        changed = np.flatnonzero(target != current)
        if changed.size:
            axis = int(changed[0])
            print(f"Plan: manual single-axis Jog J{axis+1}, delta={np.degrees(target[axis]-current[axis]):+.6f} deg, rate={100*jog_rate:g}%.")
            print("target joints rad:", format_vector(target))
            print(f"Only one axis may be outside, by <=1 deg; target is {args.clearance_deg:g} deg inside the original limit.")
        else:
            print("Plan: all joints are inside; enable existing soft limits without Jog.")
        trigger_deg = float(np.degrees(config["robot"]["initial_joint_move"].get("recovery_trigger_margin", 0.15)))
        planned_margin_deg = float(np.degrees(np.min(np.minimum(target - limits[:, 0], limits[:, 1] - target))))
        if planned_margin_deg <= trigger_deg + 0.05:
            print(f"Note: init guard requires >{trigger_deg:.3f} deg clearance; this recovery alone may not permit init.")
        print("Finish: stop Jog, power off, manual mode, enable and verify unchanged original soft limits.")
        print(f"Manual power source: {args.manual_power_source}.")
        if not args.execute:
            print("DRY RUN: no power/mode/limit/motion command sent. Re-run with --execute to confirm.")
            return
        print("Check the cell. This does not clear alarms or override safety interlocks.")
        if args.manual_power_source == "operator":
            print("Keep the physical manual enable released until WAITING FOR MANUAL ENABLE appears.")
            print("Then hold the middle position through Jog, and release at the Jog stopped prompt.")
        else:
            print("SDK manual power-on is supported only with no external enable switch or teach pendant.")
        answer = input("Type RECOVER to execute this maintenance plan: ").strip()
        if answer != "RECOVER":
            raise SystemExit("Recovery cancelled; no motion sent.")
        try:
            driver.recover_disabled_soft_limits(
                current, limits, clearance_degrees=args.clearance_deg,
                request_power_on=args.manual_power_source == "sdk",
                jog_rate=jog_rate,
            )
        except Exception:
            try:
                print_controller_logs(driver)
            except Exception as log_error:
                print(f"Controller log query failed: {log_error}")
            raise
        print("Recovery completed. No init or teleoperation was started; inspect again before normal use.")
    finally:
        driver.disconnect()


if __name__ == "__main__":
    main()
