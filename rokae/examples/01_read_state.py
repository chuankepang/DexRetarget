#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
from datetime import datetime, timezone
from pathlib import Path

from common import add_connection_args, build_driver, format_pose, format_vector, load_config


def main() -> None:
    parser = argparse.ArgumentParser(description="Read ROKAE state without motion")
    add_connection_args(parser)
    parser.add_argument("--controller-logs", action="store_true", help="Also read up to 10 recent controller logs")
    parser.add_argument("--faults-only", action="store_true", help="Read latest warning/error logs (implies --controller-logs)")
    parser.add_argument("--json-output", type=Path, help="Save read-only snapshot to a NEW JSON file; never overwrite")
    args = parser.parse_args()
    if args.json_output and args.json_output.exists():
        parser.error(f"output already exists: {args.json_output}; choose a new filename")
    config = load_config(args.config)
    driver = build_driver(args, config)
    try:
        driver.connect()
        state = driver.get_state()
        base_frame = driver.get_controller_base_frame()
        print("controller world_T_base:\n" + format_pose(base_frame))
        print(f"connected:          {state['connected']}")
        print(f"SDK:                {state['sdk_version']}")
        print(f"controller:         {state['controller_version']}")
        print(f"robot:              {state['robot_type']} ({state['joint_count']} joints)")
        print(f"power:              {state['power_state']}")
        print(f"operate mode:       {state['operate_mode']}")
        print(f"operation state:    {state['operation_state']}")
        print(f"joint position rad: {format_vector(state['joint_position'])}")
        print(f"joint velocity:     {format_vector(state['joint_velocity'])}")
        print("base_T_tcp:")
        print(format_pose(state["tcp_pose"]))
        logs = []
        if args.controller_logs or args.faults_only or args.json_output:
            logs = driver.get_controller_logs(10, faults_only=args.faults_only)
            print("Recent logs (maximum 10; not a full archive):")
            for row in logs:
                print(f"  [{row['timestamp']}] {row['id']}: {row['content']}")
                if row.get("repair"):
                    print(f"    controller guidance: {row['repair']}")
        if args.json_output:
            enabled, limits = driver.get_soft_limits()
            report = {
                "captured_at_utc": datetime.now(timezone.utc).isoformat(),
                "read_only": True,
                "scope": "Current state and latest logs; not a motion trace or impedance readiness approval",
                "state": state,
                "world_T_base": base_frame,
                "soft_limits_enabled": enabled,
                "joint_limits_rad": limits,
                "configured_payload_not_controller_readback": config["robot"].get("payload"),
                "log_filter": "warning/error" if args.faults_only else "all",
                "logs": logs,
            }
            text = json.dumps(report, ensure_ascii=False, indent=2,
                              default=lambda value: value.tolist(), allow_nan=False)
            args.json_output.parent.mkdir(parents=True, exist_ok=True)
            with args.json_output.open("x", encoding="utf-8") as stream:
                stream.write(text + "\n")
            print(f"Read-only snapshot saved: {args.json_output}")
    finally:
        driver.disconnect()


if __name__ == "__main__":
    main()
