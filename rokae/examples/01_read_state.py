#!/usr/bin/env python3
from __future__ import annotations

import argparse

from common import add_connection_args, build_driver, format_pose, format_vector, load_config


def main() -> None:
    parser = argparse.ArgumentParser(description="Read ROKAE state without motion")
    add_connection_args(parser)
    args = parser.parse_args()
    driver = build_driver(args, load_config(args.config))
    try:
        driver.connect()
        state = driver.get_state()
        print("controller world_T_base:\n" + format_pose(driver.get_controller_base_frame()))
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
    finally:
        driver.disconnect()


if __name__ == "__main__":
    main()
