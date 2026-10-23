"""Exclusive home motion with borrowed drivers; no data/camera lifecycle ownership."""
import numpy as np


def run_home(home_config, driver, execution, gate, *, cancelled, stop_event, recenter_event, hand_output=None, power_on=True, on_event=None):
    if on_event:
        on_event("home_started")
    with gate.lock:
        if cancelled.is_set():
            raise RuntimeError("Home cancelled")
        execution.stop()
    for target in home_config["joint_waypoints"] + [home_config["arm_joints"]]:
        if cancelled.is_set():
            raise RuntimeError("Home cancelled")
        driver.move_to_init(
            target, joint_speed=home_config["joint_speed"],
            timeout=home_config["timeout_s"],
            max_segment_delta=home_config["max_segment_delta"],
            power_on=power_on, cancel_event=cancelled,
        )
    if cancelled.is_set():
        raise RuntimeError("Home cancelled")
    measured = driver.get_joint_positions()
    if measured is None or not np.all(np.isfinite(measured)) or np.max(np.abs(np.asarray(measured) - home_config["arm_joints"])) > home_config["joint_tolerance_rad"]:
        raise RuntimeError("Home joint feedback is outside configured tolerance")
    with gate.lock:
        if cancelled.is_set() or stop_event.is_set():
            raise RuntimeError("Home cancelled")
        if home_config.get("hand_angles") is not None and hand_output:
            hand_output.send_angles(home_config["hand_angles"])
        # Restart with the actual post-home joints, not the startup IK state.
        execution.initial_joints = np.asarray(measured).copy()
        execution.start(power_on=power_on)
        driver.hold()
        recenter_event.set()
    if on_event:
        on_event("home_completed")
