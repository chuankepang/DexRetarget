"""Position control: the existing Cartesian or joint realtime route, plus NRT MoveL."""

from __future__ import annotations

import numpy as np
import time

from rokae.control.command import EECommand, PathSegment
from rokae.control.controllers.base import ControllerBase
from rokae.control.controllers.spec import ControllerSpec


class PositionController(ControllerBase):
    """target_pose is a position reference, not a compliant equilibrium."""

    def __init__(
        self,
        spec: ControllerSpec,
        driver,
        *,
        kinematics=None,
        initial_joints: np.ndarray | None = None,
        mock: bool = False,
    ) -> None:
        super().__init__(spec, driver)
        if spec.kind != "position":
            raise ValueError("PositionController requires kind=position")
        self.kinematics = kinematics
        self.initial_joints = None if initial_joints is None else np.asarray(initial_joints, dtype=np.float64)
        self.mock = bool(mock)
        self.ik = None
        self._last_timestamp: float | None = None

    @property
    def backend(self) -> str:
        return self.spec.position_backend

    def start(self, *, power_on: bool = True) -> None:
        if self.state == "running":
            return
        if self.backend == "pinocchio":
            if self.kinematics is None:
                raise ValueError("pinocchio position control requires a kinematic model")
            if self.mock:
                if self.initial_joints is None:
                    raise ValueError("pinocchio mock requires initial joints")
                joints = self.initial_joints
            else:
                joints = self.driver.get_joint_positions()
                if joints is None:
                    raise RuntimeError("ROKAE did not return joints for Pinocchio IK")
                translation_error, rotation_error = self.kinematics.require_controller_match(
                    joints, self.driver.get_tcp_pose()
                )
                print(
                    "Pinocchio FK matches controller TCP: "
                    f"{translation_error * 1000.0:.2f} mm / "
                    f"{np.rad2deg(rotation_error):.2f} deg"
                )
            from rokae.control.differential_ik import DifferentialIK

            self.ik = DifferentialIK(self.kinematics, np.asarray(joints, dtype=np.float64))
        self._last_timestamp = None
        self.driver.start(
            power_on=power_on,
            control_mode=self.backend,
            kinematics=self.kinematics,
        )
        if self.ik is not None:
            # Native startup may take time and narrows the effective limits.
            # Seed from its actual initial command, not a preflight-era state.
            try:
                self.ik.reset(self.driver.get_command_joints())
            except Exception:
                self.driver.stop()
                raise
        self.state = "running"

    def set_target(self, command: EECommand) -> bool:
        with self._lock:
            self._require_running()
            dt = 0.02
            if self._last_timestamp is not None:
                dt = max(command.timestamp - self._last_timestamp, 1e-3)
            self._last_timestamp = command.timestamp
            if self.backend != "pinocchio":
                accepted = bool(
                    self.driver.set_target_pose(
                        command.pose,
                        source_timestamp=command.timestamp,
                        sequence=command.sequence,
                    )
                )
                if accepted:
                    self.last_accepted_pose = command.pose.copy()
                return accepted
            if self.ik is None:
                raise RuntimeError("Pinocchio teleop has no IK solver")
            started = time.monotonic()
            joints = self.ik.step(command.pose, dt)
            if self.timing is not None:
                self.timing.record('arm_ik', time.monotonic() - started)
            accepted = bool(
                self.driver.set_target_joints(
                    joints,
                    source_timestamp=command.timestamp,
                    sequence=command.sequence,
                )
            )
            if not accepted:
                self.ik.reset(self.driver.get_command_joints())
            else:
                self.last_accepted_pose = command.pose.copy()
            return accepted

    def command_pose(self) -> np.ndarray:
        if self.backend == "pinocchio" and self.kinematics is not None:
            return np.asarray(self.kinematics.fk(self.driver.get_command_joints()), dtype=np.float64)
        return np.asarray(self.driver.get_command_pose(), dtype=np.float64)

    def reset_tracking(self) -> None:
        with self._lock:
            # A clutch/recovery starts a new target-time sequence. Do not turn
            # the preceding pause into an enlarged first IK integration step.
            self._last_timestamp = None
            if self.ik is not None:
                self.ik.reset(self.driver.get_command_joints())

    def follow(self, segments: list[PathSegment], *, power_on: bool = True) -> None:
        """NRT MoveL. Init/reset stays on MoveAbsJ and is not done here."""
        if self.state == "running":
            raise RuntimeError("stop realtime control before a position MoveL path")
        for segment in segments:
            print(f"MoveL -> {segment.name}")
            self.driver.move_cartesian(
                np.asarray(segment.pose, dtype=np.float64),
                speed=int(segment.speed),
                timeout=float(segment.timeout_s),
                max_translation_delta=float(segment.max_translation_delta),
                max_rotation_delta=float(segment.max_rotation_delta),
                power_on=power_on,
            )
            if segment.dwell_s > 0.0:
                import time

                print(f"Holding {segment.name} for {segment.dwell_s:.1f} s")
                time.sleep(segment.dwell_s)
