"""Native xCore Cartesian impedance.

target_pose is the equilibrium, not a rigid position demand. The 1 kHz loop
stays in C++. Python only publishes the latest equilibrium. Stiffness is the
SDK factor array. Damping is internal to xCore and is not sent.
"""

from __future__ import annotations

import time

import numpy as np

from rokae.control.command import EECommand, PathSegment
from rokae.control.controllers.base import ControllerBase
from rokae.control.controllers.spec import ControllerSpec
from rokae.control.reference_limit import limit_equilibrium, rotation_vector


class ImpedanceController(ControllerBase):
    def __init__(self, spec: ControllerSpec, driver) -> None:
        super().__init__(spec, driver)
        if spec.kind != "impedance":
            raise ValueError("ImpedanceController requires kind=impedance")
        if spec.impedance_backend != "sdk_cartesian":
            raise ValueError("unsupported impedance backend")

    @property
    def backend(self) -> str:
        return self.spec.impedance_backend

    def start(self, *, power_on: bool = True) -> None:
        if self.state == "running":
            return
        self.driver.start(power_on=power_on, control_mode="impedance")
        self.state = "running"

    def set_target(self, command: EECommand) -> bool:
        with self._lock:
            self._require_running()
            started = time.monotonic()
            measured = np.asarray(self.driver.get_tcp_pose(), dtype=np.float64)
            limited = limit_equilibrium(
                command.pose,
                measured,
                self.spec.max_equilibrium_translation,
                self.spec.max_equilibrium_rotation,
            )
            if self.timing is not None:
                self.timing.record('impedance_state_and_limit', time.monotonic() - started)
            accepted = bool(
                self.driver.set_target_pose(
                    limited,
                    source_timestamp=command.timestamp,
                    sequence=command.sequence,
                )
            )
            if accepted:
                self.last_accepted_pose = limited.copy()
            return accepted

    def command_pose(self) -> np.ndarray:
        return np.asarray(self.driver.get_command_pose(), dtype=np.float64)

    def follow(self, segments: list[PathSegment], *, power_on: bool = True) -> None:
        """Stream each waypoint as an equilibrium, then leave realtime mode."""
        self.start(power_on=power_on)
        sequence = 0
        try:
            for segment in segments:
                print(
                    f"Impedance equilibrium -> {segment.name} "
                    f"(error limited to {self.spec.max_equilibrium_translation * 1000:.0f} mm)"
                )
                deadline = time.monotonic() + float(segment.timeout_s)
                pose = np.asarray(segment.pose, dtype=np.float64)
                while True:
                    now = time.monotonic()
                    measured = np.asarray(self.driver.get_tcp_pose(), dtype=np.float64)
                    command = EECommand(
                        pose=pose, timestamp=now, sequence=sequence, source="path"
                    )
                    sequence += 1
                    self.set_target(command)
                    error = measured[:3, 3] - pose[:3, 3]
                    rotation = float(
                        np.linalg.norm(
                            rotation_vector(measured[:3, :3].T @ pose[:3, :3])
                        )
                    )
                    if float(np.linalg.norm(error)) < 0.015 and rotation < 0.08:
                        break
                    if now >= deadline:
                        print(
                            f"{segment.name} impedance approach stopped at "
                            f"{np.linalg.norm(error) * 1000.0:.1f} mm / "
                            f"{np.rad2deg(rotation):.1f} deg"
                        )
                        break
                    time.sleep(0.02)
                if segment.dwell_s > 0.0:
                    print(f"Holding {segment.name} for {segment.dwell_s:.1f} s")
                    time.sleep(segment.dwell_s)
        finally:
            self.stop()
