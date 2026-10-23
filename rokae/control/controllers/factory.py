"""Build the controller selected by configuration. Tasks do not branch on mode."""

from __future__ import annotations

from rokae.control.controllers.impedance import ImpedanceController
from rokae.control.controllers.position import PositionController
from rokae.control.controllers.spec import ControllerSpec, resolve_controller


def create_controller(
    spec: ControllerSpec,
    driver,
    *,
    kinematics=None,
    initial_joints=None,
    mock: bool = False,
):
    if spec.kind == "position":
        return PositionController(
            spec,
            driver,
            kinematics=kinematics,
            initial_joints=initial_joints,
            mock=mock,
        )
    if spec.kind == "impedance":
        return ImpedanceController(spec, driver)
    raise ValueError(f"unknown controller kind {spec.kind}")


__all__ = [
    "ControllerSpec",
    "create_controller",
    "resolve_controller",
]
