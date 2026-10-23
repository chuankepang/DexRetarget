"""Controller selection.

Precedence, highest first:

  1. CLI value
  2. tasks.<name>.controller
  3. control.controller
  4. position, using control.mode as the position backend

CLI values ``pinocchio`` and ``cartesian`` select the position controller
with that backend. ``impedance`` selects native Cartesian impedance.
An old file that only sets ``control.mode`` keeps today's position behavior.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

# Placeholders below the xCoreSDK 0.3.4 cobot maxima
# {1500, 1500, 1500, 100, 100, 100} N/m, Nm/rad. Not a factory recommendation.
# The SDK has no separate Cartesian damping parameter.
_DEFAULT_STIFFNESS = (300.0, 300.0, 300.0, 15.0, 15.0, 15.0)
_DEFAULT_WRENCH = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
_DEFAULT_MOCK_DAMPING = (80.0, 80.0, 80.0, 2.0, 2.0, 2.0)
_MAX_STIFFNESS = (1500.0, 1500.0, 1500.0, 100.0, 100.0, 100.0)
_MAX_WRENCH = (60.0, 60.0, 60.0, 30.0, 30.0, 30.0)


@dataclass(frozen=True)
class ControllerSpec:
    kind: str
    position_backend: str
    impedance_backend: str
    stiffness: np.ndarray
    desired_wrench: np.ndarray
    max_equilibrium_translation: float
    max_equilibrium_rotation: float
    mock_mass: float
    mock_damping: np.ndarray
    apply_load_on_impedance_start: bool = False
    expected_load_mass: float = 0.0
    expected_load_cog: tuple[float, float, float] = (0.0, 0.0, 0.0)

    @property
    def route(self) -> str:
        if self.kind == "impedance":
            return "impedance"
        return self.position_backend


def _six(values: object, name: str, maximum: tuple[float, ...]) -> np.ndarray:
    array = np.asarray(values, dtype=np.float64)
    if array.shape != (6,) or not np.all(np.isfinite(array)):
        raise ValueError(f"{name} must be six finite numbers")
    for value, limit in zip(array, maximum):
        if name.startswith("stiffness"):
            if value < 0.0 or value > limit:
                raise ValueError(f"{name} {value} is outside [0, {limit}]")
        elif abs(float(value)) > limit:
            raise ValueError(f"{name} {value} exceeds ±{limit}")
    return array


def resolve_controller(
    config: dict,
    *,
    cli: str | None = None,
    task: str | None = None,
) -> ControllerSpec:
    control = config.get("control") or {}
    tasks = config.get("tasks") or {}
    if cli:
        selected = str(cli)
    elif task and isinstance(tasks.get(task), dict) and "controller" in tasks[task]:
        selected = str(tasks[task]["controller"])
    elif "controller" in control:
        selected = str(control["controller"])
    else:
        selected = "position"

    position_backend = str(control.get("mode", "pinocchio"))
    kind = selected
    if selected in {"pinocchio", "cartesian"}:
        kind = "position"
        position_backend = selected
    if kind not in {"position", "impedance"}:
        raise ValueError(
            f"unknown controller {selected!r}; use position, impedance, "
            "pinocchio, or cartesian"
        )
    if position_backend not in {"pinocchio", "cartesian"}:
        raise ValueError("control.mode must be pinocchio or cartesian")

    impedance = control.get("impedance") or {}
    stiffness = _six(
        impedance.get("stiffness", _DEFAULT_STIFFNESS),
        "stiffness",
        _MAX_STIFFNESS,
    )
    wrench = _six(
        impedance.get("desired_wrench", _DEFAULT_WRENCH),
        "desired_wrench",
        _MAX_WRENCH,
    )
    limit = impedance.get("reference_limit") or {}
    translation = float(limit.get("translation", 0.04))
    rotation = float(limit.get("rotation", 0.30))
    if not np.isfinite(translation) or translation <= 0.0:
        raise ValueError("reference_limit.translation must be > 0 metres")
    if not np.isfinite(rotation) or not 0.0 < rotation <= np.pi:
        raise ValueError("reference_limit.rotation must be in (0, pi] radians")
    backend = str(impedance.get("backend", "sdk_cartesian"))
    if backend != "sdk_cartesian":
        raise ValueError(
            "only impedance.backend=sdk_cartesian is implemented; "
            "admittance and custom torque are not available on this build"
        )
    damping = np.asarray(impedance.get("mock_damping", _DEFAULT_MOCK_DAMPING), dtype=np.float64)
    if damping.shape != (6,) or np.any(damping < 0.0) or not np.all(np.isfinite(damping)):
        raise ValueError("mock_damping must be six finite non-negative numbers")
    mass = float(impedance.get("mock_mass", 1.0))
    if not np.isfinite(mass) or mass <= 0.0:
        raise ValueError("mock_mass must be > 0")
    payload = (config.get("robot") or {}).get("payload")
    apply_load = False
    expected_mass, expected_cog = 0.0, (0.0, 0.0, 0.0)
    if payload is not None:
        apply_load = payload.get("apply_on_impedance_start", False)
        if not isinstance(apply_load, bool):
            raise ValueError("robot.payload.apply_on_impedance_start must be a boolean")
        expected_mass = float(payload["mass_kg"])
        cog = np.asarray(payload["cog_m"], dtype=np.float64)
        if not np.isfinite(expected_mass) or expected_mass <= 0.0:
            raise ValueError("robot.payload.mass_kg must be finite and positive")
        if cog.shape != (3,) or not np.all(np.isfinite(cog)):
            raise ValueError("robot.payload.cog_m must be three finite metres")
        expected_cog = tuple(float(v) for v in cog)
    return ControllerSpec(
        kind=kind,
        position_backend=position_backend,
        impedance_backend=backend,
        stiffness=stiffness,
        desired_wrench=wrench,
        max_equilibrium_translation=translation,
        max_equilibrium_rotation=rotation,
        apply_load_on_impedance_start=apply_load,
        expected_load_mass=expected_mass,
        expected_load_cog=expected_cog,
        mock_mass=mass,
        mock_damping=damping,
    )


def apply_realtime_gains(driver: object, spec: ControllerSpec) -> None:
    """Copy impedance gains and expected payload before native configuration is built."""
    driver.apply_load_on_impedance_start = spec.apply_load_on_impedance_start
    driver.expected_load_mass = spec.expected_load_mass
    driver.expected_load_cog = spec.expected_load_cog
    driver.cartesian_stiffness = tuple(float(value) for value in spec.stiffness)
    driver.cartesian_desired_wrench = tuple(float(value) for value in spec.desired_wrench)
    driver.max_equilibrium_translation_error = float(spec.max_equilibrium_translation)
    driver.max_equilibrium_rotation_error = float(spec.max_equilibrium_rotation)
