#!/usr/bin/env python3
"""Execute dual-iiwa Cartesian targets published by DexRetarget.

The Quest process publishes absolute ``world_T_tcp_target`` and
``base_T_tcp_target`` matrices.  This bridge validates that contract, solves
the redundant 7-DoF inverse kinematics with multiple seeds, selects the
continuous joint-limit-safe candidate, and streams joint targets through the
existing :class:`KukaIiwa` SmartServo protocol.

This module intentionally refuses real output until the exact robot model,
tool transform, and world-to-base calibration are explicitly confirmed in
``cartesian_teleop.yaml``.  It does not provide collision avoidance.
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass, replace
import json
from pathlib import Path
import socket
import sys
import time
from typing import Any, Optional

import numpy as np
import yaml

PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from kuka_iwaa.kuka_iiwa import DT_JOINT_CUR_POS, KukaIiwa  # noqa: E402


SIDES = ("left", "right")
EXPECTED_SCHEMA = "dexretarget.dual_iiwa_robotiq.v2"
EXPECTED_GRIPPER_CONTRACT = "binary_position.v1"


# Kept local so the low-level robot process does not import AnyDexRetarget's
# hand-retargeting dependencies (pinocchio/nlopt).
def validate_transform(value: np.ndarray, name: str = "transform") -> np.ndarray:
    matrix = np.asarray(value, dtype=np.float64)
    if matrix.shape != (4, 4) or not np.all(np.isfinite(matrix)):
        raise ValueError(f"{name} must be a finite 4x4 matrix")
    rotation = matrix[:3, :3]
    if not np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-6):
        raise ValueError(f"{name} rotation must be orthonormal")
    if not np.isclose(np.linalg.det(rotation), 1.0, atol=1e-6):
        raise ValueError(f"{name} rotation must have determinant +1")
    if not np.allclose(matrix[3], [0.0, 0.0, 0.0, 1.0], atol=1e-8):
        raise ValueError(f"{name} last row must be [0,0,0,1]")
    return matrix.copy()


def invert_transform(value: np.ndarray) -> np.ndarray:
    matrix = validate_transform(value)
    result = np.eye(4)
    result[:3, :3] = matrix[:3, :3].T
    result[:3, 3] = -result[:3, :3] @ matrix[:3, 3]
    return result


def rotation_vector_to_matrix(value: np.ndarray) -> np.ndarray:
    vector = np.asarray(value, dtype=np.float64)
    if vector.shape != (3,) or not np.all(np.isfinite(vector)):
        raise ValueError("rotation vector must be finite and length three")
    angle = float(np.linalg.norm(vector))
    if angle < 1e-12:
        return np.eye(3)
    x, y, z = vector / angle
    skew = np.array([[0.0, -z, y], [z, 0.0, -x], [-y, x, 0.0]])
    return np.eye(3) + np.sin(angle) * skew + (1.0 - np.cos(angle)) * (skew @ skew)


def matrix_to_rotation_vector(value: np.ndarray) -> np.ndarray:
    rotation = np.asarray(value, dtype=np.float64)
    cosine = float(np.clip((np.trace(rotation) - 1.0) * 0.5, -1.0, 1.0))
    angle = float(np.arccos(cosine))
    if angle < 1e-8:
        return 0.5 * np.array(
            [rotation[2, 1] - rotation[1, 2], rotation[0, 2] - rotation[2, 0], rotation[1, 0] - rotation[0, 1]]
        )
    if np.pi - angle < 1e-6:
        # Stable eigenvector extraction at 180 degrees.
        eigenvalues, eigenvectors = np.linalg.eig(rotation)
        axis = np.real(eigenvectors[:, np.argmin(np.abs(eigenvalues - 1.0))])
        axis /= np.linalg.norm(axis)
        return angle * axis
    return angle / (2.0 * np.sin(angle)) * np.array(
        [rotation[2, 1] - rotation[1, 2], rotation[0, 2] - rotation[2, 0], rotation[1, 0] - rotation[0, 1]]
    )


def pose_distance(first: np.ndarray, second: np.ndarray) -> tuple[float, float]:
    a = validate_transform(first)
    b = validate_transform(second)
    return (
        float(np.linalg.norm(b[:3, 3] - a[:3, 3])),
        float(np.linalg.norm(matrix_to_rotation_vector(a[:3, :3].T @ b[:3, :3]))),
    )


def _vector(config: dict[str, Any], key: str, length: int = 7) -> np.ndarray:
    value = np.asarray(config[key], dtype=np.float64)
    if value.shape != (length,) or not np.all(np.isfinite(value)):
        raise ValueError(f"{key} must contain {length} finite values")
    return value


def _transform(config: dict[str, Any], key: str) -> np.ndarray:
    return validate_transform(np.asarray(config[key], dtype=np.float64), key)


def _dh_transform(theta: float, d: float, a: float, alpha: float) -> np.ndarray:
    """Standard DH transform: Rz(theta) Tz(d) Tx(a) Rx(alpha)."""
    ct, st, ca, sa = np.cos(theta), np.sin(theta), np.cos(alpha), np.sin(alpha)
    return np.array(
        [
            [ct, -st * ca, st * sa, a * ct],
            [st, ct * ca, -ct * sa, a * st],
            [0.0, sa, ca, d],
            [0.0, 0.0, 0.0, 1.0],
        ],
        dtype=np.float64,
    )


@dataclass(frozen=True)
class IKConfig:
    position_tolerance_m: float = 0.002
    rotation_tolerance_rad: float = 0.02
    max_iterations: int = 80
    damping: float = 0.03
    orientation_weight_m: float = 0.25
    max_iteration_joint_step_rad: float = 0.15
    candidate_count: int = 9
    continuity_weight: float = 1.0
    limit_weight: float = 0.02

    @classmethod
    def from_mapping(cls, value: dict[str, Any]) -> "IKConfig":
        result = cls(**{key: value[key] for key in cls.__dataclass_fields__ if key in value})
        if result.position_tolerance_m <= 0 or result.rotation_tolerance_rad <= 0:
            raise ValueError("IK tolerances must be positive")
        if result.max_iterations < 1 or result.candidate_count < 1:
            raise ValueError("IK iteration and candidate counts must be positive")
        if result.damping <= 0 or result.orientation_weight_m <= 0:
            raise ValueError("IK damping and orientation weight must be positive")
        return result


@dataclass(frozen=True)
class IKCandidate:
    joints: np.ndarray
    position_error_m: float
    rotation_error_rad: float
    score: float
    iterations: int


class IiwaKinematics:
    """Configurable 7-axis POE/DH FK plus damped-least-squares redundant IK."""

    def __init__(self, arm: dict[str, Any], solver: IKConfig):
        self.kind = str(arm.get("kinematics", "dh")).lower()
        self.zero = _vector(arm, "joint_zero_offset_rad")
        if self.kind == "poe":
            screws = np.asarray(arm["space_screw_axes"], dtype=np.float64)
            if screws.shape != (7, 6) or not np.all(np.isfinite(screws)):
                raise ValueError("space_screw_axes must be seven [wx,wy,wz,vx,vy,vz] rows")
            omega_norms = np.linalg.norm(screws[:, :3], axis=1)
            if not np.allclose(omega_norms, 1.0, atol=1e-6):
                raise ValueError("each POE angular screw axis must be unit length")
            self.screws = screws
            self.home_base_T_flange = _transform(arm, "home_base_T_flange")
        elif self.kind == "dh":
            self.d = _vector(arm, "dh_d_m")
            self.a = _vector(arm, "dh_a_m")
            self.alpha = _vector(arm, "dh_alpha_rad")
        else:
            raise ValueError("kinematics must be 'poe' or 'dh'")
        self.lower = _vector(arm, "joint_min_rad")
        self.upper = _vector(arm, "joint_max_rad")
        if np.any(self.lower >= self.upper):
            raise ValueError("every joint_min_rad must be below joint_max_rad")
        self.flange_T_tcp = _transform(arm, "flange_T_tcp")
        self.config = solver

    def forward(self, joints: np.ndarray) -> np.ndarray:
        q = np.asarray(joints, dtype=np.float64)
        if q.shape != (7,) or not np.all(np.isfinite(q)):
            raise ValueError("joints must be a finite seven-vector")
        result = np.eye(4)
        if self.kind == "poe":
            for index in range(7):
                result = result @ self._twist_exponential(
                    self.screws[index], q[index] + self.zero[index]
                )
            result = result @ self.home_base_T_flange
        else:
            for index in range(7):
                result = result @ _dh_transform(
                    q[index] + self.zero[index],
                    self.d[index],
                    self.a[index],
                    self.alpha[index],
                )
        return result @ self.flange_T_tcp

    @staticmethod
    def _twist_exponential(screw: np.ndarray, theta: float) -> np.ndarray:
        """SE(3) exponential for a unit revolute space screw [omega, v]."""
        omega = screw[:3]
        velocity = screw[3:]
        rotation = rotation_vector_to_matrix(omega * theta)
        omega_hat = np.array(
            [
                [0.0, -omega[2], omega[1]],
                [omega[2], 0.0, -omega[0]],
                [-omega[1], omega[0], 0.0],
            ]
        )
        # G(theta)v = (I theta + (1-cos)w^ + (theta-sin)w^2)v
        translation = (
            np.eye(3) * theta
            + (1.0 - np.cos(theta)) * omega_hat
            + (theta - np.sin(theta)) * (omega_hat @ omega_hat)
        ) @ velocity
        result = np.eye(4)
        result[:3, :3] = rotation
        result[:3, 3] = translation
        return result

    @staticmethod
    def _error(current: np.ndarray, target: np.ndarray) -> np.ndarray:
        return np.concatenate(
            (
                target[:3, 3] - current[:3, 3],
                matrix_to_rotation_vector(current[:3, :3].T @ target[:3, :3]),
            )
        )

    def jacobian(self, joints: np.ndarray, epsilon: float = 1e-6) -> np.ndarray:
        """Finite-difference geometric Jacobian matching :meth:`_error`."""
        q = np.asarray(joints, dtype=np.float64)
        current = self.forward(q)
        jacobian = np.empty((6, 7), dtype=np.float64)
        for joint in range(7):
            shifted = q.copy()
            shifted[joint] += epsilon
            changed = self.forward(shifted)
            jacobian[:3, joint] = (changed[:3, 3] - current[:3, 3]) / epsilon
            jacobian[3:, joint] = matrix_to_rotation_vector(
                current[:3, :3].T @ changed[:3, :3]
            ) / epsilon
        return jacobian

    def _iterate(self, target: np.ndarray, seed: np.ndarray) -> Optional[IKCandidate]:
        cfg = self.config
        q = np.clip(np.asarray(seed, dtype=np.float64), self.lower, self.upper)
        weights = np.array([1.0, 1.0, 1.0] + [cfg.orientation_weight_m] * 3)
        for iteration in range(1, cfg.max_iterations + 1):
            current = self.forward(q)
            error = self._error(current, target)
            position_error = float(np.linalg.norm(error[:3]))
            rotation_error = float(np.linalg.norm(error[3:]))
            if (
                position_error <= cfg.position_tolerance_m
                and rotation_error <= cfg.rotation_tolerance_rad
            ):
                return IKCandidate(q.copy(), position_error, rotation_error, 0.0, iteration)
            jacobian = self.jacobian(q)
            weighted_jacobian = weights[:, None] * jacobian
            weighted_error = weights * error
            system = weighted_jacobian @ weighted_jacobian.T
            system += (cfg.damping**2) * np.eye(6)
            try:
                delta = weighted_jacobian.T @ np.linalg.solve(system, weighted_error)
            except np.linalg.LinAlgError:
                return None
            norm = float(np.linalg.norm(delta))
            if norm > cfg.max_iteration_joint_step_rad:
                delta *= cfg.max_iteration_joint_step_rad / norm
            q = np.clip(q + delta, self.lower, self.upper)
        return None

    def _seeds(
        self,
        current: np.ndarray,
        candidate_count: Optional[int] = None,
        extra_seeds: Optional[list[list[float]]] = None,
    ) -> list[np.ndarray]:
        count = self.config.candidate_count if candidate_count is None else candidate_count
        seeds = [np.clip(current, self.lower, self.upper)]
        for index, value in enumerate(extra_seeds or []):
            seed = np.asarray(value, dtype=np.float64)
            if seed.shape != (7,) or not np.all(np.isfinite(seed)):
                raise ValueError(
                    f"IK extra seed {index} must contain seven finite joint angles"
                )
            seeds.append(np.clip(seed, self.lower, self.upper))
            if len(seeds) >= count:
                return seeds
        midpoint = 0.5 * (self.lower + self.upper)
        if len(seeds) < count:
            seeds.append(midpoint)
        # A fixed RNG makes broad redundant-branch exploration reproducible.
        # This is especially important for the configured startup pose: local
        # seeds can converge to q2/q6 hard limits while safe branches exist.
        rng = np.random.default_rng(4)
        while len(seeds) < min(count, 18):
            seeds.append(rng.uniform(self.lower, self.upper))
        # Add local redundant-joint offsets after broad exploration.
        for joint in (2, 4, 6, 1, 3, 5, 0):
            for sign in (1.0, -1.0):
                seed = current.copy()
                seed[joint] += sign * 0.65
                seeds.append(np.clip(seed, self.lower, self.upper))
                if len(seeds) >= count:
                    return seeds
        while len(seeds) < count:
            seeds.append(rng.uniform(self.lower, self.upper))
        return seeds[:count]

    def solve_candidates(
        self,
        target: np.ndarray,
        current_joints: np.ndarray,
        extra_seeds: Optional[list[list[float]]] = None,
    ) -> list[IKCandidate]:
        """Return all converged branches sorted by continuity/limit score."""
        desired = validate_transform(target, "base_T_tcp_target")
        current = np.asarray(current_joints, dtype=np.float64)
        if current.shape != (7,) or not np.all(np.isfinite(current)):
            raise ValueError("current_joints must be a finite seven-vector")
        span = self.upper - self.lower
        candidates: list[IKCandidate] = []
        for seed in self._seeds(current, extra_seeds=extra_seeds):
            candidate = self._iterate(desired, seed)
            if candidate is None:
                continue
            normalized_delta = (candidate.joints - current) / span
            margin = np.minimum(candidate.joints - self.lower, self.upper - candidate.joints) / span
            score = (
                self.config.continuity_weight * float(normalized_delta @ normalized_delta)
                + self.config.limit_weight * float(np.sum(1.0 / np.maximum(margin, 1e-3)))
            )
            # Deduplicate numerical convergence to the same branch.
            if any(np.linalg.norm(candidate.joints - old.joints) < 1e-3 for old in candidates):
                continue
            candidates.append(
                IKCandidate(
                    candidate.joints,
                    candidate.position_error_m,
                    candidate.rotation_error_rad,
                    score,
                    candidate.iterations,
                )
            )
        return sorted(candidates, key=lambda item: item.score)

    def solve(self, target: np.ndarray, current_joints: np.ndarray) -> IKCandidate:
        # Normal teleoperation is incremental, so the previous command is both
        # the fastest and most continuous seed. Only pay for multi-branch
        # search when that local branch cannot converge.
        desired = validate_transform(target, "base_T_tcp_target")
        current = np.asarray(current_joints, dtype=np.float64)
        local = self._iterate(desired, current)
        if local is not None:
            return local
        candidates = self.solve_candidates(desired, current)
        if not candidates:
            raise RuntimeError("IK did not converge for any candidate branch")
        return candidates[0]


def kuka_abc_pose_to_matrix(values: list[float]) -> np.ndarray:
    """Convert controller [x,y,z mm,A,B,C rad] using KUKA Rz(A)Ry(B)Rx(C)."""
    if len(values) != 6 or not np.all(np.isfinite(values)):
        raise ValueError("controller pose must contain six finite values")
    x, y, z, a, b, c = (float(value) for value in values)
    rz = rotation_vector_to_matrix(np.array([0.0, 0.0, a]))
    ry = rotation_vector_to_matrix(np.array([0.0, b, 0.0]))
    rx = rotation_vector_to_matrix(np.array([c, 0.0, 0.0]))
    result = np.eye(4)
    result[:3, :3] = rz @ ry @ rx
    result[:3, 3] = np.array([x, y, z]) * 0.001
    return result


@dataclass
class ArmRuntime:
    side: str
    config: dict[str, Any]
    kinematics: IiwaKinematics
    world_T_base: np.ndarray
    controller_pose_T_base: np.ndarray
    robot: Optional[KukaIiwa] = None
    joints: Optional[np.ndarray] = None
    last_target: Optional[np.ndarray] = None
    last_gripper: Optional[int] = None
    start_checked: bool = False
    delta_reference: Optional[np.ndarray] = None
    reference_id: Optional[int] = None
    motion_active: bool = False


class CartesianTeleopBridge:
    def __init__(self, config: dict[str, Any], dry_run: bool = False, debug: bool = False):
        self.config = config
        self.dry_run = dry_run
        self.debug = debug
        self.solver = IKConfig.from_mapping(config["solver"])
        self.safety = config["safety"]
        self.runtimes: dict[str, ArmRuntime] = {}
        for side in SIDES:
            arm = config["arms"][side]
            if not bool(arm.get("enabled", True)):
                continue
            if not dry_run and (
                not bool(arm.get("model_confirmed"))
                or not bool(arm.get("calibration_confirmed"))
            ):
                raise ValueError(
                    f"{side}: real output blocked until model_confirmed and "
                    "calibration_confirmed are true"
                )
            self.runtimes[side] = ArmRuntime(
                side,
                arm,
                IiwaKinematics(arm, self.solver),
                _transform(arm, "world_T_base"),
                _transform(
                    arm,
                    "controller_pose_T_base",
                ) if "controller_pose_T_base" in arm else np.eye(4),
            )
        if not self.runtimes:
            raise ValueError("at least one arm must be enabled")

    def connect(self) -> None:
        if self.dry_run:
            for runtime in self.runtimes.values():
                runtime.joints = 0.5 * (
                    runtime.kinematics.lower + runtime.kinematics.upper
                )
            return
        connected: list[ArmRuntime] = []
        try:
            for runtime in self.runtimes.values():
                arm = runtime.config
                robot = KukaIiwa(debug=self.debug)
                robot.connect(str(arm["robot_ip"]), int(arm["robot_port"]))
                runtime.robot = robot
                connected.append(runtime)
                if not robot.wait_for_rank(1, 3.0):
                    raise RuntimeError(f"{runtime.side}: no LINK_SUCESS")
                # Read and validate actual state as Observer before acquiring
                # motion authority. This prevents a stale configured pose from
                # becoming the first SmartServo target.
                robot.start_async()
                values = robot.wait_for_data(DT_JOINT_CUR_POS, 3.0)
                if values is None:
                    raise RuntimeError(f"{runtime.side}: no current joint state")
                runtime.joints = np.asarray(values, dtype=np.float64)
                self._verify_fk_against_controller(runtime)
                if not robot.request_controller(5.0):
                    raise RuntimeError(f"{runtime.side}: Controller authority denied")
                # Clear a controller-side target retained from a prior session,
                # then make the measured joints the first target of this one.
                robot.cancel()
                robot.start_smart_servo_stream(
                    rate_hz=float(arm.get("smart_servo_rate_hz", 100.0)),
                    target_timeout_s=float(
                        arm.get("smart_servo_target_timeout_s", 0.25)
                    ),
                )
                robot.smart_servo(runtime.joints.tolist())
                print(
                    f"{runtime.side}: FK verified as Observer; authority=Controller, "
                    "stale target cleaned, holding measured joints"
                )
                if bool(arm.get("move_to_initial_on_start", False)):
                    self._move_to_initial_pose(runtime)
        except Exception:
            for runtime in connected:
                self._stop_robot(runtime)
            raise

    def _move_to_initial_pose(self, runtime: ArmRuntime) -> None:
        """Move once to the configured world TCP pose, then wait disarmed."""
        assert runtime.robot is not None and runtime.joints is not None
        arm = runtime.config
        world_target = _transform(arm, "initial_world_T_tcp")
        base_target = invert_transform(runtime.world_T_base) @ world_target
        position = world_target[:3, 3]
        print(
            f"{runtime.side}: startup W_T_TCP target xyz="
            f"[{position[0]:.3f}, {position[1]:.3f}, {position[2]:.3f}] m"
        )

        candidate_count = int(arm.get("initial_ik_candidate_count", 32))
        initial_solver = replace(
            self.solver,
            candidate_count=candidate_count,
            max_iterations=max(self.solver.max_iterations, 120),
        )
        initial_model = IiwaKinematics(arm, initial_solver)
        candidates = initial_model.solve_candidates(
            base_target,
            runtime.joints,
            extra_seeds=arm.get("initial_ik_seed_candidates_rad"),
        )
        minimum_margin = float(arm.get("initial_min_joint_margin_rad", 0.10))
        safe_candidates = [
            item
            for item in candidates
            if float(
                np.min(
                    np.minimum(
                        item.joints - initial_model.lower,
                        initial_model.upper - item.joints,
                    )
                )
            )
            >= minimum_margin
        ]
        if not safe_candidates:
            raise RuntimeError(
                f"{runtime.side}: startup world pose has no IK candidate with "
                f">={minimum_margin:.3f} rad joint-limit margin"
            )
        chosen = safe_candidates[0]
        margin = float(
            np.min(
                np.minimum(
                    chosen.joints - initial_model.lower,
                    initial_model.upper - chosen.joints,
                )
            )
        )
        print(
            f"{runtime.side}: startup IK selected from {len(candidates)} solution(s); "
            f"min limit margin={margin:.3f} rad, q={np.round(chosen.joints, 4).tolist()}"
        )
        countdown = float(arm.get("initial_motion_countdown_s", 5.0))
        if countdown > 0.0:
            print(
                f"{runtime.side}: STARTUP ROBOT MOTION in {countdown:.1f} s; "
                "press Ctrl+C to abort"
            )
            time.sleep(countdown)
        runtime.robot.move_joints_ptp(
            chosen.joints.tolist(),
            duration=float(arm.get("initial_move_duration_s", 6.0)),
            rate_hz=float(arm.get("initial_servo_rate_hz", 100.0)),
            max_joint_speed_rad_s=float(
                arm.get("initial_max_joint_speed_rad_s", 0.6)
            ),
        )

        deadline = time.monotonic() + float(arm.get("initial_settle_timeout_s", 3.0))
        measured = runtime.joints.copy()
        while time.monotonic() < deadline:
            latest = runtime.robot.get_joints()
            if latest is not None:
                measured = np.asarray(latest, dtype=np.float64)
            if float(np.max(np.abs(measured - chosen.joints))) <= float(
                arm.get("initial_joint_tolerance_rad", 0.03)
            ):
                break
            time.sleep(0.02)
        runtime.joints = measured
        reached = runtime.kinematics.forward(measured)
        position_error, rotation_error = pose_distance(reached, base_target)
        if (
            position_error > float(arm.get("initial_position_tolerance_m", 0.01))
            or rotation_error > float(arm.get("initial_rotation_tolerance_rad", 0.05))
        ):
            runtime.robot.cancel()
            raise RuntimeError(
                f"{runtime.side}: failed to reach startup pose; residual "
                f"{position_error:.4f} m / {rotation_error:.4f} rad"
            )
        runtime.last_target = base_target.copy()
        runtime.delta_reference = base_target.copy()
        runtime.start_checked = True
        runtime.motion_active = False
        print(
            f"{runtime.side}: startup pose reached; residual "
            f"{position_error:.4f} m / {rotation_error:.4f} rad. "
            "Waiting for teleop ENABLED signal."
        )

    def _verify_fk_against_controller(self, runtime: ArmRuntime) -> None:
        assert runtime.robot is not None and runtime.joints is not None
        runtime.robot.sync_pose()
        deadline = time.monotonic() + 2.0
        pose = None
        while time.monotonic() < deadline:
            pose = runtime.robot.get_pose()
            if pose is not None:
                break
            time.sleep(0.01)
        if pose is None:
            raise RuntimeError(f"{runtime.side}: no Cartesian state for FK verification")
        measured = kuka_abc_pose_to_matrix(pose)
        predicted_base = runtime.kinematics.forward(runtime.joints)
        # The custom KUKA server used on this platform reports Cartesian state
        # in an installation-oriented frame whose origin remains at B_L. This
        # is not the full W0_T_BL transform: its 0.193/1.217 m platform
        # translations are deliberately absent from the received xyz values.
        predicted = runtime.controller_pose_T_base @ predicted_base
        position_error, rotation_error = pose_distance(predicted, measured)
        print(
            f"{runtime.side}: FK/controller error "
            f"{position_error:.4f} m, {rotation_error:.4f} rad"
        )
        if (
            position_error > float(self.safety["fk_state_position_tolerance_m"])
            or rotation_error > float(self.safety["fk_state_rotation_tolerance_rad"])
        ):
            raise RuntimeError(
                f"{runtime.side}: POE/TCP model does not match controller pose; "
                "check space screws, flange_T_tcp, controller_pose_T_base and ABC frame"
            )

    @staticmethod
    def _stop_robot(runtime: ArmRuntime) -> None:
        if runtime.robot is None:
            return
        try:
            runtime.robot.cancel()
            runtime.robot.stop_smart_servo_stream()
            runtime.robot.stop_async()
        except OSError:
            pass
        finally:
            runtime.robot.disconnect()
            runtime.robot = None

    def close(self) -> None:
        for runtime in self.runtimes.values():
            self._stop_robot(runtime)

    def _decode_target(self, runtime: ArmRuntime, arm_packet: dict[str, Any]) -> np.ndarray:
        base_target = validate_transform(
            np.asarray(arm_packet["base_T_tcp_target"], dtype=np.float64).reshape(4, 4),
            f"{runtime.side} base target",
        )
        world_target = validate_transform(
            np.asarray(arm_packet["world_T_tcp_target"], dtype=np.float64).reshape(4, 4),
            f"{runtime.side} world target",
        )
        expected = invert_transform(runtime.world_T_base) @ world_target
        position_error, rotation_error = pose_distance(expected, base_target)
        if position_error > 1e-5 or rotation_error > 1e-5:
            raise ValueError(
                f"{runtime.side}: publisher/bridge world_T_base configurations disagree"
            )
        return base_target

    def _decode_delta_target(
        self, runtime: ArmRuntime, arm_packet: dict[str, Any]
    ) -> np.ndarray:
        """Anchor publisher world deltas at the measured/held base TCP pose."""
        assert runtime.joints is not None
        reference_id = int(arm_packet["reference_id"])
        if runtime.delta_reference is None:
            runtime.delta_reference = runtime.kinematics.forward(runtime.joints)
            runtime.reference_id = reference_id
        elif reference_id != runtime.reference_id:
            # Publisher clutch/recenter starts a new zero-delta epoch. Preserve
            # the last commanded target so recentering never causes a jump.
            runtime.delta_reference = (
                runtime.last_target.copy()
                if runtime.last_target is not None
                else runtime.kinematics.forward(runtime.joints)
            )
            runtime.reference_id = reference_id

        delta_position_world = np.asarray(
            arm_packet["world_translation_delta_m"], dtype=np.float64
        )
        delta_rotation_world = np.asarray(
            arm_packet["world_rotation_delta_vector_rad"], dtype=np.float64
        )
        if (
            delta_position_world.shape != (3,)
            or delta_rotation_world.shape != (3,)
            or not np.all(np.isfinite(delta_position_world))
            or not np.all(np.isfinite(delta_rotation_world))
        ):
            raise ValueError(f"{runtime.side}: invalid world delta vectors")
        world_R_base = runtime.world_T_base[:3, :3]
        base_R_world = world_R_base.T
        delta_position_base = base_R_world @ delta_position_world
        delta_rotation_base = (
            base_R_world
            @ rotation_vector_to_matrix(delta_rotation_world)
            @ world_R_base
        )
        target = runtime.delta_reference.copy()
        target[:3, 3] += delta_position_base
        target[:3, :3] = delta_rotation_base @ runtime.delta_reference[:3, :3]
        return target

    def _execute_arm(self, runtime: ArmRuntime, target: np.ndarray) -> IKCandidate:
        assert runtime.joints is not None
        actual = runtime.kinematics.forward(runtime.joints)
        if not runtime.start_checked:
            position_error, rotation_error = pose_distance(actual, target)
            if (
                position_error > float(self.safety["max_start_position_error_m"])
                or rotation_error > float(self.safety["max_start_rotation_error_rad"])
            ):
                raise RuntimeError(
                    f"{runtime.side}: first target is {position_error:.3f} m / "
                    f"{rotation_error:.3f} rad from current TCP; disable, place the "
                    "tracked hand, then recenter before enabling"
                )
            runtime.start_checked = True
        if runtime.last_target is not None:
            position_step, rotation_step = pose_distance(runtime.last_target, target)
            if (
                position_step > float(self.safety["max_target_translation_step_m"])
                or rotation_step > float(self.safety["max_target_rotation_step_rad"])
            ):
                raise RuntimeError(
                    f"{runtime.side}: Cartesian target step too large "
                    f"({position_step:.3f} m, {rotation_step:.3f} rad)"
                )
        result = runtime.kinematics.solve(target, runtime.joints)
        delta = result.joints - runtime.joints
        max_step = float(self.safety["max_command_joint_step_rad"])
        if float(np.max(np.abs(delta))) > max_step:
            raise RuntimeError(
                f"{runtime.side}: IK joint step {np.max(np.abs(delta)):.4f} exceeds {max_step}"
            )
        if runtime.robot is not None:
            runtime.robot.smart_servo(result.joints.tolist())
        runtime.motion_active = True
        runtime.joints = result.joints.copy()
        runtime.last_target = target.copy()
        return result

    def process_packet(self, packet: dict[str, Any]) -> dict[str, IKCandidate]:
        if packet.get("schema") != EXPECTED_SCHEMA:
            raise ValueError(f"unexpected command schema: {packet.get('schema')!r}")
        results: dict[str, IKCandidate] = {}
        contract = packet.get("arm_command_contract", "absolute_tcp.v1")
        if contract not in (
            "absolute_tcp.v1",
            "reference_relative_world_delta.v1",
        ):
            raise ValueError(f"unsupported arm command contract: {contract!r}")
        gripper_contract = packet.get(
            "gripper_command_contract", EXPECTED_GRIPPER_CONTRACT
        )
        if gripper_contract != EXPECTED_GRIPPER_CONTRACT:
            raise ValueError(
                f"unsupported gripper command contract: {gripper_contract!r}"
            )
        for side, runtime in self.runtimes.items():
            arm_packet = packet["arms"][side]
            gripper_packet = packet["grippers"][side]
            if bool(packet.get("teleop_enabled")) and bool(arm_packet.get("valid")):
                target = (
                    self._decode_delta_target(runtime, arm_packet)
                    if contract == "reference_relative_world_delta.v1"
                    else self._decode_target(runtime, arm_packet)
                )
                results[side] = self._execute_arm(runtime, target)
            elif runtime.motion_active:
                if runtime.robot is not None:
                    runtime.robot.cancel()
                runtime.motion_active = False
            if bool(packet.get("teleop_enabled")) and bool(gripper_packet.get("valid")):
                command = int(gripper_packet["command"])
                if command not in (0, 1):
                    raise ValueError(f"{side}: gripper command must be 0 or 1")
                if command != runtime.last_gripper and runtime.robot is not None:
                    runtime.robot.set_gripper_binary(
                        command,
                        open_position=int(runtime.config["gripper_open_position"]),
                        close_position=int(runtime.config["gripper_close_position"]),
                        mode_settle_s=float(
                            runtime.config.get("gripper_mode_settle_s", 0.15)
                        ),
                    )
                    if self.debug:
                        action = "close" if command else "open"
                        print(f"{side}: gripper={command} ({action}) sent")
                runtime.last_gripper = command
        return results

    def run(self) -> None:
        network = self.config["network"]
        address = (str(network["listen_host"]), int(network["listen_port"]))
        timeout = float(network["command_timeout_s"])
        maximum = int(network["max_datagram_bytes"])
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.bind(address)
        sock.settimeout(min(0.05, timeout))
        print(f"Cartesian bridge listening on udp://{address[0]}:{address[1]}")
        print("DRY RUN: no robot output" if self.dry_run else "REAL OUTPUT ENABLED")
        last_packet_time = time.monotonic()
        watchdog_tripped = False
        last_sequence: Optional[int] = None
        try:
            while True:
                try:
                    payload, _peer = sock.recvfrom(maximum + 1)
                except socket.timeout:
                    if time.monotonic() - last_packet_time > timeout and not watchdog_tripped:
                        for runtime in self.runtimes.values():
                            if runtime.robot is not None:
                                runtime.robot.cancel()
                        watchdog_tripped = True
                        print("WATCHDOG: command stream timed out; motion cancelled")
                    continue
                if len(payload) > maximum:
                    print("Rejected oversized UDP datagram")
                    continue
                try:
                    packet = json.loads(payload)
                    sequence = int(packet["sequence"])
                    stream_age = time.monotonic() - last_packet_time
                    if (
                        last_sequence is not None
                        and sequence <= last_sequence
                        and stream_age <= timeout
                    ):
                        continue
                    results = self.process_packet(packet)
                    last_sequence = sequence
                    last_packet_time = time.monotonic()
                    watchdog_tripped = False
                    if self.debug and results:
                        print(
                            " | ".join(
                                f"{side}: IK {item.iterations}it "
                                f"e={item.position_error_m:.4f}m/"
                                f"{item.rotation_error_rad:.4f}rad"
                                for side, item in results.items()
                            )
                        )
                except (KeyError, TypeError, ValueError, RuntimeError, json.JSONDecodeError) as exc:
                    print(f"Rejected command: {exc}")
                    for runtime in self.runtimes.values():
                        if runtime.robot is not None:
                            runtime.robot.cancel()
        except KeyboardInterrupt:
            print("\nStopping Cartesian bridge")
        finally:
            sock.close()


def _load_config(path: Path) -> dict[str, Any]:
    with path.open("r", encoding="utf-8") as stream:
        result = yaml.safe_load(stream)
    if not isinstance(result, dict):
        raise ValueError("configuration root must be a mapping")
    for key in ("network", "solver", "safety", "arms"):
        if not isinstance(result.get(key), dict):
            raise ValueError(f"missing configuration mapping: {key}")
    for side in SIDES:
        if not isinstance(result["arms"].get(side), dict):
            raise ValueError(f"missing arm configuration: {side}")
    return result


def self_test(config: dict[str, Any]) -> None:
    solver = IKConfig.from_mapping(config["solver"])
    for side in SIDES:
        if not bool(config["arms"][side].get("enabled", True)):
            continue
        model = IiwaKinematics(config["arms"][side], solver)
        q_reference = 0.5 * (model.lower + model.upper)
        # Avoid the fully straight singular posture in the round-trip test.
        q_reference += np.array([0.2, -0.35, 0.25, -0.55, 0.2, 0.45, -0.15])
        target = model.forward(q_reference)
        candidates = model.solve_candidates(target, q_reference + 0.05)
        if not candidates:
            raise RuntimeError(f"{side}: FK/IK round-trip failed")
        best = candidates[0]
        print(
            f"{side}: {len(candidates)} IK candidate(s), best residual "
            f"{best.position_error_m:.6f} m / {best.rotation_error_rad:.6f} rad"
        )


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="UDP Cartesian targets -> dual KUKA iiwa IK/SmartServo"
    )
    parser.add_argument(
        "--config",
        type=Path,
        default=Path(__file__).with_name("cartesian_teleop.yaml"),
    )
    parser.add_argument(
        "--dry-run", action="store_true", help="receive/solve but never connect or command robots"
    )
    parser.add_argument("--self-test", action="store_true", help="run offline FK/IK round-trip")
    parser.add_argument("--debug", action="store_true")
    return parser


def main() -> None:
    args = build_parser().parse_args()
    config = _load_config(args.config)
    if args.self_test:
        self_test(config)
        return
    bridge = CartesianTeleopBridge(config, dry_run=args.dry_run, debug=args.debug)
    try:
        bridge.connect()
        bridge.run()
    finally:
        bridge.close()


if __name__ == "__main__":
    main()
