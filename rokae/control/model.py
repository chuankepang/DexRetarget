"""Pinocchio model of the 7-axis xMate ER Pro arm.

The chain is the standard spherical-revolute-spherical layout used by this
robot class: J1/J3/J5/J7 spin, J2/J4/J6 bend. Link lengths live in
``config/models/xmate_er_pro_7.yaml``.
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import numpy as np
import pinocchio as pin
import yaml

MODEL_PATH = Path(__file__).resolve().parents[2] / "config/models/xmate_er_pro_7.yaml"


def rotation_angle(rotation: np.ndarray) -> float:
    """Geodesic angle of an SO(3) matrix, in radians."""
    cosine = float(np.clip((np.trace(rotation) - 1.0) * 0.5, -1.0, 1.0))
    return float(np.arccos(cosine))


@dataclass(frozen=True)
class StreamLimits:
    """1 kHz joint-stream limits shared by the Python mirror and the C++ core."""

    max_velocity: np.ndarray
    max_acceleration: np.ndarray
    max_jerk: np.ndarray
    position_min: np.ndarray
    position_max: np.ndarray
    reference_cutoff_hz: float
    deadband: float
    max_target_delta: float
    hold_timeout: float
    stop_timeout: float
    max_source_age: float
    future_tolerance: float

    @property
    def dof(self) -> int:
        return int(self.max_velocity.size)


@dataclass(frozen=True)
class IKGains:
    damping: float
    damping_max: float
    singular_sigma: float
    posture_gain: float
    limit_gain: float
    singularity_gain: float
    null_max_speed: float
    limit_margin: float
    max_translation_speed: float
    max_angular_speed: float


class XmateErPro7:
    """Host kinematics for xMate ER Pro 7. FK, Jacobian and frame error only."""

    def __init__(self, path: Path = MODEL_PATH) -> None:
        with Path(path).open("r", encoding="utf-8") as stream:
            raw = yaml.safe_load(stream)
        if not isinstance(raw, dict):
            raise ValueError(f"invalid robot model: {path}")
        self.path = Path(path)
        self.name = str(raw["name"])
        self.d_bs = float(raw["d_bs"])
        self.d_se = float(raw["d_se"])
        self.d_ew = float(raw["d_ew"])
        self.d_wf = float(raw["d_wf"])
        self.singular_joints = tuple(int(index) for index in raw["singular_joints"])
        lower = np.deg2rad(np.asarray(raw["lower_deg"], dtype=np.float64))
        upper = np.deg2rad(np.asarray(raw["upper_deg"], dtype=np.float64))
        self.neutral = np.asarray(raw["neutral_position"], dtype=np.float64)
        self.position_min = lower
        self.position_max = upper
        self.max_velocity = np.asarray(raw["max_velocity"], dtype=np.float64)
        self.max_acceleration = np.asarray(raw["max_acceleration"], dtype=np.float64)
        self.max_jerk = np.asarray(raw["max_jerk"], dtype=np.float64)
        stream = raw["stream"]
        ik = raw["ik"]
        arrays = (
            self.neutral,
            self.position_min,
            self.position_max,
            self.max_velocity,
            self.max_acceleration,
            self.max_jerk,
        )
        if any(value.shape != (7,) for value in arrays):
            raise ValueError("xMate ER Pro 7 model arrays must have length 7")
        self.stream_limits = StreamLimits(
            max_velocity=self.max_velocity.copy(),
            max_acceleration=self.max_acceleration.copy(),
            max_jerk=self.max_jerk.copy(),
            position_min=self.position_min.copy(),
            position_max=self.position_max.copy(),
            reference_cutoff_hz=float(stream["reference_cutoff_hz"]),
            deadband=float(stream["deadband"]),
            max_target_delta=float(stream["max_target_delta"]),
            hold_timeout=float(stream["hold_timeout"]),
            stop_timeout=float(stream["stop_timeout"]),
            max_source_age=float(stream["max_source_age"]),
            future_tolerance=float(stream["future_tolerance"]),
        )
        self.ik_gains = IKGains(
            damping=float(ik["damping"]),
            damping_max=float(ik["damping_max"]),
            singular_sigma=float(ik["singular_sigma"]),
            posture_gain=float(ik["posture_gain"]),
            limit_gain=float(ik["limit_gain"]),
            singularity_gain=float(ik["singularity_gain"]),
            null_max_speed=float(ik["null_max_speed"]),
            limit_margin=float(ik["limit_margin"]),
            max_translation_speed=float(ik["max_translation_speed"]),
            max_angular_speed=float(ik["max_angular_speed"]),
        )
        self.model, self.tcp_frame = self._build()
        self.data = self.model.createData()

    @property
    def dof(self) -> int:
        return 7

    def _build(self) -> tuple[pin.Model, int]:
        model = pin.Model()

        def xyz(x: float, y: float, z: float) -> pin.SE3:
            return pin.SE3(np.eye(3), np.array([x, y, z], dtype=np.float64))

        parent = 0
        parent = model.addJoint(parent, pin.JointModelRZ(), xyz(0.0, 0.0, self.d_bs), "joint1")
        parent = model.addJoint(parent, pin.JointModelRY(), pin.SE3.Identity(), "joint2")
        parent = model.addJoint(parent, pin.JointModelRZ(), pin.SE3.Identity(), "joint3")
        parent = model.addJoint(parent, pin.JointModelRY(), xyz(0.0, 0.0, self.d_se), "joint4")
        parent = model.addJoint(parent, pin.JointModelRZ(), pin.SE3.Identity(), "joint5")
        parent = model.addJoint(parent, pin.JointModelRY(), xyz(0.0, 0.0, self.d_ew), "joint6")
        parent = model.addJoint(parent, pin.JointModelRZ(), pin.SE3.Identity(), "joint7")
        frame = pin.Frame(
            "tcp",
            parent,
            0,
            xyz(0.0, 0.0, self.d_wf),
            pin.FrameType.OP_FRAME,
        )
        frame_id = model.addFrame(frame)
        model.lowerPositionLimit[:] = self.position_min
        model.upperPositionLimit[:] = self.position_max
        model.velocityLimit[:] = self.max_velocity
        return model, int(frame_id)

    def _as_q(self, joints: np.ndarray) -> np.ndarray:
        q = np.asarray(joints, dtype=np.float64).reshape(-1)
        if q.shape != (self.dof,) or not np.all(np.isfinite(q)):
            raise ValueError("joint vector must contain 7 finite radians")
        return q

    def clip(self, joints: np.ndarray) -> np.ndarray:
        return np.clip(self._as_q(joints), self.position_min, self.position_max)

    def fk(self, joints: np.ndarray) -> np.ndarray:
        """base_T_tcp as a row-major 4x4 matrix."""
        q = self._as_q(joints)
        pin.forwardKinematics(self.model, self.data, q)
        pin.updateFramePlacements(self.model, self.data)
        placement = self.data.oMf[self.tcp_frame]
        pose = np.eye(4, dtype=np.float64)
        pose[:3, :3] = np.array(placement.rotation, dtype=np.float64)
        pose[:3, 3] = np.array(placement.translation, dtype=np.float64)
        return pose

    def jacobian(self, joints: np.ndarray) -> np.ndarray:
        """6xn Jacobian of the TCP in the current TCP frame. Rows are [v, w]."""
        q = self._as_q(joints)
        pin.forwardKinematics(self.model, self.data, q)
        pin.computeJointJacobians(self.model, self.data, q)
        pin.updateFramePlacements(self.model, self.data)
        return np.array(
            pin.getFrameJacobian(
                self.model, self.data, self.tcp_frame, pin.ReferenceFrame.LOCAL
            ),
            dtype=np.float64,
        )

    def local_error(self, joints: np.ndarray, base_T_tcp: np.ndarray) -> np.ndarray:
        """SE(3) error of ``base_T_tcp`` expressed in the current TCP frame."""
        q = self._as_q(joints)
        target = np.asarray(base_T_tcp, dtype=np.float64)
        if target.shape != (4, 4) or not np.all(np.isfinite(target)):
            raise ValueError("target pose must be a finite 4x4 matrix")
        pin.forwardKinematics(self.model, self.data, q)
        pin.updateFramePlacements(self.model, self.data)
        desired = pin.SE3(target[:3, :3], target[:3, 3])
        return np.array(
            pin.log6(self.data.oMf[self.tcp_frame].actInv(desired)).vector,
            dtype=np.float64,
        )

    def smallest_singular_value(self, joints: np.ndarray) -> float:
        singular = np.linalg.svd(self.jacobian(joints), compute_uv=False)
        return float(singular[-1])

    def pose_mismatch(
        self, joints: np.ndarray, measured_base_T_tcp: np.ndarray
    ) -> tuple[float, float]:
        """Translation (m) and rotation (rad) between host FK and a measured pose."""
        predicted = self.fk(joints)
        measured = np.asarray(measured_base_T_tcp, dtype=np.float64)
        translation = float(np.linalg.norm(predicted[:3, 3] - measured[:3, 3]))
        rotation = rotation_angle(predicted[:3, :3].T @ measured[:3, :3])
        return translation, rotation

    def require_controller_match(
        self,
        joints: np.ndarray,
        measured_base_T_tcp: np.ndarray,
        translation_tolerance: float = 0.020,
        rotation_tolerance: float = 0.10,
    ) -> tuple[float, float]:
        translation, rotation = self.pose_mismatch(joints, measured_base_T_tcp)
        if translation > translation_tolerance or rotation > rotation_tolerance:
            raise RuntimeError(
                "Pinocchio FK does not match the controller TCP at the current "
                f"joints: translation={translation * 1000.0:.1f} mm "
                f"(limit {translation_tolerance * 1000.0:.1f} mm), "
                f"rotation={np.rad2deg(rotation):.2f} deg "
                f"(limit {np.rad2deg(rotation_tolerance):.2f} deg). "
                "Edit config/models/xmate_er_pro_7.yaml before joint "
                "streaming. The Cartesian realtime route does not use this model."
            )
        return translation, rotation
