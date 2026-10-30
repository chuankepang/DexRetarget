"""Resolve Cartesian reference configuration without devices, cameras or recording."""
from dataclasses import asdict
from anydexretarget.teleop import RealtimeCartesianConfig
from dexretarget.config import load_yaml_mapping


def _config_value(argument, section, key):
    return section[key] if argument is None else argument


def configured_realtime(args, robot_config, safety_config):
    """Apply the existing CLI > YAML precedence in one explicit field mapping."""
    names = (
        "translation_cutoff_hz", "rotation_cutoff_hz", "translation_deadband", "rotation_deadband",
        "max_translation_speed", "max_angular_speed", "max_translation_acceleration", "max_angular_acceleration",
        "max_translation_jerk", "max_angular_jerk", "max_target_translation_delta", "max_target_rotation_delta",
    )
    values = {name: float(_config_value(getattr(args, 'arm_' + name), safety_config, name)) for name in names}
    for name in ['workspace_min', 'workspace_max']:
        values[name] = tuple(_config_value(getattr(args, 'arm_' + name), safety_config, name))
    values.update(
        response_hz=float(safety_config.get('response_hz', 0.0)),
        control_hz=float(robot_config['control_hz']),
        hold_timeout=float(_config_value(args.arm_tracking_timeout, safety_config, 'hold_timeout')),
        stop_timeout=float(_config_value(args.arm_stop_timeout, safety_config, 'stop_timeout')),
        max_source_age=float(safety_config['max_source_age']), future_tolerance=float(safety_config['future_tolerance']),
    )
    return RealtimeCartesianConfig(**values)


def describe_arm_config(args):
    """Resolve the same controller/CLI configuration without constructing hardware."""
    from dexretarget.config import project_path
    from rokae.control.controllers.spec import resolve_controller, effective_cartesian_safety

    path = project_path(args.arm_config)
    config = load_yaml_mapping(path)
    spec = resolve_controller(config, cli=args.arm_control)
    limits = configured_realtime(args, config["robot"], effective_cartesian_safety(config, spec))
    from anydexretarget.teleop import RelativePoseMapper
    import numpy as np
    teleop = config["teleoperation"]
    mapper = RelativePoseMapper(
        np.asarray(teleop["quest_to_robot_rotation"]).reshape(3, 3), np.eye(4),
        rotation_scale=args.arm_rotation_scale if args.arm_rotation_scale is not None else teleop["rotation_scale"],
        rotation_mapping=teleop.get("rotation_mapping", "spatial"),
        wrist_to_tool_rotation=np.asarray(teleop.get("wrist_to_tool_rotation", np.eye(3))).reshape(3, 3),
    )
    from dexretarget.teleop.tracking import TrackingHold
    tracking = TrackingHold(config.get("teleoperation", {}).get("tracking_resume_delay_s", 0.01))
    return {
        "tracking_resume_delay_s": tracking.resume_delay,
        "orientation_mapping": {
            "mode": mapper.rotation_mapping,
            "scale": mapper.rotation_scale,
            "wrist_to_tool_rotation": mapper.wrist_to_tool_rotation.tolist(),
            "axis_convention": "right-handed project wrist reference -> TCP reference; verify mounting physically",
        },
        "read_only_plan": True,
        "controller_readback_performed": False,
        "robot_config": str(path),
        "route": spec.route,
        "command_semantics": "equilibrium_pose" if spec.kind == "impedance" else "position_reference",
        "impedance": {
            "force_frame": spec.force_frame,
            "stiffness_N_m_and_Nm_rad": spec.stiffness.tolist(),
            "desired_wrench_N_and_Nm": spec.desired_wrench.tolist(),
            "damping": "controller_internal; SDK 0.3.4 has no Cartesian damping setter",
            "reference_limit_m_rad": [spec.max_equilibrium_translation, spec.max_equilibrium_rotation],
            "nominal_spring_force_at_translation_limit_N": (spec.stiffness[:3] * spec.max_equilibrium_translation).tolist(),
            "apply_payload": spec.apply_load_on_impedance_start,
            "mass_kg": spec.expected_load_mass,
            "cog_m": list(spec.expected_load_cog),
        },
        "cartesian_reference_generator": asdict(limits),
        "precedence": "CLI > impedance.reference_motion (impedance only) > safety",
        "task_home": (args.task_profile or {}).get("home"),
        "joint_limits_owner": "config/models/xmate_er_pro_7.yaml (Pinocchio only); controller soft limits are separate",
        "controller_torque_protection": "unchanged; not overridden by this project",
    }
