"""Offline wrist-axis mapping and fitted-model IK audit; never connects to a robot."""
from pathlib import Path
import argparse
import json
import sys

ROOT = Path(__file__).resolve().parents[2]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))


def audit(angle_deg=90.0, include_ik=False):
    import numpy as np
    from dexretarget.config import load_yaml_mapping
    from anydexretarget.teleop import RelativePoseMapper, make_transform
    from anydexretarget.geometry import rotation_vector_to_matrix, matrix_to_rotation_vector
    from rokae.control.model import XmateErPro7, rotation_angle
    from rokae.control.differential_ik import DifferentialIK

    if not np.isfinite(angle_deg) or not 0 < angle_deg <= 120:
        raise ValueError("angle must be in (0,120] degrees")
    config = load_yaml_mapping(ROOT/'config/robots/rokae.yaml')['teleoperation']
    home = np.asarray(load_yaml_mapping(ROOT/'config/tasks/cup.yaml')['home']['arm_joints'])
    model = XmateErPro7()
    robot_ref = model.fk(home)
    wrist_ref = make_transform([.2, .3, .4], rotation_vector_to_matrix([.3, -.2, .5]))
    angle = np.deg2rad(angle_deg)
    mapping = {}
    for mode in ('spatial', 'tool_relative'):
        mapper = RelativePoseMapper(
            np.asarray(config['quest_to_robot_rotation']).reshape(3, 3), np.eye(4),
            rotation_scale=float(config['rotation_scale']), rotation_mapping=mode,
            wrist_to_tool_rotation=np.asarray(config['wrist_to_tool_rotation']).reshape(3, 3),
        )
        mapper.set_reference(wrist_ref, robot_ref)
        axes = {}
        for index, name in enumerate('XYZ'):
            wrist = wrist_ref.copy()
            wrist[:3, :3] = wrist_ref[:3, :3] @ rotation_vector_to_matrix(np.eye(3)[index]*angle)
            result = mapper.update(wrist).base_target
            axes[name] = np.rad2deg(matrix_to_rotation_vector(robot_ref[:3, :3].T @ result[:3, :3])).tolist()
        mapping[mode] = {'wrist_body_axis_to_tool_rotvec_deg': axes}
    report = {
        'scope': 'Synthetic RH wrist input and fitted host model only; not physical tracking, controller IK or hardware reachability verification',
        'configured_mode': config['rotation_mapping'], 'rotation_scale': config['rotation_scale'],
        'mapping': mapping,
    }
    if include_ik:
        cases = {}
        for axis in (0, 2):
            for sign in (-1, 1):
                solver = DifferentialIK(model, home)
                # 4-second ramp, then 8-second hold, only in the fitted model.
                for i in range(600):
                    target = robot_ref.copy()
                    target[:3, :3] = robot_ref[:3, :3] @ rotation_vector_to_matrix(
                        np.eye(3)[axis]*sign*angle*min(1, (i+1)*.02/4))
                    solver.step(target, .02)
                reached = model.fk(solver.q)
                cases[f'tool_{"XYZ"[axis]}_{sign*angle_deg:g}_deg'] = {
                    'translation_error_mm': float(np.linalg.norm(reached[:3, 3]-target[:3, 3])*1000),
                    'rotation_error_deg': float(np.rad2deg(rotation_angle(reached[:3, :3].T @ target[:3, :3]))),
                    'final_joint_deg': np.rad2deg(solver.q).tolist(),
                }
        report['fitted_model_ik'] = cases
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--angle-deg', type=float, default=90)
    parser.add_argument('--include-ik', action='store_true')
    args = parser.parse_args()
    print(json.dumps(audit(args.angle_deg, args.include_ik), indent=2))


if __name__ == '__main__':
    main()
