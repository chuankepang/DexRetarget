"""Offline same-model IK audit; never connects to hardware or validates calibration."""
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[2]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from dexretarget.bootstrap import _ensure_conda_cpp_runtime


def audit():
    import time
    import dataclasses
    import platform
    import numpy as np
    import pinocchio as pin
    from dexretarget.config import load_yaml_mapping
    from rokae.control.model import XmateErPro7
    from rokae.control.differential_ik import DifferentialIK

    robot = XmateErPro7()
    baseline_gains = dataclasses.asdict(robot.ik_gains)
    home = robot.neutral.copy()
    cup_home = np.asarray(load_yaml_mapping(
        ROOT / 'config/tasks/cup.yaml')['home']['arm_joints'], dtype=float)
    cases = {
        'home_hold': (home, [0, 0, 0]),
        'offset_hold': (home + np.array([.2,.1,.2,-.15,.1,.1,-.1]), [0, 0, 0]),
        'home_translation': (home, [.02, -.01, .015]),
        'near_straight_translation': (np.array([0,.02,0,.02,0,.02,0]), [.01,0,-.005]),
        'cup_hold': (cup_home, [0, 0, 0]),
        'cup_translation': (cup_home, [.02, -.01, .015]),
        'cup_hold_task_posture': (cup_home, [0, 0, 0]),
        'cup_hold_model_posture': (cup_home, [0, 0, 0]),
        'cup_hold_no_secondary': (cup_home, [0, 0, 0]),
    }
    output = {}
    for name, (start, displacement) in cases.items():
        # Independent in-memory models: these comparisons never change YAML or
        # the live solver. Disabling secondary tasks is an attribution test,
        # not a deployable replacement for limit/singularity management.
        robot = XmateErPro7()
        if name == 'cup_hold_task_posture':
            robot.neutral = cup_home.copy()
        elif name == 'cup_hold_model_posture':
            robot.ik_gains = dataclasses.replace(robot.ik_gains, posture_reference='model')
        elif name == 'cup_hold_no_secondary':
            robot.ik_gains = dataclasses.replace(
                robot.ik_gains, posture_gain=0., limit_gain=0., singularity_gain=0.)
        solver = DifferentialIK(robot, start)
        target = robot.fk(start)
        target[:3,3] += displacement
        jacobian = robot.jacobian(start)
        u, singular, vh = np.linalg.svd(jacobian, full_matrices=True)
        damping = solver.gains.damping
        if singular[-1] < solver.gains.singular_sigma:
            damping += (solver.gains.damping_max-damping)*(1-singular[-1]/solver.gains.singular_sigma)**2
        damped_n = np.eye(7) - jacobian.T @ np.linalg.solve(
            jacobian @ jacobian.T + damping*damping*np.eye(6), jacobian)
        # Algebra-only comparison, not a proposed live solver or trajectory.
        rank = int(np.sum(singular > 1e-10))
        basis = vh[rank:].T
        exact_n = basis @ basis.T
        times, errors = [], []
        max_step = 0.
        max_joint_excursion = 0.
        for _ in range(500):
            previous = solver.q.copy()
            before = time.perf_counter()
            solver.step(target, .02)
            times.append(time.perf_counter()-before)
            max_step = max(max_step, float(np.max(np.abs(solver.q-previous))))
            max_joint_excursion = max(max_joint_excursion, float(np.max(np.abs(solver.q-start))))
            errors.append(robot.pose_mismatch(solver.q, target))
        translation, rotation = errors[-1]
        output[name] = {
            'dt_s': .02, 'steps': 500, 'sigma_min_start': float(singular[-1]),
            'damping_start': damping,
            'damped_JN_frobenius': float(np.linalg.norm(jacobian @ damped_n)),
            'exact_JN_frobenius': float(np.linalg.norm(jacobian @ exact_n)),
            'final_translation_error_mm': translation*1000,
            'final_rotation_error_deg': float(np.rad2deg(rotation)),
            'max_translation_error_mm': max(v[0] for v in errors)*1000,
            'joint_displacement_norm_rad': float(np.linalg.norm(solver.q-start)),
            'max_joint_step_rad': max_step,
            'peak_joint_excursion_deg': float(np.rad2deg(max_joint_excursion)),
            'final_joint_delta_deg': np.rad2deg(solver.q-start).tolist(),
            'neutral_position_rad': robot.neutral.tolist(),
            'posture_reference_mode': solver.gains.posture_reference,
            'secondary_gains': {key: getattr(solver.gains, key) for key in
                                ('posture_gain', 'limit_gain', 'singularity_gain')},
            'step_us_p50_p95_p99': (np.percentile(times,[50,95,99])*1e6).tolist(),
        }
    return {'scope': 'offline same fitted model; excludes RT filtering, robot tracking, calibration and official IK',
            'environment': {'python': platform.python_version(), 'pinocchio': pin.__version__,
                            'numpy': np.__version__, 'machine': platform.machine()},
            'model': str(robot.path.relative_to(ROOT)), 'ik_gains': baseline_gains,
            'case_notes': {'home_*': 'Legacy name: model neutral, not task home',
                           'cup_hold_task_posture': 'Offline posture-reference change only; bend/limit terms remain',
                           'cup_hold_model_posture': 'Offline comparison with legacy fixed posture intent; current strict projector',
                           'cup_hold_no_secondary': 'Offline attribution only; not suitable for live control'},
            'cases': output}


def main():
    import argparse
    import json
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path)
    args = parser.parse_args()
    report = audit()
    text = json.dumps(report, indent=2, allow_nan=False)
    if args.output:
        args.output.write_text(text+'\n')
    print(text)


if __name__ == '__main__':
    _ensure_conda_cpp_runtime()
    main()
