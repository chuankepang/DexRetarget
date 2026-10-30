"""Model preflight, shared bounds, redundancy and full mapped wrist rollout."""
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock

import numpy as np
import pytest

from anydexretarget.geometry import rotation_vector_to_matrix
from anydexretarget.teleop import RelativePoseMapper, make_transform
from dexretarget.config import load_yaml_mapping
from dexretarget.teleop.devices.rokae import RokaeXCoreDriver
from rokae.control.differential_ik import DifferentialIK
from rokae.control.model import XmateErPro7, rotation_angle
from native_support import HAS_NATIVE

ROOT = Path(__file__).resolve().parents[1]
CUP = np.asarray(load_yaml_mapping(ROOT/'config/tasks/cup.yaml')['home']['arm_joints'])


def test_virtual_fk_catches_model_error_hidden_at_the_current_pose():
    robot = XmateErPro7()
    oracle = XmateErPro7()
    queries = []

    def provider(q):
        queries.append(q.copy())
        pose = oracle.fk(q)
        if np.max(np.abs(q-CUP)) > 0.1:
            pose[0, 3] += .03
        return pose

    robot.require_controller_match(CUP, provider(CUP))
    with pytest.raises(RuntimeError, match="FK does not match"):
        robot.validate_controller_fk(provider, CUP)
    assert len(queries) > 1


@pytest.mark.parametrize('bad', [np.full((4, 4), np.nan), np.zeros((4, 4)), np.diag([-1., 1., 1., 1.])])
def test_invalid_official_fk_never_passes_validation(bad):
    with pytest.raises(ValueError, match="rigid"):
        XmateErPro7().validate_controller_fk(lambda _: bad, CUP)


def test_exact_kernel_cannot_move_a_well_conditioned_stationary_cup():
    robot = XmateErPro7()
    solver = DifferentialIK(robot, CUP)
    target = robot.fk(CUP)
    for _ in range(500):
        solver.step(target, .02)
    np.testing.assert_allclose(solver.q, CUP, atol=1e-12)
    reset = CUP + np.array([.1, .01, -.1, .01, .01, 0, .1])
    solver.reset(reset)
    np.testing.assert_array_equal(solver.posture_reference, reset)
    reset[:] = 0
    assert np.linalg.norm(solver.posture_reference) > 1


@pytest.mark.parametrize('period', [.01, 1/90, .02])
def test_nullspace_posture_improvement_preserves_the_tcp_at_different_input_rates(period):
    import dataclasses
    robot = XmateErPro7()
    robot.ik_gains = dataclasses.replace(robot.ik_gains, posture_reference='model')
    solver = DifferentialIK(robot, CUP)
    target = robot.fk(CUP)
    for _ in range(round(4/period)):
        solver.step(target, period)
    position_error, angle_error = robot.pose_mismatch(solver.q, target)
    assert position_error < 1e-5
    assert angle_error < 1e-5
    assert np.max(np.abs(solver.q-CUP)) > .01


def test_a_joint_at_its_effective_boundary_can_move_back_inward():
    robot = XmateErPro7()
    controller = np.column_stack((robot.position_min, robot.position_max))
    controller[6] = [-.2, .2]
    robot.intersect_position_limits(controller)
    start = CUP.copy()
    start[6] = .2
    destination = start.copy()
    destination[6] = .05
    solver = DifferentialIK(robot, start)
    target = robot.fk(destination)
    for _ in range(300):
        previous = solver.q.copy()
        command = solver.step(target, .02)
        assert np.all(np.abs(command-previous) <= robot.max_velocity*.02+1e-12)
        assert np.all(command <= robot.position_max+1e-12)
        assert np.all(command >= robot.position_min-1e-12)
    assert solver.q[6] < .19
    assert robot.pose_mismatch(solver.q, target)[1] < 1e-3


def test_one_effective_limit_for_model_ik_and_stream():
    robot = XmateErPro7()
    controller = np.column_stack((robot.position_min+.05, robot.position_max-.05))
    robot.intersect_position_limits(controller)
    np.testing.assert_array_equal(robot.stream_limits.position_min, controller[:, 0])
    np.testing.assert_array_equal(robot.model.upperPositionLimit, controller[:, 1])
    with pytest.raises(ValueError, match="effective"):
        DifferentialIK(robot, controller[:, 1]+.001)
    old_lower = robot.position_min.copy()
    with pytest.raises(ValueError, match="overlap"):
        robot.intersect_position_limits(np.tile([10, 11], (7, 1)))
    np.testing.assert_array_equal(robot.position_min, old_lower)


def fake_sdk_driver(*, enabled=True, bad_fk=False):
    model, oracle = XmateErPro7(), XmateErPro7()
    soft = np.column_stack((model.position_min+.05, model.position_max-.05))
    native = Mock()
    native.get_joint_positions.return_value = CUP.tolist()
    native.get_tcp_pose.return_value = oracle.fk(CUP)
    native.get_soft_limits.return_value = enabled, soft

    def fk(q):
        pose = oracle.fk(q)
        if bad_fk and not np.allclose(q, CUP):
            pose[0, 3] += .04
        return pose

    native.calculate_fk_in_base.side_effect = fk
    driver = RokaeXCoreDriver('127.0.0.1', '127.0.0.1')
    driver._driver, driver._connected = native, True
    driver._binding = SimpleNamespace(JointRealtimeConfig=lambda: SimpleNamespace(validate=lambda: None))
    return driver, native, model, soft


def test_preflight_wires_effective_boundaries_into_native_configuration():
    driver, native, model, soft = fake_sdk_driver()
    driver.start(power_on=False, control_mode='pinocchio', kinematics=model)
    cfg, power_on = native.start_realtime_joint.call_args.args
    assert not power_on
    np.testing.assert_array_equal(cfg.position_min, model.position_min)
    np.testing.assert_array_equal(cfg.position_max, soft[:, 1])
    assert native.calculate_fk_in_base.call_count == 16
    calls = [c[0] for c in native.method_calls]
    assert calls.index('start_realtime_joint') > max(i for i, name in enumerate(calls) if name == 'calculate_fk_in_base')


@pytest.mark.parametrize('options', [{'enabled': False}, {'bad_fk': True}])
def test_failed_model_or_limit_preflight_never_starts_native_control(options):
    driver, native, model, _ = fake_sdk_driver(**options)
    with pytest.raises(RuntimeError):
        driver.start(control_mode='pinocchio', kinematics=model)
    native.start_realtime_joint.assert_not_called()
    assert not driver._started


def test_motion_during_fk_queries_is_rejected_before_native_start():
    driver, native, model, _ = fake_sdk_driver()
    native.get_joint_positions.side_effect = [CUP.tolist(), (CUP+.003).tolist()]
    with pytest.raises(RuntimeError, match='moved during'):
        driver.start(control_mode='pinocchio', kinematics=model)
    native.start_realtime_joint.assert_not_called()


@pytest.mark.skipif(not HAS_NATIVE, reason='matching native binding unavailable; not validated')
def test_native_fk_frame_conversion_with_rotated_translated_reference():
    from rokae.python import _rokae_cpp
    base = make_transform([.2, -.3, .4], rotation_vector_to_matrix([.2, .4, -.3]))
    ref = make_transform([-.1, .5, .2], rotation_vector_to_matrix([-.1, -.3, .5]))
    expected = make_transform([.4, -.1, .6], rotation_vector_to_matrix([.5, -.4, .1]))
    ref_T_end = np.linalg.inv(ref) @ base @ expected
    np.testing.assert_allclose(_rokae_cpp.fk_pose_in_base(ref_T_end, base, ref), expected, atol=1e-12)
    with pytest.raises(ValueError):
        _rokae_cpp.fk_pose_in_base(np.zeros((4, 4)), base, ref)


@pytest.mark.skipif(not HAS_NATIVE, reason='matching native binding unavailable; not validated')
@pytest.mark.parametrize('sign', [-1, 1])
def test_tool_longitudinal_roll_mapping_through_ik_and_native_stream(sign):
    from rokae.python import _rokae_cpp
    from rokae.control.smoothing import JointStreamConfig
    from test_rokae_differential_ik import cpp_config

    robot = XmateErPro7()
    soft = np.deg2rad(np.array([[-165, 165], [-115, 115], [-165, 165],
                              [-115, 115], [-165, 165], [-115, 115], [-355, 355]]))
    robot.intersect_position_limits(soft)
    config = load_yaml_mapping(ROOT/'config/robots/rokae.yaml')['teleoperation']
    mapper = RelativePoseMapper(
        np.asarray(config['quest_to_robot_rotation']).reshape(3, 3), np.eye(4),
        translation_scale=config['translation_scale'], rotation_scale=config['rotation_scale'],
        rotation_mapping=config['rotation_mapping'],
        wrist_to_tool_rotation=np.asarray(config['wrist_to_tool_rotation']).reshape(3, 3))
    wrist_ref = make_transform([.2, .3, .4], rotation_vector_to_matrix([.3, -.2, .5]))
    robot_ref = robot.fk(CUP)
    mapper.set_reference(wrist_ref, robot_ref)
    solver = DifferentialIK(robot, CUP)
    core = _rokae_cpp.JointRealtimeCore(cpp_config(JointStreamConfig.from_limits(robot.stream_limits)))
    core.reset(CUP.tolist(), 0.)
    assert core.publish(CUP.tolist(), 0., 0, 0.)
    for i in range(1, 12001):
        stamp = i*.001
        if i % 20 == 0:
            wrist = wrist_ref.copy()
            wrist[:3, :3] = wrist_ref[:3, :3] @ rotation_vector_to_matrix([sign*np.pi/2*min(1, stamp/4), 0, 0])
            target = mapper.update(wrist).base_target
            assert core.publish(solver.step(target, .02).tolist(), stamp, i, stamp)
        command = np.asarray(core.step(stamp))
        assert np.all(command >= robot.position_min-1e-12)
        assert np.all(command <= robot.position_max+1e-12)
    expected = robot_ref.copy()
    expected[:3, :3] = robot_ref[:3, :3] @ rotation_vector_to_matrix([0, 0, sign*np.pi/2])
    reached = robot.fk(command)
    assert np.linalg.norm(reached[:3, 3]-expected[:3, 3]) < .0005
    assert rotation_angle(reached[:3, :3].T @ expected[:3, :3]) < np.deg2rad(.1)
