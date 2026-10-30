"""Nonaligned wrist/tool references and +/-90-degree axial rotation, no hardware."""
from pathlib import Path
import numpy as np
import pytest

from anydexretarget.teleop import RelativePoseMapper, make_transform
from anydexretarget.geometry import rotation_vector_to_matrix as exp, matrix_to_rotation_vector as log
from dexretarget.config import load_yaml_mapping
from rokae.control.model import XmateErPro7, rotation_angle
from rokae.control.differential_ik import DifferentialIK
from native_support import HAS_NATIVE

ROOT = Path(__file__).resolve().parents[1]
AXES = np.array([[0., 0., -1.], [0., 1., 0.], [1., 0., 0.]])


@pytest.mark.parametrize('angle', [-90, -20, 20, 90])
@pytest.mark.parametrize('axis', [0, 1, 2])
def test_wrist_body_rotation_maps_to_reference_tool_axes(angle, axis):
    base = make_transform([.1, .2, .3], exp([.2, .3, -.1]))
    robot = make_transform([.5, 0, .6], exp([.6, -.8, .4]))
    wrist = make_transform([.1, .4, .3], exp([.4, .7, -.5]))
    mapper = RelativePoseMapper(np.eye(3), base, rotation_mapping='tool_relative', wrist_to_tool_rotation=AXES)
    mapper.set_reference(wrist, robot)
    current = wrist.copy()
    delta = np.eye(3)[axis]*np.deg2rad(angle)
    current[:3, :3] = wrist[:3, :3] @ exp(delta)
    current[:3, 3] += [.01, .02, -.01]
    result = mapper.update(current)
    np.testing.assert_allclose(result.base_target[:3, :3], robot[:3, :3] @ exp(AXES @ delta), atol=1e-12)
    np.testing.assert_allclose(base[:3, :3] @ (result.base_target[:3, 3]-robot[:3, 3]), [.01, .02, -.01], atol=1e-12)
    np.testing.assert_allclose(result.mapped_relative[:3, :3] @ (base @ robot)[:3, :3], result.world_target[:3, :3], atol=1e-12)
    mapper.recenter(current, result.base_target)
    np.testing.assert_allclose(mapper.update(current).base_target, result.base_target, atol=1e-12)


def test_old_gain_and_spatial_axis_explain_underrotation():
    robot = make_transform(rotation=exp([0, .9, 0]))
    wrist = np.eye(4)
    current = make_transform(rotation=exp([np.pi/2, 0, 0]))
    mapper = RelativePoseMapper(np.eye(3), np.eye(4), rotation_scale=.8)
    mapper.set_reference(wrist, robot)
    local = log(robot[:3, :3].T @ mapper.update(current).base_target[:3, :3])
    assert np.rad2deg(np.linalg.norm(local)) == pytest.approx(72)
    assert abs(local[0]) > .1 and abs(local[2]) > .1


@pytest.mark.parametrize('sign', [-1, 1])
def test_configured_wrist_roll_reaches_cup_tool_z_in_fitted_model(sign):
    config = load_yaml_mapping(ROOT/'config/robots/rokae.yaml')['teleoperation']
    assert config['rotation_scale'] == 1
    model = XmateErPro7()
    start = np.asarray(load_yaml_mapping(ROOT/'config/tasks/cup.yaml')['home']['arm_joints'])
    reference = model.fk(start)
    wrist_ref = make_transform(rotation=exp([.2, -.3, .4]))
    mapper = RelativePoseMapper(np.eye(3), np.eye(4), rotation_scale=config['rotation_scale'],
        rotation_mapping=config['rotation_mapping'], wrist_to_tool_rotation=np.asarray(config['wrist_to_tool_rotation']).reshape(3, 3))
    mapper.set_reference(wrist_ref, reference)
    solver = DifferentialIK(model, start)
    for i in range(600):
        wrist = wrist_ref.copy()
        wrist[:3, :3] = wrist_ref[:3, :3] @ exp([sign*np.pi/2*min(1, (i+1)*.02/4), 0, 0])
        target = mapper.update(wrist).base_target
        solver.step(target, .02)
    expected = reference[:3, :3] @ exp([0, 0, sign*np.pi/2])
    np.testing.assert_allclose(target[:3, :3], expected, atol=1e-12)
    reached = model.fk(solver.q)
    assert np.rad2deg(rotation_angle(reached[:3, :3].T @ expected)) < .5
    assert np.linalg.norm(reached[:3, 3]-reference[:3, 3]) < .002
    # A separate exact witness: last-axis rotation achieves this TCP-Z goal.
    witness = start.copy()
    witness[6] += sign*np.pi/2
    np.testing.assert_allclose(model.fk(witness)[:3, :3], expected, atol=1e-12)


def test_invalid_rotation_mapping_and_reflection_rejected():
    with pytest.raises(ValueError, match='rotation_mapping'):
        RelativePoseMapper(np.eye(3), np.eye(4), rotation_mapping='euler')
    with pytest.raises(ValueError):
        RelativePoseMapper(np.eye(3), np.eye(4), wrist_to_tool_rotation=np.diag([1, 1, -1]))


@pytest.mark.skipif(not HAS_NATIVE, reason='Native joint reference core unavailable; not validated')
@pytest.mark.parametrize('sign', [-1, 1])
def test_roll_through_native_joint_reference_stream(sign):
    from rokae.python import _rokae_cpp
    from rokae.control.smoothing import JointStreamConfig
    from test_rokae_differential_ik import cpp_config
    model = XmateErPro7()
    start = np.asarray(load_yaml_mapping(ROOT/'config/tasks/cup.yaml')['home']['arm_joints'])
    reference = model.fk(start)
    mapper = RelativePoseMapper(np.eye(3), np.eye(4), rotation_mapping='tool_relative', wrist_to_tool_rotation=AXES)
    mapper.set_reference(np.eye(4), reference)
    solver = DifferentialIK(model, start)
    core = _rokae_cpp.JointRealtimeCore(cpp_config(JointStreamConfig.from_limits(model.stream_limits)))
    core.reset(start.tolist(), 0)
    previous = start.copy()
    for tick in range(1, 10001):
        time_s = tick*.001
        if tick % 20 == 0:
            wrist = make_transform(rotation=exp([sign*np.pi/2*min(1, time_s/4), 0, 0]))
            joints = solver.step(mapper.update(wrist).base_target, .02)
            assert core.publish(joints.tolist(), time_s, tick, time_s)
        command = np.asarray(core.step(time_s))
        assert np.max(np.abs(command-previous)/model.max_velocity) <= .0011
        previous = command
    expected = reference[:3, :3] @ exp([0, 0, sign*np.pi/2])
    reached = model.fk(command)
    assert np.rad2deg(rotation_angle(reached[:3, :3].T @ expected)) < 1
    assert np.linalg.norm(reached[:3, 3]-reference[:3, 3]) < .003
