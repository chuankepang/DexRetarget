"""Native reference convergence/limits, independent of robot dynamics."""
from dataclasses import asdict, replace

import numpy as np
import pytest

from anydexretarget.teleop import CartesianSetpointGenerator, rotation_vector_to_matrix
from dexretarget.app.cli import build_parser
from dexretarget.config import load_yaml_mapping, PROJECT_ROOT
from rokae.control.configuration import configured_realtime
from rokae.control.controllers.spec import resolve_controller, effective_cartesian_safety
from rokae.control.model import XmateErPro7
from rokae.control.smoothing import JointStreamConfig, JointStream
from native_support import HAS_NATIVE

pytestmark = pytest.mark.skipif(not HAS_NATIVE, reason='native bridge not built; not validated')


def cpp_config(cls, config):
    result = cls()
    for key, value in asdict(config).items():
        if hasattr(result, key):
            setattr(result, key, value.tolist() if isinstance(value, np.ndarray) else value)
    result.validate()
    return result


def cart_config(route):
    raw = load_yaml_mapping(PROJECT_ROOT/'config/robots/rokae.yaml')
    return configured_realtime(build_parser().parse_args([]), raw['robot'],
                               effective_cartesian_safety(raw, resolve_controller(raw, cli=route)))


@pytest.mark.parametrize('route', ['cartesian', 'impedance'])
@pytest.mark.parametrize('response', [0., 2.])
def test_cartesian_python_native_agree_and_converge_after_reversal(route, response):
    from rokae.python import _rokae_cpp as native
    cfg = replace(cart_config(route), response_hz=response)
    core = native.RealtimeCore(cpp_config(native.RealtimeConfig, cfg))
    mirror = CartesianSetpointGenerator(cfg)
    core.reset(np.eye(4), 0.)
    mirror.reset(np.eye(4), timestamp=0.)
    previous_v = np.zeros(3)
    previous_a = np.zeros(3)
    previous_angular_v = np.zeros(3)
    previous_angular_a = np.zeros(3)
    positions = []
    target = np.eye(4)
    for i in range(1, 8001):
        t = i*.001
        if i % 20 == 0:
            sign = 1 if t < 2. else -1
            target[:3, 3] = sign*.04*np.array([1., 2., 1.])/np.sqrt(6)
            target[:3, :3] = rotation_vector_to_matrix([0., 0., sign*.3])
            assert core.publish(target, t, i, t)
            assert mirror.publish_target(target, source_timestamp=t, sequence=i, receive_timestamp=t)
        actual = np.asarray(core.step(t))
        expected = mirror.step(timestamp=t)
        np.testing.assert_allclose(actual, expected.pose, atol=1e-10)
        if response > 0:
            v, a = expected.linear_velocity, expected.linear_acceleration
            assert np.linalg.norm(v) <= cfg.max_translation_speed + 1e-10
            assert np.linalg.norm(a) <= cfg.max_translation_acceleration + 1e-10
            assert np.linalg.norm((a-previous_a)/.001) <= cfg.max_translation_jerk + 1e-7
            np.testing.assert_allclose((v-previous_v)/.001, a, atol=1e-8)
            previous_v, previous_a = v.copy(), a.copy()
            v, a = expected.angular_velocity, expected.angular_acceleration
            assert np.linalg.norm(v) <= cfg.max_angular_speed + 1e-10
            assert np.linalg.norm(a) <= cfg.max_angular_acceleration + 1e-10
            assert np.linalg.norm((a-previous_angular_a)/.001) <= cfg.max_angular_jerk + 1e-7
            np.testing.assert_allclose((v-previous_angular_v)/.001, a, atol=1e-8)
            previous_angular_v, previous_angular_a = v.copy(), a.copy()
        positions.append(actual[:3, 3])
    if response > 0:
        assert np.linalg.norm(actual[:3, 3]-target[:3, 3]) < .0015
        assert np.max(np.ptp(np.array(positions[-1000:]), axis=0)) < 1e-5
        assert np.linalg.norm(actual[:3, :3]-target[:3, :3]) < .02


@pytest.mark.parametrize('response', [0., 2.])
def test_joint_python_native_agree_with_large_targets_and_hold(response):
    from rokae.python import _rokae_cpp as native
    robot = XmateErPro7()
    cfg = replace(JointStreamConfig.from_limits(robot.stream_limits), response_hz=response)
    core = native.JointRealtimeCore(cpp_config(native.JointRealtimeConfig, cfg))
    mirror = JointStream(cfg)
    core.reset(robot.neutral, 0.)
    mirror.reset(robot.neutral, 0.)
    previous_v, previous_a = np.zeros(7), np.zeros(7)
    for i in range(1, 8001):
        t = i*.001
        if i % 20 == 0 and t < 5.:
            target = robot.neutral + (.16 if t < 2. else -.16)*np.array([1,-1,1,-1,1,-1,1])
            assert core.publish(target, t, i, t)
            assert mirror.publish(target, t, i, t)
        if i == 5000:
            core.hold()
            mirror.hold()
        actual = np.asarray(core.step(t))
        np.testing.assert_allclose(actual, mirror.step(t), atol=1e-10)
        if response > 0:
            assert np.all(abs(mirror.velocity) <= cfg.max_velocity+1e-9)
            assert np.all(abs(mirror.acceleration) <= cfg.max_acceleration+1e-9)
            assert np.all(abs((mirror.acceleration-previous_a)/.001) <= cfg.max_jerk+1e-6)
            np.testing.assert_allclose((mirror.velocity-previous_v)/.001, mirror.acceleration, atol=1e-7)
            previous_v, previous_a = mirror.velocity.copy(), mirror.acceleration.copy()
    assert core.diagnostics(8.)['watchdog_state'] != 'safe_stop'
    assert np.linalg.norm(mirror.velocity) < .001


@pytest.mark.parametrize('value', [-1., float('nan'), float('inf'), 6.])
def test_response_rejected_before_native_motion(value):
    from rokae.python import _rokae_cpp as native
    for cls in (native.RealtimeConfig, native.JointRealtimeConfig):
        cfg = cls()
        cfg.response_hz = value
        with pytest.raises(ValueError, match='response_hz'):
            cfg.validate()


@pytest.mark.parametrize('response', [2., 5.])
def test_joint_large_reversing_reference_respects_actual_output_derivatives(response):
    """Differentiate emitted positions, not just the limiter's stored states."""
    from rokae.python import _rokae_cpp as native
    robot = XmateErPro7()
    cfg = replace(JointStreamConfig.from_limits(robot.stream_limits), response_hz=response)
    core = native.JointRealtimeCore(cpp_config(native.JointRealtimeConfig, cfg))
    core.reset(robot.neutral, 0.)
    position, velocity, acceleration = robot.neutral.copy(), np.zeros(7), np.zeros(7)
    for i in range(1, 20001):
        t = i*.001
        if i % 20 == 0:
            target = robot.neutral + 1.2*np.sin(t*2)*np.array([1,-1,1,-1,1,-1,1])
            target = np.clip(target, position-.34, position+.34)
            target = np.clip(target, cfg.position_min+.1, cfg.position_max-.1)
            assert core.publish(target, t, i, t)
        next_position = np.asarray(core.step(t))
        next_velocity = (next_position-position)/.001
        next_acceleration = (next_velocity-velocity)/.001
        jerk = (next_acceleration-acceleration)/.001
        assert np.all(abs(next_velocity) <= cfg.max_velocity+1e-6)
        assert np.all(abs(next_acceleration) <= cfg.max_acceleration+1e-6)
        assert np.all(abs(jerk) <= cfg.max_jerk+1e-4)
        position, velocity, acceleration = next_position, next_velocity, next_acceleration
