"""Native loader failures and ABI identity; never connects to a robot."""
import subprocess
import sys
import sysconfig
from pathlib import Path

import pytest

from dexretarget.teleop.devices.rokae import RokaeXCoreDriver
from native_support import HAS_NATIVE

ROOT = Path(__file__).resolve().parents[1]


def driver(module_path=None):
    return RokaeXCoreDriver(robot_ip='127.0.0.1', local_ip='127.0.0.1', sdk_path=module_path)


def test_native_parent_import_does_not_eagerly_import_facade():
    code = "import sys; import rokae.python; assert 'rokae.python.rokae' not in sys.modules"
    result = subprocess.run([sys.executable, '-B', '-c', code], cwd=ROOT, capture_output=True, text=True)
    assert result.returncode == 0, result.stderr


def test_wrong_abi_reports_expected_extension_without_import_cycle(tmp_path):
    (tmp_path / '_rokae_cpp.cpython-000-incompatible.so').write_bytes(b'not a matching extension')
    with pytest.raises(RuntimeError) as error:
        driver(tmp_path)._import_binding()
    message = str(error.value)
    assert 'No compatible ROKAE extension' in message
    assert sysconfig.get_config_var('EXT_SUFFIX') in message
    assert 'circular import' not in message


def test_missing_native_module_reports_build_command(tmp_path):
    with pytest.raises(FileNotFoundError, match='bash rokae/build.sh'):
        driver(tmp_path)._import_binding()


@pytest.mark.skipif(not HAS_NATIVE, reason='Matching native extension unavailable; not validated')
def test_facade_and_driver_share_one_native_module():
    from rokae.python import _rokae_cpp, RealtimeConfig
    from rokae import RealtimeConfig as public_config
    assert driver()._import_binding() is _rokae_cpp
    assert RealtimeConfig is public_config is _rokae_cpp.RealtimeConfig
    assert _rokae_cpp.SDK_VERSION == '0.3.4'


@pytest.mark.skipif(not HAS_NATIVE, reason='Matching native extension unavailable; not validated')
@pytest.mark.parametrize('statement',[
    'from rokae import SDK_VERSION',
    'from rokae.python import SDK_VERSION',
    'from rokae.python.rokae import SDK_VERSION',
])
def test_public_native_import_paths_work_in_fresh_interpreters(statement):
    result = subprocess.run([sys.executable, '-B', '-c', statement + "; assert SDK_VERSION == '0.3.4'"],
                            cwd=ROOT, capture_output=True, text=True)
    assert result.returncode == 0, result.stderr


def test_impedance_diagnostics_detects_controller_power_trip_while_paused(monkeypatch):
    from unittest.mock import Mock
    arm = driver()
    arm._started = True
    arm._connected = True
    arm._control_mode = 'impedance'
    arm._driver = Mock()
    arm._driver.get_state.return_value = {'power_state': 'off', 'operate_mode': 'automatic'}
    arm._driver.get_controller_logs.return_value = [{'message': 'Axis3 Momentum OverLimit'}]
    with pytest.raises(RuntimeError, match='Axis3 Momentum OverLimit'):
        arm.diagnostics()
    arm._driver.diagnostics.assert_not_called()
    arm._driver.start_realtime_cartesian_impedance.assert_not_called()


def test_impedance_health_queries_are_throttled(monkeypatch):
    from unittest.mock import Mock
    import dexretarget.teleop.devices.rokae as module
    arm = driver()
    arm._started = True
    arm._connected = True
    arm._control_mode = 'impedance'
    arm._driver = Mock()
    arm._driver.get_state.return_value = {'power_state': 'on', 'operate_mode': 'automatic'}
    arm._driver.diagnostics.return_value = dict.fromkeys(
        ['accepted', 'rejected_sequence', 'rejected_source_time', 'rejected_source_age',
         'rejected_target_delta', 'latest_sequence'], 0)
    monkeypatch.setattr(module.time, 'monotonic', lambda: 100.0)
    arm.diagnostics()
    arm.diagnostics()
    assert arm._driver.get_state.call_count == 1
    monkeypatch.setattr(module.time, 'monotonic', lambda: 100.6)
    arm.diagnostics()
    assert arm._driver.get_state.call_count == 2


def test_impedance_fault_contains_bounded_pretrip_joint_history(monkeypatch):
    from unittest.mock import Mock
    import dexretarget.teleop.devices.rokae as module
    arm = driver()
    arm._started = arm._connected = True
    arm._control_mode = 'impedance'
    arm._driver = Mock()
    arm._driver.diagnostics.return_value = dict.fromkeys(
        ['accepted', 'rejected_sequence', 'rejected_source_time', 'rejected_source_age',
         'rejected_target_delta', 'latest_sequence'], 0)
    for index in range(10):
        monkeypatch.setattr(module.time, 'monotonic', lambda i=index: 100.0 + i)
        arm._driver.get_state.return_value = {
            'power_state': 'on', 'operate_mode': 'automatic',
            'joint_position': [index] * 7, 'joint_velocity': [0.1] * 7}
        arm.diagnostics()
    assert len(arm._impedance_health_history) == 8
    monkeypatch.setattr(module.time, 'monotonic', lambda: 111.0)
    arm._driver.get_state.return_value = {'power_state': 'off', 'operate_mode': 'automatic'}
    arm._driver.get_controller_logs.return_value = []
    with pytest.raises(RuntimeError, match='dq_rad_s') as error:
        arm.diagnostics()
    assert 'may miss transient peaks' in str(error.value)
    assert "'last_python_target': None" in str(error.value)
    assert len(arm._impedance_health_history) == 8
    arm._driver.start_realtime_cartesian_impedance.assert_not_called()


@pytest.mark.skipif(not HAS_NATIVE, reason='Native bridge unavailable; not validated')
def test_native_initial_hold_does_not_generate_motion_without_targets():
    import numpy as np
    from rokae.python import _rokae_cpp
    core = _rokae_cpp.RealtimeCore(_rokae_cpp.RealtimeConfig())
    initial = np.eye(4)
    initial[:3, 3] = [0.4, 0.1, 0.3]
    core.reset(initial, 0.0)
    core.hold()
    for step in range(1, 2001):
        np.testing.assert_allclose(core.step(step * 0.001), initial, atol=1e-12)


def test_controller_log_filter_preserves_default_api():
    from unittest.mock import Mock
    arm = driver()
    arm._connected = True
    arm._driver = Mock()
    arm._driver.get_controller_logs.return_value = [{'id': 32006}]
    assert arm.get_controller_logs() == [{'id': 32006}]
    arm._driver.get_controller_logs.assert_called_with(10)
    assert arm.get_controller_logs(faults_only=True) == [{'id': 32006}]
    arm._driver.get_controller_logs.assert_called_with(10, True)
    arm._driver.start_realtime_cartesian_impedance.assert_not_called()
