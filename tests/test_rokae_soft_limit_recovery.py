"""Offline planning and CLI gates. No test connects to or moves a robot."""
import importlib.util
from pathlib import Path
from unittest.mock import Mock

import numpy as np
import pytest

from native_support import HAS_NATIVE

pytestmark = pytest.mark.skipif(not HAS_NATIVE, reason="Native binding unavailable; recovery not validated")
ROOT = Path(__file__).resolve().parents[1]


@pytest.fixture
def native():
    from rokae.python import _rokae_cpp
    return _rokae_cpp


@pytest.fixture
def snapshot():
    limits = np.deg2rad(np.array([[-165, 165], [-115, 115], [-165, 165],
                                 [-115, 115], [-165, 165], [-115, 115], [-355, 355]]))
    q = np.array([0.41697577, 0.26218268, 0.28432957, 0.52989569,
                  -0.45084055, 2.00796827, 0.34217898])
    return q, limits


def test_reported_j6_only_moves_inward(native, snapshot):
    q, limits = snapshot
    target = np.asarray(native.plan_disabled_soft_limit_recovery(q, limits))
    np.testing.assert_array_equal(target[[0, 1, 2, 3, 4, 6]], q[[0, 1, 2, 3, 4, 6]])
    assert target[5] == pytest.approx(limits[5, 1] - np.deg2rad(0.5))
    assert np.degrees(target[5] - q[5]) == pytest.approx(-0.548107, abs=1e-6)
    assert np.all(target > limits[:, 0]) and np.all(target < limits[:, 1])


@pytest.mark.parametrize("side", [0, 1])
def test_lower_upper_and_exact_boundary(native, snapshot, side):
    q, limits = snapshot
    q[5] = limits[5, side]
    target = native.plan_disabled_soft_limit_recovery(q, limits)
    assert target[5] == pytest.approx(limits[5, side] + (1 if side == 0 else -1) * np.deg2rad(0.5))


def test_all_inside_means_restore_without_motion(native, snapshot):
    q, limits = snapshot
    q[5] = 1.9
    np.testing.assert_array_equal(native.plan_disabled_soft_limit_recovery(q, limits), q)


def test_explicit_clearance_also_clears_init_margin(native, snapshot):
    q, limits = snapshot
    target = np.asarray(native.plan_disabled_soft_limit_recovery(q, limits, 10.0))
    assert np.degrees(target[5]) == pytest.approx(105.0)
    assert np.min(np.minimum(target - limits[:, 0], limits[:, 1] - target)) > 0.15


@pytest.mark.parametrize("clearance", [float("nan"), float("inf"), -1, 0, 0.49, 10.01])
def test_invalid_clearance_rejected(native, snapshot, clearance):
    with pytest.raises(ValueError, match="clearance"):
        native.plan_disabled_soft_limit_recovery(*snapshot, clearance)


@pytest.mark.parametrize("invalid", ["multiple", "far", "nan_q", "nan_limit", "reversed", "narrow", "shape", "empty"])
def test_unsafe_or_invalid_plan_rejected(native, snapshot, invalid):
    q, limits = snapshot
    if invalid == "multiple":
        q[1] = limits[1, 0] - 0.001
    elif invalid == "far":
        q[5] = limits[5, 1] + np.deg2rad(1.01)
    elif invalid == "nan_q":
        q[0] = np.nan
    elif invalid == "nan_limit":
        limits[0, 0] = np.nan
    elif invalid == "reversed":
        limits[0] = [1, -1]
    elif invalid == "narrow":
        limits[0] = [0, 0.001]
    elif invalid == "shape":
        q = q[:-1]
    elif invalid == "empty":
        q, limits = [], []
    with pytest.raises(ValueError):
        native.plan_disabled_soft_limit_recovery(q, limits)


def test_native_execution_requires_connection(native, snapshot):
    q, limits = snapshot
    driver = native.RokaeDriver("127.0.0.1", "127.0.0.1", "xmate-er-pro-7")
    with pytest.raises(RuntimeError, match="not connected"):
        driver.recover_disabled_soft_limits(q, limits)


@pytest.fixture
def cli(monkeypatch, snapshot):
    monkeypatch.syspath_prepend(str(ROOT / "rokae/examples"))
    spec = importlib.util.spec_from_file_location("recover_soft_limits_cli", ROOT / "rokae/examples/06_recover_soft_limits.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    q, limits = snapshot
    driver = Mock()
    driver.get_state.return_value = {"controller_version": "fake", "robot_type": "fake",
                                   "power_state": "off", "operate_mode": "manual", "operation_state": "idle"}
    driver.get_joint_positions.return_value = q
    driver.get_soft_limits.return_value = False, limits
    monkeypatch.setattr(module, "build_driver", Mock(return_value=driver))
    monkeypatch.setattr(module, "load_config", Mock(return_value={"robot": {"initial_joint_move": {}}}))
    return module, driver


def test_dry_run_sends_no_recovery_command(cli, capsys):
    module, driver = cli
    module.main([])
    assert "DRY RUN" in capsys.readouterr().out
    assert [call[0] for call in driver.method_calls] == ["connect", "get_state", "get_joint_positions", "get_soft_limits", "disconnect"]


@pytest.mark.parametrize("answer", ["", "MOVE", "recover"])
def test_execute_requires_exact_maintenance_confirmation(cli, monkeypatch, answer):
    module, driver = cli
    monkeypatch.setattr("builtins.input", lambda _: answer)
    with pytest.raises(SystemExit, match="cancelled"):
        module.main(["--execute"])
    driver.recover_disabled_soft_limits.assert_not_called()
    driver.disconnect.assert_called_once()


def test_execute_forwards_reviewed_snapshot(cli, monkeypatch, snapshot):
    module, driver = cli
    monkeypatch.setattr("builtins.input", lambda _: "RECOVER")
    module.main(["--execute"])
    driver.recover_disabled_soft_limits.assert_called_once()
    args = driver.recover_disabled_soft_limits.call_args.args
    np.testing.assert_array_equal(args[0], snapshot[0])
    np.testing.assert_array_equal(args[1], snapshot[1])
    assert driver.recover_disabled_soft_limits.call_args.kwargs == {"clearance_degrees": 0.5, "request_power_on": False, "jog_rate": 0.05}
    driver.disconnect.assert_called_once()


def test_execute_forwards_explicit_clearance(cli, monkeypatch):
    module, driver = cli
    monkeypatch.setattr("builtins.input", lambda _: "RECOVER")
    module.main(["--execute", "--clearance-deg", "10"])
    assert driver.recover_disabled_soft_limits.call_args.kwargs == {"clearance_degrees": 10.0, "request_power_on": False, "jog_rate": 0.05}


@pytest.mark.parametrize("override, expected", [(False, 0.08), (True, 0.10)])
def test_recovery_rate_config_and_cli_precedence(cli, monkeypatch, override, expected):
    module, driver = cli
    module.load_config.return_value["robot"]["initial_joint_move"]["recovery_jog_rate"] = 0.08
    monkeypatch.setattr("builtins.input", lambda _: "RECOVER")
    module.main(["--execute"] + (["--jog-rate", "0.10"] if override else []))
    assert driver.recover_disabled_soft_limits.call_args.kwargs["jog_rate"] == expected


@pytest.mark.parametrize("rate", ["nan", "inf", "0", "0.009", "0.101"])
def test_invalid_jog_rate_rejected_before_connection(cli, rate):
    module, driver = cli
    with pytest.raises(SystemExit) as error:
        module.main(["--execute", "--jog-rate", rate])
    assert error.value.code == 2
    driver.connect.assert_not_called()


@pytest.mark.parametrize("rate", [float("nan"), float("inf"), 0, 0.009, 0.101])
def test_native_jog_rate_validation_precedes_hardware(native, snapshot, rate):
    driver = native.RokaeDriver("127.0.0.1", "127.0.0.1", "xmate-er-pro-7")
    with pytest.raises(ValueError, match="jog_rate"):
        driver.recover_disabled_soft_limits(*snapshot, jog_rate=rate)


def test_sdk_power_requires_explicit_source_selection(cli, monkeypatch, capsys):
    module, driver = cli
    monkeypatch.setattr("builtins.input", lambda _: "RECOVER")
    module.main(["--execute", "--manual-power-source", "sdk"])
    assert driver.recover_disabled_soft_limits.call_args.kwargs["request_power_on"] is True
    assert "only with no external enable switch or teach pendant" in capsys.readouterr().out


def test_default_power_source_displays_hold_and_release_sequence(cli, capsys):
    module, driver = cli
    module.main([])
    assert "Manual power source: operator" in capsys.readouterr().out
    driver.recover_disabled_soft_limits.assert_not_called()


def test_power_on_failure_reads_logs_without_masking_original(cli, monkeypatch, capsys):
    module, driver = cli
    monkeypatch.setattr("builtins.input", lambda _: "RECOVER")
    driver.recover_disabled_soft_limits.side_effect = RuntimeError("power on(manual recovery): denied")
    driver.get_controller_logs.return_value = [
        {"id": 1001, "timestamp": "fake-time", "content": "fake controller diagnosis", "repair": "fake guidance"}
    ]
    with pytest.raises(RuntimeError, match="power on\\(manual recovery\\): denied"):
        module.main(["--execute"])
    driver.get_controller_logs.assert_called_once_with(10)
    assert "fake controller diagnosis" in capsys.readouterr().out
    driver.disconnect.assert_called_once()


def test_log_failure_does_not_replace_motion_failure(cli, monkeypatch, capsys):
    module, driver = cli
    monkeypatch.setattr("builtins.input", lambda _: "RECOVER")
    driver.recover_disabled_soft_limits.side_effect = RuntimeError("original controller rejection")
    driver.get_controller_logs.side_effect = RuntimeError("log read rejected")
    with pytest.raises(RuntimeError, match="original controller rejection"):
        module.main(["--execute"])
    assert "log read rejected" in capsys.readouterr().out
    driver.disconnect.assert_called_once()


@pytest.mark.parametrize("count", [0, 11])
def test_log_count_range_checked_without_connection(native, count):
    driver = native.RokaeDriver("127.0.0.1", "127.0.0.1", "xmate-er-pro-7")
    with pytest.raises(ValueError, match="1..10"):
        driver.get_controller_logs(count)


def test_log_query_requires_connection(native):
    driver = native.RokaeDriver("127.0.0.1", "127.0.0.1", "xmate-er-pro-7")
    with pytest.raises(RuntimeError, match="not connected"):
        driver.get_controller_logs()


def test_inside_but_near_limit_reports_remaining_init_guard(cli, capsys):
    module, driver = cli
    driver.get_joint_positions.return_value[5] = np.deg2rad(114.9)
    module.main([])
    output = capsys.readouterr().out
    assert "without Jog" in output
    assert "this recovery alone may not permit init" in output
    driver.recover_disabled_soft_limits.assert_not_called()


@pytest.mark.parametrize("error", [RuntimeError("SDK rejected manual Jog"), KeyboardInterrupt()])
def test_failure_or_interrupt_disconnects(cli, monkeypatch, error):
    module, driver = cli
    monkeypatch.setattr("builtins.input", lambda _: "RECOVER")
    driver.recover_disabled_soft_limits.side_effect = error
    with pytest.raises(type(error)):
        module.main(["--execute"])
    driver.disconnect.assert_called_once()


@pytest.mark.parametrize("reason", ["enabled", "busy", "multiple"])
def test_cli_preconditions_block_execution(cli, monkeypatch, snapshot, reason):
    module, driver = cli
    monkeypatch.setattr("builtins.input", lambda _: pytest.fail("must reject before confirmation"))
    q, limits = snapshot
    if reason == "enabled":
        driver.get_soft_limits.return_value = True, limits
    elif reason == "busy":
        driver.get_state.return_value["operation_state"] = "moving"
    else:
        q[1] = limits[1, 0] - 0.001
    with pytest.raises((SystemExit, RuntimeError, ValueError)):
        module.main(["--execute"])
    driver.recover_disabled_soft_limits.assert_not_called()
    driver.disconnect.assert_called_once()
