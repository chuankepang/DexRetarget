"""Init speed overrides through the Python driver to a fake SDK boundary."""
import importlib.util
import sys
from pathlib import Path
from unittest.mock import Mock

import numpy as np
import pytest

from dexretarget.teleop.devices.rokae import RokaeXCoreDriver

ROOT = Path(__file__).resolve().parents[1]


@pytest.fixture
def init_cli(monkeypatch):
    monkeypatch.syspath_prepend(str(ROOT / "rokae/examples"))
    spec = importlib.util.spec_from_file_location("init_speed_cli", ROOT / "rokae/examples/05_move_to_init.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    build = Mock()
    monkeypatch.setattr(module, "build_driver", build)
    return module, build


@pytest.mark.parametrize("options", [
    ["--speed", "4"], ["--speed", "1001"],
    ["--online-speed-scale", "0"], ["--online-speed-scale", "1.01"],
    ["--online-speed-scale", "nan"], ["--online-speed-scale", "inf"],
])
def test_bad_init_speed_rejected_before_connection(init_cli, monkeypatch, options):
    module, build = init_cli
    monkeypatch.setattr(sys, "argv", ["05_move_to_init.py", *options])
    with pytest.raises(SystemExit) as error:
        module.main()
    assert error.value.code == 2
    build.assert_not_called()


@pytest.mark.parametrize("options, expected_speed, expected_scale", [
    ([], 250, 0.50), (["--speed", "150", "--online-speed-scale", "0.75"], 150, 0.75),
])
def test_effective_speed_reaches_native_boundary(init_cli, monkeypatch, options, expected_speed, expected_scale):
    module, build = init_cli
    native = Mock()
    native.get_joint_positions.return_value = np.zeros(7)
    native.get_soft_limits.return_value = True, np.tile([-3.0, 3.0], (7, 1))
    native.get_base_frame.return_value = np.eye(4)
    native.get_tcp_pose.return_value = np.eye(4)
    driver = RokaeXCoreDriver("127.0.0.1", "127.0.0.1", nrt_online_speed_scale=expected_scale)
    driver._driver = native
    driver._connected = True
    monkeypatch.setattr(driver, "connect", Mock())
    build.return_value = driver
    monkeypatch.setattr(sys, "argv", ["05_move_to_init.py", "--execute", "--yes", *options])
    module.main()
    assert build.call_args.kwargs["nrt_online_speed_scale"] == expected_scale
    assert native.move_to_init.call_count > 0
    assert all(call.args[1] == expected_speed for call in native.move_to_init.call_args_list)
    native.disconnect.assert_called_once()
