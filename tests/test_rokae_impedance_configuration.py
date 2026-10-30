"""Configuration precedence and native handoff, without a robot connection."""
import copy
import importlib.util
import json
from pathlib import Path

import pytest

from dexretarget.app.cli import build_parser
from dexretarget.app.task import apply_task
from dexretarget.config import load_yaml_mapping
from rokae.control.configuration import describe_arm_config
from rokae.control.controllers.spec import resolve_controller, effective_cartesian_safety
from native_support import HAS_NATIVE

ROOT = Path(__file__).resolve().parents[1]


def arguments(*extra):
    args = build_parser().parse_args(['--task-config', str(ROOT/'config/tasks/cup.yaml'), *extra])
    apply_task(args, {'arm_control', 'arm_max_translation_speed'})
    return args


def test_effective_reference_limits_and_cli_precedence():
    args = arguments('--arm-control', 'impedance')
    result = describe_arm_config(args)
    assert result['command_semantics'] == 'equilibrium_pose'
    assert result['impedance']['force_frame'] == 'tool'
    assert result['cartesian_reference_generator']['max_translation_speed'] == .20
    assert result['cartesian_reference_generator']['max_translation_acceleration'] == .60
    assert not result['controller_readback_performed']
    assert result['impedance']['nominal_spring_force_at_translation_limit_N'] == [12]*3
    args.arm_max_translation_speed = .12
    assert describe_arm_config(args)['cartesian_reference_generator']['max_translation_speed'] == .12
    args.arm_control = 'cartesian'
    args.arm_max_translation_speed = None
    assert describe_arm_config(args)['cartesian_reference_generator']['max_translation_speed'] == .15


@pytest.mark.parametrize('setting', [
    {'force_frame': 'base'}, {'damping': [1]*6}, {'nullspace_stiffness': 2},
    {'reference_motion': {'max_translation_speed': -1}},
    {'reference_motion': {'max_translation_speed': float('nan')}},
    {'reference_motion': {'joint_torque_limit': 100}}, {'stiffnes': [300]*6},
])
def test_unsupported_or_invalid_settings_rejected(setting):
    with pytest.raises(ValueError):
        resolve_controller({'control': {'impedance': setting}}, cli='impedance')


def test_reference_resolution_does_not_mutate_source():
    source = load_yaml_mapping(ROOT/'config/robots/rokae.yaml')
    before = copy.deepcopy(source)
    resolved = effective_cartesian_safety(source, resolve_controller(source, cli='impedance'))
    resolved['max_translation_speed'] = 99
    assert source == before


def test_duplicate_nested_yaml_fails_with_location(tmp_path):
    path = tmp_path/'robot.yaml'
    path.write_text('control:\n  impedance:\n    force_frame: tool\n    force_frame: world\n')
    with pytest.raises(ValueError, match=':4: duplicate YAML key'):
        load_yaml_mapping(path)


def test_owned_robot_task_model_yaml_has_unique_keys():
    for folder in ['robots', 'tasks', 'models']:
        for path in (ROOT/'config'/folder).glob('*.yaml'):
            load_yaml_mapping(path)


def test_preview_cli_never_imports_runtime(monkeypatch, capsys):
    import sys
    from dexretarget.app.cli import main
    monkeypatch.setitem(sys.modules, 'dexretarget.app.runtime', None)
    monkeypatch.setattr(sys, 'argv', ['dexretarget', '--task-config', str(ROOT/'config/tasks/cup.yaml'),
                                     '--arm-control', 'impedance', '--print-arm-config'])
    main()
    assert json.loads(capsys.readouterr().out)['read_only_plan'] is True


@pytest.mark.skipif(not HAS_NATIVE, reason='Native binding unavailable; not validated')
@pytest.mark.parametrize('frame', ['tool', 'world'])
def test_force_frame_reaches_native_configuration(frame):
    from dexretarget.teleop.devices.rokae import RokaeXCoreDriver
    from rokae.control.controllers.spec import apply_realtime_gains
    from rokae.python import _rokae_cpp
    spec = resolve_controller({'control': {'impedance': {'force_frame': frame}}}, cli='impedance')
    driver = RokaeXCoreDriver('127.0.0.1', '127.0.0.1')
    apply_realtime_gains(driver, spec)
    native = driver._cpp_config(_rokae_cpp)
    assert native.impedance_force_frame == frame
    native.impedance_force_frame = 'invalid'
    with pytest.raises(ValueError, match='impedance_force_frame'):
        native.validate()


def test_standalone_and_teleop_share_configured_force_frame(tmp_path, capsys):
    import yaml
    source = load_yaml_mapping(ROOT/'config/robots/rokae.yaml')
    source['control']['impedance']['force_frame'] = 'world'
    path = tmp_path/'robot.yaml'
    path.write_text(yaml.safe_dump(source))
    module_spec = importlib.util.spec_from_file_location('fixed_run', ROOT/'rokae/impedance_test/run.py')
    module = importlib.util.module_from_spec(module_spec)
    module_spec.loader.exec_module(module)
    module.main(['--robot-config', str(path), '--stage', 'hold'])
    plan = json.loads(capsys.readouterr().out.split('\nDRY RUN:')[0])
    assert plan['force_frame'] == resolve_controller(source, cli='impedance').force_frame
    source['control']['impedance']['desired_wrench'] = [0, 0, 1, 0, 0, 0]
    path.write_text(yaml.safe_dump(source))
    with pytest.raises(ValueError, match='nonzero force'):
        module.main(['--robot-config', str(path), '--stage', 'hold'])
