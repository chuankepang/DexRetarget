"""Standalone launch/config/trace tests. Native guards are tested by CTest."""
import importlib.util
import json
from pathlib import Path
import subprocess

import numpy as np
import pytest
import yaml

ROOT = Path(__file__).resolve().parents[1]
DIRECTORY = ROOT/'rokae/impedance_test'


def load(name):
    spec = importlib.util.spec_from_file_location('fixed_'+name, DIRECTORY/(name+'.py'))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


@pytest.fixture
def applying_config(tmp_path):
    """Experimental writes need explicit authorization, independent of live settings."""
    config = yaml.safe_load((ROOT/'config/robots/rokae.yaml').read_text())
    config['robot']['payload']['apply_on_impedance_start'] = True
    path = tmp_path/'applying_robot.yaml'
    path.write_text(yaml.safe_dump(config))
    return path


def test_dry_run_never_starts_native_or_creates_output(monkeypatch, tmp_path, capsys):
    module = load('run')
    def forbidden(*args, **kwargs):
        pytest.fail('dry-run attempted subprocess/robot connection')
    monkeypatch.setattr(module.subprocess, 'Popen', forbidden)
    output = tmp_path/'not_created'
    assert module.main(['--stage', 'hold', '--output-root', str(output)]) == 0
    assert not output.exists()
    text = capsys.readouterr().out
    manifest = json.loads(text[:text.index('\nDRY RUN:')])
    assert manifest['home_rad'] == [0,.1332313,0,1.17304764,0,1.2835298642,0]
    assert manifest['mass_kg'] == pytest.approx(
        yaml.safe_load((ROOT/'config/robots/rokae.yaml').read_text())['robot']['payload']['mass_kg'])
    assert manifest['desired_wrench'] == [0]*6
    np.testing.assert_allclose(manifest['cog_m'],
        yaml.safe_load((ROOT/'config/robots/rokae.yaml').read_text())['robot']['payload']['cog_m'])
    expected_apply = yaml.safe_load((ROOT/'config/robots/rokae.yaml').read_text())['robot']['payload']['apply_on_impedance_start']
    assert manifest['apply_on_impedance_start'] is expected_apply
    assert manifest['payload_source'] == ('robot_config' if expected_apply else 'active_sdk_toolset_checked_against_config')
    encoded = module.encode_parameters(manifest, output).splitlines()
    assert float(encoded[4]) == pytest.approx(manifest['mass_kg'])
    np.testing.assert_allclose([float(x) for x in encoded[5].split()], manifest['cog_m'])


@pytest.mark.parametrize('answer', ['', 'n', 'cancel'])
@pytest.mark.parametrize('extra', [[], ['--experimental-cog-mm', '12', '8', '52']])
def test_execution_can_be_cancelled_before_connect(monkeypatch, tmp_path, extra, answer, applying_config):
    module = load('run')
    build = tmp_path/'build';build.mkdir();(build/'fixed_impedance').touch()
    monkeypatch.setattr(module, 'BUILD', build)
    monkeypatch.setattr('builtins.input', lambda _: answer)
    monkeypatch.setattr(module.subprocess, 'Popen', lambda *a, **k: pytest.fail('cancelled launch connected'))
    output = tmp_path/'outputs'
    assert module.main(['--robot-config', str(applying_config), '--stage', 'hold', '--execute', '--output-root', str(output), *extra]) == 2
    assert not output.exists()


def test_experimental_cog_units_native_transfer_and_config_preservation(monkeypatch, tmp_path, capsys, applying_config):
    module = load('run')
    config_path = applying_config
    original = config_path.read_bytes()
    monkeypatch.setattr(module.subprocess, 'Popen', lambda *a, **k: pytest.fail('dry-run connected'))
    assert module.main(['--robot-config', str(config_path), '--stage', 'hold', '--experimental-cog-mm',
                        '12.17669684527', '8.60984186271', '52.71043598571']) == 0
    text = capsys.readouterr().out
    manifest = json.loads(text[:text.index('\nDRY RUN:')])
    expected = [.01217669684527, .00860984186271, .05271043598571]
    np.testing.assert_allclose(manifest['cog_m'], expected)
    assert manifest['payload_source'] == 'experimental_cli_unverified'
    np.testing.assert_allclose(manifest['configured_cog_m'],
        yaml.safe_load(original)['robot']['payload']['cog_m'])
    encoded = module.encode_parameters(manifest, tmp_path).splitlines()
    np.testing.assert_allclose([float(x) for x in encoded[5].split()], expected)
    assert config_path.read_bytes() == original


@pytest.mark.parametrize('values', [('nan', '0', '0'), ('0', 'inf', '0'), ('301', '0', '0')])
def test_experimental_cog_rejects_invalid_values_before_connect(values, monkeypatch, applying_config):
    module = load('run')
    monkeypatch.setattr(module.subprocess, 'Popen', lambda *a, **k: pytest.fail('invalid plan connected'))
    with pytest.raises(ValueError):
        module.main(['--robot-config', str(applying_config), '--stage', 'hold', '--execute', '--experimental-cog-mm', *values])


@pytest.mark.parametrize('stage', ['inspect', 'home', 'prepare'])
def test_experimental_cog_cannot_change_other_stages(stage, applying_config):
    with pytest.raises(ValueError, match='only allowed'):
        load('run').main(['--robot-config', str(applying_config), '--stage', stage, '--experimental-cog-mm', '12', '8', '52'])


@pytest.mark.parametrize('duration', ['nan', 'inf', '0', '100'])
def test_invalid_duration_does_not_connect(duration):
    with pytest.raises(ValueError, match='duration'):
        load('run').main(['--duration', duration])


def test_trace_summary_distinguishes_moving_reference_and_actual_motion(tmp_path):
    import csv
    path = tmp_path/'trace.csv'
    columns = ['seconds','callback_period_s','stop']
    for prefix, count in [('tcp_m',16),('reference',16),('q_m',7),('dq_m',7)]:
        columns += [f'{prefix}_{i}' for i in range(count)]
    with path.open('w') as stream:
        writer = csv.DictWriter(stream, fieldnames=columns);writer.writeheader()
        for i in range(3):
            ref = np.eye(4).ravel();actual = ref.copy();actual[11] += .005*i
            row = {'seconds':i,'callback_period_s':.001,'stop':'duration_completed' if i==2 else 'none'}
            for prefix, values in [('tcp_m',actual),('reference',ref),('q_m',[.01*i]*7),('dq_m',[.01]*7)]:
                row.update({f'{prefix}_{j}':v for j,v in enumerate(values)})
            writer.writerow(row)
    result = load('analyze').summarize(path)
    assert result['reference_is_constant'] is True
    assert result['max_tcp_displacement_mm'] == pytest.approx(10)
    assert result['startup_max_tcp_displacement_mm'] == pytest.approx(5)
    assert result['last_sample_stop'] == 'duration_completed'
    assert result['power_off_readback_confirmed'] is None
    assert result['stop_source'] == 'timer'
    path.with_name('summary.txt').write_text('stop_reason=joint_speed\npower_off_readback=1\n')
    result = load('analyze').summarize(path)
    assert result['stop_source'] == 'software_guard'
    assert result['power_off_readback_confirmed'] is True
    assert result['actual_tcp_delta_mm'][2] == pytest.approx(10)
    # 启动参考早于首帧；用首帧作零点会低估护栏实际判断的偏移。
    path.with_name('summary.txt').write_text('initial_impedance_q= -0.005 0 0 0 0 0 0\n')
    result = load('analyze').summarize(path)
    assert result['joint_displacement_origin'] == 'pre_start_reference'
    assert result['max_abs_joint_displacement_rad'][0] == pytest.approx(.025)



@pytest.mark.parametrize('stage', ['home', 'prepare', 'hold'])
@pytest.mark.parametrize('answer', ['y', ' Y '])
@pytest.mark.parametrize('rt_mode', ['explicit', 'skip'])
def test_short_confirmation_launches_once(monkeypatch, tmp_path, stage, answer, rt_mode):
    if rt_mode == 'skip' and stage == 'home':
        with pytest.raises(ValueError, match='requires prepare or hold'):
            load('run').main(['--stage', stage, '--rt-load-mode', rt_mode])
        return
    from types import SimpleNamespace
    module = load('run')
    build = tmp_path/'build';build.mkdir();(build/'fixed_impedance').touch()
    monkeypatch.setattr(module, 'BUILD', build)
    prompts, commands = [], []
    def confirm(prompt):
        prompts.append(prompt)
        return answer
    def launch(command, **kwargs):
        commands.append(command)
        return SimpleNamespace(stdout=iter([]), wait=lambda: 0)
    monkeypatch.setattr('builtins.input', confirm)
    monkeypatch.setattr(module.subprocess, 'Popen', launch)
    assert module.main(['--stage', stage, '--rt-load-mode', rt_mode, '--execute', '--output-root', str(tmp_path/'out')]) == 0
    assert len(prompts) == len(commands) == 1
    assert commands[0][1] == stage
    assert commands[0][-1] == '--rt-load-mode='+rt_mode
    assert len(list((tmp_path/'out').glob('*/manifest.json'))) == 1


def test_robotassist_profile_transfers_gains_frame_and_preserves_payload(monkeypatch, capsys):
    module = load('run')
    config_path = ROOT/'config/robots/rokae.yaml'
    original = config_path.read_bytes()
    monkeypatch.setattr(module.subprocess, 'Popen', lambda *a, **k: pytest.fail('dry-run connected'))
    module.main(['--stage', 'hold'])
    baseline = json.loads(capsys.readouterr().out.split('\nDRY RUN:')[0])
    module.main(['--stage', 'hold', '--profile', 'robotassist'])
    candidate = json.loads(capsys.readouterr().out.split('\nDRY RUN:')[0])
    assert candidate['stiffness'] == [100]*6
    assert candidate['configured_stiffness'] == baseline['stiffness']
    assert candidate['force_frame'] == 'world'
    assert baseline['force_frame'] == 'tool'
    for key in ('mass_kg', 'cog_m', 'home_rad', 'guard', 'desired_wrench'):
        assert candidate[key] == baseline[key]
    lines = module.encode_parameters(candidate, ROOT).splitlines()
    assert lines[0] == 'rokae.fixed_impedance.v5'
    assert [float(v) for v in lines[7].split()] == [100]*6
    assert lines[-4:] == ['world', 'latest-compatible', 'baseline', str(candidate['apply_on_impedance_start']).lower()]
    assert config_path.read_bytes() == original


@pytest.mark.parametrize('extra', [['--stage', 'home'], ['--stage', 'inspect'],
    ['--stage', 'hold', '--experimental-cog-mm', '8', '12', '52']])
def test_robotassist_profile_rejects_mixed_experiments(extra, monkeypatch, applying_config):
    module = load('run')
    monkeypatch.setattr(module.subprocess, 'Popen', lambda *a, **k: pytest.fail('invalid launch'))
    with pytest.raises(ValueError, match='requires hold'):
        module.main(['--robot-config', str(applying_config), '--profile', 'robotassist', '--execute', *extra])


def test_startup_comparison_keeps_physics_and_guards(monkeypatch, capsys):
    module = load('run')
    monkeypatch.setattr(module.subprocess, 'Popen', lambda *a, **k: pytest.fail('dry-run connected'))
    configs = []
    for startup in ('latest-compatible', 'legacy-rt'):
        assert module.main(['--stage', 'hold', '--startup', startup]) == 0
        config = json.loads(capsys.readouterr().out.split('\nDRY RUN:')[0])
        configs.append(config)
        assert module.encode_parameters(config, ROOT).splitlines()[-3:] == [startup, 'baseline', str(config['apply_on_impedance_start']).lower()]
    for key in ('home_rad', 'mass_kg', 'cog_m', 'stiffness', 'desired_wrench', 'guard', 'force_frame'):
        assert configs[0][key] == configs[1][key]
    assert configs[0]['nrt_home_speed_sdk'] == 100
    assert all(config['home_speed_scale'] == .50 for config in configs)
    assert configs[1]['nrt_home_speed_sdk'] is None
    # Retained native protocol v1/v2 has no startup field: old startup semantics.
    old = dict(configs[0], schema='rokae.fixed_impedance.v2')
    assert module.encode_parameters(old, ROOT).splitlines()[-1] == 'tool'
    old['schema'] = 'rokae.fixed_impedance.v1'
    assert float(module.encode_parameters(old, ROOT).splitlines()[-1]) == old['duration_s']


def test_light_push_is_explicit_and_preserves_payload_and_startup_guards(capsys):
    module = load('run')
    plans = []
    for mode in ('baseline', 'light-push'):
        module.main(['--stage', 'hold', '--profile', 'robotassist', '--interaction', mode])
        plans.append(json.loads(capsys.readouterr().out.split('\nDRY RUN:')[0]))
    base, push = plans
    assert base['guard']['joint_speed_rad_s'] == .30
    assert push['guard']['joint_speed_rad_s'] == .70
    assert push['guard']['startup_joint_speed_rad_s'] == .30
    assert base['guard']['tcp_translation_m'] == .06
    assert push['guard']['tcp_translation_m'] == .10
    assert push['guard']['startup_tcp_translation_m'] == .06
    assert base['guard']['joint_delta_rad'] == .50
    assert push['guard']['joint_delta_rad'] == 1.0
    for key in ('startup_s', 'startup_joint_delta_rad', 'startup_tcp_translation_m', 'tcp_rotation_rad'):
        assert base['guard'][key] == push['guard'][key]
    for key in ('mass_kg', 'cog_m', 'home_rad', 'stiffness', 'desired_wrench'):
        assert base[key] == push[key]
    assert module.encode_parameters(push, ROOT).splitlines()[-4:] == ['world', 'latest-compatible', 'light-push', str(push['apply_on_impedance_start']).lower()]
    legacy = dict(push, schema='rokae.fixed_impedance.v3')
    assert module.encode_parameters(legacy, ROOT).splitlines()[-1] == 'latest-compatible'


@pytest.mark.parametrize('stage', ['inspect', 'home', 'prepare'])
def test_light_push_cannot_change_other_stages(stage):
    with pytest.raises(ValueError, match='requires hold'):
        load('run').main(['--stage', stage, '--interaction', 'light-push'])


def test_spring_check_changes_only_translation_stiffness(capsys):
    module = load('run')
    plans = []
    for profile in ('robotassist', 'spring-check'):
        module.main(['--stage', 'hold', '--profile', profile, '--interaction', 'light-push'])
        plans.append(json.loads(capsys.readouterr().out.split('\nDRY RUN:')[0]))
    base, spring = plans
    assert spring['stiffness'] == [300,300,300,100,100,100]
    for key in base:
        if key not in ('profile', 'stiffness'):
            assert spring[key] == base[key]
    encoded = module.encode_parameters(spring, ROOT).splitlines()
    assert [float(x) for x in encoded[7].split()] == spring['stiffness']
    assert encoded[-4] == 'world'


@pytest.mark.parametrize('args', [
    ['--stage', 'home'], ['--stage', 'hold', '--experimental-cog-mm', '8', '12', '52']])
def test_spring_check_rejects_unrelated_changes(args, applying_config):
    with pytest.raises(ValueError, match='spring-check profile requires hold'):
        load('run').main(['--robot-config', str(applying_config), '--profile', 'spring-check', *args])


def test_command_feedback_distinguishes_sent_reference_from_unavailable_sdk_feedback(tmp_path):
    import csv
    path = tmp_path/'trace.csv'
    pose = np.eye(4).ravel()
    rows = []
    for i in range(3):
        sent = pose.copy();sent[3] += .001*i
        command = pose.copy();command[3] += .002*i
        row = {'seconds':i, 'callback_period_s':.001, 'stop':'none', 'tcp_c_valid':int(i>0)}
        for prefix, values in [('tcp_m',pose),('reference',pose),('sent_tcp',sent),
                               ('tcp_c',command if i else np.zeros(16)),('q_m',[0]*7),('dq_m',[0]*7)]:
            row.update({f'{prefix}_{j}':x for j,x in enumerate(values)})
        rows.append(row)
    def write():
        with path.open('w') as stream:
            writer=csv.DictWriter(stream,fieldnames=rows[0]);writer.writeheader();writer.writerows(rows)
    write()
    result=load('analyze').summarize(path)
    assert result['reference_is_constant']
    assert result['sent_command_recorded']
    assert not result['sent_command_matches_reference']
    assert result['controller_command_valid_samples']==2
    assert result['controller_command_max_translation_from_reference_mm']==pytest.approx(4)
    for row in rows:
        row['tcp_c_valid']=0
    write()
    result=load('analyze').summarize(path)
    assert result['controller_command_max_translation_from_reference_mm'] is None


@pytest.mark.parametrize('apply', [True, False])
def test_payload_switch_is_transferred_to_native(tmp_path, capsys, apply):
    config = yaml.safe_load((ROOT/'config/robots/rokae.yaml').read_text())
    config['robot']['payload']['apply_on_impedance_start'] = apply
    path = tmp_path/'robot.yaml';path.write_text(yaml.safe_dump(config))
    module=load('run');module.main(['--stage','hold','--robot-config',str(path)])
    plan=json.loads(capsys.readouterr().out.split('\nDRY RUN:')[0])
    assert plan['apply_on_impedance_start'] is apply
    assert module.encode_parameters(plan, tmp_path).splitlines()[-1] == str(apply).lower()
    assert plan['mass_kg'] == config['robot']['payload']['mass_kg']
    legacy=dict(plan,schema='rokae.fixed_impedance.v4')
    assert module.encode_parameters(legacy,tmp_path).splitlines()[-1]=='baseline'
    if not apply:
        assert plan['payload_source']=='active_sdk_toolset_checked_against_config'
        with pytest.raises(ValueError,match='experimental CoM requires'):
            module.main(['--stage','hold','--robot-config',str(path),'--experimental-cog-mm','8','12','52'])


def test_payload_switch_rejects_string_false(tmp_path):
    config=yaml.safe_load((ROOT/'config/robots/rokae.yaml').read_text())
    config['robot']['payload']['apply_on_impedance_start']='false'
    path=tmp_path/'robot.yaml';path.write_text(yaml.safe_dump(config))
    with pytest.raises(ValueError,match='must be a boolean'):
        load('run').main(['--robot-config',str(path)])


def test_prepare_matches_hold_configuration_without_executing(monkeypatch, capsys):
    module = load('run')
    monkeypatch.setattr(module.subprocess, 'Popen', lambda *a, **k: pytest.fail('dry-run connected'))
    plans = []
    for stage in ('prepare', 'hold'):
        assert module.main(['--stage', stage, '--profile', 'robotassist']) == 0
        plans.append(json.loads(capsys.readouterr().out.split('\nDRY RUN:')[0]))
    prepare, hold = plans
    for key in hold:
        if key != 'stage':
            assert prepare[key] == hold[key]
    assert prepare['stage'] == 'prepare'


def test_force_snapshots_preserve_phases_and_missing_data(tmp_path):
    module = load('analyze')
    path = tmp_path/'summary.txt'
    def snapshot(phase, fz):
        return (f'force_snapshot_phase={phase}\nforce_snapshot_available=1\n'
                'joint_torque_measured_Nm=0 0 0 0 0 0 0\n'
                'joint_external_estimate_Nm=1 2 3 4 5 6 7\n'
                'world_cart_torque_estimate_Nm=0 0 0\n'
                f'world_cart_force_estimate_N=0 0 {fz}\n')
    path.write_text('stage=prepare\nrt_load_mode=skip\nrt_load_write_performed=0\n'+snapshot('nrt_home_powered_stationary', 6.45)+
                    snapshot('rt_after_setLoad', 3.2)+
                    'force_snapshot_phase=failed_read\nforce_snapshot_available=0\n'
                    'force_snapshot_error=unavailable\npower_off_readback=1\n'
                    'impedance_start_attempted=0\n')
    result = module.summarize_preparation(path)
    assert result['rt_load_mode'] == 'skip'
    assert result['rt_load_write_performed'] is False
    assert result['power_off_readback_confirmed'] is True
    assert result['impedance_start_attempted'] == '0'
    nrt, rt, failed = result['force_snapshots']
    assert nrt['world_cart_force_estimate_N'][2] == 6.45
    assert rt['world_cart_force_estimate_N'][2] == 3.2
    assert failed == {'phase': 'failed_read', 'available': False, 'error': 'unavailable'}
    path.write_text('force_snapshot_phase=incomplete\nforce_snapshot_available=1\n')
    assert not module.force_snapshots(path)[0]['available']
    path.write_text(snapshot('bad', 'nan'))
    with pytest.raises(ValueError, match='Invalid force snapshot'):
        module.force_snapshots(path)


def test_rt_load_skip_changes_only_rt_policy(monkeypatch, capsys):
    module = load('run')
    monkeypatch.setattr(module.subprocess, 'Popen', lambda *a, **k: pytest.fail('dry-run connected'))
    plans = []
    for mode in ('explicit', 'skip'):
        module.main(['--stage', 'hold', '--profile', 'robotassist', '--rt-load-mode', mode])
        plans.append(json.loads(capsys.readouterr().out.split('\nDRY RUN:')[0]))
    explicit, skip = plans
    for key in explicit:
        if key not in ('rt_load_mode', 'rt_load_policy'):
            assert explicit[key] == skip[key]
    assert 'unknown' in skip['rt_load_policy']
    with pytest.raises(ValueError, match='requires prepare or hold'):
        module.main(['--stage', 'inspect', '--rt-load-mode', 'skip'])
