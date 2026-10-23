"""Standalone launch/config/trace tests. Native guards are tested by CTest."""
import importlib.util
import json
from pathlib import Path
import subprocess

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[1]
DIRECTORY = ROOT/'rokae/impedance_test'


def load(name):
    spec = importlib.util.spec_from_file_location('fixed_'+name, DIRECTORY/(name+'.py'))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


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
    assert manifest['mass_kg'] == pytest.approx(.93938639463)
    assert manifest['desired_wrench'] == [0]*6


def test_execution_can_be_cancelled_before_connect(monkeypatch, tmp_path):
    module = load('run')
    build = tmp_path/'build';build.mkdir();(build/'fixed_impedance').touch()
    monkeypatch.setattr(module, 'BUILD', build)
    monkeypatch.setattr('builtins.input', lambda _: 'cancel')
    monkeypatch.setattr(module.subprocess, 'Popen', lambda *a, **k: pytest.fail('cancelled launch connected'))
    output = tmp_path/'outputs'
    assert module.main(['--stage', 'hold', '--execute', '--output-root', str(output)]) == 2
    assert not output.exists()


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
