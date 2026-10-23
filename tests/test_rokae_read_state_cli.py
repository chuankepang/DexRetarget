"""Exercise the diagnostic CLI with a read-only fake, never hardware."""
import json
from pathlib import Path
import runpy
import sys
from types import SimpleNamespace

import numpy as np
import pytest

SCRIPT = Path(__file__).resolve().parents[1] / 'rokae/examples/01_read_state.py'


def test_snapshot_has_no_motion_or_write_controller_calls(monkeypatch, tmp_path):
    calls = []
    state = dict(connected=True, sdk_version='0.3.4', controller_version='2.3.2',
                 robot_type='xMatePro3', joint_count=7, power_state='off',
                 operate_mode='automatic', operation_state='idle',
                 joint_position=[0]*7, joint_velocity=[0]*7, tcp_pose=np.eye(4))
    # No motion, power or setter methods exist on this fake.
    driver = SimpleNamespace(
        connect=lambda: calls.append('connect'),
        disconnect=lambda: calls.append('disconnect'),
        get_state=lambda: state,
        get_controller_base_frame=lambda: np.eye(4),
        get_soft_limits=lambda: (True, [[-1, 1]]*7),
        get_controller_logs=lambda count, faults_only: [{'timestamp':'now', 'id':32006, 'content':'fault'}])
    common = SimpleNamespace(
        add_connection_args=lambda p: p.add_argument('--config', default='unused'),
        build_driver=lambda a,c: driver,
        format_pose=str, format_vector=str,
        load_config=lambda p: {'robot':{'payload':{'mass_kg':0.939386}}})
    monkeypatch.setitem(sys.modules, 'common', common)
    output = tmp_path/'snapshot.json'
    monkeypatch.setattr(sys, 'argv', [str(SCRIPT), '--faults-only', '--json-output', str(output)])
    runpy.run_path(str(SCRIPT), run_name='__main__')
    report = json.loads(output.read_text())
    assert report['read_only'] is True
    assert report['state']['tcp_pose'] == np.eye(4).tolist()
    assert report['log_filter'] == 'warning/error'
    assert calls == ['connect', 'disconnect']
    with pytest.raises(SystemExit):
        runpy.run_path(str(SCRIPT), run_name='__main__')
    assert calls == ['connect', 'disconnect']  # refuse overwrite before connecting
