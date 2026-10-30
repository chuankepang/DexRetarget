"""Architecture contracts, compatibility, and fault-injected resource ownership."""
from pathlib import Path
import argparse
import io
import json
import queue
import subprocess
import sys
import threading
import time

import numpy as np
import pytest

from dexretarget.config import PROJECT_ROOT as ROOT


def isolated_import(code, blocked):
    script = '''import sys, importlib.abc
class Guard(importlib.abc.MetaPathFinder):
    def find_spec(self, fullname, path=None, target=None):
        if any(fullname == p or fullname.startswith(p + '.') for p in BLOCKED):
            raise RuntimeError('Unexpected subsystem import: ' + fullname)
sys.meta_path.insert(0, Guard())
'''.replace('BLOCKED', repr(blocked)) + code
    result = subprocess.run([sys.executable, '-c', script], cwd=ROOT, capture_output=True, text=True)
    assert result.returncode == 0, result.stderr


def test_recorder_imports_no_devices_or_control():
    isolated_import('from dexretarget.dataset.recorder import Recorder\n',
                    ['dexretarget.app', 'dexretarget.teleop', 'dexretarget.camera', 'pyrealsense2', 'pinocchio'])


def test_camera_imports_no_recorder_control_or_sdk_until_worker_runs():
    isolated_import('from dexretarget.camera.processes import CameraProcesses\n',
                    ['dexretarget.dataset', 'dexretarget.teleop', 'dexretarget.app', 'pyrealsense2', 'h5py', 'pinocchio'])


def test_control_imports_no_recording_or_camera():
    isolated_import('from dexretarget.teleop.loop import TeleopLoop\nfrom dexretarget.teleop.devices.rokae import MockRokaeDriver\n',
                    ['dexretarget.dataset', 'dexretarget.camera', 'dexretarget.app', 'pyrealsense2', 'h5py', 'pinocchio'])


def test_quest_communication_imports_no_robot_camera_or_recorder():
    isolated_import('from dexretarget.teleop.devices.quest_relay import QuestRelayServer, QuestRelayInput\n',
                    ['dexretarget.app', 'dexretarget.dataset', 'dexretarget.camera', 'rokae',
                     'dexretarget.teleop.devices.rokae', 'dexretarget.teleop.devices.inspire',
                     'pyrealsense2', 'h5py', 'pinocchio'])


def test_cli_help_requires_no_device_or_storage_packages():
    isolated_import("from dexretarget.app.cli import build_parser\nbuild_parser().parse_args(['--help'])\n",
                    ['numpy', 'h5py', 'pyrealsense2', 'pinocchio', 'dexretarget.teleop', 'dexretarget.dataset'])


def test_legacy_imports_preserve_class_identity():
    from integrations.input.quest3 import Quest3 as old_quest
    from dexretarget.teleop.devices.quest3 import Quest3
    from integrations.output.real.drivers_rokae import MockRokaeDriver as old_arm
    from dexretarget.teleop.devices.rokae import MockRokaeDriver
    from integrations.output.real.drivers_inspire import InspireSerialOutput as old_hand
    from dexretarget.teleop.devices.inspire import InspireSerialOutput
    from anydexretarget.teleop.pose import make_transform as old_transform
    from anydexretarget.geometry import make_transform
    assert (old_quest, old_arm, old_hand, old_transform) == (Quest3, MockRokaeDriver, InspireSerialOutput, make_transform)


def test_cli_and_realtime_settings_match_pre_refactor_fixtures():
    from dexretarget.app.cli import build_parser
    from dexretarget.teleop.rokae import configured_realtime
    parser = build_parser()
    actual = [{key: str(getattr(a,key)) if isinstance(getattr(a,key),Path) else getattr(a,key)
               for key in ['option_strings','dest','nargs','default','choices','required']}
              for a in parser._actions if a.dest not in {'print_arm_config', 'config', 'quest3_usb', 'quest3_source', 'quest3_relay_socket', 'timing_report'}]
    assert json.loads(json.dumps(actual)) == json.loads((ROOT/'tests/fixtures/legacy_cli.json').read_text())
    fixture = json.loads((ROOT/'tests/fixtures/legacy_realtime.json').read_text())
    for case in fixture['cases']:
        args = parser.parse_args([])
        for key,value in case['override'].items():
            setattr(args,key,value)
        result = configured_realtime(args,fixture['robot'],fixture['safety'])
        actual = vars(result).copy()
        assert actual.pop('response_hz') == 0.0  # Legacy configs preserve their recurrence.
        assert json.loads(json.dumps(actual)) == case['expected']


def test_resource_cleanup_continues_after_error_and_runs_once():
    from dexretarget.lifecycle import close_resource, close_resources
    calls = []
    def fail():
        calls.append('fail')
        raise OSError('close failed')
    resources = {'first': lambda: calls.append('first'), 'broken': fail, 'last': lambda: calls.append('last')}
    assert len(close_resources(resources)) == 1
    close_resources(resources)
    assert calls == ['last','fail','first'] and resources == {}


def test_camera_snapshot_and_wire_values_do_not_reload_collection(monkeypatch):
    from dexretarget.camera import processes
    from dexretarget.config import load_camera_config
    import base64
    record = {'t':123.456,'sequence':7,'payload':{
        'jpeg':base64.b64encode(b'jpeg-bytes').decode(), 'depth_png':'',
        'device_time_ms':456.789, 'capture_t':123.45, 'calibration_json':'{}'}}
    class Pipe(io.BytesIO):
        def close(self):
            self.snapshot = self.getvalue()
            super().close()
    class Process:
        def __init__(self,*args,**kwargs):
            self.stdin = Pipe()
            self.stdout = io.BytesIO((processes.PREFIX+json.dumps(record)+'\n').encode())
        def poll(self): return 0
        def wait(self,timeout=None): return 0
    monkeypatch.setattr(processes.subprocess,'Popen',Process)
    options = load_camera_config()
    records = []
    cameras = processes.CameraProcesses(options,lambda *args:records.append(args),warmup_s=0)
    options['width'] = 1
    cameras.start()
    for thread in cameras.threads: thread.join(1)
    cameras.close()
    assert len(records) == 2
    for values in records:
        assert values[1:3] == (123.456,7)
        assert values[3]['jpeg'] == b'jpeg-bytes' and values[4] == 456.789
    assert json.loads(cameras.processes[0].stdin.snapshot)['cameras']['width'] == 640
    with pytest.raises(RuntimeError): cameras.start()


def test_camera_partial_start_closes_first_process(monkeypatch):
    from dexretarget.camera import processes
    from dexretarget.config import load_camera_config
    created = []
    class Process:
        def __init__(self,*args,**kwargs):
            if created: raise OSError('second camera launch failed')
            self.stdin = io.BytesIO()
            self.stdout = io.BytesIO()
            self.waited = False
            created.append(self)
        def poll(self): return 0
        def wait(self,timeout=None): self.waited=True; return 0
    monkeypatch.setattr(processes.subprocess,'Popen',Process)
    cameras = processes.CameraProcesses(load_camera_config(),lambda *args:None)
    with pytest.raises(OSError,match='second camera'):
        cameras.start()
    assert created[0].waited and created[0].stdout.closed and created[0].stdin.closed


def prime_recorder(tmp_path):
    from dexretarget.dataset.recorder import Recorder
    from dexretarget.app.simulation import SyntheticRig
    recorder = Recorder()
    recorder.config['output_root'] = str(tmp_path)
    recorder.config['min_free_disk_gb'] = 0
    rig = SyntheticRig(recorder)
    rig.start()
    deadline = time.monotonic()+2
    while recorder.readiness():
        assert time.monotonic() < deadline
        time.sleep(.01)
    return recorder,rig


def test_detached_writer_remains_owned_until_closed(tmp_path):
    recorder,rig = prime_recorder(tmp_path)
    try:
        recorder.start_episode('test detached owner')
        writer,_ = recorder.detach_episode()
        with pytest.raises(RuntimeError): recorder.start_episode('must finish pending')
        recorder.close()
        assert not writer.is_running and recorder._pending == {}
        assert not json.loads((recorder.last_episode/'metadata.json').read_text())['success']
    finally:
        rig.close()
        recorder.close()


def test_start_metadata_failure_drains_writer_and_keeps_partial_invalid(tmp_path,monkeypatch):
    from dexretarget.dataset import recorder as module
    recorder,rig = prime_recorder(tmp_path)
    def fail(*args,**kwargs): raise OSError('fsync failure')
    monkeypatch.setattr(module,'write_json_atomic',fail)
    try:
        with pytest.raises(OSError,match='fsync failure'):
            recorder.start_episode('fail start')
        assert recorder.writer is None and not recorder._pending
        metadata = json.loads((recorder.last_episode/'metadata.json').read_text())
        assert metadata['status']=='invalid' and recorder.last_episode.name.startswith('.partial-')
        assert not any(t.name=='episode-hdf5-writer' and t.is_alive() for t in threading.enumerate())
    finally:
        rig.close()
        recorder.close()


def test_writer_timeout_also_bounds_full_queue_sentinel_wait(tmp_path):
    from dexretarget.dataset.storage import EpisodeWriter
    class BlockedThread:
        def is_alive(self): return True
        def join(self,timeout=None): pass
    writer = EpisodeWriter.__new__(EpisodeWriter)
    writer._thread = BlockedThread()
    writer.queue = queue.Queue(1)
    writer.queue.put('occupied')
    writer.partial = tmp_path
    start = time.monotonic()
    with pytest.raises(TimeoutError): writer.drain(timeout=.02)
    assert time.monotonic()-start < .5


@pytest.mark.parametrize('measured,error',[(np.zeros(7),'max-joint-delta'),(np.full(7,np.nan),'Invalid measured')])
def test_joint_tool_enforces_advertised_delta_before_motion(monkeypatch,measured,error):
    from tools.robot import rokae_joint_tool as tool
    calls=[]
    class Driver:
        def __init__(self,**kwargs): pass
        def connect(self): pass
        def get_joint_positions(self): return measured
        def get_tcp_pose(self): return np.eye(4)
        def move_to_init(self,*args,**kwargs): calls.append('move')
        def disconnect(self): calls.append('close')
    monkeypatch.setattr(tool,'RokaeXCoreDriver',Driver)
    monkeypatch.setattr(sys,'argv',['rokae_joint_tool','move','--joints','1','0','0','0','0','0','0','--max-joint-delta','.05','--execute','--yes'])
    with pytest.raises(ValueError,match=error):
        tool.main()
    assert calls==['close']


@pytest.mark.parametrize('option',['--max-joint-delta','--timeout'])
def test_joint_tool_rejects_nan_limits_before_connect(monkeypatch,option):
    from tools.robot import rokae_joint_tool as tool
    def unexpected_connect(**kwargs): raise AssertionError('Must reject arguments before creating driver')
    monkeypatch.setattr(tool,'RokaeXCoreDriver',unexpected_connect)
    monkeypatch.setattr(sys,'argv',['rokae_joint_tool','move','--joints',*(['0']*7),option,'nan','--execute','--yes'])
    with pytest.raises(SystemExit) as error:
        tool.main()
    assert error.value.code==2


def test_camera_compatibility_command_preserves_exit_status(monkeypatch):
    import runpy,types
    fake=types.ModuleType('tools.diagnostics.check_cameras')
    fake.main=lambda:7
    monkeypatch.setitem(sys.modules,'tools.diagnostics.check_cameras',fake)
    with pytest.raises(SystemExit) as exit:
        runpy.run_path(str(ROOT/'realsense/scripts/check_cameras.py'),run_name='__main__')
    assert exit.value.code==7


def test_control_target_and_timestamp_are_published_together_and_copied():
    from dexretarget.teleop.loop import TeleopLoop
    from types import SimpleNamespace
    published=threading.Event()
    target=np.arange(6,dtype=np.float64)
    source_timestamp = time.monotonic()
    class Input:
        def get_hand_tracking_data(self,hand):
            return {'fingers':np.zeros((21,3)),'landmarks_timestamp':source_timestamp,'wrist':None}
    class Retargeter:
        def retarget(self,points): return target
    class Observer:
        def tracking(self,tracking,sequence): published.set()
    loop=TeleopLoop(Input(),'right',SimpleNamespace(lock=threading.RLock(),enabled=False),retargeter=Retargeter(),observer=Observer())
    loop.start()
    try:
        assert published.wait(1)
        deadline=time.monotonic()+1
        while loop.snapshot()[0] is None:
            assert time.monotonic()<deadline
            time.sleep(.001)
        qpos,command,timestamp=loop.snapshot()
        assert timestamp==source_timestamp and qpos.dtype==np.float32
        np.testing.assert_equal(qpos,target)
        target[:]=0
        qpos[:]=-1
        np.testing.assert_equal(loop.snapshot()[0],np.arange(6))
    finally:
        loop.close()
    assert not loop.thread.is_alive() and loop.error is None


def test_sources_borrow_devices_and_join_readers_before_return():
    from dexretarget.app.sources import CaptureSources
    from dexretarget.dataset.config import load_config
    from types import SimpleNamespace
    records=[]
    arm_read=threading.Event();hand_read=threading.Event()
    class Arm:
        def get_state(self):
            arm_read.set()
            return {'tcp_pose':np.eye(4),'joint_position':np.zeros(7),'joint_velocity':np.zeros(7),'timestamp':123}
        def disconnect(self): raise AssertionError('Borrowed arm was disconnected')
    class Hand:
        def enable_feedback(self,timeout): self.timeout=timeout
        def read_feedback(self,**kwargs):
            hand_read.set()
            now=time.monotonic()
            return {'angle':np.zeros(6),'read_start':now,'read_end':now}
        def close(self): raise AssertionError('Borrowed hand was closed')
    sink=SimpleNamespace(emit=lambda *values:records.append(values),event=lambda *values:None)
    hand=Hand()
    sources=CaptureSources(load_config(),sink)
    sources.start(Arm(),hand,cameras=False)
    assert arm_read.wait(1) and hand_read.wait(1)
    sources.close()
    count=len(records)
    time.sleep(.03)
    assert len(records)==count and all(not thread.is_alive() for thread in sources.threads)
    assert hand.timeout==sources.config['hand_feedback_timeout_s']
    arm_record=next(record for record in records if record[0]=='arm_state')
    assert arm_record[4]==123 and arm_record[3]['read_start']<=arm_record[1]
    with pytest.raises(RuntimeError): sources.start(cameras=False)


def test_camera_failed_shutdown_reports_reader_without_blocking_stream_close():
    from dexretarget.camera.processes import CameraProcesses
    from dexretarget.config import load_camera_config
    class LockedReader:
        closed=False
        def close(self): raise AssertionError('Must not close beneath blocked readline')
    class Process:
        stdin=None
        stdout=LockedReader()
        def poll(self): return 0
        def wait(self,timeout=None): return 0
    class StuckThread:
        name='stuck-camera-reader'
        def join(self,timeout=None): assert timeout==2
        def is_alive(self): return True
    sources=CameraProcesses(load_camera_config(),lambda *args:None)
    sources.processes=[Process()];sources.threads=[StuckThread()]
    with pytest.raises(RuntimeError,match='did not stop'):
        sources.close()


def test_owned_module_graph_and_document_links():
    from tools.check_project import check
    assert check()['errors']==[]


@pytest.mark.parametrize('mode',['plain','paused','stale'])
def test_episode_schema_and_contents_match_pre_refactor_fingerprint(tmp_path,mode):
    from test_data_collection import fixture_episode
    from episode_fingerprint import fingerprint
    expected=json.loads((ROOT/'tests/fixtures/legacy_episode.json').read_text())['cases'][mode]
    episode=fixture_episode(tmp_path,paused=mode=='paused',stale=mode=='stale')
    assert fingerprint(episode)==expected
