"""Workflow tests: shared gate, task precedence, asynchronous save, home exclusion."""
from argparse import Namespace
from pathlib import Path
import threading
import time

import pytest
import yaml

from anydexretarget.teleop.session import TeleopSession
from anydexretarget.teleop.task import ROOT, apply_task, load_task


class FakeCollector:
    def __init__(self):
        self.writer = None
        self.controls = []
        self.labels = []
        self.saving = threading.Event()
        self.saved = threading.Event()
        self.saved.set()

    def control(self, enabled, reason):
        self.controls.append((enabled, reason))

    def start_episode(self, task):
        if self.writer:
            raise RuntimeError('already recording')
        self.writer = task
        return 'partial'

    def detach_episode(self):
        if self.writer is None:
            raise RuntimeError('not recording')
        result, self.writer = self.writer, None
        return result

    def finish_episode(self, detached, success, reason="operator"):
        self.saving.set()
        self.saved.wait(2)
        self.labels.append((detached, success))
        return 'complete'


def make_session(collector=None, home=None, ready=lambda: None):
    changes = []
    return TeleopSession(collector=collector, task='pick cup', set_following=changes.append,
                         ready=ready, home=home, cancel_home=lambda: None,
                         recenter=lambda: None, quit_event=threading.Event()), changes


def finish(session):
    session.worker.join(2)
    assert not session.worker.is_alive()


def test_enable_before_record_and_pause_does_not_home():
    home_calls = []
    collector = FakeCollector()
    session, changes = make_session(collector, lambda cancel: home_calls.append('home'))
    with pytest.raises(RuntimeError):
        session.handle('r')
    session.handle('e')
    session.handle('r')
    finish(session)
    # No home condition is imposed on start; repositioning before r is allowed.
    assert collector.writer == 'pick cup'
    session.handle('p')
    assert changes == [True, False] and not session.enabled and not home_calls
    with pytest.raises(RuntimeError):
        session.handle('h')
    session.handle('e')
    assert session.enabled and collector.writer is not None
    session.handle('f')
    finish(session)
    assert collector.labels == [('pick cup', False)]
    session.close()


def test_save_is_detached_before_disk_work_and_controls_remain_responsive():
    collector = FakeCollector()
    session, _ = make_session(collector)
    session.handle('e')
    session.handle('r')
    finish(session)
    collector.saved.clear()
    session.handle('s')
    assert collector.writer is None
    assert collector.saving.wait(1)
    session.handle('p')
    assert not session.enabled and session.operation == 'save'
    with pytest.raises(RuntimeError):
        session.handle('r')
    collector.saved.set()
    finish(session)
    assert collector.labels == [('pick cup', True)]
    session.close()


def test_home_is_exclusive_paused_and_does_not_auto_resume():
    entered, release = threading.Event(), threading.Event()
    def home(cancel):
        entered.set()
        assert release.wait(2)
    session, _ = make_session(home=home)
    session.handle('e')
    with pytest.raises(RuntimeError):
        session.handle('h')
    session.handle('p')
    session.handle('h')
    assert entered.wait(1)
    with pytest.raises(RuntimeError):
        session.handle('e')
    release.set()
    finish(session)
    assert not session.enabled and session.fault is None
    session.close()


def test_home_failure_keeps_following_disabled():
    def bad_home(cancel):
        raise OSError('feedback timeout')
    session, _ = make_session(home=bad_home)
    session.handle('h')
    finish(session)
    assert 'feedback timeout' in session.fault
    with pytest.raises(RuntimeError):
        session.handle('e')
    session.close()


def test_stale_input_rejects_enable():
    def stale():
        raise RuntimeError('stale')
    session, changes = make_session(ready=stale)
    with pytest.raises(RuntimeError):
        session.handle('e')
    assert not session.enabled and changes == []
    session.close()


@pytest.mark.parametrize("reason", ["quest_hand_stale", "quest_wrist_timeout"])
def test_automatic_pause_preserves_true_reason_for_console_and_recording(reason, capsys):
    collector = FakeCollector()
    session, changes = make_session(collector)
    session.handle('e')
    session.following(False, reason=reason)
    assert changes == [True, False]
    assert not session.enabled and session.following_reason == reason
    assert collector.controls[-1] == (False, reason)
    assert f'PAUSED (reason={reason})' in capsys.readouterr().out
    session.handle('e')
    assert session.following_reason == 'keyboard_enable'
    session.handle('p')
    assert session.following_reason == 'keyboard_pause'
    session.close()


def test_task_reference_and_explicit_cli_precedence():
    args = Namespace(task_config=ROOT/'config/tasks/cup.yaml', task='CLI instruction',
                     collection_config=None, capture=False, inspire_port='/dev/test',
                     arm_config='legacy', hand='left')
    profile = apply_task(args, {'task', 'inspire_port', 'capture'})
    assert args.task == 'CLI instruction' and args.inspire_port == '/dev/test'
    assert args.hand == 'right' and args.capture is False
    assert args.arm_config == str(ROOT/'config/robots/rokae.yaml')
    assert args.collection_config == ROOT/'config/collection/collection.json'
    assert len(profile['profile_sha256']) == 64
    assert profile['home']['arm_joints'] != [0]*7


@pytest.mark.parametrize('change', [
    lambda p: p['home'].update(arm_joints=[0]*6),
    lambda p: p['home'].update(hand_angles=[1001]*6),
    lambda p: p['home'].update(joint_speed=float('nan')),
    lambda p: p.update(teleop={'inspire_prt': '/dev/ttyUSB0'}),
    lambda p: p.update(collection={'enabled': 'false'}),
])
def test_invalid_task_rejected_before_hardware(tmp_path, change):
    profile = yaml.safe_load((ROOT/'config/tasks/cup.yaml').read_text())
    change(profile)
    path = tmp_path/'task.yaml'
    path.write_text(yaml.safe_dump(profile))
    with pytest.raises((ValueError, TypeError)):
        load_task(path)


def test_home_cancel_latches_fault_and_does_not_run_later_steps():
    entered = threading.Event()
    calls = []
    def home(cancel):
        calls.append('begin')
        entered.set()
        assert cancel.wait(2)
        # Worker must not schedule the next waypoint after cancellation.
        if not cancel.is_set():
            calls.append('next')
    session, _ = make_session(home=home)
    session.handle('h')
    assert entered.wait(1)
    session.handle('p')
    finish(session)
    assert calls == ['begin'] and session.fault
    assert not session.enabled
    session.close()


def test_resolved_control_overrides_affect_export_compatibility():
    from dexretarget.dataset.processing import control_signature
    first = {'context': {'robot_config': {}, 'effective_control': {'translation_scale': .8}}}
    second = {'context': {'robot_config': {}, 'effective_control': {'translation_scale': .6}}}
    assert control_signature(first) != control_signature(second)
    # Task home is allowed to differ when the actual policy/control contract agrees.
    second['context']['effective_control'] = {'translation_scale': .8}
    second['context']['task_profile'] = {'task_id': 'different_task'}
    assert control_signature(first) == control_signature(second)


def test_keyboard_single_key_tty_and_terminal_restoration(monkeypatch):
    import os
    import sys
    import termios
    from anydexretarget.teleop.keyboard import Keyboard
    master, slave = os.openpty()
    stream = os.fdopen(os.dup(slave), 'r')
    before = termios.tcgetattr(slave)
    quit_event = threading.Event()
    keyboard = Keyboard(quit_event)
    try:
        monkeypatch.setattr(sys, 'stdin', stream)
        keyboard.start()
        os.write(master, b'e')  # No newline.
        deadline = time.monotonic() + 1
        received = []
        while not received and time.monotonic() < deadline:
            received.extend(keyboard.drain())
            time.sleep(.01)
        assert received == ['e']
        os.write(master, b'q')
        assert quit_event.wait(1)
        keyboard.close()
        assert termios.tcgetattr(slave) == before
    finally:
        keyboard.close()
        stream.close()
        os.close(master)
        os.close(slave)


def test_setup_failure_closes_started_quest_receiver(monkeypatch):
    import importlib
    from dexretarget.app import runtime as entry
    from dexretarget.app.cli import build_parser
    closed = []
    class Receiver:
        def __init__(self, **kwargs):
            pass
        def stop(self):
            closed.append('Quest')
    quest_module = importlib.import_module('dexretarget.teleop.devices.quest3')
    monkeypatch.setattr(quest_module, 'Quest3', Receiver)
    def invalid_config(*args):
        raise ValueError('invalid retarget config')
    monkeypatch.setattr(entry.Retargeter, 'from_yaml', invalid_config)
    args = build_parser().parse_args(['--enable-hand'])
    with pytest.raises(ValueError, match='invalid retarget config'):
        entry.run_teleop(args)
    assert closed == ['Quest']


def test_failed_enable_rolls_back_both_outputs():
    session, changes = make_session()
    def transition(enabled):
        changes.append(enabled)
        if enabled:
            raise RuntimeError('reference acquisition failed')
    session._set_following = transition
    with pytest.raises(RuntimeError, match='reference acquisition'):
        session.handle('e')
    assert not session.enabled and changes == [True, False]
    session.close()


def test_recording_limit_uses_same_background_save_path():
    collector = FakeCollector()
    session, _ = make_session(collector)
    session.handle('e')
    session.handle('r')
    finish(session)
    collector.saved.clear()
    collector.stop_for_limit('max_episode_duration')
    assert collector.writer is None and collector.saving.wait(1)
    session.handle('p')  # Limit-triggered disk drain must not block pause.
    assert not session.enabled
    collector.saved.set()
    finish(session)
    assert collector.labels == [('pick cup', False)]
    session.close()


def test_save_thread_start_failure_pauses_and_finishes_detached_writer(monkeypatch):
    collector = FakeCollector()
    session,_ = make_session(collector)
    session.handle('e')
    session.handle('r')
    finish(session)
    def failed_start(self): raise RuntimeError('cannot start worker')
    monkeypatch.setattr(threading.Thread,'start',failed_start)
    with pytest.raises(RuntimeError,match='cannot start worker'):
        session.handle('s')
    assert not session.enabled and session.operation is None
    assert collector.labels == [('pick cup',True)]
    session.close()
