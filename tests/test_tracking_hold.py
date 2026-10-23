"""Loss/recovery semantics and the actual control-loop publication gate."""
import threading
import time
from types import SimpleNamespace

import numpy as np
import pytest

from dexretarget.teleop.tracking import TrackingHold, HandResumeBlend, fresh_timestamp
from dexretarget.teleop.loop import TeleopLoop
from test_teleop_session import make_session, FakeCollector


def test_loss_requires_continuously_fresh_input_and_does_not_replay():
    policy = TrackingHold()
    assert policy.update(False, 0) == 'hold'
    assert policy.update(False, 5) is None
    assert policy.update(True, 5.01) is None
    assert policy.update(False, 5.1) is None
    assert policy.update(True, 6) is None
    assert policy.update(True, 6.21) == 'resume'
    assert not policy.held
    assert policy.update(True, 7) is None


@pytest.mark.parametrize('timestamp', [None, float('nan'), float('inf'), 10.01, 9.0])
def test_missing_stale_future_and_nonfinite_input_is_not_fresh(timestamp):
    assert not fresh_timestamp(timestamp, 10, .2)


def test_hand_resume_starts_at_last_target_and_blends_to_current_not_old_frame():
    blend = HandResumeBlend()
    np.testing.assert_equal(blend.target(np.zeros(6), 1), 0)
    blend.resume(10)
    np.testing.assert_equal(blend.target(np.ones(6), 10), 0)
    np.testing.assert_allclose(blend.target(np.ones(6)*2, 10.25), 1)
    np.testing.assert_equal(blend.target(np.ones(6)*3, 10.5), 3)


def test_hold_does_not_clear_intent_but_manual_pause_does_and_blocks_recording():
    session, calls = make_session(FakeCollector())
    session.following(True)
    session.tracking_held = True
    assert session.enabled
    with pytest.raises(RuntimeError):
        session.handle('r')
    session.handle('p')
    session.tracking_held = False  # input recovery cannot overwrite user intent
    assert not session.enabled and calls == [True, False]


def test_input_loop_cannot_publish_during_tracking_hold_then_reanchors_on_gap():
    gate = SimpleNamespace(lock=threading.RLock(), enabled=True, tracking_held=True)
    stop = threading.Event()
    observed = threading.Event()
    calls = []
    pose = np.eye(4)
    pose[0,3] = .4
    class Input:
        def get_hand_tracking_data(self, hand):
            return {'fingers': None, 'landmarks_timestamp': None,
                    'wrist': {'timestamp': time.monotonic(), 'position': [10,0,0], 'rotation': np.eye(3)}}
    class Controller:
        command_timeout = .15
        status = 'active'
        enabled = True
        def recenter(self, wrist, robot):
            calls.append('recenter')
            np.testing.assert_equal(robot, pose)
        def update(self, wrist, timestamp):
            calls.append('update')
            return SimpleNamespace(base_target_raw=pose, timestamp=timestamp)
    class Execution:
        last_accepted_pose = pose
        def hold(self): calls.append('hold')
        def command_pose(self): return pose
        def reset_tracking(self): calls.append('reset')
        def set_target(self, target):
            calls.append('publish')
            observed.set()
            stop.set()
            return True
    loop = TeleopLoop(Input(), 'right', gate, arm_controller=Controller(), execution=Execution(), stop_event=stop)
    loop.last_wrist_timestamp = time.monotonic() - 1
    loop.start()
    try:
        assert not observed.wait(.03)
        assert calls == []
        with gate.lock:
            gate.tracking_held = False
        assert observed.wait(1)
    finally:
        loop.close()
    assert loop.error is None
    assert calls == ['hold','recenter','reset','update','publish']


def test_recovery_coordinator_gates_callbacks_events_and_respects_keyboard_pause():
    from dexretarget.app.tracking import TrackingRecovery
    collector = FakeCollector()
    session, calls = make_session(collector)
    session.following(True)
    recovery = TrackingRecovery(session)
    transitions = []
    def hold():
        assert session.enabled and session.tracking_held
        transitions.append('hold')
    def resume():
        assert session.enabled and session.tracking_held
        transitions.append('anchor_before_publish')
    def step(fresh, now):
        recovery.update(fresh, now, hold=hold, resume=resume)
    step(False, 1)
    step(False, 5)  # a long loss keeps holding, never changes keyboard intent
    step(True, 6)
    assert session.tracking_held
    step(True, 6.21)
    assert session.enabled and not session.tracking_held
    assert calls == [True]
    assert transitions == ['hold', 'anchor_before_publish']
    assert collector.controls[-2:] == [(False,'tracking_hold'), (False,'tracking_recovered_recenter')]
    step(False, 7)
    session.handle('p')
    step(True, 8)
    step(True, 9)
    assert not session.enabled
    assert transitions == ['hold','anchor_before_publish','hold']


def test_reanchor_failure_does_not_open_output_gate():
    from dexretarget.app.tracking import TrackingRecovery
    session, _ = make_session()
    session.following(True)
    recovery = TrackingRecovery(session)
    def fail(): raise RuntimeError('IK reset failed')
    recovery.update(False, 0, hold=lambda: None, resume=fail)
    recovery.update(True, 1, hold=lambda: None, resume=fail)
    with pytest.raises(RuntimeError, match='IK reset failed'):
        recovery.update(True, 2, hold=lambda: None, resume=fail)
    assert session.tracking_held
