"""Concurrency and numerical regressions for host latency improvements."""
import threading
import time
from types import SimpleNamespace

import numpy as np
import pytest

from dexretarget.teleop.loop import TeleopLoop
from dexretarget.teleop.timing import TimingStats
from rokae.control.model import XmateErPro7
from test_data_collection import fake_hand


def test_arm_publication_does_not_wait_for_a_running_hand_solve():
    solving, release, published = threading.Event(), threading.Event(), threading.Event()
    gate = SimpleNamespace(lock=threading.RLock(), enabled=True, tracking_held=False)
    pose = np.eye(4)
    class Input:
        def get_hand_tracking_data(self, hand):
            return dict(fingers=np.zeros((21, 3)), landmarks_timestamp=time.monotonic(),
                        wrist=dict(position=np.zeros(3), rotation=np.eye(3), timestamp=time.monotonic()))
    class Retargeter:
        def retarget(self, fingers):
            solving.set()
            assert release.wait(2)
            return np.zeros(12)
    class Controller:
        command_timeout, status, enabled = .2, 'active', True
        def update(self, wrist, timestamp):
            return SimpleNamespace(base_target_raw=pose, timestamp=timestamp)
    class Execution:
        last_accepted_pose = pose
        def set_target(self, command):
            published.set()
            return True
        def hold(self): pass
    loop = TeleopLoop(Input(), 'right', gate, retargeter=Retargeter(),
                      arm_controller=Controller(), execution=Execution())
    # Previously completed hand frame stays fresh while the next solve runs.
    loop.last_landmarks_timestamp = time.monotonic()
    loop.start()
    try:
        assert solving.wait(1)
        assert published.wait(.15), 'arm blocked behind hand optimizer'
    finally:
        release.set()
        loop.close()
    assert loop.error is None


def test_serial_ack_wait_releases_gate_and_pause_blocks_next_write():
    hand = fake_hand()
    gate = SimpleNamespace(lock=threading.RLock(), enabled=True, tracking_held=False)
    waiting, release = threading.Event(), threading.Event()
    result, errors = [], []
    original = hand._read_packet
    def delayed_ack(*args):
        waiting.set()
        assert release.wait(2)
        return original(*args)
    hand._read_packet = delayed_ack
    def send():
        try:
            result.append(hand.send_angles([500]*6, gate=gate, source_timestamp=time.monotonic()))
        except Exception as exc:
            errors.append(exc)
    worker = threading.Thread(target=send)
    worker.start()
    try:
        assert waiting.wait(1)
        assert gate.lock.acquire(timeout=.1), 'ACK waiting blocked arm/pause gate'
        try:
            gate.enabled = False
        finally:
            gate.lock.release()
    finally:
        release.set()
        worker.join(2)
    assert not worker.is_alive() and not errors
    assert result[0]['acknowledged']
    assert hand.send_angles([600]*6, gate=gate) is None
    assert len(hand._port.writes) == 1


def test_serial_rechecks_stale_landmarks_after_waiting_for_port():
    hand = fake_hand()
    gate = SimpleNamespace(lock=threading.RLock(), enabled=True, tracking_held=False)
    assert hand.send_angles([500]*6, gate=gate, source_timestamp=time.monotonic() - 1) is None
    assert not hand._port.writes


def test_no_capture_ack_does_not_wait_for_serial_silence():
    hand = fake_hand()
    hand._feedback_enabled = False
    original = hand._port.read
    def no_extra_reads(count):
        assert hand._port.incoming, 'unnecessary read after a complete ACK'
        return original(count)
    hand._port.read = no_extra_reads
    result = hand.send_angles([500]*6)
    assert result['acknowledged'] is False  # original no-capture contract


def test_combined_kinematics_preserves_error_and_jacobian():
    robot = XmateErPro7()
    rng = np.random.default_rng(42)
    for _ in range(30):
        q = robot.neutral + rng.uniform(-.2, .2, 7)
        target = robot.fk(q + rng.uniform(-.15, .15, 7))
        expected_error = robot.local_error(q, target)
        expected_jacobian = robot.jacobian(q).copy()
        error, jacobian = robot.error_and_jacobian(q, target)
        np.testing.assert_allclose(error, expected_error, atol=1e-13)
        np.testing.assert_allclose(jacobian, expected_jacobian, atol=1e-13)
        # The reused SVD gives the same damped least-squares primary step.
        u, s, vh = np.linalg.svd(jacobian, full_matrices=True)
        direct = jacobian.T @ np.linalg.solve(jacobian @ jacobian.T + .05**2*np.eye(6), error)
        reused = vh[:6].T @ ((s / (s*s + .05**2)) * (u.T @ error))
        np.testing.assert_allclose(reused, direct, atol=1e-12)


def test_timing_is_bounded_and_rejects_invalid_samples():
    timing = TimingStats(capacity=8)
    for i in range(20):
        timing.record('test', i * .001)
    for value in [-1, float('nan'), float('inf')]:
        timing.record('test', value)
    result = timing.snapshot()['test']
    assert result['count'] == 20 and result['window_samples'] == 8
    assert result['max_ms'] == pytest.approx(19)
