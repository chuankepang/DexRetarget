"""Quest-to-arm/hand control loop. No recorder, camera, task, or UI imports."""
from __future__ import annotations

import threading
import time
import numpy as np

from anydexretarget.geometry import make_transform
from dexretarget.contracts import ControlObserver


class TeleopLoop:
    def __init__(self, input_device, hand, gate, *, retargeter=None,
                 arm_controller=None, execution=None, initial_command=None,
                 sequence=0, observer: ControlObserver | None=None, debug=None,
                 stop_event=None, recenter_event=None, timing=None):
        self.input = input_device
        self.hand = hand
        self.gate = gate
        self.retargeter = retargeter
        self.arm_controller = arm_controller
        self.execution = execution
        self.observer = observer
        self.debug = debug
        self.stop_event = stop_event or threading.Event()
        self.recenter_event = recenter_event or threading.Event()
        self.state_lock = threading.Lock()
        self.latest_qpos = None
        self.latest_command = initial_command
        self.last_landmarks_timestamp = None
        self.last_wrist_timestamp = None if initial_command is None else initial_command.timestamp
        self.hand_frame_count = self.arm_frame_count = 0
        self.sequence = sequence
        self.error = None
        self.thread = None
        self.hand_thread = None
        self.timing = timing

    def _time(self, name, start):
        if self.timing is not None:
            self.timing.record(name, time.monotonic() - start)

    def snapshot(self):
        with self.state_lock:
            return (None if self.latest_qpos is None else self.latest_qpos.copy(),
                    self.latest_command, self.last_landmarks_timestamp)

    def set_command(self, command):
        with self.state_lock:
            self.latest_command = command

    def start(self):
        if self.thread is not None:
            raise RuntimeError('Teleoperation input loop already started')
        self.thread = threading.Thread(target=self._run, name='quest-teleop-input', daemon=True)
        if self.retargeter is not None:
            self.hand_thread = threading.Thread(target=self._run_hand, name='quest-hand-retarget', daemon=True)
            self.hand_thread.start()
        self.thread.start()

    def close(self):
        self.stop_event.set()
        if self.thread:
            self.thread.join()
        if self.hand_thread:
            self.hand_thread.join()

    def _run_hand(self):
        # One owner of the optimizer; latest input only, never a frame backlog.
        while not self.stop_event.is_set():
            try:
                tracking = self.input.get_hand_tracking_data(self.hand)
                timestamp = tracking['landmarks_timestamp']
                from .tracking import fresh_timestamp
                if (tracking['fingers'] is not None
                        and fresh_timestamp(timestamp, time.monotonic(), .20)
                        and timestamp != self.last_landmarks_timestamp):
                    start = time.monotonic()
                    qpos = self.retargeter.retarget(tracking['fingers'])
                    self._time('hand_retarget', start)
                    with self.state_lock:
                        self.latest_qpos = np.array(qpos, dtype=np.float32, copy=True)
                        self.last_landmarks_timestamp = timestamp
                        self.hand_frame_count += 1
                else:
                    self.stop_event.wait(.001)
            except Exception as exc:
                self.error = exc
                return

    def _run(self):
        while not self.stop_event.is_set():
            try:
                tracking = self.input.get_hand_tracking_data(self.hand)
                if self.observer:
                    self.observer.tracking(tracking, self.hand_frame_count)
                if self.arm_controller is None:
                    continue
                wait_start = time.monotonic()
                with self.gate.lock:
                    self._time('arm_gate_wait', wait_start)
                    if not self.gate.enabled or getattr(self.gate, 'tracking_held', False):
                        continue
                    from .tracking import fresh_timestamp
                    if self.retargeter is not None and not fresh_timestamp(
                            self.last_landmarks_timestamp, time.monotonic(), .20):
                        self.execution.hold()
                        self.recenter_event.set()
                        continue
                    wrist = tracking['wrist']
                    if wrist is None:
                        continue
                    timestamp = float(wrist['timestamp'])
                    from .tracking import fresh_timestamp
                    if not fresh_timestamp(timestamp, time.monotonic(), self.arm_controller.command_timeout):
                        continue
                    if self.last_wrist_timestamp is not None and timestamp <= self.last_wrist_timestamp:
                        continue
                    if self.last_wrist_timestamp is not None and timestamp - self.last_wrist_timestamp > self.arm_controller.command_timeout:
                        self.execution.hold()
                        self.recenter_event.set()
                    self.last_wrist_timestamp = timestamp
                    pose = make_transform(wrist['position'], wrist['rotation'])
                    if self.recenter_event.is_set() or self.arm_controller.status == 'hold_timeout':
                        if self.observer:
                            self.observer.control(False, 'recenter')
                        self.arm_controller.recenter(pose, self.execution.command_pose())
                        self.execution.reset_tracking()
                        self.recenter_event.clear()
                        print('Arm reference re-centered at the current commanded TCP.')
                    start = time.monotonic()
                    command = self.arm_controller.update(pose, timestamp)
                    self._time('arm_mapping', start)
                    if not self.arm_controller.enabled:
                        self.set_command(command)
                        continue
                    from rokae.control.command import EECommand
                    sequence = self.sequence
                    self.sequence += 1
                    start = time.monotonic()
                    accepted = self.execution.set_target(EECommand(
                        pose=command.base_target_raw, timestamp=timestamp,
                        sequence=sequence, source='quest'))
                    self._time('arm_controller_publish', start)
                    if not accepted:
                        continue
                    self._time('wrist_receive_to_arm_accept', timestamp)
                    self.set_command(command)
                    self.arm_frame_count += 1
                    if self.observer:
                        self.observer.arm_target(command, self.execution.last_accepted_pose, sequence)
                    if self.debug:
                        self.debug(command, sequence)
            except Exception as exc:
                self.error = exc
                return
            finally:
                # Never sleep while owning the shared enable/recenter lock.
                self.stop_event.wait(.001)
