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
                 stop_event=None, recenter_event=None):
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
        self.thread.start()

    def close(self):
        self.stop_event.set()
        if self.thread:
            self.thread.join()

    def _run(self):
        while not self.stop_event.is_set():
            try:
                tracking = self.input.get_hand_tracking_data(self.hand)
                if self.observer:
                    self.observer.tracking(tracking, self.hand_frame_count)
                timestamp = tracking['landmarks_timestamp']
                if self.retargeter is not None and tracking['fingers'] is not None and timestamp is not None and timestamp != self.last_landmarks_timestamp:
                    qpos = self.retargeter.retarget(tracking['fingers'])
                    with self.state_lock:
                        # Publish target and its timestamp atomically.
                        self.latest_qpos = np.array(qpos, dtype=np.float32, copy=True)
                        self.last_landmarks_timestamp = timestamp
                        self.hand_frame_count += 1
                if self.arm_controller is None:
                    self.stop_event.wait(.002)
                    continue
                with self.gate.lock:
                    if not self.gate.enabled or getattr(self.gate, 'tracking_held', False):
                        self.stop_event.wait(.001)
                        continue
                    from .tracking import fresh_timestamp
                    if self.retargeter is not None and not fresh_timestamp(
                            self.last_landmarks_timestamp, time.monotonic(), .20):
                        self.execution.hold()
                        self.recenter_event.set()
                        self.stop_event.wait(.001)
                        continue
                    wrist = tracking['wrist']
                    if wrist is None:
                        self.stop_event.wait(.002)
                        continue
                    timestamp = float(wrist['timestamp'])
                    from .tracking import fresh_timestamp
                    if not fresh_timestamp(timestamp, time.monotonic(), self.arm_controller.command_timeout):
                        self.stop_event.wait(.001)
                        continue
                    if self.last_wrist_timestamp is not None and timestamp <= self.last_wrist_timestamp:
                        self.stop_event.wait(.001)
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
                    command = self.arm_controller.update(pose, timestamp)
                    if not self.arm_controller.enabled:
                        self.set_command(command)
                        self.stop_event.wait(.001)
                        continue
                    from rokae.control.command import EECommand
                    sequence = self.sequence
                    self.sequence += 1
                    accepted = self.execution.set_target(EECommand(
                        pose=command.base_target_raw, timestamp=timestamp,
                        sequence=sequence, source='quest'))
                    if not accepted:
                        self.stop_event.wait(.001)
                        continue
                    self.set_command(command)
                    self.arm_frame_count += 1
                    if self.observer:
                        self.observer.arm_target(command, self.execution.last_accepted_pose, sequence)
                    if self.debug:
                        self.debug(command, sequence)
            except Exception as exc:
                self.error = exc
                return
