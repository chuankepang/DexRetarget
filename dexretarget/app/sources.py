"""Composition adapters: read borrowed robot/hand state and own camera sources.

No commands are sent except the hand feedback-protocol setup. Control devices
are created by the application once; the recorder only receives emitted records.
"""
from __future__ import annotations

import threading
import time
import numpy as np

from dexretarget.camera.processes import CameraProcesses
from dexretarget.contracts import EventSink


class CaptureSources:
    def __init__(self, config, sink: EventSink, *, read_failures=None, camera_factory=None):
        self.config, self.sink = config, sink
        self.read_failures = read_failures if read_failures is not None else {'arm_state': 0, 'hand_state': 0}
        self.camera_factory = camera_factory or CameraProcesses
        self.stopping = threading.Event()
        self.threads = []
        self.cameras = None
        self.started = False

    def start(self, arm=None, hand=None, cameras=True):
        if self.started or self.stopping.is_set():
            raise RuntimeError('Capture sources can only be started once')
        self.started = True
        try:
            if cameras:
                self.cameras = self.camera_factory(
                    self.config['cameras'], self.sink.emit,
                    python=self.config['camera_python'], warmup_s=self.config['camera_warmup_s'],
                    jpeg_quality=self.config['jpeg_quality'])
                self.cameras.start()
            if arm is not None:
                self._thread(self._arm_states, arm, 'capture-arm-state')
            if hand is not None:
                hand.enable_feedback(self.config['hand_feedback_timeout_s'])
                self._thread(self._hand_states, hand, 'capture-hand-feedback')
        except BaseException:
            try:
                self.close()
            except Exception:
                pass  # Preserve the startup failure; caller still owns device handles.
            raise

    def _thread(self, target, source, name):
        thread = threading.Thread(target=target, args=(source,), name=name, daemon=True)
        thread.start()
        self.threads.append(thread)

    def _event(self, name, detail):
        self.sink.event(name, str(detail))

    def _arm_states(self, arm):
        sequence = 0
        period = 1 / self.config["arm_state_hz"]
        while not self.stopping.is_set():
            begin = time.monotonic()
            try:
                # get_state queries are outside the realtime callback. The
                # read interval is saved because these SDK fields are not an
                # atomic robot-clock snapshot and can incur network latency.
                state = arm.get_state()
                end = time.monotonic()
                self.sink.emit("arm_state", end, sequence, {
                    "tcp": np.asarray(state["tcp_pose"]), "qpos": np.asarray(state["joint_position"]),
                    "qvel": np.asarray(state["joint_velocity"]), "read_start": begin, "read_end": end,
                }, state["timestamp"])
                sequence += 1
            except Exception as exc:
                self.read_failures["arm_state"] += 1
                self._event("arm_read_failure", str(exc))
            self.stopping.wait(max(0, period - (time.monotonic() - begin)))

    def _hand_states(self, hand):
        sequence = 0
        period = 1 / self.config["hand_feedback_hz"]
        while not self.stopping.is_set():
            begin = time.monotonic()
            try:
                feedback = hand.read_feedback(read_force=self.config["read_hand_force"])
                self.sink.emit("hand_state", feedback["read_end"], sequence, feedback)
                sequence += 1
            except Exception as exc:
                self.read_failures["hand_state"] += 1
                self._event("hand_read_failure", str(exc))
            self.stopping.wait(max(0, period - (time.monotonic() - begin)))


    def check_health(self):
        errors = list(self.cameras.errors) if self.cameras else []
        if errors:
            raise RuntimeError('; '.join(errors))

    def close(self):
        self.stopping.set()
        try:
            if self.cameras:
                self.cameras.close()
        finally:
            for thread in self.threads:
                # Device handles cannot be disconnected under outstanding SDK
                # reads. These are bounded by the device adapter's I/O timeout.
                thread.join()
