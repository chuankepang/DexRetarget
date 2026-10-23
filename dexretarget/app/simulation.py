"""Marked synthetic input for end-to-end checks; never commands robot hardware."""

import threading
import time

import cv2
import numpy as np


class SyntheticRig:
    def __init__(self, collector, cameras=True):
        self.collector = collector
        self.cameras = cameras
        self.stop = threading.Event()
        self.thread = threading.Thread(target=self.run, name="synthetic-sensors", daemon=True)

    def start(self):
        self.thread.start()

    def close(self):
        self.stop.set()
        self.thread.join(timeout=2)

    def run(self):
        config = self.collector.config
        image = np.zeros((config["cameras"]["height"], config["cameras"]["width"], 3), dtype=np.uint8)
        cv2.putText(image, "SYNTHETIC TEST DATA", (30, 80), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
        jpeg = cv2.imencode(".jpg", image)[1].tobytes()
        rates = {"arm_state": config["arm_state_hz"], "hand_state": config["hand_feedback_hz"],
                 "arm_command": 50, "hand_command": 100, "quest": 50, "control": 100}
        if self.cameras:
            rates.update({"camera_wrist": config["cameras"]["fps"], "camera_global": config["cameras"]["fps"]})
        deadlines = {key: time.monotonic() for key in rates}
        sequences = {key: 0 for key in rates}
        while not self.stop.is_set():
            now = time.monotonic()
            measured = np.eye(4)
            measured[:3, 3] = [0.4 + 0.01 * np.sin(now - 0.15), 0, 0.3]
            target = measured.copy()
            target[0, 3] = 0.4 + 0.01 * np.sin(now)
            angle = np.full(6, int(500 + 100 * np.sin(now - 0.15)), dtype="i2")
            target_angle = np.full(6, int(500 + 100 * np.sin(now)), dtype="i2")
            for name, hz in rates.items():
                if now < deadlines[name]:
                    continue
                if name == "arm_state":
                    payload = {"tcp": measured, "qpos": np.zeros(7), "qvel": np.zeros(7), "read_start": now, "read_end": now}
                elif name == "hand_state":
                    payload = {"angle": angle, "force": np.zeros(6, dtype="i2"), "read_start": now,
                               "read_end": now, "force_valid": True}
                elif name == "arm_command":
                    payload = {"requested_tcp": target, "accepted_tcp": target, "enabled": True}
                elif name == "hand_command":
                    payload = {"angle": target_angle, "acknowledged": True}
                elif name == "quest":
                    payload = {"wrist": target, "landmarks": np.zeros((21, 3)),
                               "wrist_timestamp": now, "landmarks_timestamp": now}
                elif name == "control":
                    payload = {"enabled": True}
                else:
                    payload = {"jpeg": jpeg, "depth_png": b"", "capture_t": now, "device_time_ms": now * 1000,
                               "clock_offset_s": 0, "timestamp_domain": "synthetic", "calibration_json": "{}"}
                self.collector.emit(name, now, sequences[name], payload)
                sequences[name] += 1
                deadlines[name] += 1 / hz
                if deadlines[name] < now - 1 / hz:
                    deadlines[name] = now + 1 / hz
            self.stop.wait(0.001)


def run_simulation(args):
    from dexretarget.dataset.recorder import Recorder
    from .sources import CaptureSources
    from dexretarget.lifecycle import close_resources
    if args.seconds <= 0:
        raise ValueError('seconds must be positive')
    recorder = Recorder(args.config, {'simulated': True, 'control_mode': 'synthetic',
                                      'real_cameras': args.real_cameras})
    if args.output_root:
        recorder.config['output_root'] = str(args.output_root.resolve())
    sources = CaptureSources(recorder.config, recorder, read_failures=recorder.read_failures)
    rig = SyntheticRig(recorder, cameras=not args.real_cameras)
    resources = {'sources': sources.close, 'recorder': recorder.close, 'synthetic rig': rig.close}
    try:
        sources.start(cameras=args.real_cameras)
        rig.start()
        deadline = time.monotonic() + 20
        while recorder.readiness():
            sources.check_health()
            recorder.check_health()
            if time.monotonic() >= deadline:
                raise TimeoutError('Sources not ready: ' + str(recorder.readiness()))
            time.sleep(.05)
        recorder.start_episode('Synthetic pipeline validation; do not train a robot on this episode')
        deadline = time.monotonic() + args.seconds
        while time.monotonic() < deadline:
            sources.check_health()
            recorder.check_health()
            time.sleep(.01)
        print('Saved:', recorder.stop_episode(True, 'simulation_completed'))
    finally:
        close_resources(resources)
