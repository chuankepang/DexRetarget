"""Test-only entrypoint wrapper. Replaces cameras, never the production capture code."""
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from dexretarget.bootstrap import _ensure_conda_cpp_runtime
_ensure_conda_cpp_runtime()

import threading
import time
import cv2
import numpy as np
from dexretarget.app import sources


class SyntheticCameras:
    def __init__(self, options, emit, **kwargs):
        self.emit, self.config = emit, {"cameras": options}
        self.errors = []
        self.stop = threading.Event()

    def start(self):
        def run():
            config = self.config['cameras']
            shape = (config['height'], config['width'], 3)
            image = np.full(shape, 64, dtype='u1')
            jpeg = cv2.imencode('.jpg', image)[1].tobytes()
            sequence = 0
            while not self.stop.is_set():
                now = time.monotonic()
                for role in ['wrist', 'global']:
                    self.emit('camera_' + role, now, sequence, {
                        'jpeg': jpeg, 'depth_png': b'', 'capture_t': now-.001,
                        'device_time_ms': now*1000, 'clock_offset_s': 0,
                        'timestamp_domain': 'test_synthetic', 'calibration_json': '{}',
                    })
                sequence += 1
                self.stop.wait(1/30)
        self.thread = threading.Thread(target=run, daemon=True)
        self.thread.start()

    def close(self):
        self.stop.set()
        self.thread.join(2)


sources.CameraProcesses = SyntheticCameras
from dexretarget.app.cli import main
main()
