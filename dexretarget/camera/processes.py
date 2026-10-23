"""Own one camera process per role and forward existing frame records to a sink."""
from __future__ import annotations

import base64
import json
import os
import subprocess
import sys
import threading

from dexretarget.config import PROJECT_ROOT
from .config import validate_camera_config, validate_encoding
from .worker import PREFIX


class CameraProcesses:
    def __init__(self, options, emit, *, python=None, warmup_s=5., jpeg_quality=95):
        self.options = validate_camera_config(options)
        validate_encoding(warmup_s, jpeg_quality)
        self.settings = {'cameras': self.options, 'camera_warmup_s': warmup_s, 'jpeg_quality': jpeg_quality}
        self.python = python or sys.executable
        self.emit = emit
        self.processes = []
        self.threads = []
        self.errors = []
        self.stopping = threading.Event()
        self.started = False

    def start(self):
        if self.started or self.stopping.is_set():
            raise RuntimeError('Camera sources can only be started once')
        self.started = True
        environment = os.environ.copy()
        environment['PYTHONPATH'] = str(PROJECT_ROOT) + os.pathsep + environment.get('PYTHONPATH', '')
        try:
            for camera in self.options['cameras']:
                role = camera['role']
                process = subprocess.Popen([
                    self.python, '-m', 'dexretarget.camera.worker', '--role', role,
                ], cwd=PROJECT_ROOT, env=environment, stdin=subprocess.PIPE, stdout=subprocess.PIPE)
                self.processes.append(process)
                # Parent settings and recorded metadata now share a snapshot;
                # worker startup never reloads a mutable collection config file.
                process.stdin.write((json.dumps(self.settings) + '\n').encode())
                process.stdin.close()
                thread = threading.Thread(target=self._read, args=(process, role), daemon=True,
                                          name=f'camera-{role}-reader')
                thread.start()
                self.threads.append(thread)
        except BaseException:
            try:
                self.close()
            except Exception as exc:
                self.errors.append(f'Camera startup cleanup: {exc}')
            raise

    def _read(self, process, role):
        try:
            for line in process.stdout:
                text = line.decode("utf-8")
                if not text.startswith(PREFIX):
                    continue  # Vendor SDK console diagnostics are not data records.
                record = json.loads(text[len(PREFIX):])
                payload = record["payload"]
                for key in ["jpeg", "depth_png"]:
                    payload[key] = base64.b64decode(payload[key], validate=True)
                self.emit("camera_" + role, record["t"], record["sequence"], payload,
                          payload["device_time_ms"])
            if not self.stopping.is_set():
                self.errors.append(f"Camera {role} exited ({process.wait()})")
        except Exception as exc:
            self.errors.append(f"Camera {role}: {type(exc).__name__}: {exc}")

    def close(self):
        self.stopping.set()
        errors = []
        for process in self.processes:
            try:
                if process.poll() is None:
                    process.terminate()
            except Exception as exc:
                errors.append(exc)
        for process in self.processes:
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                try:
                    process.kill()
                    process.wait(timeout=2)
                except Exception as exc:
                    errors.append(exc)
            except Exception as exc:
                errors.append(exc)
        live_readers = False
        for thread in self.threads:
            thread.join(timeout=2)
            if thread.is_alive():
                live_readers = True
                errors.append(RuntimeError(f'{thread.name} did not stop after camera shutdown'))
        for process in self.processes:
            # Closing a buffered reader underneath a blocked readline can
            # itself deadlock on its internal lock. Report a failed shutdown
            # and allow a later close() to retry after the process exits.
            streams = [process.stdin] if live_readers else [process.stdin, process.stdout]
            for stream in streams:
                if stream is not None and not stream.closed:
                    try:
                        stream.close()
                    except Exception as exc:
                        errors.append(exc)
        if errors:
            raise RuntimeError('; '.join(str(exc) for exc in errors))
