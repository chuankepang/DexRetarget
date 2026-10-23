"""Device-independent recording: ingest, episode lifecycle, storage and quality."""

from __future__ import annotations

import json
import importlib.metadata
import shutil
import sys
import threading
import time
from pathlib import Path

import numpy as np

from .config import DEFAULT_CONFIG, load_config, project_path
from .storage import EpisodeWriter, trash_episode, write_json_atomic


class Recorder:
    def __init__(self, config_path=DEFAULT_CONFIG, context=None):
        self.config_path = Path(config_path).resolve()
        self.config = load_config(self.config_path)
        self.context = context or {}
        self.context["python"] = sys.version
        self.context["packages"] = {}
        for package in ["numpy", "h5py", "pyserial", "pyrealsense2", "opencv-python"]:
            try:
                self.context["packages"][package] = importlib.metadata.version(package)
            except importlib.metadata.PackageNotFoundError:
                self.context["packages"][package] = None
        self.latest = {}
        self.writer = None
        self._pending = {}
        self.last_episode = None
        self.errors = []
        self.read_failures = {"arm_state": 0, "hand_state": 0}
        self.lock = threading.RLock()
        self.lifecycle = threading.RLock()
        self.stopping = threading.Event()
        self.stop_for_limit = None

    def emit(self, stream, timestamp, sequence, payload, source_timestamp=None):
        values = {k: v.copy() if isinstance(v, np.ndarray) else v for k, v in payload.items()}
        record = (stream, timestamp, sequence, values, source_timestamp)
        with self.lock:
            self.latest[stream] = record
            if self.writer:
                self.writer.submit(*record)

    def event(self, name, detail=""):
        with self.lock:
            self.emit("event", time.monotonic(), 0, {"name": name, "detail": str(detail)})

    def control(self, enabled, reason=""):
        with self.lock:
            self.emit("control", time.monotonic(), 0, {"enabled": bool(enabled)})
        if reason:
            self.event("control", reason)

    def readiness(self):
        now = time.monotonic()
        required = ["camera_wrist", "camera_global", "arm_state", "hand_state", "quest", "control"]
        with self.lock:
            stale = [name for name in required if name not in self.latest or now - self.latest[name][1] > 0.25]
            if not stale and not self.latest["control"][3]["enabled"]:
                stale.append("control_not_active")
        return stale

    def start_episode(self, task):
        with self.lifecycle:
            if self.stopping.is_set():
                raise RuntimeError("Recorder is closed")
            if self.writer or self._pending:
                raise RuntimeError("Finish the current episode first")
            self.check_health()
            missing = self.readiness()
            if missing:
                raise RuntimeError("Sources not ready: " + ", ".join(missing))
            writer = EpisodeWriter(self.config, task, self.context)
            with self.lock:
                # The episode begins when producers can actually attach, not
                # before metadata fsync/HDF5 worker startup has completed.
                writer.metadata["start_monotonic"] = time.monotonic()
                writer.metadata["start_unix"] = time.time()
                # Prime history with real last observations, never invented zeros.
                for name, record in self.latest.items():
                    if not name.endswith("_command") and name != "event":
                        writer.submit(*record)
                self.writer = writer
            try:
                write_json_atomic(writer.partial / "metadata.json", writer.metadata)
            except Exception as exc:
                writer.error = f"Start metadata update failed: {exc}"
                self.errors.append(writer.error)
                detached = self.detach_episode()
                self.finish_episode(detached, False, "start_failed")
                raise
            self.event("episode_started")
            return writer.partial

    def detach_episode(self):
        """Cut the episode at the operator event without waiting for disk I/O."""
        with self.lifecycle:
            with self.lock:
                writer, self.writer = self.writer, None
                stopped_at = time.monotonic()
            if writer is None:
                raise RuntimeError("No active episode")
            writer.metadata["read_failures"] = dict(self.read_failures)
            detached = writer, stopped_at
            self._pending[id(writer)] = detached
            return detached

    def finish_episode(self, detached, success=False, reason="operator"):
        with self.lifecycle:
            writer, stopped_at = detached
            try:
                self.last_episode = writer.finish(success, reason, stop=stopped_at)
                return self.last_episode
            finally:
                if not writer.is_running:
                    self._pending.pop(id(writer), None)

    def stop_episode(self, success=False, reason="operator"):
        with self.lifecycle:
            return self.finish_episode(self.detach_episode(), success, reason)

    def discard_last(self):
        with self.lifecycle:
            if self.writer:
                raise RuntimeError("Stop recording before discarding an episode")
            if self.last_episode is None:
                raise RuntimeError("No episode to discard")
            path = trash_episode(self.last_episode.parent, self.last_episode.name)
            self.last_episode = None
            return path

    def check_health(self):
        errors = list(self.errors)
        with self.lock:
            writer = self.writer
        if writer and writer.error:
            errors.append(writer.error)
        if writer:
            elapsed = time.monotonic() - writer.metadata["start_monotonic"]
            stop_for_limit = self.stop_for_limit or (lambda reason: self.stop_episode(False, reason))
            if elapsed > self.config["max_episode_s"]:
                stop_for_limit("max_episode_duration")
            elif shutil.disk_usage(writer.root).free < self.config["min_free_disk_gb"] * 1e9:
                stop_for_limit("disk_space_limit")
                raise OSError("Recording stopped: low free disk space")
        if errors:
            raise RuntimeError("; ".join(errors))

    def close(self):
        self.stopping.set()
        errors = []
        with self.lifecycle:
            if self.writer:
                self.detach_episode()
            for detached in list(self._pending.values()):
                try:
                    writer, _ = detached
                    if writer.accepting:
                        self.finish_episode(detached, False, "shutdown_or_interrupt")
                    else:
                        # A prior finish may have timed out during I/O. Drain
                        # resources but never publish that incomplete episode.
                        writer.drain()
                        self._pending.pop(id(writer), None)
                        raise RuntimeError(f"Earlier finalization failed; inspect {writer.partial}")
                except Exception as exc:
                    errors.append(exc)
        if errors:
            raise RuntimeError("; ".join(str(exc) for exc in errors))
