"""Single-owner HDF5 writing, bounded nonblocking producers, atomic episodes."""

from __future__ import annotations

import datetime
import json
import os
import queue
import shutil
import threading
import time
import uuid
from pathlib import Path

import h5py
import numpy as np

from .config import SCHEMA, project_path, snapshot


def write_json_atomic(path, value):
    path = Path(path)
    temporary = path.with_name(path.name + ".tmp")
    with temporary.open("w") as stream:
        json.dump(value, stream, ensure_ascii=False, indent=2, allow_nan=False)
        stream.write("\n")
        stream.flush()
        os.fsync(stream.fileno())
    temporary.replace(path)


class EpisodeWriter:
    def __init__(self, config, task, context, start=None):
        if not task.strip():
            raise ValueError("An episode needs a nonempty language instruction")
        self.config = config
        self.root = project_path(config["output_root"]) / config["dataset_name"]
        self.root.mkdir(parents=True, exist_ok=True)
        if shutil.disk_usage(self.root).free < config["min_free_disk_gb"] * 1e9:
            raise OSError("Insufficient free space to start an episode")
        self.episode_id = "episode_" + datetime.datetime.now().strftime("%Y%m%d_%H%M%S_") + uuid.uuid4().hex[:10]
        self.partial = self.root / (".partial-" + self.episode_id)
        self.partial.mkdir()
        self.metadata = {
            "episode_id": self.episode_id, "schema_version": config["schema_version"],
            "task": task.strip(), "context": context,
            "start_monotonic": time.monotonic() if start is None else float(start),
            "start_unix": time.time(), "status": "recording", "success": False,
            "config": snapshot(config), "clock": "time.monotonic on a single Linux host",
        }
        write_json_atomic(self.partial / "metadata.json", self.metadata)
        self.queue = queue.Queue(config["queue_capacity"])
        self.accepting = True
        self.error = None
        self.dropped = 0
        self._lock = threading.Lock()
        self._thread = threading.Thread(target=self._run, name="episode-hdf5-writer", daemon=True)
        self._thread.start()

    def submit(self, stream, timestamp, sequence, payload, source_timestamp=None):
        # Only a short queue operation is held under this lock, never disk I/O.
        with self._lock:
            if not self.accepting or self.error:
                return False
            try:
                self.queue.put_nowait((stream, float(timestamp), int(sequence), payload,
                                       float(timestamp if source_timestamp is None else source_timestamp)))
                return True
            except queue.Full:
                self.dropped += 1
                self.error = "Episode writer queue overflow; episode cannot be used for training"
                return False

    @staticmethod
    def _append(group, name, values, dtype, shape=(), unit=""):
        variable = h5py.check_dtype(vlen=np.dtype(dtype))
        if name not in group:
            group.create_dataset(name, shape=(0, *shape), maxshape=(None, *shape),
                                 chunks=(128, *shape), dtype=dtype,
                                 compression=None if variable is not None else "lzf")
            group[name].attrs["unit"] = unit
        dataset = group[name]
        row = dataset.shape[0]
        dataset.resize(row + len(values), axis=0)
        if variable is not None:
            # Ragged byte arrays need explicit scalar writes; timestamp and
            # numerical arrays are all appended in one operation per batch.
            for offset, value in enumerate(values):
                dataset[row + offset] = value
        else:
            array = np.asarray(values, dtype=dtype)
            if array.shape != (len(values), *shape):
                raise ValueError(f"{group.name}/{name}: invalid batch shape {array.shape}")
            dataset[row:row + len(values)] = array

    def _write_batch(self, streams, batch):
        grouped = {}
        for record in batch:
            grouped.setdefault(record[0], []).append(record)
        for stream, records in grouped.items():
            group = streams.require_group(stream)
            timestamps = [r[1] for r in records]
            sources = [r[4] for r in records]
            if not np.all(np.isfinite(timestamps)) or not np.all(np.isfinite(sources)):
                raise ValueError("Raw stream timestamps must be finite")
            self._append(group, "t", timestamps, "f8", unit="host_receive_monotonic_s")
            self._append(group, "source_t", sources, "f8", unit="stream_specific_source_clock")
            self._append(group, "sequence", [r[2] for r in records], "i8")
            payloads = [r[3] for r in records]
            if stream.startswith("camera_"):
                for key in ["jpeg", "depth_png"]:
                    self._append(group, key, [np.frombuffer(p.get(key, b""), dtype="u1") for p in payloads],
                                 h5py.vlen_dtype(np.dtype("u1")))
                for key, unit in [("capture_t", "estimated_monotonic_s"), ("device_time_ms", "device_ms"),
                                  ("clock_offset_s", "unix_minus_monotonic_s")]:
                    self._append(group, key, [p[key] for p in payloads], "f8", unit=unit)
                for key in ["timestamp_domain", "calibration_json"]:
                    self._append(group, key, [p[key] for p in payloads], h5py.string_dtype("utf-8"))
                self._append(group, "depth_sequence", [p.get("depth_sequence", -1) for p in payloads], "i8")
                self._append(group, "depth_device_time_ms", [p.get("depth_device_time_ms", np.nan) for p in payloads], "f8")
                self._append(group, "depth_timestamp_domain", [p.get("depth_timestamp_domain", "disabled_or_missing") for p in payloads],
                             h5py.string_dtype("utf-8"))
            elif stream == "event":
                self._append(group, "json", [json.dumps(p, ensure_ascii=False, allow_nan=False) for p in payloads],
                             h5py.string_dtype("utf-8"))
            else:
                for name, (dtype, shape, unit) in SCHEMA[stream].items():
                    self._append(group, name, [p[name] for p in payloads], dtype, shape, unit)

    def _run(self):
        try:
            with h5py.File(self.partial / "episode.h5", "w") as file:
                file.attrs["schema_version"] = self.config["schema_version"]
                streams = file.create_group("streams")
                finished = False
                while not finished:
                    first = self.queue.get()
                    if first is None:
                        break
                    batch = [first]
                    deadline = time.monotonic() + 0.1
                    while len(batch) < 256:
                        remaining = deadline - time.monotonic()
                        if remaining <= 0:
                            break
                        try:
                            item = self.queue.get(timeout=remaining)
                        except queue.Empty:
                            break
                        if item is None:
                            finished = True
                            break
                        batch.append(item)
                    self._write_batch(streams, batch)
                file.flush()
        except Exception as exc:
            self.error = f"{type(exc).__name__}: {exc}"

    @property
    def is_running(self):
        return self._thread.is_alive()

    def drain(self, timeout=60):
        """Bound both sentinel enqueue and join; usable again after an I/O stall."""
        deadline = time.monotonic() + timeout
        while self._thread.is_alive():
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                raise TimeoutError(f"Writer did not drain; incomplete data remains at {self.partial}")
            try:
                self.queue.put(None, timeout=min(.1, remaining))
                break
            except queue.Full:
                continue
        self._thread.join(timeout=max(0, deadline - time.monotonic()))
        if self._thread.is_alive():
            raise TimeoutError(f"Writer did not finish; incomplete data remains at {self.partial}")

    def finish(self, success=False, reason="operator", stop=None):
        stop = time.monotonic() if stop is None else float(stop)
        with self._lock:
            if not self.accepting:
                raise RuntimeError("Episode was already stopped")
            self.accepting = False
        self.drain()
        self.metadata.update({"stop_monotonic": stop, "success": bool(success),
                              "stop_reason": reason, "writer_error": self.error,
                              "dropped_records": self.dropped, "status": "invalid" if self.error else "complete"})
        if not self.error:
            try:
                from .alignment import align_episode
                self.metadata["quality"] = align_episode(self.partial / "episode.h5", self.metadata)
            except Exception as exc:
                self.metadata["status"] = "invalid"
                self.metadata["alignment_error"] = f"{type(exc).__name__}: {exc}"
        write_json_atomic(self.partial / "metadata.json", self.metadata)
        if self.metadata["status"] == "invalid":
            return self.partial
        destination = self.root / self.episode_id
        # UUID paths and no replacement of an existing completed episode.
        if destination.exists():
            raise FileExistsError(destination)
        self.partial.rename(destination)
        return destination


def trash_episode(root, episode_id):
    """Reversible deletion only, never follow user-supplied directory paths."""
    if not episode_id.startswith("episode_") or Path(episode_id).name != episode_id:
        raise ValueError("Specify an episode ID, not a path")
    root = Path(root).resolve()
    source = root / episode_id
    if source.is_symlink() or not source.is_dir() or not (source / "metadata.json").is_file():
        raise ValueError("Not a completed episode directory")
    metadata = json.loads((source / "metadata.json").read_text())
    if metadata["status"] != "complete":
        raise ValueError("Cannot delete an active or incomplete episode")
    trash = root / ".trash"
    trash.mkdir(exist_ok=True)
    destination = trash / (episode_id + "_" + uuid.uuid4().hex[:8])
    source.rename(destination)
    write_json_atomic(destination / "trash.json", {"original_path": str(source), "deleted_unix": time.time()})
    return destination


def restore_episode(root, trash_id):
    if Path(trash_id).name != trash_id:
        raise ValueError("Specify a trash ID, not a path")
    root = Path(root).resolve()
    source = root / ".trash" / trash_id
    if source.is_symlink() or not source.is_dir():
        raise ValueError("Not a trash directory")
    metadata = json.loads((source / "metadata.json").read_text())
    destination = root / metadata["episode_id"]
    if destination.exists() or destination.parent != root or destination.name != Path(metadata["episode_id"]).name:
        raise ValueError("Invalid or already existing restore target")
    source.rename(destination)
    return destination
