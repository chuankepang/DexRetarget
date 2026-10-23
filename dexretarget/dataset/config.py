"""Validated acquisition configuration and an explicit per-stream schema."""

from __future__ import annotations

import copy
import hashlib
import json
import math
from pathlib import Path

from dexretarget.config import PROJECT_ROOT, DEFAULT_COLLECTION_CONFIG as DEFAULT_CONFIG, project_path
from dexretarget.config import load_camera_config

# dtype, trailing shape, physical unit. Time is host monotonic seconds unless
# explicitly named device_time_ms or unix_time. Missing optional values are NaN.
SCHEMA = {
    "arm_state": {
        "tcp": ("f8", (4, 4), "base_T_tcp"),
        "qpos": ("f4", (7,), "rad"), "qvel": ("f4", (7,), "rad/s"),
        "read_start": ("f8", (), "monotonic_s"),
        "read_end": ("f8", (), "monotonic_s"),
    },
    "hand_state": {
        "angle": ("i2", (6,), "native_0_1000"),
        "force": ("i2", (6,), "native_force_register"),
        "read_start": ("f8", (), "monotonic_s"),
        "read_end": ("f8", (), "monotonic_s"),
        "force_valid": ("?", (), "bool"),
    },
    "arm_command": {
        "requested_tcp": ("f8", (4, 4), "base_T_tcp"),
        "accepted_tcp": ("f8", (4, 4), "base_T_tcp"),
        "enabled": ("?", (), "bool"),
    },
    "hand_command": {
        "angle": ("i2", (6,), "native_0_1000"),
        "acknowledged": ("?", (), "bool"),
    },
    "quest": {
        "wrist": ("f8", (4, 4), "converted_quest_T_wrist"),
        "landmarks": ("f4", (21, 3), "converted_quest_world_m"),
        "wrist_timestamp": ("f8", (), "host_receive_monotonic_s"),
        "landmarks_timestamp": ("f8", (), "host_receive_monotonic_s"),
    },
    "control": {"enabled": ("?", (), "bool")},
}


def load_config(path=DEFAULT_CONFIG):
    config = json.loads(Path(path).read_text())
    if config.get("schema_version") != "dexretarget.episode.v1":
        raise ValueError("Unsupported acquisition schema")
    for key in ["sample_hz", "arm_state_hz", "hand_feedback_hz", "max_episode_s"]:
        if type(config[key]) not in (int, float) or not math.isfinite(config[key]) or config[key] <= 0:
            raise ValueError(f"{key} must be positive")
    if int(config["sample_hz"]) != config["sample_hz"]:
        raise ValueError("sample_hz must be an integer for LeRobot export")
    if config["sample_hz"] > min(config["arm_state_hz"], config["hand_feedback_hz"]):
        raise ValueError("Observation sampling cannot exceed configured state/feedback rates")
    if not 1 <= config["jpeg_quality"] <= 100:
        raise ValueError("jpeg_quality must be 1..100")
    for key in ["queue_capacity"]:
        if type(config[key]) is not int or config[key] < 1:
            raise ValueError(f"{key} must be a positive integer")
    name = config["dataset_name"]
    if not name or any(c not in "abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789_-" for c in name):
        raise ValueError("dataset_name must be one safe path component")
    if any(type(v) not in (int, float) or not math.isfinite(v) or v <= 0 for v in config["sync"].values()):
        raise ValueError("Synchronization thresholds must be positive")
    if not math.isfinite(config["min_free_disk_gb"]) or config["min_free_disk_gb"] < 0:
        raise ValueError("min_free_disk_gb must be finite and nonnegative")
    rep = config["representation"]
    if (rep["frame"], rep["tcp"], rep["hand"], rep["state_dim"], rep["action_dim"]) != (
        "robot_base", "xyz_m_rotvec_rad", "inspire_angle_register_div_1000", 12, 12
    ):
        raise ValueError("This schema implements base-frame TCP6 + native hand6 only")
    if rep["model_action_dim"] != 32:
        raise ValueError("Keep pi05's pretrained 32-dimensional projections")
    cameras = load_camera_config(project_path(config["camera_config"]))
    if cameras["fps"] < config["sample_hz"]:
        raise ValueError("Camera FPS must cover the training sampling frequency")
    if cameras["color_format"] != "rgb8":
        raise ValueError("RGB8 is required")
    if config["storage"] != {
        "container": "hdf5", "color_encoding": "jpeg", "depth_encoding": "png16",
        "numeric_compression": "lzf", "deletion": "atomic rename to dataset/.trash; no permanent delete command"
    }:
        raise ValueError("Unsupported storage settings for this schema")
    config["cameras"] = cameras
    return config


def snapshot(config):
    result = copy.deepcopy(config)
    result["stream_schema"] = SCHEMA
    result["config_sha256"] = hashlib.sha256(json.dumps(config, sort_keys=True).encode()).hexdigest()
    return result
