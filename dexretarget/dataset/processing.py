"""Inspect immutable episodes and export contiguous valid runs through LeRobot."""

from __future__ import annotations

import json
import uuid
from pathlib import Path

import h5py
import numpy as np

from .alignment import align_episode
from .storage import write_json_atomic


def validate_episode(path):
    path = Path(path)
    metadata = json.loads((path / "metadata.json").read_text())
    if metadata["status"] != "complete" or path.name.startswith(".partial-"):
        raise ValueError("Episode is incomplete/invalid; never export an active or partial writer")
    with h5py.File(path / "episode.h5", "r") as file:
        if file.attrs["schema_version"] != "dexretarget.episode.v1":
            raise ValueError("Unknown raw schema")
        for name, group in file["streams"].items():
            t = group["t"][:]
            if not np.all(np.isfinite(t)) or np.any(np.diff(t) < 0):
                raise ValueError(f"Invalid timeline: {name}")
            if any(len(field) != len(t) for field in group.values()):
                raise ValueError(f"Broken stream lengths: {name}")
        group = file["samples"]
        for field in ["state", "action"]:
            values = group[field][:]
            if values.shape != (len(group["t"]), 12):
                raise ValueError(f"Incorrect {field} dimensions")
            if not np.all(np.isfinite(values[group["valid"][:]])):
                raise ValueError(f"Nonfinite valid {field}")
        if len(group["t"]) > 1 and not np.allclose(np.diff(group["t"][:]), 1 / metadata["config"]["sample_hz"], atol=1e-6):
            raise ValueError("Training time grid is not uniform")
        segments = group["segments"][:]
        for begin, end in segments:
            if not 0 <= begin < end <= len(group["t"]) or not np.all(group["valid"][begin:end]):
                raise ValueError("Invalid contiguous segment")
        # Check image references and decode representative images, preserving
        # their raw indices rather than assuming camera frames equal row IDs.
        import cv2
        for column, name in [(0, "camera_wrist"), (1, "camera_global")]:
            if name not in file["streams"]:
                if len(segments):
                    raise ValueError(f"Missing {name}")
                continue
            camera = file["streams"][name]
            refs = group["raw_indices"][:, column][group["valid"][:]]
            if np.any(refs < 0) or np.any(refs >= len(camera["t"])):
                raise ValueError("Invalid camera frame index")
            for index in sorted(set([0, len(camera["t"]) - 1])):
                if index < 0:
                    continue
                image = cv2.imdecode(camera["jpeg"][index], cv2.IMREAD_COLOR)
                expected = metadata["config"]["cameras"]
                if image is None or image.shape != (expected["height"], expected["width"], 3):
                    raise ValueError(f"Invalid image: {name}/{index}")
    return metadata


def control_signature(record):
    """Include resolved CLI overrides, not only the original YAML defaults."""
    return json.dumps({key: record["context"].get(key) for key in
                       ["control_mode", "robot_config", "hand_config_text", "effective_control", "hand_side"]},
                      sort_keys=True)


def export_lerobot(episodes, output, repo_id, allow_simulated=False, include_failed=False):
    # Import the revision used by OpenPI, rather than claiming v3 and v2.1 are
    # interchangeable. Never silently install a different dataset library.
    from lerobot.common.datasets.lerobot_dataset import CODEBASE_VERSION, LeRobotDataset
    if CODEBASE_VERSION != "v2.1":
        raise RuntimeError("Use OpenPI's pinned LeRobot v2.1 dependency for this exporter")
    output = Path(output).resolve()
    if output.exists():
        raise FileExistsError(f"Refusing to overwrite dataset: {output}")
    metadata = [validate_episode(path) for path in episodes]
    for record in metadata:
        if record["context"].get("simulated", False) and not allow_simulated:
            raise ValueError("Synthetic episodes require explicit --allow-simulated")
        if not record["success"] and not include_failed:
            raise ValueError("Failed/interrupted episode excluded; explicitly select --include-failed if intended")
        if not record["quality"]["has_training_segment"]:
            raise ValueError("No continuous valid segment long enough to export")
    if not metadata:
        raise ValueError("Select at least one episode")
    # Reject a mixture of incompatible representations/settings in one dataset.
    fingerprints = {(r["config"]["sample_hz"], json.dumps(r["config"]["representation"], sort_keys=True),
                     r["config"]["cameras"]["width"], r["config"]["cameras"]["height"],
                     control_signature(r),
                     r["config"]["config_sha256"])
                    for r in metadata}
    if len(fingerprints) != 1:
        raise ValueError("Do not mix controller modes or acquisition configurations in a training dataset")
    config = metadata[0]["config"]
    shape = (config["cameras"]["height"], config["cameras"]["width"], 3)
    output.parent.mkdir(parents=True, exist_ok=True)
    staging = output.with_name(".partial-" + output.name + "-" + uuid.uuid4().hex[:8])
    dataset = LeRobotDataset.create(
        repo_id=repo_id, root=staging, robot_type="rokae_er3pro_inspire6", fps=config["sample_hz"],
        use_videos=False, image_writer_threads=2,
        features={
            "observation.images.global": {"dtype": "image", "shape": shape, "names": ["height", "width", "channel"]},
            "observation.images.wrist": {"dtype": "image", "shape": shape, "names": ["height", "width", "channel"]},
            "observation.state": {"dtype": "float32", "shape": (12,), "names": ["state"]},
            "action": {"dtype": "float32", "shape": (12,), "names": ["action"]},
        },
    )
    import cv2
    provenance = []
    try:
        for path, record in zip(episodes, metadata):
            with h5py.File(Path(path) / "episode.h5", "r") as file:
                samples = file["samples"]
                for begin, end in samples["segments"][:]:
                    for row in range(begin, end):
                        indices = samples["raw_indices"][row]
                        frame = {"observation.state": samples["state"][row], "action": samples["action"][row],
                                 "task": record["task"]}
                        for column, role in [(0, "wrist"), (1, "global")]:
                            encoded = file[f"streams/camera_{role}/jpeg"][indices[column]]
                            image = cv2.imdecode(encoded, cv2.IMREAD_COLOR)
                            if image is None:
                                raise ValueError(f"Corrupt image at sample {row}")
                            frame[f"observation.images.{role}"] = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
                        dataset.add_frame(frame)
                    dataset.save_episode()
                    provenance.append({"raw_episode": record["episode_id"], "sample_begin": int(begin),
                                       "sample_end_exclusive": int(end), "simulated": record["context"].get("simulated", False),
                                       "task": record["task"], "config_sha256": record["config"]["config_sha256"],
                                       "task_id": (record["context"].get("task_profile") or {}).get("task_id"),
                                       "task_profile_sha256": (record["context"].get("task_profile") or {}).get("profile_sha256")})
        if hasattr(dataset, "finalize"):
            dataset.finalize()
    finally:
        dataset.stop_image_writer()
    write_json_atomic(staging / "raw_provenance.json", {"source_segments": provenance, "representation": config["representation"]})
    # Verify the official reader before publishing the dataset directory.
    loaded = LeRobotDataset(repo_id=repo_id, root=staging)
    if len(loaded) == 0:
        raise RuntimeError("Official LeRobot reader returned no frames")
    first = loaded[0]
    if tuple(first["action"].shape) != (12,) or tuple(first["observation.state"].shape) != (12,):
        raise RuntimeError("LeRobot readback shape mismatch")
    staging.rename(output)
    return {"output": str(output), "frames": len(loaded), "episodes": len(provenance), "format": CODEBASE_VERSION}
