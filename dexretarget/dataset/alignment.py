"""Offline causal observations and future-slot command labels, with explicit masks."""

from __future__ import annotations

import h5py
import numpy as np

from .representation import encode_state


def valid_segments(valid, min_frames=1):
    edges = np.diff(np.r_[False, valid, False].astype(int))
    return [(int(a), int(b)) for a, b in zip(np.flatnonzero(edges == 1), np.flatnonzero(edges == -1))
            if b - a >= min_frames]


def align_episode(path, metadata):
    config = metadata["config"]
    sync = config["sync"]
    hz = config["sample_hz"]
    dt = 1 / hz
    start, stop = metadata["start_monotonic"], metadata["stop_monotonic"]
    # Subtracting large monotonic timestamps can put an exact slot count just
    # below an integer. Tolerate sub-nanosecond rounding, not a missing slot.
    times = start + np.arange(max(0, int(np.floor((stop - start) * hz + 1e-8)))) * dt
    required = ["camera_wrist", "camera_global", "arm_state", "hand_state", "quest", "control",
                "arm_command", "hand_command"]
    with h5py.File(path, "r+") as file:
        raw = file["streams"]
        for name, stream in raw.items():
            t = stream["t"][:]
            if np.any(~np.isfinite(t)) or np.any(np.diff(t) < 0):
                raise ValueError(f"Nonmonotonic timestamps: {name}")
            if any(len(field) != len(t) for field in stream.values()):
                raise ValueError(f"Incomplete raw record: {name}")
        n = len(times)
        state = np.full((n, 12), np.nan, dtype="f4")
        action = np.full((n, 12), np.nan, dtype="f4")
        indices = np.full((n, len(required)), -1, dtype="i8")
        valid = np.ones(n, dtype=bool)
        reasons = np.zeros(n, dtype="u4")
        ages = np.full((n, 4), np.nan, dtype="f4")
        camera_skew = np.full(n, np.nan, dtype="f4")
        action_skew = np.full(n, np.nan, dtype="f4")
        missing = [name for name in required if name not in raw or len(raw[name]["t"]) == 0]
        time_arrays = {name: raw[name]["t"][:] for name in required if name not in missing}
        camera_gaps = {}
        for name in required[:2]:
            if name in time_arrays:
                gap_indices = np.flatnonzero(np.diff(raw[name]["sequence"][:]) != 1)
                camera_gaps[name] = [(time_arrays[name][i], time_arrays[name][i + 1]) for i in gap_indices]
        for row, t in enumerate(times):
            if missing:
                reasons[row] |= 1
                continue
            if any(a <= t <= b for gaps in camera_gaps.values() for a, b in gaps):
                reasons[row] |= 256
            for column, name in enumerate(required):
                values = time_arrays[name]
                if name.endswith("_command"):
                    index = int(np.searchsorted(values, t, side="left"))
                    if index >= len(values) or values[index] >= t + dt:
                        index = -1
                else:
                    index = int(np.searchsorted(values, t, side="right")) - 1
                indices[row, column] = index
            if np.any(indices[row] < 0):
                reasons[row] |= 1
                continue
            ref = dict(zip(required, indices[row]))
            for column, (name, limit) in enumerate([
                ("camera_wrist", sync["max_camera_age_s"]), ("camera_global", sync["max_camera_age_s"]),
                ("arm_state", sync["max_arm_age_s"]), ("hand_state", sync["max_hand_age_s"]),
            ]):
                ages[row, column] = t - time_arrays[name][ref[name]]
                if ages[row, column] > limit:
                    reasons[row] |= 2
            captures = [float(raw[name]["capture_t"][ref[name]]) for name in required[:2]]
            if any(not np.isfinite(c) or c > t + 0.005 or t - c > sync["max_camera_age_s"] for c in captures):
                reasons[row] |= 2
            camera_skew[row] = abs(captures[0] - captures[1])
            if not np.isfinite(camera_skew[row]) or camera_skew[row] > sync["max_camera_skew_s"]:
                reasons[row] |= 4
            arm_index, hand_index = ref["arm_command"], ref["hand_command"]
            action_skew[row] = abs(time_arrays["arm_command"][arm_index] - time_arrays["hand_command"][hand_index])
            if action_skew[row] > sync["max_action_skew_s"]:
                reasons[row] |= 8
            controls = raw["control"]
            in_slot = (time_arrays["control"] >= t) & (time_arrays["control"] < t + dt)
            if not bool(controls["enabled"][ref["control"]]) or np.any(~controls["enabled"][:][in_slot]):
                reasons[row] |= 16
            if not bool(raw["arm_command"]["enabled"][arm_index]) or not bool(raw["hand_command"]["acknowledged"][hand_index]):
                reasons[row] |= 16
            quest = raw["quest"]
            for name in ["wrist_timestamp", "landmarks_timestamp"]:
                age = t - float(quest[name][ref["quest"]])
                if not np.isfinite(age) or age < 0 or age > sync["max_quest_age_s"]:
                    reasons[row] |= 32
            for name in ["arm_state", "hand_state"]:
                group, index = raw[name], ref[name]
                if group["read_end"][index] - group["read_start"][index] > sync["max_state_read_s"]:
                    reasons[row] |= 64
            try:
                observed_hand = raw["hand_state"]["angle"][ref["hand_state"]]
                target_hand = raw["hand_command"]["angle"][hand_index]
                if np.any((observed_hand < 0) | (observed_hand > 1000)) or np.any((target_hand < 0) | (target_hand > 1000)):
                    raise ValueError("Invalid hand register")
                state[row] = encode_state(raw["arm_state"]["tcp"][ref["arm_state"]], observed_hand)
                action[row] = encode_state(raw["arm_command"]["accepted_tcp"][arm_index], target_hand)
                if not np.all(np.isfinite(state[row])) or not np.all(np.isfinite(action[row])):
                    raise ValueError("Nonfinite state/action")
            except ValueError:
                reasons[row] |= 128
        valid = reasons == 0
        if "samples" in file:
            del file["samples"]
        aligned = file.create_group("samples")
        for name, values in {"t": times, "state": state, "action": action, "valid": valid,
                             "invalid_reasons": reasons, "raw_indices": indices,
                             "observation_age_s": ages, "camera_skew_s": camera_skew,
                             "action_skew_s": action_skew}.items():
            aligned.create_dataset(name, data=values)
        aligned.attrs["raw_index_streams"] = required
        aligned.attrs["invalid_reason_bits"] = (
            "1=missing,2=stale,4=camera_skew,8=action_skew,16=disabled_or_unacknowledged,"
            "32=quest_stale,64=slow_state_read,128=invalid_state_or_action,256=camera_frame_gap"
        )
        segments = valid_segments(valid, sync["min_segment_frames"])
        aligned.create_dataset("segments", data=np.asarray(segments, dtype="i8").reshape(-1, 2))
        return {"frames": n, "valid_frames": int(valid.sum()), "segments": segments,
                "missing_streams": missing, "has_training_segment": bool(segments),
                "invalid_reason_counts": {str(bit): int(np.count_nonzero(reasons & bit))
                                          for bit in [1, 2, 4, 8, 16, 32, 64, 128, 256]},
                "camera_frame_gap_intervals": {name: len(gaps) for name, gaps in camera_gaps.items()},
                "stream_counts": {name: len(stream["t"]) for name, stream in raw.items()}}
