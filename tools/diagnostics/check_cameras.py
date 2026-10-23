"""Inventory or exercise both cameras without commanding any robot.

Only the Python RealSense SDK and standard library are required. Images are
received but not retained. JSON reports contain timing, configuration and USB
metadata, not images. This is a transport/capture check, not an encoder test.
"""

import sys
import pathlib
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[2]))

import argparse
import collections
import datetime
import json
import pathlib
import statistics
import threading
import time
import traceback

import pyrealsense2 as rs

ROOT = pathlib.Path(__file__).resolve().parents[2]


def info(device, key):
    return device.get_info(key) if device.supports(key) else None


def inventory(context):
    return [
        {
            "name": info(d, rs.camera_info.name),
            "serial": info(d, rs.camera_info.serial_number),
            "firmware": info(d, rs.camera_info.firmware_version),
            "usb_type": info(d, rs.camera_info.usb_type_descriptor),
            "physical_port": info(d, rs.camera_info.physical_port),
        }
        for d in context.query_devices()
    ]


def resolve_cameras(config, devices):
    resolved = []
    for camera in config["cameras"]:
        serial = camera.get("serial")
        candidates = [d for d in devices if (
            d["serial"] == serial if serial else camera["model"].upper() in d["name"].upper()
        )]
        if len(candidates) != 1:
            raise ValueError(f"Expected exactly one camera for {camera}, found {candidates}")
        resolved.append({**camera, "serial": candidates[0]["serial"]})
    if len({c["serial"] for c in resolved}) != len(resolved):
        raise ValueError("Each role must reference a different camera")
    return resolved


def percentile(values, fraction):
    if not values:
        return None
    return sorted(values)[round((len(values) - 1) * fraction)]


def summarize(samples, duration, requested_fps):
    if not samples:
        return {"frames": 0, "passed": False}
    numbers = [s[0] for s in samples]
    receipt = [s[1] for s in samples]
    intervals = [(b - a) * 1000 for a, b in zip(receipt, receipt[1:])]
    gaps = sum(max(0, b - a - 1) for a, b in zip(numbers, numbers[1:]))
    regressions = sum(b <= a for a, b in zip(numbers, numbers[1:]))
    span = receipt[-1] - receipt[0]
    fps = (len(samples) - 1) / span if span > 0 else 0
    sensor_intervals = [b[2] - a[2] for a, b in zip(samples, samples[1:])]
    return {
        "frames": len(samples), "host_fps": fps,
        "frames_per_test_second": len(samples) / duration,
        "frame_number_gaps": gaps, "duplicate_or_regressed_frame_numbers": regressions,
        "host_interval_ms_median": statistics.median(intervals) if intervals else None,
        "host_interval_ms_p99": percentile(intervals, 0.99),
        "host_interval_ms_max": max(intervals, default=None),
        "sensor_interval_ms_median": statistics.median(sensor_intervals) if sensor_intervals else None,
        "timestamp_domains": sorted({s[3] for s in samples}),
        "passed": fps >= requested_fps * 0.97 and len(samples) >= duration * requested_fps * 0.95
        and gaps == 0 and regressions == 0,
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=pathlib.Path, default=ROOT / "config/cameras/realsense.json")
    parser.add_argument("--inventory", action="store_true")
    parser.add_argument("--bind-serials", action="store_true", help="Bind uniquely matched models to role serials")
    parser.add_argument("--depth", action="store_true", help="Also capture Z16 depth on both cameras")
    parser.add_argument("--only", choices=["wrist", "global"], help="Isolate one camera for diagnosis")
    parser.add_argument("--unpaired", action="store_true", help="Measure independent sensor callbacks, bypassing frameset reuse")
    parser.add_argument("--color-exposure", type=float, help="Manual RGB exposure in sensor SDK units; default is auto")
    parser.add_argument("--seconds", type=float, default=60)
    parser.add_argument("--warmup", type=float, default=5)
    parser.add_argument("--output", type=pathlib.Path)
    args = parser.parse_args()
    if args.seconds <= 0 or args.warmup < 0:
        parser.error("seconds must be positive and warmup nonnegative")
    ctx = rs.context()
    devices = inventory(ctx)
    print(json.dumps(devices, ensure_ascii=False, indent=2), flush=True)
    if args.inventory and not args.bind_serials:
        return 0 if devices else 1
    from dexretarget.config import load_camera_config
    config = load_camera_config(args.config, require_bound=False)
    cameras = resolve_cameras(config, devices)
    if args.bind_serials:
        config["cameras"] = cameras
        args.config.write_text(json.dumps(config, ensure_ascii=False, indent=2) + "\n")
        print(f"Saved camera role bindings: {args.config}", flush=True)
        if args.inventory:
            return 0
    if args.only:
        cameras = [c for c in cameras if c["role"] == args.only]
    depth = args.depth or config.get("enable_depth", False)
    if config.get("color_format", "rgb8") != "rgb8" or config.get("depth_format", "z16") != "z16":
        raise ValueError("This checker supports rgb8 color and z16 depth only")
    stop = threading.Event()
    samples = {c["role"]: collections.defaultdict(list) for c in cameras}
    errors = []
    timeouts = collections.Counter()
    profiles = {}
    sensor_options = {}
    pipelines, threads, raw_sensors = [], [], []
    window = {"start": float("inf"), "end": float("inf")}

    def record_frame(role, frame, now):
        kind = frame.get_profile().stream_type()
        key = "color" if kind == rs.stream.color else "depth" if kind == rs.stream.depth else None
        if key is not None and window["start"] <= now < window["end"]:
            samples[role][key].append((frame.get_frame_number(), now, frame.get_timestamp(),
                                       str(frame.get_frame_timestamp_domain())))

    def raw_callback(role, frame):
        try:
            if not stop.is_set():
                record_frame(role, frame, time.monotonic())
        except Exception:
            errors.append({"role": role, "error": traceback.format_exc()})
            stop.set()

    def receive(role, pipeline):
        try:
            while not stop.is_set():
                success, frames = pipeline.try_wait_for_frames(1000)
                now = time.monotonic()
                if not success:
                    if window["start"] <= now < window["end"]:
                        timeouts[role] += 1
                    continue
                if not window["start"] <= now < window["end"]:
                    continue
                for key, frame in (("color", frames.get_color_frame()), ("depth", frames.get_depth_frame())):
                    if frame:
                        record_frame(role, frame, now)
        except Exception:
            errors.append({"role": role, "error": traceback.format_exc()})
            stop.set()

    try:
        for camera in cameras:
            pipeline = rs.pipeline(ctx)
            cfg = rs.config()
            cfg.enable_device(camera["serial"])
            cfg.enable_stream(rs.stream.color, config["width"], config["height"], rs.format.rgb8, config["fps"])
            if depth:
                cfg.enable_stream(rs.stream.depth, config["width"], config["height"], rs.format.z16, config["fps"])
            active = cfg.resolve(pipeline) if args.unpaired else pipeline.start(cfg)
            if not args.unpaired:
                pipelines.append(pipeline)
            profiles[camera["role"]] = [
                {"stream": str(p.stream_type()), "format": str(p.format()), "fps": p.fps(),
                 "width": p.as_video_stream_profile().width(), "height": p.as_video_stream_profile().height()}
                for p in active.get_streams()
            ]
            # Keep auto exposure from reducing the requested color frame rate.
            for sensor in active.get_device().query_sensors():
                if sensor.supports(rs.option.auto_exposure_priority):
                    sensor.set_option(rs.option.auto_exposure_priority, float(config.get("color_auto_exposure_priority", False)))
                if sensor.is_color_sensor():
                    if args.color_exposure is None:
                        sensor.set_option(rs.option.enable_auto_exposure, float(config.get("color_auto_exposure", True)))
                    else:
                        sensor.set_option(rs.option.enable_auto_exposure, 0)
                        sensor.set_option(rs.option.exposure, args.color_exposure)
                    sensor_options[camera["role"]] = {
                        str(option): sensor.get_option(option)
                        for option in [rs.option.enable_auto_exposure, rs.option.exposure,
                                       rs.option.auto_exposure_priority, rs.option.power_line_frequency]
                        if sensor.supports(option)
                    }
            if args.unpaired:
                for sensor in active.get_device().query_sensors():
                    ids = {p.unique_id() for p in sensor.get_stream_profiles()}
                    selected = [p for p in active.get_streams() if p.unique_id() in ids]
                    if selected:
                        sensor.open(selected)
                        raw_sensors.append(sensor)
                        sensor.start(lambda frame, role=camera["role"]: raw_callback(role, frame))
            else:
                thread = threading.Thread(target=receive, args=(camera["role"], pipeline), daemon=True)
                thread.start()
                threads.append(thread)
        print(f"Warming up {len(cameras)} camera(s) for {args.warmup:g}s", flush=True)
        stop.wait(args.warmup)
        window["start"] = time.monotonic()
        window["end"] = window["start"] + args.seconds
        while not stop.is_set() and time.monotonic() < window["end"]:
            stop.wait(min(10, max(0, window["end"] - time.monotonic())))
            print({r: {k: len(v) for k, v in s.items()} for r, s in samples.items()}, flush=True)
    finally:
        stop.set()
        for thread in threads:
            thread.join(timeout=2)
        for pipeline in reversed(pipelines):
            pipeline.stop()
        for sensor in reversed(raw_sensors):
            try:
                sensor.stop()
            finally:
                sensor.close()
    report = {
        "utc": datetime.datetime.now(datetime.timezone.utc).isoformat(),
        "devices": devices, "camera_roles": cameras, "profiles": profiles,
        "color_sensor_options_at_start": sensor_options,
        "duration_seconds": args.seconds, "warmup_seconds": args.warmup,
        "depth_enabled": depth, "errors": errors, "timeouts": dict(timeouts),
        "capture_mode": "independent_sensor_callbacks" if args.unpaired else "pipeline_framesets",
        "scope": "Concurrent capture only; excludes encoding, disk recording, policy inference and robot load.",
        "criteria": "Each stream >=97% requested FPS, >=95% expected frames, zero frame counter gaps/regressions, no timeouts/errors.",
        "streams": {},
    }
    for camera in cameras:
        role = camera["role"]
        report["streams"][role] = {
            key: summarize(samples[role][key], args.seconds, config["fps"])
            for key in (["color", "depth"] if depth else ["color"])
        }
    report["passed"] = not errors and not any(timeouts.values()) and all(
        s["passed"] for streams in report["streams"].values() for s in streams.values()
    )
    pixels_per_second = len(cameras) * config["width"] * config["height"] * config["fps"]
    report["decoded_payload_MB_per_second"] = pixels_per_second * (5 if depth else 3) / 1e6
    report["payload_note"] = "Decoded RGB8/Z16 array size, not measured USB wire traffic. USB format/overhead may differ."
    output = args.output or ROOT / "data/camera_tests" / (
        datetime.datetime.now().strftime("%Y%m%d_%H%M%S") + ("_rgbd.json" if depth else "_rgb.json")
    )
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n")
    print(json.dumps(report, ensure_ascii=False, indent=2), flush=True)
    print(f"Report saved: {output}", flush=True)
    return 0 if report["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
