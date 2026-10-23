"""RealSense SDK/encoding worker. Settings arrive as an immutable stdin snapshot."""
import argparse
import base64
import json
import signal
import sys
import threading
import time
from .config import validate_camera_config, validate_encoding

PREFIX = "DXFRAME "


def camera_worker(config, role):
    import cv2
    import numpy as np
    import pyrealsense2 as rs

    options = config["cameras"]
    camera = next(c for c in options["cameras"] if c["role"] == role)
    stopped = threading.Event()
    signal.signal(signal.SIGTERM, lambda *_: stopped.set())
    pipeline = rs.pipeline()
    settings = rs.config()
    settings.enable_device(camera["serial"])
    settings.enable_stream(rs.stream.color, options["width"], options["height"], rs.format.rgb8, options["fps"])
    if options.get("enable_depth", False):
        settings.enable_stream(rs.stream.depth, options["width"], options["height"], rs.format.z16, options["fps"])
    active = pipeline.start(settings)
    try:
        device = active.get_device()
        depth_scale = device.first_depth_sensor().get_depth_scale() if options.get("enable_depth") else None
        for sensor in device.query_sensors():
            if sensor.supports(rs.option.global_time_enabled):
                sensor.set_option(rs.option.global_time_enabled, 1)
            if sensor.is_color_sensor():
                sensor.set_option(rs.option.enable_auto_exposure, float(options.get("color_auto_exposure", True)))
                if sensor.supports(rs.option.auto_exposure_priority):
                    sensor.set_option(rs.option.auto_exposure_priority, float(options.get("color_auto_exposure_priority", False)))
        calibration = {
            "serial": camera["serial"], "role": role, "depth_scale_m": depth_scale,
            "firmware": device.get_info(rs.camera_info.firmware_version),
            "usb_type": device.get_info(rs.camera_info.usb_type_descriptor),
            "extrinsics_to_robot": None, "streams": [],
        }
        for profile in active.get_streams():
            v = profile.as_video_stream_profile()
            intr = v.get_intrinsics()
            calibration["streams"].append({
                "type": str(profile.stream_type()), "format": str(profile.format()), "fps": profile.fps(),
                "width": intr.width, "height": intr.height, "fx": intr.fx, "fy": intr.fy,
                "ppx": intr.ppx, "ppy": intr.ppy, "distortion": str(intr.model), "coeffs": list(intr.coeffs),
            })
        calibration_json = json.dumps(calibration)
        ready_at = time.monotonic() + config["camera_warmup_s"]
        previous = None
        while not stopped.is_set():
            ok, frames = pipeline.try_wait_for_frames(1000)
            if not ok:
                raise TimeoutError(f"{role} camera frame timeout")
            frame = frames.get_color_frame()
            received = time.monotonic()
            if not frame or received < ready_at or frame.get_frame_number() == previous:
                continue
            previous = frame.get_frame_number()
            # Epoch-valued SDK global time can be mapped to monotonic; retain
            # the per-frame mapping and raw clock so its accuracy is auditable.
            offset = time.time() - time.monotonic()
            domain = str(frame.get_frame_timestamp_domain())
            capture = frame.get_timestamp() / 1000 - offset if "global_time" in domain else received
            rgb = np.asanyarray(frame.get_data())
            encoded_ok, jpeg = cv2.imencode(".jpg", cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR),
                                            [cv2.IMWRITE_JPEG_QUALITY, config["jpeg_quality"]])
            if not encoded_ok:
                raise RuntimeError("JPEG encoding failed")
            depth_bytes = b""
            depth_sequence = -1
            depth_device_time_ms = float("nan")
            depth_domain = "disabled_or_missing"
            depth_frame = frames.get_depth_frame()
            if depth_frame:
                encoded_ok, png = cv2.imencode(".png", np.asanyarray(depth_frame.get_data()))
                if not encoded_ok:
                    raise RuntimeError("Depth encoding failed")
                depth_bytes = png.tobytes()
                depth_sequence = depth_frame.get_frame_number()
                depth_device_time_ms = depth_frame.get_timestamp()
                depth_domain = str(depth_frame.get_frame_timestamp_domain())
            payload = {
                "jpeg": base64.b64encode(jpeg).decode("ascii"),
                "depth_png": base64.b64encode(depth_bytes).decode("ascii"),
                "capture_t": capture, "device_time_ms": frame.get_timestamp(),
                "clock_offset_s": offset, "timestamp_domain": domain,
                "calibration_json": calibration_json,
                "depth_sequence": depth_sequence, "depth_device_time_ms": depth_device_time_ms,
                "depth_timestamp_domain": depth_domain,
            }
            message = {"t": received, "sequence": previous, "payload": payload}
            print(PREFIX + json.dumps(message, separators=(",", ":")), flush=True)
    finally:
        pipeline.stop()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--role', choices=['wrist', 'global'], required=True)
    args = parser.parse_args()
    config = json.loads(sys.stdin.readline())
    config['cameras'] = validate_camera_config(config['cameras'])
    validate_encoding(config['camera_warmup_s'], config['jpeg_quality'])
    camera_worker(config, args.role)


if __name__ == '__main__':
    main()
