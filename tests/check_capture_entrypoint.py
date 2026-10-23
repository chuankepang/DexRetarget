"""Opt-in integration: fake Quest UDP + fake Inspire PTY + mock arm + real cameras.

Never connects to or commands a real robot/hand. All episodes are marked mock.
Run manually, not during ordinary unit-test discovery.
"""

import argparse
import json
import os
from pathlib import Path
import select
import socket
import struct
import subprocess
import sys
import tempfile
import threading
import time

ROOT = Path(__file__).resolve().parents[1]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--seconds", type=float, default=3)
    parser.add_argument("--synthetic-cameras", action="store_true")
    parser.add_argument("--teleop-only", action="store_true")
    parser.add_argument("--report", type=Path)
    parser.add_argument("--arm-control", choices=["pinocchio", "cartesian", "impedance"], default="cartesian")
    args = parser.parse_args()
    master, slave = os.openpty()
    port_name = os.ttyname(slave)
    stopped = threading.Event()
    counts = {"writes": 0, "reads": 0}
    probe = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    probe.bind(("127.0.0.1", 0))
    udp_port = probe.getsockname()[1]
    probe.close()

    def hand_peer():
        pending = bytearray()
        angles = [800] * 6
        while not stopped.is_set():
            if not select.select([master], [], [], 0.05)[0]:
                continue
            try:
                pending.extend(os.read(master, 4096))
            except OSError:
                continue
            while len(pending) >= 4:
                index = pending.find(b"\xeb\x90")
                if index < 0:
                    pending[:] = pending[-1:]
                    break
                if index:
                    del pending[:index]
                if len(pending) < 4 or len(pending) < pending[3] + 5:
                    break
                length = pending[3] + 5
                frame = pending[:length]
                del pending[:length]
                if sum(frame[2:-1]) & 255 != frame[-1]:
                    continue
                address = frame[5] | frame[6] << 8
                if frame[4] == 0x12:
                    angles = list(struct.unpack("<6h", frame[7:-1]))
                    payload = b"\x01"
                    counts["writes"] += 1
                else:
                    payload = struct.pack("<6h", *(angles if address == 1546 else [0] * 6))
                    counts["reads"] += 1
                reply = bytearray([0x90, 0xEB, frame[2], len(payload) + 3, frame[4], frame[5], frame[6]])
                reply.extend(payload)
                reply.append(sum(reply[2:]) & 255)
                os.write(master, reply)

    quest_shift = [0.0]
    quest_paused = threading.Event()

    def quest_peer():
        client = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        points = [[0, 0, 0]]
        for x in [-0.04, -0.02, 0, 0.02, 0.04]:
            points.extend([[x, 0.025 + joint * 0.025, 0.005 * joint] for joint in range(4)])
        landmarks = "Right landmarks:," + ",".join(str(v) for p in points for v in p)
        try:
            while not stopped.wait(0.02):
                if quest_paused.is_set():
                    continue
                packet = (f"Right wrist:,0,{quest_shift[0]},0,0,0,0,1\n" + landmarks + "\n").encode()
                client.sendto(packet, ("127.0.0.1", udp_port))
        finally:
            client.close()

    config = json.loads((ROOT / "config/collection/collection.json").read_text())
    config["output_root"] = str(ROOT / "data/camera_tests/entrypoint_smoke")
    config_path = Path(tempfile.mkdtemp(prefix="dex-entrypoint-")) / "collection.json"
    config_path.write_text(json.dumps(config))
    threads = [threading.Thread(target=hand_peer, daemon=True), threading.Thread(target=quest_peer, daemon=True)]
    for thread in threads:
        thread.start()
    process = subprocess.Popen([
        sys.executable, "-u", *([str(ROOT / "tests/run_capture_with_synthetic_cameras.py")] if args.synthetic_cameras else ["-m", "dexretarget"]),
        "--task-config", str(ROOT / "config/tasks/cup.yaml"),
        "--arm-translation-scale", "0.8",  # Explicit fixture; do not depend on operator tuning.
        "--mock-rokae", "--arm-control", args.arm_control, "--enable-hand", "--enable-arm",
        "--inspire-port", port_name, "--quest3-port", str(udp_port), "--quest3-protocol", "udp",
        "--no-capture" if args.teleop_only else "--capture", "--collection-config", str(config_path), "--task", "synthetic entrypoint integration test",
        "--arm-print-hz", "0",
    ], cwd=ROOT, stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
        text=True, bufsize=1)
    lines = []
    reader = threading.Thread(target=lambda: [lines.append(line) for line in process.stdout], daemon=True)
    reader.start()

    def wait_for(predicate, timeout=40):
        deadline = time.monotonic() + timeout
        while not predicate():
            if process.poll() is not None or time.monotonic() > deadline:
                raise RuntimeError("Entrypoint did not reach expected state:\n" + "".join(lines))
            time.sleep(0.05)

    try:
        def send(key):
            process.stdin.write(key + "\n")
            process.stdin.flush()

        wait_for(lambda: any("键盘控制" in line for line in lines))
        time.sleep(.4 if args.synthetic_cameras or args.teleop_only else 7)
        assert counts["writes"] == 0, "Paused startup must not drive the hand"
        if args.teleop_only:
            send("r")
            wait_for(lambda: any("Recording is disabled" in line for line in lines))
            send("e")
            wait_for(lambda: any("臂手跟随: ENABLED" in line for line in lines))
            time.sleep(args.seconds)
            assert counts["writes"] > 0 and counts["reads"] == 0
            send("p")
            wait_for(lambda: any("臂手跟随: PAUSED" in line for line in lines))
            paused_writes = counts["writes"]
            time.sleep(.25)
            assert counts["writes"] == paused_writes
            send("h")
            wait_for(lambda: any("Home complete" in line for line in lines))
            send("q")
            process.wait(timeout=15)
            reader.join(timeout=2)
            assert process.returncode == 0, "".join(lines)
            assert not any(line.startswith(("Saved:", "Recording:")) for line in lines)
            report = {"teleop_only": True, "serial_transactions": counts,
                      "startup_pause_enable_pause_home_quit": True}
            if args.report:
                args.report.write_text(json.dumps(report, indent=2) + "\n")
            print(json.dumps(report, indent=2))
            return
        send("r")
        wait_for(lambda: any("Press e" in line for line in lines))
        send("e")
        wait_for(lambda: any("臂手跟随: ENABLED" in line for line in lines))
        # Pre-position away from the reference before r. No home requirement.
        quest_shift[0] = .01
        time.sleep(.4)
        assert counts["writes"] > 0
        send("r")
        wait_for(lambda: any("Recording:" in line for line in lines))
        time.sleep(args.seconds)
        holds_before = sum("Tracking HOLD:" in line for line in lines)
        resumes_before = sum("Tracking recovered:" in line for line in lines)
        quest_paused.set()
        wait_for(lambda: sum("Tracking HOLD:" in line for line in lines) > holds_before)
        held_writes = counts["writes"]
        time.sleep(.30)
        assert counts["writes"] == held_writes, "Hand writes leaked through tracking HOLD"
        quest_paused.clear()
        wait_for(lambda: sum("Tracking recovered:" in line for line in lines) > resumes_before)
        wait_for(lambda: counts["writes"] > held_writes)
        # No extra e key was sent: automatic recovery retained the enable intent.
        time.sleep(args.seconds)
        send("p")
        wait_for(lambda: any("臂手跟随: PAUSED" in line for line in lines))
        paused_writes = counts["writes"]
        time.sleep(.35)
        assert counts["writes"] == paused_writes, "Hand writes leaked through pause"
        send("h")
        wait_for(lambda: any("Finish recording with s/f before home" in line for line in lines))
        send("e")
        wait_for(lambda: sum("臂手跟随: ENABLED" in line for line in lines) >= 2)
        time.sleep(args.seconds)
        send("s")
        wait_for(lambda: any("Saved:" in line for line in lines))
        send("p")
        wait_for(lambda: sum("臂手跟随: PAUSED" in line for line in lines) >= 2)
        send("h")
        wait_for(lambda: any("Home complete" in line for line in lines))
        post_home_writes = counts["writes"]
        time.sleep(.15)
        assert counts["writes"] == post_home_writes
        # Re-enable and fail-save a second episode after the RT restart.
        send("e")
        wait_for(lambda: sum("臂手跟随: ENABLED" in line for line in lines) >= 3)
        send("r")
        wait_for(lambda: sum("Recording:" in line for line in lines) >= 2)
        time.sleep(1.2)
        send("f")
        wait_for(lambda: sum("Saved:" in line for line in lines) >= 2)
        send("q")
        process.wait(timeout=15)
        reader.join(timeout=2)
        if process.returncode:
            raise RuntimeError("".join(lines))
        # Assert only this process's saved episodes. Other test invocations
        # may legitimately save to the same dataset concurrently.
        episodes = [Path(line.removeprefix("Saved:").strip())
                    for line in lines if line.startswith("Saved:")]
        assert len(episodes) == 2
        from dexretarget.dataset.processing import validate_episode
        import h5py
        import numpy as np
        reports = []
        for episode in episodes:
            metadata = validate_episode(episode)
            assert metadata["context"]["simulated"]
            assert metadata["context"]["task_profile"]["task_id"] == "cup_pick_place"
            assert metadata["context"]["effective_control"]["translation_scale"] == .8
            assert metadata["context"]["control_mode"] == args.arm_control
            reports.append({"episode": str(episode), "success": metadata["success"], "quality": metadata["quality"]})
        assert reports[0]["success"] and not reports[1]["success"]
        assert reports[0]["quality"]["has_training_segment"]
        with h5py.File(episodes[0]/"episode.h5") as file:
            controls = file["streams/control"]
            paused = controls["t"][:][~controls["enabled"][:]]
            assert len(paused) > 0
            # Separate each false interval: HOLD can now resume before a later
            # keyboard pause. Combining first/last false would include valid motion.
            events = [json.loads(value) for value in file['streams/event/json'][:]]
            event_t = file['streams/event/t'][:]
            details = [value.get('detail') for value in events]
            assert 'tracking_hold' in details and 'tracking_recovered_recenter' in details
            begin = event_t[details.index('keyboard_pause')]
            end = next(t for t, event in zip(event_t, events) if t > begin and event.get('detail') == 'keyboard_enable')
            samples = file["samples"]
            for a, b in samples["segments"][:]:
                assert samples["t"][b-1] < begin or samples["t"][a] > end, "Training segment crossed pause"
            assert np.any(samples["valid"][:][samples["t"][:] < begin])
            assert np.any(samples["valid"][:][samples["t"][:] > end])
            ct, enabled = controls['t'][:], controls['enabled'][:]
            edges = np.r_[0, np.flatnonzero(enabled[1:] != enabled[:-1]) + 1, len(enabled)]
            intervals = [(ct[a], ct[b] if b < len(ct) else ct[-1])
                         for a,b in zip(edges[:-1], edges[1:]) if not enabled[a]]
            if args.arm_control == "pinocchio":
                with h5py.File(episodes[1]/"episode.h5") as second:
                    assert np.linalg.norm(samples["state"][0,:3] - second["samples/state"][0,:3]) > 1e-5, "r unexpectedly required the home pose"
            for name in ["arm_command", "hand_command"]:
                t = file[f"streams/{name}/t"][:]
                for start, stop in intervals:
                    assert not np.any((t > start + .05) & (t < stop - .05)), name
        report = {"episodes": reports, "serial_transactions": counts,
                  "synthetic_cameras": args.synthetic_cameras}
        if args.report:
            args.report.write_text(json.dumps(report, indent=2) + "\n")
        print(json.dumps(report, indent=2))
    finally:
        if process.poll() is None:
            process.terminate()
            try:
                process.wait(timeout=10)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait()
        stopped.set()
        for thread in threads:
            thread.join(timeout=1)
        os.close(master)
        os.close(slave)


if __name__ == "__main__":
    sys.path.insert(0, str(ROOT))
    main()
