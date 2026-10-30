"""Synthetic TCP -> Unix relay benchmark, no Quest/robot or saved raw logs."""
from __future__ import annotations

import argparse
import json
from pathlib import Path
import socket
import sys
import tempfile
import threading
import time

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
import numpy as np
from dexretarget.teleop.devices.quest_relay import QuestRelayServer, QuestRelayInput


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--duration', type=float, default=5.)
    parser.add_argument('--hz', type=float, default=70.)
    args = parser.parse_args()
    if not 1 <= args.duration <= 60 or not 1 <= args.hz <= 1000:
        parser.error('duration must be 1..60 seconds, hz 1..1000')
    with tempfile.TemporaryDirectory(prefix='quest-bench-') as temp:
        server = QuestRelayServer(Path(temp)/'quest.sock', port=0)
        stop = threading.Event()
        errors = []
        def serve():
            try:
                server.run(stop)
            except BaseException as exc:
                errors.append(str(exc))
        worker = threading.Thread(target=serve)
        worker.start()
        consumer = QuestRelayInput(server.path)
        sender = socket.create_connection(('127.0.0.1', server.receiver._port))
        sender.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        csv = ','.join(f'{v*.001:.6f}' for v in range(63))
        sent = [0, 0]
        def transmit():
            deadline = time.monotonic()
            for i in range(round(args.duration*args.hz)):
                packet = ''.join(f'{side} wrist:,{i},2,3,0,0,0,1\n{side} landmarks:,{csv}\n'
                                 for side in ('Left', 'Right')).encode()
                sender.sendall(packet)
                sent[0] += 1
                sent[1] += len(packet)
                deadline += 1/args.hz
                time.sleep(max(0., deadline-time.monotonic()))
        source = threading.Thread(target=transmit)
        ages = []
        direct_ages = []
        last = direct_last = None
        cpu_start, wall_start = time.process_time(), time.monotonic()
        try:
            consumer.wait_until_ready()
            source.start()
            while source.is_alive() or time.monotonic()-wall_start < args.duration+.1:
                direct = server.receiver.get_wrist_pose('right')
                if direct is not None and direct['timestamp'] != direct_last:
                    direct_last = direct['timestamp']
                    direct_ages.append((time.monotonic()-direct_last)*1000)
                wrist = consumer.get_wrist_pose('right')
                if wrist is not None and wrist['timestamp'] != last:
                    last = wrist['timestamp']
                    ages.append((time.monotonic()-last)*1000)
                time.sleep(.0005)
            source.join()
            if consumer.error is not None:
                errors.append(str(consumer.error))
            elapsed = time.monotonic()-wall_start
            def percentiles(values):
                return dict(zip(['p50', 'p95', 'p99', 'max'],
                                np.percentile(values, [50, 95, 99, 100]).round(4).tolist())) if values else {}
            stats = server.receiver.stats
            result = {
                'synthetic_only': True, 'hz_per_hand': args.hz, 'duration_s': round(elapsed, 3),
                'sent_frames_per_hand': sent[0], 'observed_relay_frames_right': len(ages),
                'valid_wrists_received_both_hands': stats['valid_wrist'],
                'source_bytes_sent': sent[1], 'source_bytes_received': stats['bytes'],
                'source_bytes_exact': sent[1] == stats['bytes'],
                'local_snapshot_bytes_total': server.local_bytes,
                'local_snapshot_bytes_per_s': round(server.local_bytes/elapsed, 1),
                'direct_receive_to_probe_ms': percentiles(direct_ages),
                'relay_receive_to_probe_ms': percentiles(ages),
                'combined_process_cpu_percent_one_core': round((time.process_time()-cpu_start)/elapsed*100, 2),
                'slow_subscriber_skips': server.skipped_updates, 'errors': errors,
                'scope': 'Same-process threads and real TCP/Unix sockets; includes 0.5ms probe scheduling. Not USB, cross-process latency or hard realtime certification.',
            }
            print(json.dumps(result, ensure_ascii=False, indent=2))
            return bool(errors or not result['source_bytes_exact'] or not ages)
        finally:
            if source.is_alive():
                source.join(2)
            sender.close()
            consumer.stop()
            stop.set()
            worker.join(3)
            server.close()


if __name__ == '__main__':
    raise SystemExit(main())
