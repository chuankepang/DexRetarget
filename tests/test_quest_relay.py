"""Real TCP + local sockets: stream lifetime, timestamps, load and isolation."""
import socket
import os
import signal
import select
import subprocess
import sys
import threading
import time

import numpy as np
import pytest

from dexretarget.teleop.devices.quest3 import QuestTrackingState
from dexretarget.teleop.devices.quest_relay import (
    QuestRelayServer, QuestRelayInput, encode_snapshot, apply_snapshot, _PACKET,
)


def until(predicate, timeout=3):
    deadline = time.monotonic()+timeout
    while not predicate():
        if time.monotonic() >= deadline:
            pytest.fail('Timed out waiting for Quest relay')
        time.sleep(.001)


@pytest.fixture
def relay(tmp_path):
    path = tmp_path/'quest.sock'
    server = QuestRelayServer(path, port=0)
    stop = threading.Event()
    errors = []
    def serve():
        try:
            server.run(stop)
        except BaseException as exc:
            errors.append(exc)
    thread = threading.Thread(target=serve)
    thread.start()
    try:
        yield server
    finally:
        stop.set()
        thread.join(3)
        server.close()
        assert not thread.is_alive()
        assert not errors
        assert not path.exists()


def connect_hts(server):
    return socket.create_connection(('127.0.0.1', server.receiver._port))


def send_hand(sock, x=1., side='Right'):
    # Whole-hand payload ~1 KB, matching the actual CSV channel layout.
    landmarks = ','.join(str(i*.001) for i in range(63))
    sock.sendall(f'{side} wrist:,{x},2,3,0,0,0,1\n{side} landmarks:,{landmarks}\n'.encode())


def test_consumers_restart_without_disconnecting_hts(relay):
    with connect_hts(relay) as hts:
        send_hand(hts)
        until(lambda: relay.receiver.stats['valid_right_landmarks'] == 1)
        # Start/stop local input repeatedly while keeping the SAME TCP sender.
        for i in range(4):
            consumer = QuestRelayInput(relay.path)
            try:
                consumer.wait_until_ready()
                send_hand(hts, x=10+i)
                until(lambda: consumer.get_wrist_pose('right') is not None and
                      consumer.get_wrist_pose('right')['position'][1] == -(10+i))
                expected = relay.receiver.get_hand_tracking_data('right')
                actual = consumer.get_hand_tracking_data('right')
                np.testing.assert_allclose(actual['wrist']['rotation'], expected['wrist']['rotation'])
                np.testing.assert_allclose(actual['fingers'], expected['fingers'])
                assert actual['wrist']['timestamp'] == expected['wrist']['timestamp']
                assert consumer.stats['valid_right_wrist'] > 0
                assert consumer.error is None
            finally:
                consumer.stop()
            assert relay.receiver.error is None
        send_hand(hts, x=99)
        until(lambda: relay.receiver.stats['valid_right_wrist'] == 6)


def test_idle_heartbeat_never_refreshes_source_timestamps(relay):
    with connect_hts(relay) as hts:
        send_hand(hts)
        until(lambda: relay.receiver.stats['valid_right_landmarks'] == 1)
        consumer = QuestRelayInput(relay.path)
        try:
            consumer.wait_until_ready()
            first = consumer.get_hand_tracking_data('right')
            time.sleep(.65)  # Cross one heartbeat and both tracking age limits.
            later = consumer.get_hand_tracking_data('right')
            assert later['wrist']['timestamp'] == first['wrist']['timestamp']
            assert later['landmarks_timestamp'] == first['landmarks_timestamp']
            assert time.monotonic()-later['wrist']['timestamp'] > .2
            assert consumer.error is None
        finally:
            consumer.stop()


def test_empty_service_and_separate_hand_channels(relay):
    consumer = QuestRelayInput(relay.path)
    try:
        consumer.wait_until_ready()
        assert consumer.get_wrist_pose('right') is None
        with connect_hts(relay) as left, connect_hts(relay) as right:
            send_hand(left, x=5, side='Left')
            send_hand(right, x=9)
            until(lambda: consumer.get_wrist_pose('left') is not None and
                  consumer.get_wrist_pose('right') is not None)
            assert consumer.get_wrist_pose('left')['position'][1] == -5
            assert consumer.get_wrist_pose('right')['position'][1] == -9
    finally:
        consumer.stop()


def test_slow_subscriber_never_blocks_hts_or_other_consumers(relay):
    # A connected subscriber that deliberately never reads.
    with socket.socket(socket.AF_UNIX, socket.SOCK_SEQPACKET) as slow, connect_hts(relay) as hts:
        slow.connect(str(relay.path))
        consumer = QuestRelayInput(relay.path)
        try:
            consumer.wait_until_ready()
            for i in range(160):
                send_hand(hts, x=i)
                time.sleep(.001)
            until(lambda: relay.receiver.stats['valid_right_landmarks'] == 160)
            until(lambda: consumer.get_wrist_pose('right') is not None and
                  consumer.get_wrist_pose('right')['position'][1] == -159)
            assert relay.skipped_updates > 0
            assert relay.receiver.error is None
        finally:
            consumer.stop()


def test_duplicate_service_cannot_steal_path_or_receiver(relay):
    with pytest.raises(OSError):
        QuestRelayServer(relay.path, port=0)
    assert relay.path.exists()
    consumer = QuestRelayInput(relay.path)
    try:
        consumer.wait_until_ready()
        assert consumer.error is None
    finally:
        consumer.stop()


def test_tcp_bind_failure_cleans_only_its_own_local_endpoint(tmp_path):
    path = tmp_path/'failed.sock'
    with socket.socket() as occupied:
        occupied.bind(('127.0.0.1', 0))
        occupied.listen()
        with pytest.raises(RuntimeError, match='cannot listen'):
            QuestRelayServer(path, port=occupied.getsockname()[1])
    assert not path.exists()


def test_bad_local_subscriber_does_not_stop_upstream(relay):
    with socket.socket(socket.AF_UNIX, socket.SOCK_SEQPACKET) as bad, connect_hts(relay) as hts:
        bad.connect(str(relay.path))
        bad.send(b'not a control command')
        bad.settimeout(2)
        # An initial snapshot may precede the disconnect.
        try:
            while bad.recv(4096):
                pass
        except ConnectionResetError:
            pass
        send_hand(hts)
        until(lambda: relay.receiver.stats['valid_right_wrist'] == 1)
        assert relay.receiver.error is None


def test_service_shutdown_invalidates_consumer_and_releases_ports(tmp_path):
    server = QuestRelayServer(tmp_path/'quest.sock', port=0)
    port = server.receiver._port
    stop = threading.Event()
    thread = threading.Thread(target=server.run, args=(stop,))
    thread.start()
    consumer = QuestRelayInput(server.path)
    try:
        consumer.wait_until_ready()
        with connect_hts(server) as hts:
            send_hand(hts)
            until(lambda: consumer.get_wrist_pose('right') is not None)
            stop.set()
            thread.join(3)
            server.close()
            until(lambda: consumer.error is not None)
            assert consumer.get_wrist_pose('right') is None
        with socket.socket() as reusable:
            reusable.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            reusable.bind(('127.0.0.1', port))
    finally:
        consumer.stop()
        stop.set()
        thread.join(3)
        server.close()


def test_invalid_binary_snapshot_is_rejected():
    state = QuestTrackingState()
    packet = encode_snapshot(state)
    assert len(packet) == 1224
    with pytest.raises(ValueError):
        apply_snapshot(state, packet[:-1])
    with pytest.raises(ValueError):
        apply_snapshot(state, b'badmagic'+packet[8:])
    fields = list(_PACKET.unpack(packet))
    fields[9] = float('nan')
    with pytest.raises(ValueError):
        apply_snapshot(state, _PACKET.pack(*fields))


def test_auto_runtime_uses_relay_without_adb_or_tcp_bind(relay, monkeypatch):
    from dexretarget.app.cli import build_parser
    from dexretarget.app import runtime, quest_usb
    import dexretarget.teleop.devices.quest3 as quest
    calls = []
    monkeypatch.setattr(quest_usb, 'prepare_usb', lambda *args: pytest.fail('ADB belongs to the communication service'))
    monkeypatch.setattr(quest, 'Quest3', lambda **kwargs: pytest.fail('Teleop must not bind the HTS port'))
    def cancel(receiver, *args, **kwargs):
        calls.append(receiver)
        raise KeyboardInterrupt
    monkeypatch.setattr(quest_usb, 'wait_for_usb_tracking', cancel)
    args = build_parser().parse_args(['--quest3-usb', '--quest3-relay-socket', str(relay.path), '--no-capture'])
    with pytest.raises(KeyboardInterrupt):
        runtime.run_teleop(args)
    assert len(calls) == 1
    assert not calls[0]._thread.is_alive()
    assert relay.receiver._thread.is_alive()


def test_missing_required_relay_never_falls_back_to_robot_or_tcp(tmp_path, monkeypatch):
    from dexretarget.app.cli import build_parser
    from dexretarget.app import runtime
    import dexretarget.teleop.devices.quest3 as quest
    monkeypatch.setattr(quest, 'Quest3', lambda **kwargs: pytest.fail('Unexpected direct input fallback'))
    args = build_parser().parse_args(['--quest3-source', 'relay',
                                     '--quest3-relay-socket', str(tmp_path/'missing.sock')])
    with pytest.raises(RuntimeError, match='python -m dexretarget.quest'):
        runtime.run_teleop(args)


def test_terminal_service_survives_separate_client_processes(tmp_path):
    path = tmp_path/'quest.sock'
    with socket.socket() as reservation:
        reservation.bind(('127.0.0.1', 0))
        port = reservation.getsockname()[1]
    process = subprocess.Popen([sys.executable, '-m', 'dexretarget.quest', '--no-adb',
                                '--port', str(port), '--socket', str(path)],
                               stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
    hts = None
    try:
        until(path.exists, timeout=5)
        def connect():
            nonlocal hts
            try:
                hts = socket.create_connection(('127.0.0.1', port), timeout=.2)
            except OSError:
                return False
            return True
        until(connect)
        for i in range(3):
            send_hand(hts, x=42)
            script = '''
import sys
from dexretarget.teleop.devices.quest_relay import QuestRelayInput
receiver=QuestRelayInput(sys.argv[1])
try:
    receiver.wait_until_ready()
    assert receiver.get_wrist_pose('right')['position'][1] == -42
finally:
    receiver.stop()
'''
            result = subprocess.run([sys.executable, '-c', script, str(path)],
                                    capture_output=True, text=True, timeout=5)
            assert result.returncode == 0, result.stderr
            assert process.poll() is None
        # Independent teleop input clients exited, but this TCP socket is live.
        send_hand(hts, x=43)
        process.terminate()
        output, error = process.communicate(timeout=5)
        assert process.returncode == 0, error
        assert 'Quest 通信已关闭' in output
        assert not path.exists()
    finally:
        if hts is not None:
            hts.close()
        if process.poll() is None:
            process.kill()
        process.communicate(timeout=5)


def test_stale_socket_is_reclaimed_without_manual_deletion(tmp_path):
    path = tmp_path/'quest.sock'
    with socket.socket(socket.AF_UNIX, socket.SOCK_SEQPACKET) as abandoned:
        abandoned.bind(str(path))
        abandoned.listen()
    server = QuestRelayServer(path, port=0)
    try:
        assert server.receiver.error is None
        assert path.exists()
    finally:
        server.close()
    assert not path.exists()


def test_live_legacy_socket_without_process_lock_is_not_removed(tmp_path):
    path = tmp_path/'quest.sock'
    with socket.socket(socket.AF_UNIX, socket.SOCK_SEQPACKET) as legacy:
        legacy.bind(str(path))
        legacy.listen()
        inode = path.stat().st_ino
        with pytest.raises(OSError, match='still active'):
            QuestRelayServer(path, port=0)
        assert path.stat().st_ino == inode


@pytest.mark.parametrize('symlink', [False, True])
def test_non_socket_or_symlink_is_never_removed(tmp_path, symlink):
    path = tmp_path/'quest.sock'
    target = tmp_path/'keep.txt'
    target.write_text('keep')
    if symlink:
        path.symlink_to(target)
    else:
        path.write_text('keep')
    with pytest.raises(OSError, match='Refusing to replace'):
        QuestRelayServer(path, port=0)
    assert path.read_text() == 'keep'
    assert target.read_text() == 'keep'


def test_receiver_stop_error_still_cleans_endpoint_and_releases_lock(tmp_path, monkeypatch):
    path = tmp_path/'quest.sock'
    server = QuestRelayServer(path, port=0)
    original = server.receiver.stop
    def fail():
        original()
        raise RuntimeError('injected stop error')
    monkeypatch.setattr(server.receiver, 'stop', fail)
    with pytest.raises(RuntimeError, match='injected stop error'):
        server.close()
    assert not path.exists()
    replacement = QuestRelayServer(path, port=0)
    replacement.close()


def wait_output(process, token):
    # stdout is unbuffered in the child; read bytes to avoid TextIO prefetch
    # hiding a ready line from select().
    output = b''
    deadline = time.monotonic()+5
    while token not in output:
        remaining = deadline-time.monotonic()
        assert remaining > 0, (output, process.poll())
        assert select.select([process.stdout], [], [], remaining)[0], output
        data = os.read(process.stdout.fileno(), 4096)
        assert data, (output, process.poll())
        output += data
    return output


def service_process(tmp_path, *, delayed_stop=False):
    path = tmp_path/'terminal.sock'
    with socket.socket() as reservation:
        reservation.bind(('127.0.0.1', 0))
        port = reservation.getsockname()[1]
    if delayed_stop:
        script = '''
import runpy,time
from dexretarget.teleop.devices.quest3 import Quest3
original=Quest3.stop
def delayed(self):
    print('CLEANUP_ENTERED',flush=True)
    time.sleep(.4)
    original(self)
Quest3.stop=delayed
runpy.run_module('dexretarget.quest',run_name='__main__')
'''
        command = [sys.executable, '-u', '-c', script]
    else:
        command = [sys.executable, '-u', '-m', 'dexretarget.quest']
    process = subprocess.Popen([*command, '--no-adb', '--port', str(port), '--socket', str(path)],
                               stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    return process, path


def test_repeated_ctrl_c_during_cleanup_does_not_interrupt_or_leave_socket(tmp_path):
    process, path = service_process(tmp_path, delayed_stop=True)
    try:
        output = wait_output(process, '通信服务已就绪'.encode())
        process.send_signal(signal.SIGINT)
        output += wait_output(process, b'CLEANUP_ENTERED')
        process.send_signal(signal.SIGINT)
        time.sleep(.03)
        process.send_signal(signal.SIGINT)
        final, error = process.communicate(timeout=5)
        assert process.returncode == 0, error
        assert b'KeyboardInterrupt' not in error
        assert '通信已关闭'.encode() in output+final
        assert not path.exists()
    finally:
        if process.poll() is None:
            process.kill()
        process.communicate(timeout=5)


def test_sigkill_leftover_endpoint_is_recovered_on_process_restart(tmp_path):
    first, path = service_process(tmp_path)
    replacement = None
    try:
        wait_output(first, '通信服务已就绪'.encode())
        first.kill()
        first.communicate(timeout=5)
        assert path.exists()
        replacement, same_path = service_process(tmp_path)
        assert same_path == path
        wait_output(replacement, '通信服务已就绪'.encode())
        replacement.send_signal(signal.SIGINT)
        _, error = replacement.communicate(timeout=5)
        assert replacement.returncode == 0, error
        assert not path.exists()
    finally:
        for process in (first, replacement):
            if process is not None:
                if process.poll() is None:
                    process.kill()
                process.communicate(timeout=5)
