"""USB startup ordering, failures and real TCP reconnects, without a robot."""
import socket
import threading
import time
from types import SimpleNamespace

import pytest

from dexretarget.app.cli import build_parser
from dexretarget.app.task import apply_task
from dexretarget.app import quest_usb
from dexretarget.config import PROJECT_ROOT
from dexretarget.teleop.devices.quest3 import Quest3


def arguments(*extra):
    parser = build_parser()
    args = parser.parse_args(list(extra))
    explicit = {parser._option_string_actions[t.split('=')[0]].dest
                for t in extra if t.split('=')[0] in parser._option_string_actions}
    apply_task(args, explicit)
    return args


def test_cup_usb_defaults_and_wireless_override():
    cup = str(PROJECT_ROOT/'config/tasks/cup.yaml')
    args = arguments('--task-config', cup)
    assert (args.quest3_usb, args.quest3_protocol, args.quest3_port) == (True, 'tcp', 8000)
    args = arguments('--task-config', cup, '--quest3-protocol', 'udp', '--quest3-port', '9000')
    assert (args.quest3_usb, args.quest3_protocol, args.quest3_port) == (False, 'udp', 9000)
    assert not arguments('--task-config', cup, '--no-quest3-usb').quest3_usb
    args = arguments('--quest3-usb', '--quest3-port', '8765')
    assert (args.quest3_protocol, args.quest3_port) == ('tcp', 8765)
    assert arguments('--quest3-usb').quest3_port == 8000
    args = arguments()
    assert (args.quest3_usb, args.quest3_protocol, args.quest3_port) == (False, 'udp', 9000)
    with pytest.raises(ValueError, match='requires'):
        arguments('--quest3-usb', '--quest3-protocol', 'udp')


def test_usb_keeps_task_custom_port():
    args = SimpleNamespace(quest3_usb=True, quest3_protocol='tcp', quest3_port=8765,
                           task_profile={'teleop': {'quest3_port': 8765}})
    quest_usb.resolve_usb_transport(args)
    assert args.quest3_port == 8765


def test_adb_selects_authorized_device_and_opens_only_hts(monkeypatch):
    monkeypatch.delenv('ANDROID_SERIAL', raising=False)
    calls = []
    def adb(*args):
        calls.append(args)
        return 'List of devices attached\nquest123\tdevice' if args == ('devices',) else ''
    monkeypatch.setattr(quest_usb, '_adb', adb)
    serial = quest_usb.prepare_usb(8000)
    quest_usb.open_hts(serial)
    assert calls[1] == ('-s', 'quest123', 'reverse', 'tcp:8000', 'tcp:8000')
    assert calls[2][-1] == 'com.wengmister.handtrackingstreamer'
    assert all('--remove-all' not in call and 'force-stop' not in call for call in calls)


@pytest.mark.parametrize('listing', ['', 'q\tunauthorized', 'q\tdevice\nr\tdevice'])
def test_adb_refuses_missing_unauthorized_or_ambiguous_device(monkeypatch, listing):
    monkeypatch.delenv('ANDROID_SERIAL', raising=False)
    monkeypatch.setattr(quest_usb, '_adb', lambda *args: 'List of devices attached\n'+listing)
    with pytest.raises(RuntimeError):
        quest_usb.prepare_usb(8000)


def test_adb_respects_selected_serial(monkeypatch):
    monkeypatch.setenv('ANDROID_SERIAL', 'r')
    calls = []
    def adb(*args):
        calls.append(args)
        return 'List of devices attached\nq\tdevice\nr\tdevice' if args == ('devices',) else ''
    monkeypatch.setattr(quest_usb, '_adb', adb)
    assert quest_usb.prepare_usb(8000) == 'r'
    assert calls[1][1] == 'r'


def test_hts_launch_error_not_silent(monkeypatch):
    monkeypatch.setattr(quest_usb, '_adb', lambda *args: 'Error: unable to resolve Intent')
    with pytest.raises(RuntimeError, match='Cannot open HTS'):
        quest_usb.open_hts('q')


def test_delayed_stream_and_stale_wrong_hand_landmarks(monkeypatch):
    # Simulated delay exceeds the old 15-second startup window.
    clock = [0.0]
    monkeypatch.setattr(quest_usb.time, 'monotonic', lambda: clock[0])
    monkeypatch.setattr(quest_usb.time, 'sleep', lambda dt: clock.__setitem__(0, clock[0]+1))
    class Receiver:
        error = None
        stats = {'bytes': 10}
        def get_hand_tracking_data(self, hand):
            assert hand == 'right'
            return {'wrist': {'timestamp': clock[0]},
                    'fingers': [] if clock[0] >= 20 else None,
                    'landmarks_timestamp': clock[0] if clock[0] >= 25 else 0}
    quest_usb.wait_for_usb_tracking(Receiver(), 'right', require_landmarks=True)
    assert clock[0] == 25


def test_wait_surfaces_receiver_error():
    receiver = SimpleNamespace(error=OSError('port occupied'))
    with pytest.raises(RuntimeError, match='before robot initialization'):
        quest_usb.wait_for_usb_tracking(receiver, 'right', require_landmarks=False)


def test_usb_startup_interrupt_closes_receiver_before_any_output(monkeypatch):
    from dexretarget.app import runtime
    import dexretarget.teleop.devices.quest3 as device
    calls = []
    class Receiver:
        def __init__(self, **kwargs):
            calls.append('receiver')
        def wait_until_ready(self):
            calls.append('listen')
        def stop(self):
            calls.append('close')
    monkeypatch.setattr(device, 'Quest3', Receiver)
    monkeypatch.setattr(quest_usb, 'prepare_usb', lambda port: calls.append('adb') or 'q')
    monkeypatch.setattr(quest_usb, 'open_hts', lambda serial: calls.append('app'))
    def cancel(*args, **kwargs):
        calls.append('wait')
        raise KeyboardInterrupt
    monkeypatch.setattr(quest_usb, 'wait_for_usb_tracking', cancel)
    monkeypatch.setattr(runtime.InspireSerialOutput, '__init__',
                        lambda *args, **kwargs: pytest.fail('hand opened during wait'))
    import dexretarget.teleop.rokae as arm
    monkeypatch.setattr(arm, 'create_arm', lambda *args: pytest.fail('arm initialized during wait'))
    args = arguments('--quest3-usb', '--no-capture')
    with pytest.raises(KeyboardInterrupt):
        runtime.run_teleop(args)
    assert calls == ['adb', 'receiver', 'listen', 'app', 'wait', 'close']


def wait_until(predicate):
    deadline = time.monotonic()+2
    while not predicate():
        if time.monotonic() >= deadline:
            pytest.fail('Timed out waiting for TCP input')
        time.sleep(.005)


def test_real_tcp_listener_accepts_late_stream_and_reconnect():
    receiver = Quest3(host='127.0.0.1', protocol='tcp', port=0)
    try:
        receiver.wait_until_ready()
        port = receiver._port
        assert receiver.get_wrist_pose('right') is None
        # HTS makes a short-lived connection probe before its streaming sockets.
        with socket.create_connection(('127.0.0.1', port)):
            pass
        with socket.create_connection(('127.0.0.1', port)) as client:
            client.sendall(b'Right wrist:,1,2,3,0,0,0,1\n')
            wait_until(lambda: receiver.stats['valid_right_wrist'] == 1)
        initial_timestamp = receiver.get_wrist_pose('right')['timestamp']
        wait_until(lambda: not any(t.is_alive() for t in receiver._conn_threads))
        assert receiver.error is None
        with socket.create_connection(('127.0.0.1', port)) as client:
            # CSV split across reads must still parse after reconnect.
            client.sendall(b'Right wrist:,2,3,')
            client.sendall(b'4,0,0,0,1\nRight landmarks:,'+b','.join([b'0']*63)+b'\n')
            quest_usb.wait_for_usb_tracking(receiver, 'right', require_landmarks=True)
            assert receiver.get_wrist_pose('right')['timestamp'] > initial_timestamp
            assert receiver.stats['valid_right_wrist'] == 2
    finally:
        receiver.stop()
    assert not receiver._thread.is_alive()
    assert not any(t.is_alive() for t in receiver._conn_threads)
    # Server releases its port even when stop closes an idle client.
    with socket.socket() as reusable:
        reusable.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        reusable.bind(('127.0.0.1', port))


def test_bind_error_is_reported_and_socket_released():
    with socket.socket() as occupied:
        occupied.bind(('127.0.0.1', 0))
        occupied.listen()
        receiver = Quest3(host='127.0.0.1', protocol='tcp', port=occupied.getsockname()[1])
        try:
            with pytest.raises(RuntimeError, match='cannot listen'):
                receiver.wait_until_ready()
        finally:
            receiver.stop()
