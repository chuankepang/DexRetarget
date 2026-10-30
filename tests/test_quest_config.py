"""Transport precedence, direct UDP and optional relay, without real outputs."""
import os
from pathlib import Path
import select
import signal
import socket
import subprocess
import sys
import time

import pytest
import yaml

from dexretarget.app.cli import build_parser
from dexretarget.app.task import apply_task, load_task
from dexretarget.config import PROJECT_ROOT
from dexretarget.quest_config import load_quest_config
from dexretarget.teleop.devices.quest_relay import QuestRelayInput

UDP = 'config/quest/udp.yaml'
USB = 'config/quest/usb_tcp.yaml'


def resolve(*extra):
    parser = build_parser()
    args = parser.parse_args(list(extra))
    explicit = {parser._option_string_actions[t.split('=')[0]].dest
                for t in extra if t.split('=')[0] in parser._option_string_actions}
    apply_task(args, explicit)
    return args


@pytest.mark.parametrize('path,protocol,port,usb,host', [
    (UDP, 'udp', 9000, False, '0.0.0.0'),
    (USB, 'tcp', 8000, True, '127.0.0.1'),
])
def test_config_overrides_task_transport_atomically(path, protocol, port, usb, host):
    args = resolve('--task-config', 'config/tasks/cup.yaml', '--config', path)
    assert (args.quest3_protocol, args.quest3_port, args.quest3_usb, args.quest3_host) == (protocol, port, usb, host)
    assert args.quest3_source == ('relay' if usb else 'direct')
    assert args.quest3_transport_config['mode'] == ('usb_tcp' if usb else 'udp')
    assert args.task_profile['home']['arm_joints']


def test_routes_have_distinct_local_endpoints():
    assert load_quest_config(UDP)['socket_path'] != load_quest_config(USB)['socket_path']
    assert load_quest_config(USB)['socket_path'].endswith('/quest.sock')


def test_custom_port_uses_matching_consumer_and_service_path():
    args = resolve('--config', UDP, '--quest3-port', '9988')
    service = load_quest_config(UDP, port=9988)
    assert str(args.quest3_relay_socket) == service['socket_path']
    assert args.quest3_port == service['port']
    # A task's default route still applies when only the port is overridden.
    args = resolve('--task-config', 'config/tasks/cup.yaml', '--quest3-port', '8001')
    assert (args.quest3_protocol, args.quest3_usb, args.quest3_port) == ('tcp', True, 8001)


@pytest.mark.parametrize('auto', [False, True])
def test_udp_runtime_receives_directly_without_adb_or_service(tmp_path, monkeypatch, auto):
    from anydexretarget import Retargeter
    from dexretarget.app import runtime, quest_usb
    from dexretarget.teleop.devices import quest3, quest_relay
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as reserve:
        reserve.bind(('127.0.0.1', 0))
        port = reserve.getsockname()[1]
    # Even an existing relay path must not affect UDP's auto/direct route.
    unused = tmp_path/'unused.sock'
    unused.write_text('do not connect')
    flags = ['--quest3-source', 'auto'] if auto else []
    args = resolve('--config', UDP, '--quest3-port', str(port), '--enable-hand',
                   '--no-capture', '--quest3-relay-socket', str(unused), *flags)
    receivers = []
    original = quest3.Quest3
    def capture(**kwargs):
        receivers.append(original(**kwargs))
        return receivers[-1]
    monkeypatch.setattr(quest3, 'Quest3', capture)
    def forbidden(*args, **kwargs):
        pytest.fail('UDP must not use ADB or a local communication service')
    monkeypatch.setattr(quest_usb, 'prepare_usb', forbidden)
    monkeypatch.setattr(quest_usb, 'open_hts', forbidden)
    monkeypatch.setattr(quest_relay, 'QuestRelayInput', forbidden)
    def check_before_outputs(*args):
        receiver = receivers[0]
        receiver.wait_until_ready()
        assert receiver._protocol == 'udp'
        assert receiver._host == '0.0.0.0'
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sender:
            sender.sendto(b'Right wrist:,1,2,3,0,0,0,1\n', ('127.0.0.1', port))
            wait_until(lambda: receiver.get_wrist_pose('right') is not None)
        assert receiver.get_wrist_pose('right')['position'][1] == -1
        raise RuntimeError('end test before hardware initialization')
    monkeypatch.setattr(Retargeter, 'from_yaml', check_before_outputs)
    with pytest.raises(RuntimeError, match='end test before hardware'):
        runtime.run_teleop(args)
    assert args.quest3_effective_source == 'direct'
    assert not receivers[0]._thread.is_alive()
    assert unused.read_text() == 'do not connect'


@pytest.mark.parametrize('flags', [
    ['--config', UDP, '--quest3-protocol', 'tcp'],
    ['--config', UDP, '--quest3-usb'],
    ['--config', USB, '--no-quest3-usb'],
])
def test_conflicting_legacy_mode_flags_fail_before_hardware(flags):
    with pytest.raises(ValueError, match='conflicts'):
        resolve(*flags)


@pytest.mark.parametrize('change', [
    {'mode': 'wireless_tcp'}, {'port': 0}, {'port': 65536}, {'port': True},
    {'bind_host': 'not-an-ip'}, {'bind_host': None},
    {'mode': 'usb_tcp', 'bind_host': '0.0.0.0'},
    {'schema_version': 'typo'}, {'porrt': 9000},
])
def test_invalid_transport_config_is_rejected(tmp_path, change):
    config = yaml.safe_load((PROJECT_ROOT/UDP).read_text())
    config.update(change)
    path = tmp_path/'invalid.yaml'
    path.write_text(yaml.safe_dump(config))
    with pytest.raises(ValueError):
        load_quest_config(path)


def test_task_has_one_transport_reference_and_rejects_duplicate_definitions(tmp_path):
    path = PROJECT_ROOT/'config/tasks/cup.yaml'
    profile = yaml.safe_load(path.read_text())
    assert profile['teleop']['quest_config'] == USB
    assert not {'quest3_protocol', 'quest3_port', 'quest3_usb'} & profile['teleop'].keys()
    profile['teleop']['quest3_port'] = 9000
    invalid = tmp_path/'task.yaml'
    invalid.write_text(yaml.safe_dump(profile))
    with pytest.raises(ValueError, match='not both'):
        load_task(invalid)


def wait_until(predicate, timeout=4):
    end = time.monotonic()+timeout
    while not predicate():
        assert time.monotonic() < end, 'Timed out waiting for UDP relay'
        time.sleep(.001)


def test_udp_cli_never_calls_adb_and_survives_consumer_restarts(tmp_path):
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as reserve:
        reserve.bind(('127.0.0.1', 0))
        port = reserve.getsockname()[1]
    path = tmp_path/'udp.sock'
    script = '''
import runpy
from dexretarget.app import quest_usb
def forbidden(*args):
    raise AssertionError('UDP must not call ADB or open the USB app')
quest_usb.prepare_usb=quest_usb.open_hts=forbidden
runpy.run_module('dexretarget.quest', run_name='__main__')
'''
    process = subprocess.Popen([sys.executable, '-u', '-c', script, '--config', UDP,
                                '--port', str(port), '--socket', str(path)],
                               stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    try:
        output = b''
        end = time.monotonic()+5
        while '通信服务已就绪'.encode() not in output:
            remaining = end-time.monotonic()
            assert remaining > 0 and select.select([process.stdout], [], [], remaining)[0], output
            chunk = os.read(process.stdout.fileno(), 4096)
            assert chunk, output
            output += chunk
        assert b'udp 0.0.0.0:' in output
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sender:
            for value in (1, 2, 3):
                consumer = QuestRelayInput(path)
                try:
                    consumer.wait_until_ready()
                    packet = f'Right wrist:,{value},2,3,0,0,0,1\n'.encode()
                    sender.sendto(packet, ('127.0.0.1', port))
                    wait_until(lambda: consumer.get_wrist_pose('right') is not None and
                               consumer.get_wrist_pose('right')['position'][1] == -value)
                    assert consumer.stats['valid_right_wrist'] == value
                finally:
                    consumer.stop()
                assert process.poll() is None
        process.send_signal(signal.SIGINT)
        _, error = process.communicate(timeout=5)
        assert process.returncode == 0, error
        assert not path.exists()
    finally:
        if process.poll() is None:
            process.kill()
        process.communicate(timeout=5)


def test_usb_config_runs_adb_setup_with_configured_port(tmp_path, monkeypatch):
    from dexretarget import quest
    from dexretarget.app import quest_usb
    from dexretarget.teleop.devices.quest_relay import QuestRelayServer
    with socket.socket() as reserve:
        reserve.bind(('127.0.0.1', 0))
        port = reserve.getsockname()[1]
    calls = []
    monkeypatch.setattr(quest_usb, 'prepare_usb', lambda port: calls.append(('adb', port)) or 'device')
    monkeypatch.setattr(quest_usb, 'open_hts', lambda serial: calls.append(('app', serial)))
    def stop(server, event):
        assert server.receiver._protocol == 'tcp'
        assert server.receiver._host == '127.0.0.1'
        event.set()
    monkeypatch.setattr(QuestRelayServer, 'run', stop)
    quest.main(['--config', USB, '--port', str(port), '--socket', str(tmp_path/'usb.sock')])
    assert calls == [('adb', port), ('app', 'device')]


def test_cli_preview_with_udp_config_never_imports_runtime(monkeypatch, capsys):
    import json
    from dexretarget.app.cli import main
    monkeypatch.setitem(sys.modules, 'dexretarget.app.runtime', None)
    monkeypatch.setattr(sys, 'argv', ['dexretarget', '--config', UDP,
                                     '--task-config', 'config/tasks/cup.yaml', '--print-arm-config'])
    main()
    assert json.loads(capsys.readouterr().out)['read_only_plan']
