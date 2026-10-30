"""Shared Quest transport presets; no hardware or application imports."""
from __future__ import annotations

import ipaddress
import os
from pathlib import Path

from .config import load_yaml_mapping, project_path

LEGACY_MODE_FLAGS = {'quest3_protocol', 'quest3_usb'}


def transport_socket_path(mode='usb_tcp', port=8000):
    # Preserve the established USB default endpoint; different routes/ports
    # get separate endpoints so a consumer cannot select the wrong service.
    name = 'quest.sock' if (mode, port) == ('usb_tcp', 8000) else f'quest_{mode}_{port}.sock'
    return Path(f'/tmp/dexretarget-{os.getuid()}')/name


def load_quest_config(path, *, port=None, socket_path=None):
    path = project_path(path).resolve()
    config = load_yaml_mapping(path)
    if set(config) != {'schema_version', 'mode', 'bind_host', 'port'}:
        raise ValueError('Quest config requires only schema_version, mode, bind_host and port')
    if config['schema_version'] != 'dexretarget.quest.v1':
        raise ValueError('Expected Quest schema dexretarget.quest.v1')
    if config['mode'] not in ('udp', 'usb_tcp'):
        raise ValueError('Quest mode must be udp or usb_tcp')
    if not isinstance(config['bind_host'], str):
        raise ValueError('Quest bind_host must be an IPv4 address string')
    try:
        ipaddress.IPv4Address(config['bind_host'])
    except ipaddress.AddressValueError as exc:
        raise ValueError('Quest bind_host must be an IPv4 address') from exc
    if config['mode'] == 'usb_tcp' and config['bind_host'] != '127.0.0.1':
        raise ValueError('USB TCP bind_host must be 127.0.0.1 for ADB reverse')
    if type(config['port']) is not int or not 1 <= config['port'] <= 65535:
        raise ValueError('Quest port must be an integer in [1,65535]')
    if port is not None:
        if type(port) is not int or not 1 <= port <= 65535:
            raise ValueError('Quest port override must be in [1,65535]')
        config['port'] = port
    config['protocol'] = 'tcp' if config['mode'] == 'usb_tcp' else 'udp'
    config['socket_path'] = str(Path(socket_path) if socket_path is not None else
                                transport_socket_path(config['mode'], config['port']))
    config['config_path'] = str(path)
    return config


def apply_quest_config(args, explicit=()):
    """A selected preset overrides task transport defaults as one unit."""
    config = load_quest_config(args.config,
                               port=args.quest3_port if 'quest3_port' in explicit else None,
                               socket_path=(args.quest3_relay_socket
                                            if 'quest3_relay_socket' in explicit else None))
    usb = config['mode'] == 'usb_tcp'
    if 'quest3_protocol' in explicit and args.quest3_protocol != config['protocol']:
        raise ValueError('--quest3-protocol conflicts with --config mode')
    if 'quest3_usb' in explicit and args.quest3_usb != usb:
        raise ValueError('--quest3-usb/--no-quest3-usb conflicts with --config mode')
    args.quest3_protocol, args.quest3_port, args.quest3_usb = config['protocol'], config['port'], usb
    args.quest3_host = config['bind_host']
    args.quest3_relay_socket = Path(config['socket_path'])
    if 'quest3_source' not in explicit:
        # UDP has no connection to preserve: keep the original single-process
        # receiver. Only USB TCP needs a relay to survive teleop restarts.
        args.quest3_source = 'relay' if usb else 'direct'
    args.config = Path(config['config_path'])
    args.quest3_transport_config = config
