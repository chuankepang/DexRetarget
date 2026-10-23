"""Task profiles are independent of sensor/storage collection configuration."""
from __future__ import annotations

import copy
import hashlib
import json
from pathlib import Path

import numpy as np
import yaml

from dexretarget.config import PROJECT_ROOT as ROOT, project_path as _project_path
TELEOP_KEYS = {
    'arm_config', 'arm_control', 'hand', 'optimizer', 'hand_config',
    'quest3_port', 'quest3_protocol', 'inspire_port', 'inspire_baudrate',
    'inspire_hand_id', 'rokae_robot_ip', 'rokae_local_ip', 'arm_print_hz',
}


def project_path(value):
    return _project_path(value).resolve()


def _vector(value, size, name):
    result = np.asarray(value, dtype=float)
    if result.shape != (size,) or not np.all(np.isfinite(result)):
        raise ValueError(f'{name} must contain {size} finite values')
    return result.tolist()


def load_task(path):
    path = Path(path).resolve()
    profile = yaml.safe_load(path.read_text())
    if not isinstance(profile, dict) or profile.get('schema_version') != 'dexretarget.task.v1':
        raise ValueError('Expected task schema dexretarget.task.v1')
    allowed = {'schema_version', 'task_id', 'instruction', 'teleop', 'collection', 'home', 'scene_setup'}
    if set(profile) - allowed:
        raise ValueError(f'Unknown task keys: {set(profile) - allowed}')
    for key in ['task_id', 'instruction']:
        if not isinstance(profile.get(key), str) or not profile[key].strip():
            raise ValueError(f'Task {key} is required')
    teleop = profile.setdefault('teleop', {})
    if not isinstance(teleop, dict) or set(teleop) - TELEOP_KEYS:
        raise ValueError('Unknown/invalid task teleop settings')
    choices = {'hand': {'left', 'right'}, 'optimizer': {'adaptive', 'vector'},
               'quest3_protocol': {'udp', 'tcp'},
               'arm_control': {'pinocchio', 'cartesian', 'impedance'}}
    for key, values in choices.items():
        if key in teleop and teleop[key] not in values:
            raise ValueError(f'Invalid task teleop.{key}')
    for key, lower, upper in [('quest3_port', 1, 65535), ('inspire_baudrate', 1, 4000000), ('inspire_hand_id', 1, 253)]:
        if key in teleop and (type(teleop[key]) is not int or not lower <= teleop[key] <= upper):
            raise ValueError(f'Invalid task teleop.{key}')
    if 'arm_print_hz' in teleop:
        rate = teleop['arm_print_hz']
        if isinstance(rate, bool) or not isinstance(rate, (int, float)) or not np.isfinite(rate) or rate < 0:
            raise ValueError('teleop.arm_print_hz must be finite and nonnegative')
    for key in ['arm_config', 'hand_config']:
        if key in teleop and not project_path(teleop[key]).is_file():
            raise FileNotFoundError(teleop[key])
    for key in ['inspire_port', 'rokae_robot_ip', 'rokae_local_ip']:
        if key in teleop and (not isinstance(teleop[key], str) or not teleop[key].strip()):
            raise ValueError(f'teleop.{key} must be a nonempty string')
    collection = profile.setdefault('collection', {})
    if not isinstance(collection, dict) or set(collection) - {'enabled', 'config'}:
        raise ValueError('Task collection accepts enabled and config reference only')
    if type(collection.get('enabled', False)) is not bool:
        raise ValueError('collection.enabled must be boolean')
    if 'config' in collection and not project_path(collection['config']).is_file():
        raise FileNotFoundError(collection['config'])
    home = profile.get('home')
    if home is not None:
        if not isinstance(home, dict) or set(home) - {'on_start', 'arm_joints', 'hand_angles', 'joint_waypoints', 'joint_speed', 'timeout_s', 'max_segment_delta', 'joint_tolerance_rad'}:
            raise ValueError('Unknown/invalid home settings')
        if type(home.setdefault('on_start', False)) is not bool:
            raise ValueError('home.on_start must be boolean')
        home['arm_joints'] = _vector(home['arm_joints'], 7, 'home.arm_joints')
        home['joint_waypoints'] = [_vector(q, 7, 'home waypoint') for q in home.get('joint_waypoints', [])]
        if home.get('hand_angles') is not None:
            angles = _vector(home['hand_angles'], 6, 'home.hand_angles')
            if any(v < 0 or v > 1000 or v != int(v) for v in angles):
                raise ValueError('home.hand_angles must be native integer registers 0..1000')
            home['hand_angles'] = [int(v) for v in angles]
        for key, default, upper in [('joint_speed', .05, 1), ('timeout_s', 30, 300), ('max_segment_delta', .15, .5), ('joint_tolerance_rad', .02, .1)]:
            value = home.setdefault(key, default)
            if isinstance(value, bool) or not isinstance(value, (int, float)) or not np.isfinite(value) or not 0 < value <= upper:
                raise ValueError(f'home.{key} must be in (0,{upper}]')
    canonical = json.dumps(profile, sort_keys=True, ensure_ascii=False, allow_nan=False)
    profile['profile_sha256'] = hashlib.sha256(canonical.encode()).hexdigest()
    profile['profile_path'] = str(path)
    return profile


def apply_task(args, explicit=()):
    """CLI > profile > existing defaults. Paths in profiles are project-relative."""
    profile = load_task(args.task_config) if args.task_config else None
    if profile:
        for key, value in profile['teleop'].items():
            if key not in explicit:
                if key in {'arm_config', 'hand_config'} and value is not None:
                    value = str(project_path(value))
                setattr(args, key, value)
        if 'task' not in explicit:
            args.task = profile['instruction']
        if args.collection_config is None and profile['collection'].get('config'):
            args.collection_config = project_path(profile['collection']['config'])
        if args.capture is None:
            args.capture = profile['collection'].get('enabled', False)
    if args.collection_config is None:
        args.collection_config = ROOT / 'config/collection/collection.json'
    args.capture = bool(args.capture)
    args.task_profile = copy.deepcopy(profile)
    return profile
