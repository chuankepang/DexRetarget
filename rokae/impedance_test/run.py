#!/usr/bin/env python3
"""Plan and launch a direct SDK test. Default: no connection and no motion."""
from __future__ import annotations

import argparse
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
import time

import yaml

ROOT = Path(__file__).resolve().parents[2]
BUILD = ROOT / 'rokae/build/impedance_test'


def vector(value, size, name):
    if not isinstance(value, (list, tuple)) or len(value) != size:
        raise ValueError(f'{name}: expected {size} values')
    result = [float(x) for x in value]
    if not all(math.isfinite(x) for x in result):
        raise ValueError(f'{name}: values must be finite')
    return result


def plan(args):
    robot_path = args.robot_config.resolve()
    task_path = args.task_config.resolve()
    robot_config = yaml.safe_load(robot_path.read_text())
    task_config = yaml.safe_load(task_path.read_text())
    robot = robot_config['robot']
    if robot['type'] != 'xmate-er-pro-7':
        raise ValueError('This isolated test supports the current xMatePro3 seven-axis assembly only')
    payload = robot['payload']
    mass = float(payload['mass_kg'])
    if not math.isfinite(mass) or not 0 < mass <= 3:
        raise ValueError('payload mass must be in (0, 3] kg')
    cog = vector(payload['cog_m'], 3, 'payload.cog_m')
    if any(abs(x) > .3 for x in cog):
        raise ValueError('payload.cog_m: metres within 0.3 m per axis required')
    home = vector(task_config['home']['arm_joints'], 7, 'home.arm_joints')
    stiffness = vector(robot_config['control']['impedance']['stiffness'], 6, 'stiffness')
    if any(not 0 < x <= (1500 if i < 3 else 100) for i, x in enumerate(stiffness)):
        raise ValueError('stiffness outside SDK 0.3.4 ER Pro limits')
    if not math.isfinite(args.duration) or not 3 <= args.duration <= 60:
        raise ValueError('duration must be 3..60 seconds')
    return {
        'schema': 'rokae.fixed_impedance.v1', 'stage': args.stage,
        'robot_config': str(robot_path), 'task_config': str(task_path),
        'robot_ip': robot['ip'], 'local_ip': robot['local_ip'],
        'mass_kg': mass, 'cog_m': cog, 'home_rad': home,
        'stiffness': stiffness, 'desired_wrench': [0]*6,
        'duration_s': args.duration, 'home_speed_scale': .05,
        'control_point': 'existing flange-centre TCP; nonzero TCP refused',
        'force_frame': 'tool/flange; not world',
        'inertia': 'preserve current controller value; zero remains unverified',
        'guard': {'startup_s': 2, 'startup_joint_delta_rad': .02,
                  'joint_delta_rad': .25, 'joint_speed_rad_s': .15,
                  'tcp_translation_m': .03, 'tcp_rotation_rad': .15},
    }


def encode_parameters(config, output):
    lines = [config['schema'], config['robot_ip'], config['local_ip'], str(output),
             str(config['mass_kg'])]
    for key in ('cog_m', 'home_rad', 'stiffness'):
        lines.append(' '.join(str(x) for x in config[key]))
    lines.append(str(config['duration_s']))
    if any('\n' in str(x) or '\r' in str(x) for x in lines[:5]):
        raise ValueError('Newline in native connection/path parameters')
    return '\n'.join(lines) + '\n'


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--robot-config', type=Path, default=ROOT/'config/robots/rokae.yaml')
    parser.add_argument('--task-config', type=Path, default=ROOT/'config/tasks/cup.yaml')
    parser.add_argument('--stage', choices=('inspect', 'home', 'hold'), default='inspect')
    parser.add_argument('--duration', type=float, default=5, help='hold wall duration, 3..60 s; first 2 s no touch')
    parser.add_argument('--execute', action='store_true', help='Actually connect; home/hold request power and motion')
    parser.add_argument('--output-root', type=Path, default=ROOT/'data/diagnostics/impedance_test')
    args = parser.parse_args(argv)
    config = plan(args)
    print(json.dumps(config, ensure_ascii=False, indent=2), flush=True)
    if not args.execute:
        print('DRY RUN: no robot connection. Use --execute for the selected stage.')
        return 0
    executable = BUILD/'fixed_impedance'
    if not executable.is_file():
        raise RuntimeError('Build first: cmake -S rokae/impedance_test -B rokae/build/impedance_test; cmake --build rokae/build/impedance_test --parallel')
    if args.stage != 'inspect':
        token = 'MOVE_HOME' if args.stage == 'home' else 'START_IMPEDANCE'
        print('Verify the cell, payload and reachable physical stop before executing.', flush=True)
        if input(f'Type {token} to execute {args.stage}: ').strip() != token:
            print('Cancelled; no connection.')
            return 2
    args.output_root.mkdir(parents=True, exist_ok=True)
    output = Path(tempfile.mkdtemp(prefix=f'{args.stage}_', dir=args.output_root)).resolve()
    (output/'manifest.json').write_text(json.dumps(config, ensure_ascii=False, indent=2)+'\n')
    parameters = output/'parameters.txt'
    parameters.write_text(encode_parameters(config, output))
    # Set child runtime explicitly; does not change or install system libraries.
    env = os.environ.copy()
    env['LD_LIBRARY_PATH'] = str(Path(sys.prefix)/'lib') + (':'+env['LD_LIBRARY_PATH'] if env.get('LD_LIBRARY_PATH') else '')
    command = [str(executable), args.stage, str(parameters), '--execute']
    print(f'Outputs: {output}', flush=True)
    with (output/'console.txt').open('w') as capture:
        process = subprocess.Popen(command, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                                   text=True, env=env, start_new_session=True)
        try:
            assert process.stdout is not None
            for line in process.stdout:
                print(line, end='', flush=True)
                capture.write(line)
                capture.flush()
            return process.wait()
        except KeyboardInterrupt:
            # Child is in its own session so forward once, then let native cleanup
            # stopMove/stopLoop/power-off and save the trace; do not kill it mid-cleanup.
            process.send_signal(signal.SIGINT)
            print('Stop requested. Waiting for SDK cleanup; use the physical stop for immediate danger.', flush=True)
            while True:
                try:
                    remaining, _ = process.communicate(timeout=.5)
                    print(remaining, end='')
                    capture.write(remaining)
                    break
                except subprocess.TimeoutExpired:
                    continue
                except KeyboardInterrupt:
                    print('SDK cleanup still pending; check the physical stop/controller.', flush=True)
            return process.returncode


if __name__ == '__main__':
    try:
        raise SystemExit(main())
    except (ValueError, KeyError, RuntimeError) as exc:
        raise SystemExit(f'Configuration/test error: {exc}')
