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


ROOT = Path(__file__).resolve().parents[2]
BUILD = ROOT / 'rokae/build/impedance_test'
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))
from dexretarget.config import load_yaml_mapping
from rokae.control.controllers import resolve_controller



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
    robot_config = load_yaml_mapping(robot_path)
    task_config = load_yaml_mapping(task_path)
    robot = robot_config['robot']
    if robot['type'] != 'xmate-er-pro-7':
        raise ValueError('This isolated test supports the current xMatePro3 seven-axis assembly only')
    if args.interaction != 'baseline' and args.stage != 'hold':
        raise ValueError('light-push requires hold stage')
    if args.rt_load_mode == 'skip' and args.stage not in ('prepare', 'hold'):
        raise ValueError('RT load skip comparison requires prepare or hold stage')
    payload = robot['payload']
    apply_payload = payload.get('apply_on_impedance_start', False)
    if not isinstance(apply_payload, bool):
        raise ValueError('payload.apply_on_impedance_start must be a boolean')
    if not apply_payload and args.experimental_cog_mm is not None:
        raise ValueError('experimental CoM requires apply_on_impedance_start=true')
    mass = float(payload['mass_kg'])
    if not math.isfinite(mass) or not 0 < mass <= 3:
        raise ValueError('payload mass must be in (0, 3] kg')
    cog = vector(payload['cog_m'], 3, 'payload.cog_m')
    configured_cog = cog.copy()
    if args.profile in ('robotassist', 'spring-check') and (args.stage not in ('hold', 'prepare') or args.experimental_cog_mm is not None):
        raise ValueError(f'{args.profile} profile requires hold or prepare and the configured payload; remove experimental CoM override')
    if args.experimental_cog_mm is not None:
        if args.stage != 'hold':
            raise ValueError('--experimental-cog-mm is only allowed for the fixed hold stage')
        # 仅本次试验覆盖；输入 mm，原生 SDK 始终接收 m。原 YAML 不修改。
        cog = [x / 1000.0 for x in vector(args.experimental_cog_mm, 3, 'experimental CoM mm')]
    if any(abs(x) > .3 for x in cog):
        raise ValueError('payload.cog_m: metres within 0.3 m per axis required')
    home = vector(task_config['home']['arm_joints'], 7, 'home.arm_joints')
    spec = resolve_controller(robot_config, cli='impedance')
    if any(spec.desired_wrench):
        raise ValueError('Fixed no-contact test requires desired_wrench=0; nonzero force is not silently ignored')
    stiffness = spec.stiffness.tolist()
    configured_stiffness = stiffness.copy()
    if args.profile == 'robotassist':
        # 用户成功 RL 截图：只匹配六轴刚度及世界方向，不冒充完整 tool1/FcInit 复现。
        stiffness = [100.0] * 6
    elif args.profile == 'spring-check':
        # 与 robotassist 对照仅提高平移刚度；力控系、转动刚度、负载和护栏保持一致。
        stiffness = [300.0, 300.0, 300.0, 100.0, 100.0, 100.0]
    if any(not 0 < x <= (1500 if i < 3 else 100) for i, x in enumerate(stiffness)):
        raise ValueError('stiffness outside SDK 0.3.4 ER Pro limits')
    if not math.isfinite(args.duration) or not 3 <= args.duration <= 60:
        raise ValueError('duration must be 3..60 seconds')
    return {
        'schema': 'rokae.fixed_impedance.v5', 'stage': args.stage,
        'apply_on_impedance_start': apply_payload,
        'rt_load_mode': args.rt_load_mode,
        'rt_load_policy': ('explicitly sync validated active SDK load after entering RT; no assumption of automatic tool selection'
                           if args.rt_load_mode == 'explicit' else
                           'experimental: skip RT setLoad; existing RT compensation is unknown and not independently read back'),
        'interaction': args.interaction,
        'robot_config': str(robot_path), 'task_config': str(task_path),
        'robot_ip': robot['ip'], 'local_ip': robot['local_ip'],
        'mass_kg': mass, 'cog_m': cog, 'home_rad': home,
        'configured_cog_m': configured_cog,
        'payload_source': ('experimental_cli_unverified' if args.experimental_cog_mm is not None
                           else 'robot_config' if apply_payload else 'active_sdk_toolset_checked_against_config'),
        'stiffness': stiffness, 'configured_stiffness': configured_stiffness,
        'profile': args.profile, 'desired_wrench': [0]*6,
        'duration_s': args.duration, 'home_speed_scale': .50,
        'on_duration_completed': 'stop impedance, request power-off and verify; normal timed completion',
        'startup': args.startup,
        'sdk_runtime_required': '0.3.4',
        'nrt_home_speed_sdk': 100 if args.startup == 'latest-compatible' else None,
        'preparation': ('NRT MoveAbsJ home -> stationary 0.5 s -> RT setup -> restart subscription -> 3 packets'
                        if args.startup == 'latest-compatible' else
                        'legacy RT MoveJ home -> stationary 0.5 s -> RT setup'),
        'home_settle_position_rad': .002, 'home_settle_speed_rad_s': .01,
        'control_point': 'existing flange-centre TCP; nonzero TCP refused',
        'force_frame': 'world' if args.profile in ('robotassist', 'spring-check') else spec.force_frame,
        'inertia': 'preserve current controller value; zero remains unverified',
        'guard': {'startup_s': 2, 'startup_joint_delta_rad': .04,
                  'joint_delta_rad': 1.0 if args.interaction == 'light-push' else .50,
                  'startup_joint_speed_rad_s': .30,
                  'joint_speed_rad_s': .70 if args.interaction == 'light-push' else .30,
                  'startup_tcp_translation_m': .06,
                  'tcp_translation_m': .10 if args.interaction == 'light-push' else .06,
                  'tcp_rotation_rad': .30},
    }


def encode_parameters(config, output):
    lines = [config['schema'], config['robot_ip'], config['local_ip'], str(output),
             str(config['mass_kg'])]
    for key in ('cog_m', 'home_rad', 'stiffness'):
        lines.append(' '.join(str(x) for x in config[key]))
    lines.append(str(config['duration_s']))
    if config['schema'] in ('rokae.fixed_impedance.v2', 'rokae.fixed_impedance.v3', 'rokae.fixed_impedance.v4', 'rokae.fixed_impedance.v5'):
        lines.append(config['force_frame'])
    if config['schema'] in ('rokae.fixed_impedance.v3', 'rokae.fixed_impedance.v4', 'rokae.fixed_impedance.v5'):
        lines.append(config['startup'])
    if config['schema'] in ('rokae.fixed_impedance.v4', 'rokae.fixed_impedance.v5'):
        lines.append(config['interaction'])
    if config['schema'] == 'rokae.fixed_impedance.v5':
        lines.append('true' if config['apply_on_impedance_start'] else 'false')
    if any('\n' in str(x) or '\r' in str(x) for x in lines[:5]):
        raise ValueError('Newline in native connection/path parameters')
    return '\n'.join(lines) + '\n'


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--robot-config', type=Path, default=ROOT/'config/robots/rokae.yaml')
    parser.add_argument('--task-config', type=Path, default=ROOT/'config/tasks/cup.yaml')
    parser.add_argument('--stage', choices=('inspect', 'home', 'prepare', 'hold'), default='inspect',
                        help='prepare: home, configure RT and capture force estimates, then power off before starting impedance')
    parser.add_argument('--profile', choices=('configured', 'robotassist', 'spring-check'), default='configured',
                        help='robotassist: world stiffness 100x6; spring-check: world stiffness 300x3+100x3; keep configured payload and guards')
    parser.add_argument('--startup', choices=('latest-compatible', 'legacy-rt'), default='latest-compatible',
                        help='latest-compatible: backport NRT home and fresh RT reference using SDK 0.3.4; legacy-rt: old startup for comparison')
    parser.add_argument('--rt-load-mode', choices=('explicit', 'skip'), default='explicit',
                        help='skip: isolated RT setLoad comparison for prepare/hold; does not reset or verify existing RT compensation')
    parser.add_argument('--interaction', choices=('baseline', 'light-push'), default='baseline',
                        help='baseline: no touch; light-push: after 2 seconds, small push/release with 0.70 rad/s host guard')
    parser.add_argument('--duration', type=float, default=5, help='hold wall duration, 3..60 s; first 2 s no touch')
    parser.add_argument('--experimental-cog-mm', nargs=3, type=float, metavar=('X', 'Y', 'Z'),
                        help='Unverified CoM override in flange-frame millimetres; fixed hold only; does not edit YAML')
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
        action = ('回 home 后开启阻抗测试' if args.stage == 'hold' else
                  '回 home 并配置 RT 诊断（不启动阻抗）' if args.stage == 'prepare' else '回 home')
        if args.experimental_cog_mm is not None:
            print('EXPERIMENTAL CoM: remain clear and DO NOT TOUCH for the entire test. '
                  'The test writes SDK/RT load; exit does not explicitly restore the previous load.', flush=True)
        print('This stage first moves to task HOME in position control. Verify the entire return path, payload and physical stop.', flush=True)
        print(f"回 home 速度倍率：{config['home_speed_scale']:g}（原 0.05 的 10 倍）；到位后仍需静止确认。", flush=True)
        if args.stage == 'hold':
            print(f'本次阻抗观察 {args.duration:g} 秒，到时程序自动停止并下电；不会一直保持使能。', flush=True)
        if args.stage == 'prepare':
            print('PREPARE: 比较上电 NRT/RT 配置后的力矩估计；不调用阻抗 startMove，不启动控制回调，完成后下电。', flush=True)
        if args.rt_load_mode == 'skip':
            print('RT LOAD COMPARISON: 跳过 RT setLoad；不是将整套负载质量改为零，也不保证继承了正确的 RT 负载。', flush=True)
        if args.interaction == 'light-push':
            print('轻推测试：前 2 秒不触碰，提示后先缓慢推移 10–20 mm 再松手；位移边界 100 mm，不是自由拖拽。', flush=True)
        if input(f'{action}？输入 y 回车开始，直接回车取消 [y/N]: ').strip().lower() != 'y':
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
    command = [str(executable), args.stage, str(parameters), '--execute',
               '--rt-load-mode='+config['rt_load_mode']]
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
