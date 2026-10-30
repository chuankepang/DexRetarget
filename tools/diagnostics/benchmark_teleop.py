"""Offline IK cost and native reference step response; never connects hardware."""
from __future__ import annotations

import argparse
from dataclasses import asdict, replace
import json
from pathlib import Path
import sys
import time

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from dexretarget.config import load_yaml_mapping, PROJECT_ROOT
from dexretarget.app.cli import build_parser
from rokae.control.configuration import configured_realtime
from rokae.control.controllers.spec import resolve_controller, effective_cartesian_safety
from rokae.control.differential_ik import DifferentialIK
from rokae.control.model import XmateErPro7
from rokae.control.smoothing import JointStreamConfig
from rokae.python import _rokae_cpp as native


def native_config(cls, config):
    result = cls()
    for name, value in asdict(config).items():
        if hasattr(result, name):
            setattr(result, name, value.tolist() if isinstance(value, np.ndarray) else value)
    result.validate()
    return result


def step_response(route, response_hz):
    """50 Hz target / 1 kHz callback, unchanged v/a/jerk limits, no hardware."""
    if route == 'pinocchio':
        robot = XmateErPro7()
        config = replace(JointStreamConfig.from_limits(robot.stream_limits), response_hz=response_hz)
        core = native.JointRealtimeCore(native_config(native.JointRealtimeConfig, config))
        initial = robot.neutral.copy()
        target = initial.copy()
        target[0] += .1
        core.reset(initial, 0.)
        value = lambda state: float(state[0] - initial[0])
        amplitude = .1
    else:
        raw = load_yaml_mapping(PROJECT_ROOT/'config/robots/rokae.yaml')
        args = build_parser().parse_args([])
        spec = resolve_controller(raw, cli=route)
        config = replace(configured_realtime(args, raw['robot'], effective_cartesian_safety(raw, spec)),
                         response_hz=response_hz)
        core = native.RealtimeCore(native_config(native.RealtimeConfig, config))
        initial = np.eye(4)
        target = initial.copy()
        target[0, 3] = .01
        core.reset(initial, 0.)
        value = lambda state: float(state[0, 3])
        amplitude = .01
    samples = []
    for index in range(1, 5001):
        t = index * .001
        if index % 20 == 0:
            assert core.publish(target if t >= .1 else initial, t, index, t)
        samples.append(value(np.asarray(core.step(t))))
    samples = np.asarray(samples)
    reached = np.flatnonzero(samples >= amplitude * .5)
    v = np.diff(samples) / .001
    a = np.diff(v) / .001
    j = np.diff(a) / .001
    return dict(response_hz=response_hz, amplitude=amplitude,
                units='rad' if route == 'pinocchio' else 'm',
                time_to_half_target_ms=float((reached[0]+1)*1.-100) if len(reached) else None,
                final_error=float(amplitude-samples[-1]),
                overshoot=float(max(0., samples.max()-amplitude)),
                peak_speed=float(abs(v).max()), peak_acceleration=float(abs(a).max()),
                peak_jerk=float(abs(j).max()))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path)
    args = parser.parse_args()
    robot = XmateErPro7()
    solver = DifferentialIK(robot, robot.neutral)
    times = []
    for i in range(1200):
        q = robot.neutral.copy()
        q[0] += .05*np.sin(i*.02)
        q[6] += .2*np.sin(i*.01)
        target = robot.fk(q)
        start = time.perf_counter()
        solver.step(target, .01)
        if i >= 200:
            times.append((time.perf_counter()-start)*1000)
    report = dict(schema='dexretarget.offline_latency.v1', hardware_connected=False,
                  ik_ms=dict(zip(['p50', 'p95', 'p99', 'max'], np.percentile(times, [50,95,99,100]).tolist())),
                  reference_step={route: [step_response(route, hz) for hz in (0., 2.)]
                                  for route in ['pinocchio', 'cartesian', 'impedance']},
                  scope='Native reference generation only, not physical robot response, SDK filtering or network latency. '
                        'response_hz=0 reproduces legacy a/v coupling; no Quest prediction.')
    text = json.dumps(report, indent=2, ensure_ascii=False)
    print(text)
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(text+'\n')


if __name__ == '__main__':
    main()
