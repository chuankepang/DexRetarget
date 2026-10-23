#!/usr/bin/env python3
"""Summarize a saved fixed-reference trace. Does not connect to a robot."""
from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path

import numpy as np


def summarize(path: Path):
    with path.open() as stream:
        rows = list(csv.DictReader(stream))
    if not rows:
        raise ValueError('No callback samples; inspect console.txt and summary.txt')
    def array(prefix, width):
        return np.array([[float(row[f'{prefix}_{i}']) for i in range(width)] for row in rows])
    measured, reference = array('tcp_m', 16), array('reference', 16)
    q, dq = array('q_m', 7), array('dq_m', 7)
    seconds = np.array([float(row['seconds']) for row in rows])
    periods = np.array([float(row['callback_period_s']) for row in rows])
    if not all(np.isfinite(value).all() for value in (measured, reference, q, dq, seconds, periods)):
        raise ValueError('Trace contains invalid state; inspect summary stop_reason before analysis')
    displacement = np.linalg.norm(measured[:, [3, 7, 11]] - reference[:, [3, 7, 11]], axis=1)
    initial = seconds < 2
    summary_path = path.with_name('summary.txt')
    summary = {}
    if summary_path.is_file():
        for line in summary_path.read_text().splitlines():
            if '=' in line:
                key, value = line.split('=', 1)
                summary[key] = value
    stop = summary.get('stop_reason', rows[-1]['stop'])
    guard_reasons = {'startup_joint_drift', 'joint_excursion', 'joint_speed',
                     'tcp_displacement', 'tcp_rotation', 'invalid_state', 'trace_buffer_full'}
    stop_source = ('software_guard' if stop in guard_reasons else
                   'controller_health_check' if stop == 'controller_power_or_mode_fault' else
                   'timer' if stop == 'duration_completed' else 'other_or_unknown')
    result = {
        'samples': len(rows), 'duration_sampled_s': float(seconds[-1]),
        'last_sample_stop': rows[-1]['stop'],
        'runtime_stop_reason': stop,
        'stop_source': stop_source,
        'power_off_readback_confirmed': (summary['power_off_readback'] == '1'
                                        if 'power_off_readback' in summary else None),
        'actual_tcp_delta_mm': ((measured[-1, [3, 7, 11]] - measured[0, [3, 7, 11]])*1000).tolist(),
        'reference_is_constant': bool(np.all(reference == reference[0])),
        'max_tcp_displacement_mm': float(displacement.max()*1000),
        'final_tcp_displacement_mm': float(displacement[-1]*1000),
        'startup_max_tcp_displacement_mm': float(displacement[initial].max()*1000),
        'max_abs_joint_displacement_rad': np.max(np.abs(q-q[0]), axis=0).tolist(),
        'max_abs_joint_speed_rad_s': np.max(np.abs(dq), axis=0).tolist(),
        'max_callback_period_ms': float(periods.max()*1000),
        'scope': 'Callback samples only; not controller servo timing, a stiffness measurement, or safety certification',
        'force_measurement': 'No calibrated external force recorded; cannot calculate actual stiffness or damping from this trace alone',
    }
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('trace', type=Path)
    args = parser.parse_args()
    print(json.dumps(summarize(args.trace), ensure_ascii=False, indent=2))


if __name__ == '__main__':
    main()
