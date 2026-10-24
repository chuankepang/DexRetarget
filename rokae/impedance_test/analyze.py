#!/usr/bin/env python3
"""Summarize a saved fixed-reference trace. Does not connect to a robot."""
from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path

import numpy as np


def force_snapshots(path: Path):
    """Preserve repeated phase records; never overwrite NRT with the last RT row."""
    records = []
    current = None
    widths = {'joint_torque_measured_Nm': 7, 'joint_external_estimate_Nm': 7,
              'world_cart_torque_estimate_Nm': 3, 'world_cart_force_estimate_N': 3}
    for line in path.read_text().splitlines():
        if '=' not in line:
            continue
        key, value = line.split('=', 1)
        if key == 'force_snapshot_phase':
            current = {'phase': value, 'available': False}
            records.append(current)
        elif current is not None:
            if key == 'force_snapshot_available':
                current['available'] = value == '1'
            elif key == 'force_snapshot_error':
                current['error'] = value
            elif key in widths:
                values = [float(x) for x in value.split()]
                if len(values) != widths[key] or not np.isfinite(values).all():
                    raise ValueError(f'Invalid force snapshot {key}')
                current[key] = values
    for record in records:
        if record['available'] and not all(key in record for key in widths):
            record['available'] = False
            record['error'] = 'incomplete force snapshot'
    return records


def summarize_preparation(path: Path):
    fields = {}
    for line in path.read_text().splitlines():
        if '=' in line:
            key, value = line.split('=', 1)
            fields[key] = value
    return {'stage': fields.get('stage'),
            'rt_load_mode': fields.get('rt_load_mode'),
            'rt_load_write_performed': {'0': False, '1': True}.get(fields.get('rt_load_write_performed')),
            'power_off_readback_confirmed': (fields['power_off_readback'] == '1'
                                            if 'power_off_readback' in fields else None),
            'impedance_start_attempted': fields.get('impedance_start_attempted'),
            'force_snapshots': force_snapshots(path),
            'scope': 'Controller model estimates at separate RPC reads; not synchronized independent force measurements or proof of calibration quality'}


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
    q_origin = q[0]
    origin_source = 'first_callback_sample'
    if 'initial_impedance_q' in summary:
        q_origin = np.array([float(x) for x in summary['initial_impedance_q'].split()])
        if q_origin.shape != (7,) or not np.isfinite(q_origin).all():
            raise ValueError('Invalid initial_impedance_q in summary')
        origin_source = 'pre_start_reference'
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
        'max_abs_joint_displacement_rad': np.max(np.abs(q-q_origin), axis=0).tolist(),
        'joint_displacement_origin': origin_source,
        'final_joint_speed_rad_s': dq[-1].tolist(),
        'max_abs_joint_speed_rad_s': np.max(np.abs(dq), axis=0).tolist(),
        'max_callback_period_ms': float(periods.max()*1000),
        'scope': 'Callback samples only; not controller servo timing, a stiffness measurement, or safety certification',
        'force_measurement': 'No calibrated external force recorded; cannot calculate actual stiffness or damping from this trace alone',
    }
    # 老 trace 只有参考点，不能据此声称 SDK/控制器实际收到恒定目标。
    result['sent_command_recorded'] = 'sent_tcp_0' in rows[0]
    if result['sent_command_recorded']:
        sent = array('sent_tcp', 16)
        result['sent_command_finite'] = bool(np.isfinite(sent).all())
        result['sent_command_matches_reference'] = bool(np.array_equal(sent, reference))
    if 'tcp_c_valid' in rows[0]:
        controller = array('tcp_c', 16)
        mask = np.array([row['tcp_c_valid'] == '1' for row in rows])
        mask &= np.isfinite(controller).all(axis=1)
        result['controller_command_valid_samples'] = int(mask.sum())
        result['controller_command_valid_fraction'] = float(mask.mean())
        result['controller_command_max_translation_from_reference_mm'] = (
            float(np.linalg.norm(controller[mask][:, [3, 7, 11]] - reference[mask][:, [3, 7, 11]], axis=1).max()*1000)
            if mask.any() else None)
        result['controller_command_scope'] = 'SDK pos_c feedback; validity is not independent verification of internal impedance or servo execution'
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('trace', type=Path, help='trace.csv, summary.txt or a run directory')
    args = parser.parse_args()
    path = args.trace
    if path.is_dir():
        path = path/('trace.csv' if (path/'trace.csv').exists() else 'summary.txt')
    result = summarize_preparation(path) if path.name == 'summary.txt' else summarize(path)
    print(json.dumps(result, ensure_ascii=False, indent=2))


if __name__ == '__main__':
    main()
