#!/usr/bin/env python3
"""Offline coordinate and ideal spring checks. Never connects or chooses a robot load."""
import argparse
import itertools
import json
from pathlib import Path
import numpy as np
import yaml

ROOT = Path(__file__).resolve().parents[2]


def signed_axis_maps():
    """48 signed permutations: only 24 are right-handed rotations."""
    for order in itertools.permutations(range(3)):
        for signs in itertools.product((-1, 1), repeat=3):
            matrix = np.eye(3)[list(order)] * np.array(signs)[:, None]
            yield matrix


def audit(config):
    payload = config['robot']['payload']
    mass, cog = float(payload['mass_kg']), np.asarray(payload['cog_m'], dtype=float)
    if cog.shape != (3,) or not np.isfinite(cog).all() or not np.isfinite(mass) or mass <= 0:
        raise ValueError('invalid payload')
    # Hypothetical gravity direction only: this is NOT a measured robot pose.
    force = np.array([0., 0., -9.81 * mass])
    variants = []
    invariance_error = 0.
    stiffness = np.diag([300., 300., 300.])
    for matrix in signed_axis_maps():
        determinant = round(np.linalg.det(matrix))
        transformed = matrix @ cog
        if determinant == 1:
            invariance_error = max(invariance_error,
                np.linalg.norm(matrix @ np.cross(cog, force) - np.cross(transformed, matrix @ force)),
                np.linalg.norm(matrix @ stiffness @ matrix.T - stiffness))
        variants.append({'matrix': matrix.tolist(), 'determinant': determinant,
                         'cog_m': transformed.tolist(),
                         'hypothetical_gravity_moment_Nm': np.cross(transformed, force).tolist()})
    # Analytical critically damped 1-DOF examples, not identified ROKAE dynamics.
    scenarios = []
    t = .35
    for k in (150., 300., 600.):
        for residual in (0., 3.):
            effective_mass = 1.
            omega = np.sqrt(k/effective_mass)
            x = residual/k * (1-(1+omega*t)*np.exp(-omega*t))
            scenarios.append({'stiffness_N_m': k, 'assumed_mass_kg': effective_mass,
                              'assumed_damping_Ns_m': 2*np.sqrt(k*effective_mass),
                              'assumed_residual_force_N': residual,
                              'displacement_at_0_35s_mm': float(x*1000),
                              'equilibrium_displacement_mm': residual/k*1000})
    return {'scope': 'Offline mathematics only; no firmware, actual damping, inertia, friction or stability validation',
            'mass_kg': mass, 'configured_cog_m': cog.tolist(),
            'proper_rotations': sum(v['determinant'] == 1 for v in variants),
            'reflections_not_frame_rotations': sum(v['determinant'] == -1 for v in variants),
            'rotation_covariance_and_isotropic_stiffness_error': float(invariance_error),
            'candidate_ranking': 'none: no independent force/geometry ground truth',
            'axis_variants': variants, 'ideal_spring_scenarios': scenarios}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--robot-config', type=Path, default=ROOT/'config/robots/rokae.yaml')
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    result = audit(yaml.safe_load(args.robot_config.read_text()))
    with args.output.open('x') as stream:
        json.dump(result, stream, indent=2)
        stream.write('\n')
    print(json.dumps({k:v for k,v in result.items() if k != 'axis_variants'}, indent=2))


if __name__ == '__main__':
    main()
