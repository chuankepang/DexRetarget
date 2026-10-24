"""Physical consistency checks, not simulation of the proprietary controller."""
import importlib.util
from pathlib import Path
import numpy as np

path = Path(__file__).resolve().parents[1]/'rokae/impedance_test/offline_audit.py'
spec = importlib.util.spec_from_file_location('offline_audit', path)
audit = importlib.util.module_from_spec(spec)
spec.loader.exec_module(audit)


def test_axis_maps_and_force_covariance():
    result = audit.audit({'robot': {'payload': {'mass_kg': 1, 'cog_m': [.02, -.03, .05]}}})
    matrices = [np.array(v['matrix']) for v in result['axis_variants']]
    assert len({tuple(m.ravel()) for m in matrices}) == 48
    assert result['proper_rotations'] == result['reflections_not_frame_rotations'] == 24
    assert result['rotation_covariance_and_isotropic_stiffness_error'] < 1e-12
    for value in result['axis_variants']:
        assert np.isclose(np.linalg.norm(value['cog_m']), np.linalg.norm([.02,-.03,.05]))


def test_residual_force_has_offset_and_zero_input_does_not_drift():
    result = audit.audit({'robot': {'payload': {'mass_kg': 1, 'cog_m': [0,0,.05]}}})
    for row in result['ideal_spring_scenarios']:
        if row['assumed_residual_force_N'] == 0:
            assert row['displacement_at_0_35s_mm'] == 0
        else:
            assert 0 < row['displacement_at_0_35s_mm'] < row['equilibrium_displacement_mm']
            assert np.isclose(row['equilibrium_displacement_mm'], 3000/row['stiffness_N_m'])
