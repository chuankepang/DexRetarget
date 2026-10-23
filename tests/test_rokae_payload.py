"""Calibration units and native startup guard; no robot connections."""
from pathlib import Path

import numpy as np
import pytest
import yaml

from dexretarget.teleop.devices.rokae import RokaeXCoreDriver
from rokae.control.controllers.spec import apply_realtime_gains, resolve_controller
from native_support import HAS_NATIVE

ROOT = Path(__file__).resolve().parents[1]


def calibrated_spec():
    config = yaml.safe_load((ROOT / 'config/robots/rokae.yaml').read_text())
    return resolve_controller(config, cli='impedance')


def test_calibration_uses_metres_and_full_assembly_mass():
    spec = calibrated_spec()
    assert spec.expected_load_mass == pytest.approx(0.93938639463)
    np.testing.assert_allclose(spec.expected_load_cog,
        np.array([-52.71043598571, 12.17669684527, 8.60984186271]) / 1000)


@pytest.mark.parametrize('payload', [
    {'mass_kg': 0, 'cog_m': [0, 0, 0]},
    {'mass_kg': float('nan'), 'cog_m': [0, 0, 0]},
    {'mass_kg': 1, 'cog_m': [0, 0]},
    {'mass_kg': 1, 'cog_m': [0, float('inf'), 0]},
])
def test_invalid_calibration_rejected(payload):
    with pytest.raises(ValueError):
        resolve_controller({'robot': {'payload': payload}})


@pytest.mark.skipif(not HAS_NATIVE, reason='Native bridge unavailable; not validated')
def test_config_to_native_and_controller_tool_matching():
    from rokae.python import _rokae_cpp
    driver = RokaeXCoreDriver('127.0.0.1', '127.0.0.1')
    spec = calibrated_spec()
    apply_realtime_gains(driver, spec)
    cfg = driver._cpp_config(_rokae_cpp)
    assert cfg.apply_load_on_impedance_start is True
    assert cfg.expected_load_mass == spec.expected_load_mass
    np.testing.assert_allclose(cfg.expected_load_cog, spec.expected_load_cog)
    # RobotAssist displays rounded values; zero inertia is reported, not invented.
    cfg.validate_impedance_load(0.939386, [-0.05271043, 0.012176697, 0.008609842], [0, 0, 0])
    for mass, cog, inertia in [
        (0, spec.expected_load_cog, [0, 0, 0]),
        (1.25, spec.expected_load_cog, [0, 0, 0]),
        (spec.expected_load_mass, [0, 0, 0], [0, 0, 0]),
        (spec.expected_load_mass, np.array(spec.expected_load_cog) * 1000, [0, 0, 0]),
        (spec.expected_load_mass, spec.expected_load_cog, [float('nan'), 0, 0]),
        (spec.expected_load_mass, spec.expected_load_cog, [-1, 0, 0]),
    ]:
        with pytest.raises(RuntimeError):
            cfg.validate_impedance_load(mass, cog, inertia)


@pytest.mark.skipif(not HAS_NATIVE, reason='Native bridge unavailable; not validated')
def test_native_legacy_config_and_invalid_expected_load():
    from rokae.python import _rokae_cpp
    cfg = _rokae_cpp.RealtimeConfig()
    cfg.validate_impedance_load(1, [0, 0, 0], [0, 0, 0])
    cfg.expected_load_mass = float('nan')
    with pytest.raises(ValueError):
        cfg.validate()


def test_payload_application_is_explicit_and_boolean():
    assert not resolve_controller({}).apply_load_on_impedance_start
    with pytest.raises(ValueError, match="must be a boolean"):
        resolve_controller({'robot': {'payload': {
            'mass_kg': 1, 'cog_m': [0, 0, 0], 'apply_on_impedance_start': 'false'}}})


@pytest.mark.skipif(not HAS_NATIVE, reason='Native bridge unavailable; not validated')
def test_native_apply_requires_calibrated_mass():
    from rokae.python import _rokae_cpp
    cfg = _rokae_cpp.RealtimeConfig()
    assert not cfg.apply_load_on_impedance_start
    cfg.apply_load_on_impedance_start = True
    with pytest.raises(ValueError, match='positive calibrated mass'):
        cfg.validate()
