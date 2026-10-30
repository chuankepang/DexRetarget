"""Offline attribution distinguishes TCP accuracy from redundancy motion."""
import dataclasses
import unittest

import numpy as np

from rokae.control.model import XmateErPro7
from tools.diagnostics.audit_rokae_ik import audit


class IKAuditTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.report = audit()

    def test_fixed_tcp_does_not_imply_minimal_joint_motion(self):
        cases = self.report['cases']
        current = cases['cup_hold']
        self.assertLess(current['final_translation_error_mm'], 0.2)
        self.assertLess(current['peak_joint_excursion_deg'], 1e-8)
        # Fixed model posture still deliberately changes redundant joints;
        # strict projection alone must not be confused with holding posture.
        self.assertGreater(cases['cup_hold_model_posture']['peak_joint_excursion_deg'], 1.)
        attribution = cases['cup_hold_no_secondary']
        self.assertLess(attribution['peak_joint_excursion_deg'], 1e-8)
        self.assertLess(attribution['final_translation_error_mm'], 1e-8)
        self.assertLess(cases['cup_hold_task_posture']['peak_joint_excursion_deg'], 1e-8)

    def test_experiments_keep_default_gains_and_reference_isolated(self):
        robot = XmateErPro7()
        self.assertEqual(self.report['ik_gains'], dataclasses.asdict(robot.ik_gains))
        np.testing.assert_array_equal(
            self.report['cases']['cup_hold']['neutral_position_rad'], robot.neutral)
        self.assertGreater(self.report['cases']['cup_hold']['secondary_gains']['posture_gain'], 0)
        for case in self.report['cases'].values():
            self.assertEqual(case['steps'], 500)
            self.assertTrue(np.isfinite(case['final_rotation_error_deg']))
            self.assertLess(case['exact_JN_frobenius'], 1e-12)
            self.assertEqual(len(case['final_joint_delta_deg']), 7)
