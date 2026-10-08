"""Unit regression for FDI cooldown and paired fault transitional safety."""
import unittest
import numpy as np
from mode29_mujoco import BlindMotorFaultDetector, MotorModel
import tomllib
from pathlib import Path


class RecoveryCooldownTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.cfg = tomllib.loads(Path(__file__).resolve().parents[1].joinpath("config_opposite_pair.toml").read_text(encoding="utf-8"))

    def new_detector(self):
        return BlindMotorFaultDetector(
            self.cfg["fault_detection"],
            self.cfg["vehicle"],
            MotorModel(self.cfg["motor_model"]),
            0.0025, 12.0,
        )

    def test_detector_recovery_hold_clears_wrong_motor_allocation(self):
        d = self.new_detector()
        d.confirmed = True
        d.detected_id = 1
        d.candidate_id = 1
        d.loss_estimate_percent = 87.0
        d.begin_cooldown(30.0)
        self.assertFalse(d.confirmed)
        self.assertEqual(d.detected_id, 0)
        self.assertEqual(d.candidate_id, 0)
        self.assertEqual(d.loss_estimate_percent, 0.0)
        self.assertAlmostEqual(d.cooldown_until_s, 32.0)
        self.assertIsNone(d.prev_cmd)
        self.assertIsNone(d.prev_w_nominal)

    def test_saturated_allocator_freezes_confirmed_severity(self):
        d = self.new_detector()
        d.confirmed = True
        d.detected_id = 1
        d.loss_estimate_percent = 83.7
        d.sigma_valid = True
        d.prev_omega = np.zeros(3)
        d.record_control(np.array([13., 1., 1., 0.]),
                         np.array([80., 80., 80., 80.]))
        # An impossible observed angular acceleration does not imply
        # the rotor instantaneously deteriorated by thousands of percent.
        d.update(16.25, np.array([15., -20., 9.]),
                 freeze_confirmed_estimate=True)
        self.assertAlmostEqual(d.loss_estimate_percent, 83.7)
        self.assertEqual(d.frozen_estimate_samples, 1)
        self.assertFalse(d.loss_instant_percent > 100.)

    def test_implausible_innovation_is_rejected_without_magnitude_lock(self):
        d = self.new_detector()
        d.confirmed = True
        d.detected_id = 1
        d.loss_estimate_percent = 76.
        d.sigma_valid = True
        d.prev_omega = np.zeros(3)
        d.signature_filtered[0] = np.array([0.04, 0.03, .01])
        d.record_control(np.array([13., 0., 0., 0.]),
                         np.array([20., 20., 20., 20.]))
        d.update(16.3, np.array([17., -10., 8.]),
                 freeze_confirmed_estimate=False)
        self.assertAlmostEqual(d.loss_estimate_percent, 76.)
        self.assertGreater(d.implausible_innovation_count, 0)
        self.assertLessEqual(abs(d.loss_instant_percent), 100.)

    def test_cooldown_reanchors_baseline_without_false_confirmation(self):
        d = self.new_detector()
        d.begin_cooldown(30.0)
        for k in range(200):
            t = 30.0025 * 1.0 + k * 0.0025
            omega = np.array([0.02, -0.01, 0.1], dtype=float)
            d.record_control(np.zeros(4), np.ones(4) * 43.0)
            d.update(t, omega)
            self.assertFalse(d.confirmed)
            self.assertEqual(d.candidate_id, 0)


if __name__ == "__main__":
    unittest.main()
