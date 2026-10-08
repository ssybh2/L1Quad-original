"""Unit regression for FDI cooldown and paired fault transitional safety."""
import unittest
import numpy as np
from mode29_mujoco import BlindMotorFaultDetector, MotorModel
import tomllib
from pathlib import Path


class RecoveryCooldownTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.cfg = tomllib.loads(Path("config_opposite_pair.toml").read_text())

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
