"""Offline regression tests for observer-gated opposite-motor pairing.

Unit tests exercise the map, two-efficiency allocator, mirrored thrust model,
and experimental truth-as-veto gating, not physical flight stability.
"""
import math
import unittest
from types import SimpleNamespace
import numpy as np
from opposite_pair_sim import (
    OppositePairExperiment, allocate_opposite_pair,
    apply_opposite_model_loss, opposite_motor,
)


class ToyMotor:
    def thrust(self, w):
        return float(w) * 0.14

    def w_from_thrust(self, thrust):
        return max(0.0, min(100.0, float(thrust) / 0.14))


class ToyMixer:
    L = 0.28
    D = 0.28
    motor = ToyMotor()


class OppositePairTests(unittest.TestCase):
    def test_diagonal_indices(self):
        self.assertEqual([opposite_motor(i) for i in range(1, 5)], [2, 1, 4, 3])

    def test_nominal_symmetric_hover_wrench(self):
        mixer = ToyMixer()
        desired = [15.0, 0.0, 0.0, 0.0]
        for faulty in range(1, 5):
            cmd = allocate_opposite_pair(mixer, desired, faulty, 60.0)
            eta = np.ones(4)
            eta[faulty-1] = 0.4
            eta[opposite_motor(faulty)-1] = 0.4
            actual_f = np.array([mixer.motor.thrust(w) for w in cmd])*eta
            roll = .14 * (-actual_f[0]+actual_f[1]+actual_f[2]-actual_f[3])
            pitch = .14 * (actual_f[0]-actual_f[1]+actual_f[2]-actual_f[3])
            self.assertAlmostEqual(float(actual_f.sum()), 15., places=5)
            self.assertAlmostEqual(float(roll), 0., places=5)
            self.assertAlmostEqual(float(pitch), 0., places=5)

    def test_injected_and_mirrored_loss_separate(self):
        motor = ToyMotor()
        w = np.array([50., 50., 50., 50.])
        after_primary = w.copy()
        after_primary[0] = motor.w_from_thrust(.4 * motor.thrust(w[0]))
        after_mirror = apply_opposite_model_loss(after_primary, motor, 2, 60.)
        self.assertAlmostEqual(after_mirror[0], 20.)
        self.assertAlmostEqual(after_mirror[1], 20.)
        self.assertAlmostEqual(after_mirror[2], 50.)
        self.assertAlmostEqual(after_mirror[3], 50.)

    def test_fdi_guard_and_engagement(self):
        cfg = {"enabled": True}
        vehicle = {"mass_kg": 1.0, "gravity_mps2": 9.8}
        pair = OppositePairExperiment(cfg, vehicle, ToyMotor(), 30.)
        fault = SimpleNamespace(motor_id=1, loss_percent=60.)
        detector = SimpleNamespace(
            confirmed=True, detected_id=1,
            loss_estimate_percent=62., residual_ratio=.1
        )
        pair.update(5., [0., 0., -1.], [0., 0., 0.], 1., True, fault, detector)
        self.assertFalse(pair.active, "FDI must settle before synthetic mirroring")
        self.assertTrue(pair.retry_pending)
        pair.update(5.2, [0., 0., -1.], [0., 0., 0.], 1., True, fault, detector)
        self.assertTrue(pair.active)
        self.assertEqual(pair.opposite_motor_id, 2)
        self.assertEqual(pair.estimated_loss_percent, 62.)

        pair.update(5.21, [.7, 0., -1.], [0., 0., 0.], 1., True, fault, detector)
        self.assertFalse(pair.active)
        self.assertFalse(pair.inhibited, "transient position guard must be retryable")
        self.assertTrue(pair.retry_pending)
        self.assertGreater(pair.retry_at_s, 5.21)
        pair.update(5.7, [0., 0., -1.], [0., 0., 0.], 1., True, fault, detector)
        self.assertTrue(pair.active, "pair must re-arm after state returns inside guard")

    def test_wrong_motor_is_blocked(self):
        pair = OppositePairExperiment({"enabled": True},
                                      {"mass_kg": 1., "gravity_mps2": 9.8},
                                      ToyMotor(), 30.)
        fault = SimpleNamespace(motor_id=1, loss_percent=60.)
        detector = SimpleNamespace(
            confirmed=True, detected_id=3,
            loss_estimate_percent=60., residual_ratio=.1
        )
        pair.update(5., [0., 0., -1.], [0., 0., 0.], 1., True, fault, detector)
        self.assertFalse(pair.active)
        self.assertTrue(pair.inhibited)

    def test_arbitrary_loss_fractions(self):
        motor = ToyMotor()
        for loss in (0.0, 0.1, 1.0, 5.0, 12.5, 25.0,
                     40.0, 50.0, 60.0, 75.0, 90.0, 99.0, 100.0):
            with self.subTest(loss=loss):
                w = np.array([60., 60., 60., 60.])
                mirrored = apply_opposite_model_loss(w, motor, 2, loss)
                self.assertAlmostEqual(motor.thrust(mirrored[1]),
                                       (1.0-loss/100.)*motor.thrust(60.), places=9)
                self.assertEqual(mirrored[0], 60.)
                self.assertEqual(mirrored[2], 60.)
                self.assertEqual(mirrored[3], 60.)

    def test_rank_degeneracy_is_not_a_special_percentage_threshold(self):
        # Full 100% diagonal loss is a rank-two actuator plant, not an
        # arbitrary 'maximum allowed percentage' in the input model.
        with self.assertRaisesRegex(RuntimeError, "rank-deficient"):
            allocate_opposite_pair(ToyMixer(), [15., 0., 0.], 1, 100.)
        with self.assertRaises(ValueError):
            allocate_opposite_pair(ToyMixer(), [15., 0., 0.], 1, 101.)
        for loss in (0.0, 20.0, 60.0, 75.0):
            w = allocate_opposite_pair(ToyMixer(), [10., 0., 0.], 1, loss)
            self.assertTrue(np.isfinite(w).all())

    def test_higher_than_70_can_engage_if_authority_exists(self):
        pair = OppositePairExperiment({"enabled": True},
                                      {"mass_kg": 1., "gravity_mps2": 9.8},
                                      ToyMotor(), 30.)
        fault = SimpleNamespace(motor_id=1, loss_percent=75.)
        detector = SimpleNamespace(
            confirmed=True, detected_id=1,
            loss_estimate_percent=75., residual_ratio=.1
        )
        pair.update(5., [0., 0., -1.], [0., 0., 0.], 1., True, fault, detector)
        pair.update(5.2, [0., 0., -1.], [0., 0., 0.], 1., True, fault, detector)
        self.assertTrue(pair.active)

    def test_transient_underestimate_is_retryable(self):
        pair = OppositePairExperiment({"enabled": True},
                                      {"mass_kg": 1., "gravity_mps2": 9.8},
                                      ToyMotor(), 30.)
        fault = SimpleNamespace(motor_id=1, loss_percent=70.)
        detector = SimpleNamespace(confirmed=True, detected_id=1,
                                   loss_estimate_percent=41., residual_ratio=.1)
        pair.update(5.0, [0., 0., -1.], [0., 0., 0.], 1., True, fault, detector)
        self.assertFalse(pair.active)
        self.assertFalse(pair.inhibited, "early observer bias must not permanently block pair")
        self.assertTrue(pair.retry_pending)
        detector.loss_estimate_percent = 70.
        pair.update(5.2, [0., 0., -1.], [0., 0., 0.], 1., True, fault, detector)
        self.assertFalse(pair.active)
        pair.update(5.4, [0., 0., -1.], [0., 0., 0.], 1., True, fault, detector)
        self.assertTrue(pair.active)
        self.assertEqual(pair.failed_motor_id, 1)
        self.assertEqual(pair.opposite_motor_id, 2)

    def test_confirmed_identity_quality_survives_low_innovation_snr(self):
        pair = OppositePairExperiment({"enabled": True},
                                      {"mass_kg": 1., "gravity_mps2": 9.8},
                                      ToyMotor(), 30.)
        fault = SimpleNamespace(motor_id=1, loss_percent=70.)
        # The identity was confidently fitted before reallocation, while
        # the near-zero innovation has poor normalized fit as it converges.
        detector = SimpleNamespace(confirmed=True, detected_id=1,
                                   loss_estimate_percent=70.,
                                   residual_ratio=0.92,
                                   identity_fit_ratio=0.03)
        pair.update(5., [0., 0., -1.], [0., 0., 0.], 1., True, fault, detector)
        self.assertFalse(pair.active)
        pair.update(5.2, [0., 0., -1.], [0., 0., 0.], 1., True, fault, detector)
        self.assertTrue(pair.active)
        self.assertEqual(pair.opposite_motor_id, 2)

    def test_exact_zero_injection_is_no_fault(self):
        pair = OppositePairExperiment({"enabled": True, "source": "oracle"},
                                      {"mass_kg": 1., "gravity_mps2": 9.8},
                                      ToyMotor(), 30.)
        fault = SimpleNamespace(motor_id=1, loss_percent=0.0)
        detector = SimpleNamespace(confirmed=False, detected_id=0,
                                   loss_estimate_percent=0., residual_ratio=1.0)
        pair.update(5., [0., 0., -1.], [0., 0., 0.], 1., False, fault, detector)
        self.assertFalse(pair.active)
        self.assertFalse(pair.inhibited)


if __name__ == "__main__":
    unittest.main()
