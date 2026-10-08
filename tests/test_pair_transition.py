"""Generic pairing transition tests: retries and smooth mirror activation.

These tests do not rely on any privileged failure percentage or the MuJoCo
GUI and do not claim feasibility for every real motor/airframe.
"""
import unittest
from types import SimpleNamespace
import numpy as np

from opposite_pair_sim import OppositePairExperiment, opposite_motor
from yaw_rate_schedule import paired_primary_feasibility, allocate_pair_yaw_control


class ToyMotor:
    def thrust(self, w):
        return float(w) * 0.14

    def w_from_thrust(self, f):
        return np.clip(float(f) / .14, 0., 100.)

    def moment(self, w):
        return .006 * float(w)


class ToyMixer:
    L = .28
    D = .28
    motor = ToyMotor()


class RecoveryTest(unittest.TestCase):
    def make_pair(self, loss, motor_id=1):
        pair = OppositePairExperiment({
            "enabled": True, "source": "fdi",
            "feasible_hold_s": .15, "retry_delay_s": .3,
            "mirror_ramp_rate_pp_s": 70.,
        }, {"mass_kg": 1., "gravity_mps2": 9.8, "L_m": .28, "D_m": .28},
                                      ToyMotor(), 30.)
        fault = SimpleNamespace(motor_id=motor_id, loss_percent=loss)
        fdi = SimpleNamespace(confirmed=True, detected_id=motor_id,
                              loss_estimate_percent=loss, residual_ratio=.02,
                              identity_fit_ratio=.02)
        pair.update(5., [0., 0., -1.], [0., 0., 0.], 1., True, fault, fdi)
        pair.update(5.2, [0., 0., -1.], [0., 0., 0.], 1., True, fault, fdi)
        self.assertTrue(pair.active)
        return pair, fault, fdi

    def test_dynamic_infeasibility_waits_then_ramps(self):
        pair, fault, fdi = self.make_pair(84.)
        # This wrench is impossible with both weakened motors at 84%.
        severe = [12.194, -1.663, 1.627, 0.0]
        at_full = paired_primary_feasibility(ToyMixer(), severe, 1, 84., 84.)
        self.assertFalse(at_full["feasible"])
        pair.plan_mirror(5.2, .0025, severe, ToyMixer())
        self.assertEqual(pair.mirror_loss_percent, 0.)
        self.assertFalse(pair.active)
        self.assertTrue(pair.retry_pending)
        self.assertFalse(pair.inhibited, "one impossible moment request cannot permanently latch")

        mild = [10.29, -.083, .007, 0.]
        self.assertTrue(paired_primary_feasibility(ToyMixer(), mild, 1, 84., 84.)["feasible"])
        pair.update(5.55, [0., 0., -1.], [0., 0., 0.], 1., True, fault, fdi)
        self.assertTrue(pair.active, "re-enter after transient with previously confirmed FDI")
        pair.plan_mirror(5.55, .0025, mild, ToyMixer())
        self.assertEqual(pair.mirror_loss_percent, 0.)
        pair.plan_mirror(5.72, .0025, mild, ToyMixer())
        self.assertGreater(pair.mirror_loss_percent, 0.)
        self.assertLess(pair.mirror_loss_percent, 84.)
        old = pair.mirror_loss_percent
        pair.plan_mirror(5.7225, .0025, mild, ToyMixer())
        self.assertGreater(pair.mirror_loss_percent, old)

    def test_transient_fallback_retries_without_permanent_lock(self):
        pair, fault, fdi = self.make_pair(84.)
        pair.defer(5.23, "temporary F/Mx/My infeasible")
        self.assertFalse(pair.active)
        self.assertFalse(pair.inhibited)
        self.assertTrue(pair.retry_pending)
        self.assertEqual(pair.retry_count, 1)
        pair.update(5.35, [0., 0., -1.], [0., 0., 0.], 1., True, fault, fdi)
        self.assertFalse(pair.active)
        pair.update(5.55, [0., 0., -1.], [0., 0., 0.], 1., True, fault, fdi)
        self.assertTrue(pair.active)
        self.assertFalse(pair.inhibited)

    def test_geometry_and_envelope_accept_arbitrary_loss(self):
        for loss in (0., 1.0, 11.2, 37.7, 60., 80., 84., 93.):
            for motor_id in range(1, 5):
                with self.subTest(loss=loss, motor=motor_id):
                    opp = opposite_motor(motor_id)
                    self.assertIn(opp, (1, 2, 3, 4))
                    ok = paired_primary_feasibility(
                        ToyMixer(), [8., 0., 0.], motor_id, loss, loss
                    )
                    self.assertTrue(ok["feasible"])
        for motor_id in range(1, 5):
            self.assertFalse(paired_primary_feasibility(
                ToyMixer(), [8., 0., 0.], motor_id, 100., 100.
            )["feasible"])

    def test_position_priority_nullspace_with_partial_mirror(self):
        motor = ToyMixer()
        for opposite_loss in (0., 7., 20., 40., 62.):
            w, diag = allocate_pair_yaw_control(
                motor, [8., 0., 0.], 1, 62., .2,
                mirror_loss_percent=opposite_loss
            )
            f = np.array([motor.motor.thrust(x) for x in w])
            f *= np.array([.38, 1. - opposite_loss/100, 1., 1.])
            b = np.vstack((np.ones(4),
                           .14*np.array([-1., 1., 1., -1.]),
                           .14*np.array([1., -1., 1., -1.])))
            np.testing.assert_allclose(b@f, [8., 0., 0.], atol=1e-5)
            self.assertTrue(diag["primary_wrench_feasible"])


if __name__ == "__main__":
    unittest.main()
