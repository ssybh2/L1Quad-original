"""Reproduce clipped pseudoinverse thrust inflation across all fault severities."""
import unittest
import numpy as np
from bounded_allocator import bounded_allocate
from mode29_mujoco import Mixer, MotorModel
from pathlib import Path
import tomllib


class BoundedAllocatorTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        p = Path(__file__).resolve().parents[1] / "config_opposite_pair.toml"
        with p.open("rb") as fh:
            cfg = tomllib.load(fh)
        cls.motor = MotorModel(cfg["motor_model"])
        cls.mixer = Mixer(cfg["vehicle"]["L_m"], cfg["vehicle"]["D_m"],
                          cls.motor)
        cls.fmax = cls.motor.thrust(100.0)

    def check_forces(self, request, motor_id, loss_pct):
        w, d = bounded_allocate(self.mixer, request, motor_id, loss_pct)
        eta = np.ones(4)
        eta[motor_id-1] = 1.0-loss_pct/100.
        f = np.array([self.motor.thrust(x) for x in w])*eta
        self.assertTrue(np.isfinite(w).all())
        self.assertTrue((w >= 0.0).all())
        self.assertTrue((w <= 100.0).all())
        self.assertTrue((f >= 0.0).all())
        self.assertTrue((f <= eta*self.fmax + 1e-6).all())
        self.assertAlmostEqual(float(sum(f)), float(d["collective_achievable_n"]),
                               delta=5e-4)
        self.assertAlmostEqual(float(sum(f)), float(d["collective_predicted_n"]),
                               delta=5e-4)
        return d, f

    def test_reproduces_grossly_infeasible_moment_without_thrust_inflation(self):
        # The old algorithm clipped a negative rotor individually after a
        # minimum-norm solve and gave ~24.7 N for an 11.6 N request.
        for faulty in range(1, 5):
            with self.subTest(motor=faulty):
                request = [11.625, -1.9, +1.78, +0.15]
                diag, f = self.check_forces(request, faulty, 90.)
                self.assertTrue(diag["primary_saturated"])
                self.assertLess(abs(diag["collective_error_n"]), 5e-4)
                self.assertAlmostEqual(float(sum(f)), 11.625, delta=5e-4)

    def test_extreme_and_fractional_losses_and_all_motor_indices(self):
        for motor_id in range(1, 5):
            for loss in (0., 0.01, 7.3, 33.5, 60., 70., 80., 90., 98.7, 100.):
                for requested in ([13., 0., 0., .04],
                                  [10.47, -1.90, 1.78, .15],
                                  [15., .2, -.12, -.08],
                                  [100., 4., -5., .2]):
                    with self.subTest(motor=motor_id, loss=loss, request=requested):
                        self.check_forces(requested, motor_id, loss)

    def test_feasible_primary_wrench_preserved(self):
        for loss in (0., 25., 60., 85., 95.):
            diag, f = self.check_forces([10., .0, .0, .0], 1, loss)
            self.assertFalse(diag["primary_saturated"])
            b = np.vstack((
                np.ones(4),
                .14*np.array([-1., 1., 1., -1.]),
                .14*np.array([1., -1., 1., -1.])
            ))
            np.testing.assert_allclose(b @ f, [10., 0., 0.], atol=4e-4)


if __name__ == "__main__":
    unittest.main()
