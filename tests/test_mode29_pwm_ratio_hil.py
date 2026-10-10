#!/usr/bin/env python3
"""Unpowered-HIL source contract and independent PWM-ratio model regressions.

These numerical tests are NOT closed-loop validation, not a firmware compile,
and not evidence of propeller-on safety. They never feed injection truth to FDI.
"""
import math
import pathlib
import unittest

ROOT = pathlib.Path(__file__).resolve().parents[1]
SRC = (ROOT / "L1AC_customization/ArduCopter/mode_adaptive.cpp").read_text()
STATE = (ROOT / "L1AC_customization/ArduCopter/mode.h").read_text()
DEAD_F = 4.47703190
DEAD_M = 6.24726216
F = (-2.62683159e-05, 4.01680390e-03, 4.05756758e-08)
M = (-2.50608401e-07, 3.84208303e-05, 5.42809055e-04)
ROLL = (0.14, -0.14, -0.14, 0.14)
PITCH = (-0.14, 0.14, -0.14, 0.14)
YAW = (-1., -1., 1., 1.)


def polynomial(w, dead, coefficients):
    x = max(0., w - dead)
    a, b, c = coefficients
    return max(0., ((a*x+b)*x+c)*x)


def thrust(w):
    return polynomial(w, DEAD_F, F)


def moment(w):
    return polynomial(w, DEAD_M, M)


def missing_signature(motor, commanded, loss):
    applied = commanded * (1. - loss)
    df = thrust(commanded) - thrust(applied)
    dm = moment(commanded) - moment(applied)
    return (ROLL[motor] * df, PITCH[motor] * df, YAW[motor] * dm)


def norm3(v):
    return math.sqrt(sum(x*x for x in v))


def infer_single_sample(innovation, commands):
    """Independent static reference; dynamic delays require separate replay."""
    options = []
    for motor, w in enumerate(commands):
        for k in range(1, 21):
            loss = k / 20.
            sig = missing_signature(motor, w, loss)
            diff = tuple(innovation[j]-sig[j] for j in range(3))
            options.append((norm3(diff), motor, loss))
    return min(options)


class PwmRatioModelTests(unittest.TestCase):
    def test_pwm_mapping_and_endpoints(self):
        for w in (0., 15., 40., 100.):
            for p in (0., .2, .5, .8, 1.):
                applied = w * (1. - p)
                self.assertAlmostEqual(1000 + 10*applied,
                                       1000 + (10*w)*(1.-p))
                self.assertGreaterEqual(applied, 0.)
                self.assertLessEqual(applied, 100.)

    def test_thrust_ratio_is_not_pwm_ratio(self):
        w = 40.
        loss_pwm = .8
        self.assertAlmostEqual(w*(1-loss_pwm), 8.)
        self.assertGreater(1. - thrust(8.)/thrust(w), .95)
        self.assertLess(thrust(8.)/thrust(w), .05)

    def test_forward_inverse_consistency(self):
        for eta in (.05, .2, .5, .8, 1.):
            max_force = thrust(100*eta)
            for fraction in (0., .1, .3, .6, .9, 1.):
                target = max_force*fraction
                lo, hi = 0., 100.*eta
                for _ in range(30):
                    mid=(lo+hi)*.5
                    if thrust(mid)<target:
                        lo=mid
                    else:
                        hi=mid
                cmd = ((lo+hi)*.5)/eta
                self.assertLessEqual(cmd, 100.00001)
                self.assertAlmostEqual(thrust(eta*cmd),target,
                                       delta=max(.0001,max_force*.0001))

    def test_all_motors_and_severities_identifiable_with_valid_excitation(self):
        # Ideal noiseless observation only; not a delay-performance promise.
        cmds=(42.,39.,45.,41.)
        for motor in range(4):
            for loss in (.2, .4, .6, .8, 1.0):
                residual=missing_signature(motor,cmds[motor],loss)
                fit, selected, estimated=infer_single_sample(residual,cmds)
                self.assertEqual(selected,motor)
                self.assertAlmostEqual(estimated,loss,places=6)
                self.assertLess(fit,1e-8)

    def test_no_fault_produces_zero_fault_signature(self):
        for i,w in enumerate((35.,40.,50.,60.)):
            self.assertAlmostEqual(norm3(missing_signature(i,w,0.)),0.)

    def test_source_contract_fdi_has_no_injection_truth(self):
        self.assertIn("MOTOR_PWM_FDI_GRID = 21U", STATE)
        start=SRC.index("EXPERIMENTAL BLIND PWM-RATIO FDI")
        stop=SRC.index("#endif",start)
        fdi=SRC[start:stop]
        for forbidden in ("motor_degradation_motor_id",
                          "motor_degradation_loss_pct",
                          "motor_bounded_injected_motor_id",
                          "motor_bounded_injected_loss_pct",
                          "motor_bounded_recovery_active",
                          "motorPWM[motor_index]"):
            self.assertNotIn(forbidden,fdi)

    def test_source_contract_controller_has_no_truth(self):
        start=SRC.index("Strict firewall: use only independent blind FDI")
        stop=SRC.index("Mode29BoundedDiag allocation;",start)
        allocator=SRC[start:stop]
        for forbidden in ("motor_degradation_motor_id",
                          "motor_degradation_loss_pct",
                          "motor_bounded_injected_motor_id",
                          "motor_bounded_injected_loss_pct",
                          "motor_bounded_recovery_active"):
            self.assertNotIn(forbidden,allocator)
        self.assertIn("pwm_remaining[motor_fault_detected_id-1U]",allocator)
        self.assertIn("M29_PAIR_EN=0",SRC)

    def test_bounded_model_uses_pwm_not_thrust_effectiveness(self):
        self.assertIn("cap[i] = softdrone_thrust_from_w(100.0f * eta[i]);", SRC)
        self.assertIn("softdrone_thrust_from_w(eta[i]*w[i]);", SRC)
        self.assertIn("effectiveness*w_nom,0.0f,100.0f", SRC)
        self.assertIn('AP::logger().Write("L1PW"', SRC)
        self.assertIn("motor_pwm_fdi_started",STATE)


if __name__ == "__main__":
    unittest.main()
