#!/usr/bin/env python3
"""Offline HIL-only FDI gating regression.

These tests check source integration and conservative numerical
anti-windup behavior; they are NOT an ArduPilot closed-loop or flight test.
"""
import math
import pathlib
import unittest

ROOT = pathlib.Path(__file__).resolve().parents[1]
CODE = (ROOT / "L1AC_customization/ArduCopter/mode_adaptive.cpp").read_text()
STATE = (ROOT / "L1AC_customization/ArduCopter/mode.h").read_text()


def replay_confirmed_fdi(initial, innovations, pair=False):
    """Minimal independent discrete approximation of the firmware gates.

    Each innovation is (time_ms, residual_fraction, fit_ratio, cmd_w,
    primary_sat). Implements wait and 30pp/s upward, 100pp/s downward.
    """
    value = initial
    accepted = 0
    prev_sign = 0
    consecutive = 0
    for elapsed, residual, fit, cmd, saturated in innovations:
        if pair or elapsed < 125 or saturated or cmd < 16 or fit > .35:
            continue
        if not math.isfinite(residual) or abs(residual) > 1:
            continue
        if abs(residual) < .03:
            consecutive = 0
            prev_sign = 0
            continue
        sign = 1 if residual > 0 else -1
        consecutive = consecutive + 1 if sign == prev_sign else 1
        prev_sign = sign
        if consecutive < 6:
            continue
        dv = min(.075, max(-.25, residual))  # 1.00 * raw fraction
        value = min(100, max(0, value + dv))
        accepted += 1
    return value, accepted


class FdiAntiWindupTests(unittest.TestCase):
    def test_source_is_hil_gated(self):
        self.assertIn("if (motor_bounded_enabled_this_run) {", CODE)
        self.assertIn("OBSERVER_SETTLE_MS=125U", CODE)
        self.assertIn("MAX_RISE_PCT_PER_CYCLE=0.075f", CODE)
        self.assertIn("MAX_FALL_PCT_PER_CYCLE=0.25f", CODE)
        self.assertIn('AP::logger().Write("L1FI"', CODE)
        self.assertIn("motor_fdi_confirmed_at_ms", STATE)
        self.assertIn("motor_fdi_consistent_samples", STATE)
        self.assertIn("} // legacy FDI path", CODE)
        self.assertIn(
            "motor_bounded_enabled_this_run ? motor_fault_confirmed :", CODE
        )

    def test_log67_delayed_residual_no_immediate_overestimate(self):
        # The old integrator 0.05 * 0.7 * 100 per 400-Hz step would
        # elevate 69% above 90% in 6 samples, reaching 100% in 9 samples.
        samples = [(k * 2.5, .7, .1, 45, False) for k in range(36)]
        new_value, updates = replay_confirmed_fdi(69, samples)
        self.assertEqual(new_value, 69)
        self.assertEqual(updates, 0)

    def test_real_complete_failure_can_still_reach_100(self):
        samples = [(k * 2.5, .8, .1, 55, False) for k in range(650)]
        value, accepted = replay_confirmed_fdi(70, samples)
        self.assertEqual(value, 100)
        self.assertGreater(accepted, 0)

    def test_reject_sat_bad_fit_and_low_excitation(self):
        samples = []
        for k in range(200):
            t = 200 + k*2.5
            samples.append((t, .7, .5, 50, False))  # bad fit
            samples.append((t, .7, .1, 10, False))  # small motor thrust
            samples.append((t, .7, .1, 50, True))   # saturated wrench
        value, updates = replay_confirmed_fdi(70, samples)
        self.assertEqual((value, updates), (70, 0))

    def test_noise_hysteresis_and_faster_negative_recovery(self):
        noisy = [(200+i*2.5, (0.6 if i%2 else -0.6), .1, 50, False)
                 for i in range(100)]
        value, accepted = replay_confirmed_fdi(70, noisy)
        self.assertEqual((value, accepted), (70, 0))
        recovery = [(200+i*2.5, -.7, .1, 45, False)
                    for i in range(140)]
        value, accepted = replay_confirmed_fdi(70, recovery)
        self.assertLess(value, 40)
        self.assertGreater(accepted, 0)


if __name__ == "__main__":
    unittest.main()
