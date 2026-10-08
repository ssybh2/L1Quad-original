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



def guarded_timeline(initial, events):
    """Independent time-stepped model for *transition invalidation*.

    Each event: (ms, innovation_fraction, residual_fit, nominal_w,
                 primary_saturated, paired, staged_recovery).
    A timestamp denotes the *last* measurement invalidation, not a
    fixed delay only at first detection.
    """
    value = initial
    last_invalidation_ms = 0.0
    last_w = None
    streak = 0
    prev_sign = 0
    accepted_times = []
    saturation_streak = 0
    for (ms, innovation, fit, nominal_w, saturated, paired, recovering) in events:
        if saturated or paired or recovering:
            last_invalidation_ms = ms
            last_w = None
            prev_sign = 0
            streak = 0
            saturation_streak = saturation_streak + 1 if saturated else 0
            continue
        saturation_streak = 0
        if last_w is not None and abs(nominal_w - last_w) >= 12.0:
            last_invalidation_ms = ms
            streak = 0
            prev_sign = 0
        last_w = nominal_w
        if ms - last_invalidation_ms < 125.0:
            continue
        if not math.isfinite(innovation) or abs(innovation) > 1.0:
            continue
        if nominal_w < 16.0 or not math.isfinite(fit) or fit > 0.35:
            continue
        if abs(innovation) < 0.03:
            streak = 0
            prev_sign = 0
            continue
        sign = 1 if innovation > 0 else -1
        streak = streak + 1 if sign == prev_sign else 1
        prev_sign = sign
        if streak < 6:
            continue
        dv = min(0.075, max(-0.25, innovation))
        value = max(0.0, min(100.0, value + dv))
        accepted_times.append(ms)
    return value, accepted_times


def hil_candidate(loss_fraction, fit_ratio, nominal_w, rp_moment, samples,
                  bounded=True, pair=False):
    """HIL numerical contract for the additional low-severity study."""
    threshold = 0.0 if pair else (0.15 if bounded else 0.60)
    weak = bounded and loss_fraction < 0.60
    moment_floor = 0.02 if pair else (0.10 if bounded else 0.20)
    fit_max = 0.25 if weak else 0.35
    count_min = 40 if weak else 24
    return (loss_fraction > threshold and rp_moment >= moment_floor and
            fit_ratio <= fit_max and (not weak or nominal_w >= 16.0) and
            samples >= count_min)


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
        self.assertIn("motor_fdi_saturation_streak_samples", STATE)
        self.assertIn("motor_fdi_confirmed_loss_pct", STATE)
        self.assertIn("motor_fdi_confirmed_loss_pct = motor_fault_loss_estimate_pct;", CODE)
        self.assertIn("0.5f * motor_fdi_confirmed_loss_pct", CODE)
        self.assertIn("motor_fdi_last_excitation_valid", STATE)
        self.assertIn("motor_fdi_confirmed_at_ms = AP_HAL::millis();", CODE)
        self.assertIn("motor_fdi_saturation_streak_samples >= 200U", CODE)
        self.assertIn("EXCITATION_STEP_W=12.0f", CODE)
        self.assertIn("motor_fdi_gate_code = 12U", CODE)
        self.assertIn('"raw,step,ratio,w,gate,held,stall,est"', CODE)
        self.assertIn('"ffffBIIf"', CODE)
        self.assertIn("weak_hil_candidate ? 40U : MOTOR_FDI_CONFIRM_SAMPLES", CODE)
        self.assertIn("weak_hil_candidate ? 0.25f : MOTOR_FDI_MAX_RESIDUAL_RATIO", CODE)
        self.assertIn("motor_bounded_enabled_this_run ? 0.15f : MOTOR_FDI_MIN_LOSS_FRACTION", CODE)

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

    def test_saturated_estimate_unobservable_until_125ms_after_clear(self):
        events = []
        for i in range(400):
            ms = i * 2.5
            events.append((ms, .8, .1, 50, 150 <= i < 240, False, False))
        _, accepted = guarded_timeline(70, events)
        self.assertTrue(accepted)
        self.assertTrue(all(t >= 240 * 2.5 + 125 for t in accepted
                            if t >= 240 * 2.5))
        self.assertFalse(any(150 * 2.5 <= t < 240 * 2.5 + 125
                             for t in accepted))

    def test_pair_exit_restarts_settling_window(self):
        events = [(i * 2.5, .6, .1, 45, False, 60 <= i < 120, False)
                  for i in range(260)]
        _, accepted = guarded_timeline(68, events)
        self.assertTrue(any(t > 120 * 2.5 + 125 for t in accepted))
        self.assertFalse(any(60 * 2.5 <= t < 120 * 2.5 + 125
                             for t in accepted))

    def test_large_excitation_step_restarts_observer_wait(self):
        events = [(i * 2.5, .7, .1, 50 if i < 160 else 70,
                   False, False, False) for i in range(300)]
        _, accepted = guarded_timeline(65, events)
        self.assertTrue(any(t < 160 * 2.5 for t in accepted))
        self.assertFalse(any(160 * 2.5 <= t < 160 * 2.5 + 125
                             for t in accepted))
        self.assertTrue(any(t >= 160 * 2.5 + 125 for t in accepted))

    def test_low_loss_requires_confidence_and_persistence(self):
        self.assertFalse(hil_candidate(.15, .08, 45, .20, 60))
        self.assertFalse(hil_candidate(.30, .30, 45, .20, 60))
        self.assertFalse(hil_candidate(.30, .10, 14, .20, 60))
        self.assertFalse(hil_candidate(.30, .10, 45, .20, 39))
        self.assertTrue(hil_candidate(.30, .10, 45, .20, 40))
        self.assertFalse(hil_candidate(.30, .10, 45, .20, 40,
                                       bounded=False))
        self.assertTrue(hil_candidate(.70, .28, 45, .20, 24))


    def test_weak_fault_does_not_self_clear_at_35_percent(self):
        # For the 15%-60% HIL study, recovery must use hysteresis relative
        # to the *confirmed* value, rather than a hard-coded 35%-loss cutoff.
        def release_threshold(confirmed):
            return min(35.0, max(5.0, confirmed * 0.5))
        self.assertAlmostEqual(release_threshold(20), 10)
        self.assertAlmostEqual(release_threshold(30), 15)
        self.assertAlmostEqual(release_threshold(50), 25)
        self.assertAlmostEqual(release_threshold(70), 35)
        self.assertGreater(30, release_threshold(30))
        self.assertGreater(20, release_threshold(20))



if __name__ == "__main__":
    unittest.main()
