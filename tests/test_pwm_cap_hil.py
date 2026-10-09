#!/usr/bin/env python3
"""HIL-only fixed pre-fault PWM-ceiling source and numerical contracts.

No autopilot closed-loop behavior is simulated or flight safety certified.
"""
import math
import pathlib
import unittest
ROOT=pathlib.Path(__file__).resolve().parents[1]
CODE=(ROOT/"L1AC_customization/ArduCopter/mode_adaptive.cpp").read_text()
STATE=(ROOT/"L1AC_customization/ArduCopter/mode.h").read_text()

def force(w):
    x=max(0.0,float(w)-4.47703190)
    return max(0.0,((-2.62683159e-5*x+4.01680390e-3)*x+4.05756758e-8)*x)

def cap(baseline,loss,requested):
    assert 0<=baseline<=100 and 0<=loss<=100
    return min(max(0.0,requested),baseline*(1.0-loss/100.0))

class PwmCapHILTests(unittest.TestCase):
    def test_pre_fault_sample_is_latched_once(self):
        self.assertIn("motor_pwm_cap_baseline=motor_pwm_last_sent;",CODE)
        self.assertIn("motor_pwm_last_sent=motorPWM;",CODE)
        self.assertIn("if (!motor_pwm_cap_latched)",CODE)
        self.assertIn("motor_pwm_last_sent_valid",STATE)

    def test_allocator_and_actuator_both_use_pwm_caps(self):
        self.assertIn("softdrone_thrust_from_w(pwm_caps[i])",CODE)
        self.assertIn("softdrone_w_from_thrust(f)",CODE)
        self.assertIn("motorPWM[i]=MIN(motorPWM[i],injected_cap_pwm[i]);",CODE)
        self.assertIn("motor_pwm_cap_baseline[idx]*",CODE)
        self.assertIn('AP::logger().Write("L1PC"',CODE)
        self.assertNotIn("eta[i]*softdrone_thrust_from_w(w[i])",CODE)

    def test_80pct_cap_of_40_is_8_not_20pct_of_thrust(self):
        before=40.0
        outputs=[cap(before,80,w) for w in [40,50,60,80,100,100]]
        self.assertEqual(outputs,[8.0]*len(outputs))
        self.assertAlmostEqual(force(outputs[-1]),force(8))
        self.assertGreater(abs(force(8)-0.2*force(40)),.01)

    def test_all_four_motor_indices_and_full_loss_range(self):
        for idx in range(4):
            for loss in (0,.1,10,40,60,70,80,90,99.9,100):
                for baseline in (0,6,15,40,60,80,100):
                    for req in (0,3,40,100):
                        cmd=[100.0]*4
                        cmd[idx]=cap(baseline,loss,req)
                        self.assertGreaterEqual(cmd[idx],0)
                        self.assertLessEqual(cmd[idx],
                                             baseline*(1-loss/100)+1e-6)
        self.assertEqual(cap(40,100,99),0)
        self.assertEqual(cap(40,0,99),40)

    def test_pair_uses_frozen_opposite_baseline_during_ramp(self):
        primary_baseline,opposite_baseline=40.0,42.0
        self.assertEqual(cap(primary_baseline,80,100),8.0)
        previous=float("inf")
        for elapsed in range(0,1200,50):
            mirror_loss=min(80.0,70.0*elapsed/1000.0)
            mirror=cap(opposite_baseline,mirror_loss,100)
            self.assertLessEqual(mirror,previous+1e-5)
            previous=mirror
        self.assertAlmostEqual(cap(opposite_baseline,80,100),8.4)

    def test_release_expands_pwm_limit_not_thrust_effectiveness(self):
        vals=[cap(40,l,100) for l in (80,70,50,30,10,0)]
        for got,want in zip(vals,(8,12,20,28,36,40)):
            self.assertAlmostEqual(got,want)
        self.assertEqual(sorted(vals),vals)

    def test_legacy_thrust_loss_does_not_apply_in_balloc_branch(self):
        fragment=CODE.split("if (motor_bounded_enabled_this_run) {\n        // Independent post-allocation ceiling enforcement",1)[1]
        self.assertIn("} else if (motor_degradation_active)",fragment[:900])
        self.assertIn("apply_modelled_motor_loss(",fragment)

    def test_oracle_assistance_is_explicit_not_blind_fdi(self):
        self.assertIn("HIL ORACLE-ASSISTED",CODE)
        self.assertIn("HIL oracle-assisted allocation",CODE)
        self.assertIn("motor_pair_target_loss_pct=target",CODE)
        self.assertIn("motor_pwm_cap_latched",CODE)

if __name__=="__main__":
    unittest.main()
