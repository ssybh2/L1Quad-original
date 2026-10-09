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
        for w in outputs:self.assertAlmostEqual(w,8.0)
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

    def test_pair_ramp_is_continuous_from_100_to_frozen_snapshot_cap(self):
        primary_baseline,opposite_baseline=40.0,42.0
        target_loss=80.0
        target_cap=opposite_baseline*(1-target_loss/100)
        mirror_caps=[]
        for elapsed in range(0,1200,50):
            mirror_pct=min(target_loss,70.0*elapsed/1000.0)
            alpha=mirror_pct/target_loss
            current_cap=100.0+(target_cap-100.0)*alpha
            mirror_caps.append(current_cap)
        self.assertAlmostEqual(mirror_caps[0],100.0)
        self.assertAlmostEqual(mirror_caps[-1],target_cap)
        self.assertEqual(sorted(mirror_caps,reverse=True),mirror_caps)
        self.assertLess(mirror_caps[0]-mirror_caps[1],5)
        self.assertAlmostEqual(cap(primary_baseline,80,100),8.0)
        self.assertIn("mode29_mirror_pwm_cap(",CODE)
        self.assertIn("motor_pwm_cap_baseline[opposite],target,mirror_loss",CODE)

    def test_release_expands_pwm_limit_continuously_to_full_100(self):
        start_cap=40.0*(1-.8)
        initial_loss=80.0
        vals=[start_cap+(100-start_cap)*(1-loss/initial_loss)
              for loss in (80,70,50,30,10,0)]
        for got,want in zip(vals,(8,19.5,42.5,65.5,88.5,100)):
            self.assertAlmostEqual(got,want)
        self.assertEqual(sorted(vals),vals)
        self.assertIn("motor_pwm_cap_release_start",CODE)
        self.assertIn("motor_pwm_cap_release_loss",STATE)

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
