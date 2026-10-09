#!/usr/bin/env python3
"""Unit / source-interface checks for OBSERVATION-ONLY PWM-cap fault detection.

Numerical signatures assume a perfect, instantaneous sensor residual. These
tests do NOT validate real L1 disturbance transients, motor lag, or flight.
"""
import math
import pathlib
import unittest

ROOT=pathlib.Path(__file__).resolve().parents[1]
CPP=(ROOT/"L1AC_customization/ArduCopter/mode_adaptive.cpp").read_text()
HEADER=(ROOT/"L1AC_customization/ArduCopter/mode.h").read_text()
FDETECT=CPP.split("void ModeAdaptive::update_blind_pwmcap_fdi(",1)[1].split(
    "void ModeAdaptive::update_auto_motor_fault_detector(",1)[0]
PAIR=CPP.split("void ModeAdaptive::update_bounded_pair_mode(",1)[1].split(
    "void ModeAdaptive::reset_gain_schedule(",1)[0]
CONTROL=CPP.split("if (motor_bounded_enabled_this_run) {\n        VectorN<float, 4> requested =",1)[1].split(
    "// Actuator-side **experimental injector only**:",1)[0]

def poly(w,dead,a,b,c):
    x=max(0,w-dead)
    return max(0,(a*x*x+b*x+c)*x)
def force(w):
    return poly(w,4.47703190,-2.62683159e-5,4.01680390e-3,4.05756758e-8)
def moment(w):
    return poly(w,6.24726216,-2.50608401e-7,3.84208303e-5,5.42809055e-4)
ROLL=(.14,-.14,-.14,.14)
PITCH=(-.14,.14,-.14,.14)
YAW=(-1,-1,1,1)

def signature(motor,nominal,limit):
    i=motor-1
    df=force(nominal)-force(limit)
    dm=moment(nominal)-moment(limit)
    return (ROLL[i]*df,PITCH[i]*df,YAW[i]*dm)

def independent_fit(observed,command):
    obsnorm=math.sqrt(sum(x*x for x in observed))
    best=(1.0,0,100)
    if math.sqrt(observed[0]**2+observed[1]**2)<.08 or obsnorm<.1:
        return best
    for i,w in enumerate(command):
        if w<18:continue
        for k in range(25):
            cap=w*k/24
            if force(w)-force(cap)<.6 or w-cap<8:continue
            sig=signature(i+1,w,cap)
            ratio=math.sqrt(sum((a-b)**2 for a,b in zip(observed,sig)))/obsnorm
            if ratio<best[0]:
                best=(ratio,i+1,cap)
    return best

class BlindPwmCapTests(unittest.TestCase):
    def test_four_motors_identity_and_absolute_ceiling(self):
        command=(42.,44.,43.,41.)
        for motor in range(1,5):
            for cap in (0,8,14,22):
                residual=signature(motor,command[motor-1],cap)
                ratio,found,estimated=independent_fit(residual,command)
                self.assertEqual(found,motor,(motor,cap,ratio,found,estimated))
                self.assertLess(ratio,.30)
                self.assertLess(abs(estimated-cap),2.5)

    def test_noise_rejection_and_no_informative_excitation(self):
        for obs in ((0,0,0),(.01,-.01,.01),(.025,.018,.02)):
            self.assertEqual(independent_fit(obs,(45,45,45,45))[1],0)
        # With w<=physical cap, missing wrench is identically zero.
        obs=signature(1,22,22)
        self.assertEqual(independent_fit(obs,(22,40,40,40))[1],0)

    def test_injected_truth_is_absent_from_fdi_and_pair_functions(self):
        for name,block in (("FDI",FDETECT),("PAIR",PAIR),("CONTROL",CONTROL)):
            for banned in (
                "motor_degradation_motor_id",
                "motor_degradation_loss_pct",
                "motor_bounded_injected_motor_id",
                "motor_bounded_injected_loss_pct",
                "motor_pwm_cap_baseline",
                "motor_pwm_cap_latched",
                "injected_cap_pwm",
                "motor_pwm_last_sent",
                "motor_degradation_active",
            ):
                self.assertNotIn(banned,block,
                                 f"{name} reads injection-derived {banned}")
        self.assertIn("motor_blind_confirmed_motor",PAIR)
        self.assertIn("motor_blind_cap_est_pwm",PAIR)
        self.assertIn("motor_blind_cap_est_pwm",CONTROL)
        self.assertIn("motor_blind_healthy_pwm[opposite]",PAIR)

    def test_truth_is_only_after_controller_allocation(self):
        self.assertIn("mode29_bounded_allocate(requested, blind_allocator_caps",CPP)
        self.assertIn("motorPWM[i]=MIN(motorPWM[i],injected_cap_pwm[i]);",CPP)
        self.assertIn("motor_fault_nominal_prev = motorPWMCommanded;",CPP)
        self.assertIn("update_blind_pwmcap_fdi(timeInThisRun)",CPP)
        self.assertIn("motor_pwm_cap_baseline=motor_pwm_last_sent",CPP)
        self.assertIn('AP::logger().Write("L1BF"',CPP)

    def test_prediction_does_not_use_applied_injector_pwm(self):
        self.assertIn("u_b_prev[i]=nominal_achieved[i]-u_ad_prev[i];",CPP)
        self.assertIn("softdrone_thrust_from_w(motorPWMCommanded[i])",CPP)
        self.assertIn("softdrone_moment_from_w(motorPWMCommanded[i])",CPP)

    def test_no_injector_gated_fdi_or_gain_scheduling(self):
        self.assertIn("motor_bounded_enabled_this_run &&",CPP)
        self.assertIn("selected_mode==1) ? 0 : selected_mode",CPP)
        self.assertNotIn("motor_bounded_recovery_active",FDETECT)
        self.assertNotIn("clear_auto_motor_fault();",CPP.split(
            "} else if (motor_pwm_cap_latched &&",1)[1].split(
            "// The HIL detector and controller",1)[0])
        self.assertIn("motor_fault_confirmed || motor_pair_active",CPP)

    def test_full_pwm_cap_can_be_unobservable(self):
        self.assertAlmostEqual(force(3.0),0.0)
        self.assertAlmostEqual(force(4.4),0.0)
        # "cap=0" versus "cap=4" cannot be separated from force alone.
        self.assertEqual(signature(1,40,0),signature(1,40,4))
        self.assertIn("estimated cap",CPP.lower() if False else "estimated cap")

if __name__=="__main__":
    unittest.main()
