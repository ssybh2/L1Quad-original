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
            for loss in (.2, .4, .6, .8):
                residual=missing_signature(motor,cmds[motor],loss)
                fit, selected, estimated=infer_single_sample(residual,cmds)
                self.assertEqual(selected,motor)
                self.assertAlmostEqual(estimated,loss,places=6)
                self.assertLess(fit,1e-8)

    def test_extreme_loss_cannot_be_identified_below_deadzone(self):
        # Physical observability limit: nominal w=42 and 90% loss yields
        # actual w=4.2, below the thrust and reaction-torque dead zones.
        # 90% and 100% produce IDENTICAL wrench: no estimator can tell.
        a=missing_signature(0,42.,.9)
        b=missing_signature(0,42.,1.)
        self.assertEqual(a,b)

    def test_late_threshold_start_bootstraps_observed_filter(self):
        # A fault starts before the measured (LP-filtered) moment exceeds
        # the FDI onset threshold. The old estimator resets predicted LP
        # candidates to ZERO at threshold crossing, hence overestimates
        # PWM loss to catch up to a partially-risen observation.
        w=40.
        candidates=[missing_signature(0,w,k/20.) for k in range(21)]
        true=candidates[6]  # 30% PWM loss
        obs=[0.,0.,0.]
        started=False
        old=[[0.,0.,0.] for _ in candidates]
        boot=[[0.,0.,0.] for _ in candidates]
        old_best=None
        boot_best=None
        ticks=0
        for t in range(100):
            obs=[.95*y+.05*x for y,x in zip(obs,true)]
            if not started:
                if norm3(obs[:2])<.08 or norm3(obs)<.10:
                    continue
                started=True
                for k,h in enumerate(candidates):
                    projection=sum(a*b for a,b in zip(obs,h))
                    energy=sum(a*a for a in h)
                    frac=min(1.,max(0.,projection/energy)) if energy>1e-8 else 0.
                    boot[k]=[frac*x for x in h]
            for k,h in enumerate(candidates):
                old[k]=[.95*a+.05*b for a,b in zip(old[k],h)]
                boot[k]=[.95*a+.05*b for a,b in zip(boot[k],h)]
            ticks+=1
            if ticks==24:
                old_best=min(range(21),key=lambda k:
                    norm3([a-b for a,b in zip(obs,old[k])]))
                boot_best=min(range(21),key=lambda k:
                    norm3([a-b for a,b in zip(obs,boot[k])]))
                break
        self.assertTrue(started)
        self.assertIsNotNone(boot_best)
        self.assertLess(abs(boot_best-6),abs(old_best-6))

    def test_staged_authority_and_pwm_step_guards(self):
        # 400-Hz rate limits used in firmware. Even when FDI suddenly
        # reports 42%, allocation model cannot jump from 1.0 to 0.58.
        authority=1.0
        target=.58
        seen=[]
        for _ in range(100):
            authority+=min(.0045,max(-.0045,target-authority))
            seen.append(authority)
        self.assertAlmostEqual(seen[0],.9955)
        self.assertGreater(seen[0],.99)
        self.assertAlmostEqual(seen[-1],target)
        for x,y in zip(seen,seen[1:]):
            self.assertLessEqual(abs(x-y),.00450001)
        # 200-us legacy Run76 M1 jump becomes <=8us per 400-Hz tick.
        old_w,new_w=49.5,78.7
        limited=old_w+min(.8,max(-.8,new_w-old_w))
        self.assertLessEqual((limited-old_w)*10.,8.000001)

    def test_source_fdi_onset_uses_measured_only(self):
        start=SRC.index("Run76 replay: threshold crossing")
        end=SRC.index("// Every hypothesis predicts the MISSING",start)
        boot=SRC[start:end]
        self.assertIn("motor_pwm_hypothesis_lp[i][k]=h*initial_fraction",boot)
        self.assertIn("obs*h",boot)
        for forbidden in ("motor_degradation_motor_id",
                          "motor_degradation_loss_pct",
                          "motor_bounded_injected_loss_pct",
                          "motor_bounded_injected_motor_id"):
            self.assertNotIn(forbidden,boot)

    def test_source_allocator_has_slew_and_post_slew_prediction(self):
        self.assertIn("PWM_AUTHORITY_SLEW_PER_TICK=0.0045f",SRC)
        self.assertIn("PWM_MAX_NOMINAL_STEP_W=0.8f",SRC)
        self.assertIn("motor_pwm_last_nominal=motorPWMCommanded",SRC)
        self.assertIn("motor_bounded_predicted_collective_n=achieved_f",SRC)
        self.assertIn("ALLOCATION_SETTLE_MS=150U",SRC)
        self.assertIn('AP::logger().Write("L1TR"',SRC)
        self.assertIn("motor_pwm_alloc_remaining[4]",STATE)

    def test_no_fault_produces_zero_fault_signature(self):
        for i,w in enumerate((35.,40.,50.,60.)):
            self.assertAlmostEqual(norm3(missing_signature(i,w,0.)),0.)

    def test_causal_hypothesis_filters_recover_motor_and_severity(self):
        # Offline idealized 400-Hz observer response, not actual airborne
        # validation. Onset is identified from residual, not injection time.
        commands=(40.,40.,40.,40.)
        for failed in range(4):
            for injected_loss in (.3,.5,.8):
                observed=[0.,0.,0.]
                hypotheses=[[[0.,0.,0.] for _ in range(21)]
                            for _ in range(4)]
                started=False
                confirmed_id=None
                candidate=None
                count=0
                value=0.
                for tick in range(250):
                    # Truth belongs only to independent plant in this test.
                    plant=(missing_signature(failed,40.,injected_loss)
                           if tick>=20 else (0.,0.,0.))
                    observed=[.95*a+.05*b
                              for a,b in zip(observed,plant)]
                    magnitude=norm3(observed)
                    planar=norm3(observed[:2])
                    if not started:
                        if planar<.08 or magnitude<.10:
                            continue
                        started=True
                    fitness=[1000.]*4
                    best_loss=[0.]*4
                    for motor in range(4):
                        for k in range(21):
                            h=missing_signature(motor,40.,k/20.)
                            old=hypotheses[motor][k]
                            new=[.95*a+.05*b for a,b in zip(old,h)]
                            hypotheses[motor][k]=new
                            ratio=norm3([a-b for a,b in zip(observed,new)])
                            ratio/=max(magnitude,.1)
                            if ratio<fitness[motor]:
                                fitness[motor]=ratio
                                best_loss[motor]=5.*k
                    idx=min(range(4),key=lambda i:fitness[i])
                    competitor=min(fitness[i] for i in range(4)
                                   if i!=idx)
                    plausible=(magnitude>=.1 and planar>=.08
                               and fitness[idx]<.36
                               and competitor-fitness[idx]>.06
                               and best_loss[idx]>=10.)
                    if confirmed_id is None:
                        if not plausible:
                            candidate=None
                            count=0
                            continue
                        if candidate!=idx or abs(value-best_loss[idx])>20.:
                            candidate=idx
                            value=best_loss[idx]
                            count=1
                        else:
                            value+=.1*(best_loss[idx]-value)
                            count+=1
                        if count>=24:
                            confirmed_id=idx
                    elif fitness[confirmed_id]<.42:
                        target_loss=best_loss[confirmed_id]
                        value+=max(-.25,min(.25,.05*(target_loss-value)))
                self.assertEqual(confirmed_id,failed)
                self.assertAlmostEqual(value,100.*injected_loss,delta=5.)

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
