# Mode29 Blind PWM-Ceiling Fault Estimation — UNVALIDATED HIL PROTOTYPE

Branch: `feature/mode29-blind-pwmcap-fdi-hil-20261009`, derived from
`d1daf8e`. This branch is **NOT FLIGHT-QUALIFIED**. Use only unpowered
simulation, hardware-in-the-loop with motors disabled, or offboard replay.

## Design objective and information firewall

A user/Orange-Pi injection defines an **unknown** actuator output
`w_actual=min(w_controller,w_before_injection*(1-loss))`.
The normalized `w=(PWM_us-1000)/10` convention is used throughout.

The flight controller's autonomous **FDI and control allocation** may
use *only* previous nominal **pre-injector** motor commands and measured
L1 matched-moment observer residuals, not the source of the impairment.

### Allowed
- Mode29's control request and four-motor nominal *pre-injector* PWM.
- Gyro/IMU and L1 observer moment residuals and normal motor geometry.
- Pre-existing 6S static PWM→force / reaction moment model.
- Previous healthy controller-command reference; no captured injector
  `motor_pwm_cap_baseline`.
- Purely independently estimated `motor_blind_confirmed_motor`,
  `motor_blind_cap_est_pwm`, and confidence.

### Strictly prohibited in FDI/allocator/opposite-pair
- `motor_degradation_motor_id` and `motor_degradation_loss_pct`.
- `motor_bounded_injected_motor_id` or injected loss percentage.
- `motor_pwm_cap_baseline`, `motor_pwm_last_sent`, or `L1PC` truth.
- Post-injector motor output `motorPWM` or `L1DG.a*`.
- State flags that encode injection onset/release.

The software injector ONLY applies the true cap at the **final actuator
stage**, after the controller's independent allocation, then records
`L1PC` for **offline grading only**.

## Nonlinear inverse model

For each nominal controller command `w_i`, candidate faulted motor
`i`, and unknown cap `c∈[0,w_i]`, predict missing thrust and moment:

```
DeltaF = F_model(w_i) - F_model(c)
DeltaM = M_model(w_i) - M_model(c)
h_i(c) = [
  sign_x[i] * (L/2) * DeltaF,
  sign_y[i] * (D/2) * DeltaF,
  sign_z[i] * DeltaM
]
```

Evaluate four hypotheses and 25 static candidate caps/motor.
Minimize `||L1_filtered_moment_residual - h_i(c)||` normalized by
the total observed moment norm. Require significant Roll/Pitch residual,
plausible thrust loss, residual fit <0.30 and persistent/stable motor
identity and absolute cap for >=40 control cycles (~100ms).

The pre-fault normalized PWM reference used for reporting a loss percentage
(and setting an **optional** diagonal mirror cap) is estimated from a slow
healthy command reference, frozen at candidate onset and confirmation.
It is not read from the injector.

Identifiability caveat: if `w_cmd<=cap` then `DeltaF≈DeltaM≈0`;
the cap is unobservable in that state. Below the motor deadzone,
different actual cap values also look identical. A confirmed estimate is
therefore *model inferred*, not guaranteed measured. This branch does not
perform extra rotor probing and does not pretend to know full recovery.

## Control and recovery

Before independent FDI confirmation, the allocator assumes four normal
actuators and therefore does NOT know the injected limit. Sensor residuals
can reveal lost authority, though severe faults may become uncontrollable
before the estimator becomes confident.

After confirmation, `mode29_bounded_allocate()` uses only the inferred
primary PWM cap. A separate plant/injector clamp still uses real injected
truth, after the controller has finished.

Optional `M29_PAIR_EN=1` *can* voluntarily decrease the opposite
motor's command range after independent identification, if feasibility
and yaw braking guards allow. It uses the estimated primary cap and the
opposite motor's *healthy controller PWM estimate*, not the injected
loss. This is not a physical second motor failure; it is a constrained
control choice. The initial robust baseline recommendation is
`M29_PAIR_EN=0` in simulation until primary blind estimation works.

When the external injector is turned off, no detector reset, new motor ID
or normal-ability declaration is passed to control. **Known release is
not blind proof of recovery.** Confirmed cap remains conservatively held
until Mode29 is exited/disarmed. An eventual autonomous recovery detector
requires independent evidence of restored authority (e.g. safe, validated
active excitation or independent RPM/thrust telemetry).

`M29_GS_MODE=1` (injected-oracle calibration scheduler) is DISABLED
for the HIL bounded branch; mode=2 uses only blind confirmed PWM-cap
loss normalized to a healthy pre-fault command reference. Recommended
initial isolated test: `M29_GS_MODE=0`.

## Diagnostics

- New `L1BF`: `id,cap,pre,pct,fit,conf,streak`. **Controller blind estimate**;
  `id=0` means not confirmed. `cap` is normalized PWM, `pre`
  is healthy command reference, `pct` is estimated PWM headroom loss.
- Existing `L1FD.state/motor/cand/loss`: independent, new detector.
- `L1DG.c1..c4`: controller's independent nominal commands; `a*`:
  post-injector commands actually sent to PWM output.
- `L1PC.valid/motor/base/cap/opp/bopp/copp/clip`:
  **injected truth, logged only for retrospective evaluation**.
- `RCOU`: actual FC PWM output record (still not ESC RPM/actual thrust).
- `L1BA.sat/Fpred/Er/Ep`: allocator feasibility under estimates.

### HIL-only falsification protocol (mandatory)

1. Test no-fault hover and yaw/roll/pitch perturbations first; require
   zero false confirmations and sensible L1 residual baseline.
2. Test all four actuator locations at pre-injection PWM values 25..60
   with cap fractions 0%, 20%, 40%, 60%, 80%, 90%, 100%.
3. Withhold all injection labels from detector and controller; log truth
   to a separate channel, examine delay, identification accuracy, cap MAE,
   confidence and false-positive rate AFTER the run.
4. Check estimates based on model residuals across actual L1 filter and
   motor lag; ideal synthetic motor signatures alone are INSUFFICIENT.
5. Measure FDI confirmation time, altitude and position tracking, yaw
   braking authority, primary wrench saturation, and motor PWM ranges.
6. Test switching injector off while a blind cap is latched: controller
   must not magically know real authority has returned.
7. Test missing data, drift, false M2/M3/M4 candidates and real RPM
   mismatch. Any dangerous false cap must block use on real hardware.
8. Only consider subsequent powered trials after independent physical
   testing and supervisory review. No propeller-on test authorized
   by compiler success.

Run72 on the older oracle-assisted controller showed **0/7 confirmed FDI**
and therefore cannot validate autonomous compensation in this new
closed-loop architecture. This code is a hypothesis needing real HIL data.
