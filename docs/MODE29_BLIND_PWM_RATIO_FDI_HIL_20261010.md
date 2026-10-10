# Mode29 Blind PWM-Ratio Fault Identification & Constrained Allocation
**Experimental / HIL ONLY — NOT FLIGHT QUALIFIED.**

- Baseline **exact** Git commit: `f3f12d901e82c779cd842246556091ba21492838`
- Experiment branch: `feature/mode29-blind-pwm-ratio-fdi-hil-20261010`
- Builds target Pixhawk6C but **firmware compilation is not an approval to use powered propellers**.
- Only `M29_BALLOC=1` selects this new experiment. Legacy `M29_BALLOC=0` remains on the older thrust-effectiveness fault definition.

## Two distinct fault models

**Old Run71 (thrust loss)**:
`F_applied = (1-loss)*F_model(w_nominal)`.

**New HIL (normalized PWM ratio loss)**:
`w_applied = (1-loss)*w_nominal`, using `w=(PWM_us-1000)/10` and `0<=w<=100`.
Corresponding physical force:
`F_applied=F_model((1-loss)*w_nominal)`.
`loss=80%`, `w_nominal=100` -> `w_applied=20` -> 1200 us, **not 80% thrust loss**.
This is also **NOT a fixed ceiling**: `w_applied` scales with `w_nominal`.
At low PWM values it can correspond to almost complete thrust loss.

The RC9-12 Orange Pi command format is unchanged. For `M29_BALLOC=1`, RC11 now means *normalized PWM ratio* loss. 80% PWM loss in this motor model is not comparable to 80% thrust loss in Run71.

## Strict information firewall

**Allowed to independent FDI and allocator**:
- `motor_fault_nominal_prev`: pre-injector motor commands from prior cycle
- L1 matched-moment observer (`sigma_m_hat_prev`) and IMU/gyro dynamics
- nominal calibrated `F_model(w)`, `M_model(w)`, quad geometry
- FDI-estimated motor ID, PWM-ratio severity, residual fit and confidence

**Forbidden as inputs to FDI or allocator**:
- `motor_degradation_motor_id/loss_pct`, injected cap or PWM, measured post-injector `motorPWM`
- `motor_bounded_injected_motor_id/loss_pct`, known fault onset or release
- pairing/recovery oracle checks from the old HIL experiment

Those truth values remain inside the laboratory software injector and offline `L1DG`/log evaluation. Controller **does not infer fault release from the injector**. FDI identity remains latched until Mode29 exit; PWM severity can continue updating on valid measured residuals.

## Model and algorithm

For candidate motor `i` and PWM loss fraction `l`:

```
deltaF_i(l,w) = F(w) - F((1-l)w)
deltaM_i(l,w) = M(w) - M((1-l)w)
h_i = [-roll_arm_i*deltaF, -pitch_arm_i*deltaF, -yaw_sign_i*deltaM]
```

In `update_auto_motor_fault_detector()` with `M29_BALLOC=1`, scan four candidate motors and 21 PWM loss grid points (0%, 5%, ... 100%).
Each hypothesis is **low-pass filtered** with alpha=0.05 (400Hz ~49ms) to match the logged L1 moment innovation filter.
A thresholded onset starts the independent hypothesis states; motor ID requires a good residual fit, separation from other IDs, and 24 consistent samples (~60ms of persistence after a plausible fit, **not a promise of 60ms detection**). Fault severity is retained and updated continuously after confirmation when excitation and fit permit.

Important limitations: the hypothesis filter does not model complete ESC/rotor lag or nonlinear aerodynamic changes, and starts at a detected onset rather than the true onset. Static thrust/torque fits were originally calibrated for a specific 6S motor/prop/ESC setup, partially extrapolated above normalized `w=80`. Noise, gyro drift, saturation, externally induced torques and delay mismatch can create bias. The `L1PW.conf` value is a **heuristic fit score, not a statistically calibrated probability**. No motor-specific force, RPM, or actual aircraft thrust is directly measured.

## Fault-tolerant allocation

HIL allocator receives only independent FDI estimate `l_hat`, with `eta_pwm=1-l_hat`:
- physical maximum force: `F_model(100*eta_pwm)`;
- inverse command for requested force `f`: `w_nom=F_model_inverse(f)/eta_pwm` (zero handled separately);
- actual predicted motor force: `F_model(eta_pwm*w_nom)`;
- yaw moment evaluated at the **physical** PWM, after collective/roll/pitch feasible allocation.

The L1 predictor uses the attainable **pre-injector** wrench computed from actual nominal commands, avoiding allocation saturation being incorrectly absorbed into the fault residual.
Control allocation uses `l_hat` only *after motor ID confirmation*; before then it assumes healthy actuators.

## Deliberate features excluded to prevent accidental oracle dependence

- `M29_PAIR_EN=1` with `M29_BALLOC=1`: Mode29 entry rejected. Do not inject a second synthetic fault until the single-fault estimator is independently validated.
- `M29_GS_MODE` gain scheduling is **forced to baseline** for HIL: old anchors describe thrust loss and mode 1 reads injector truth. `L1GS.mode=0` indicates effective baseline.
- The known injector release may ramp the **plant fault** smoothly, but never directly resets FDI or removes the conservative fault authority constraints.
- Yaw-free behavior, primary allocation and navigation containment do not use injected-fault identity, onset or release.

## Log channels
- `L1DG.c1..c4`: controller nominal **pre-injector** normalized PWM.
- `L1DG.a1..a4`: command after plant-side fault injection (not ESC RPM or physical thrust).
- `L1FD`: FDI candidate/confirmed state, ID, estimated *PWM loss percent*, L1 residual fit, estimated roll/pitch/yaw disturbance.
- `L1FI`: gating and per-tick severity innovation (experimental).
- `L1PW`: `id,pct,fit,conf,w,gate,sat`. Only independent FDI estimates.
- `L1BA`: allocator saturation, requested/predicted thrust and RP errors.
- `L1PR`: pairing experimental logging (pair disabled in this mode).
- `L1GS`: effective baseline gain mode for HIL.
- The injected truth in logs is **for offline grading only**.

## Required validation before any powered experiment

1. Run `python3 -m unittest discover -s tests -p 'test_mode29_pwm_ratio_hil.py' -v`.
2. Compile the Pixhawk6C target via the new isolated GitHub Actions workflow.
3. Unpowered/no-prop SITL/HIL: inject no-fault sequences, all four IDs and 0/10/20/30/40/50/60/80/100 PWM loss at multiple healthy PWM operating points.
4. Repeat when nominal commands change quickly; quantify residual model lag, estimated severity bias, all 4 IDs, spurious detections and delays without ever giving FDI injection truth.
5. On actual HIL replay, identify independent `t_inject` only from separate plant log *after* the run; report detection latency, ID confusion matrix, severity MAE, false positives, wrench saturation, yaw-braking and position error.
6. Test injector release; estimate must not magically reset from the injected command alone. Validate yaw-free emergency behavior, NaN guards and watchdog timeouts.
7. Only after independent simulation, actuator bench calibration, measurement latency analysis and safety review can an actual propeller-on experiment even be considered.

**This is source-level algorithm integration, not a tested flight-capable firmware**.
