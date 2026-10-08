# Mode29 observer-gated opposite-motor matching experiment

**Research prototype / not flight validated.** This does not change the two frozen 2026-10-06 milestone refs.

- Pixhawk firmware working branch: `feature/mode29-pixhawk-next-from-20261006`
- Paired Orange Pi working branch: `feature/orangepi-mode29-next-from-20261006`
- Common original baseline: `2c606d7e532c54416fc850995449b9acfd6f6a1b`
- No 2026-10-07 `M29_YAW_KD`, `M29_YAW_RMAX`, or `M29_YAW_MMAX` logic has been imported.

## Hypothesis, not a proven flight result

If motor M1 (or M3) loses an inferred fraction of thrust, apply the **same fractional thrust effectiveness reduction** to its opposite motor M2 (or M4). This equalizes the diagonal pairs' *modelled effectiveness*, not necessarily their instantaneous absolute thrust values. The other two motors retain full effectiveness. A single pair has one spin direction, the other pair has the opposite, so an unequal reaction moment should allow free yaw acceleration. The existing Mode29 geometric position loop still requests NED `(0, 0, -M29_TKOFF_ALT)`, which is `(0,0,-1)` when configured altitude is 1 m.

**This is NOT a guaranteed position-hold or yaw-stable controller.** Reaction torque, gyroscopic coupling, EKF/mocap performance under fast spin, saturation and motor dynamics have not been proven safe. The baseline's geometric controller and yaw-free allocator are deliberately retained; no October 7 reduced-attitude/yaw-damping controller is reused.

## Activation and estimator design

1. Orange Pi sends the ordinary single-motor RC9..RC12 motor derating command. FDI does not read the injected loss/motor ID to *estimate* the failure.
2. The existing matched-moment L1 observer and four single-motor signatures estimate fault motor and fraction. Confirmation currently requires: loss >= 60%, relative signature residual <= 0.35, 24 consistent cycles (nominal 60 ms at 400 Hz), and the existing roll/pitch excitation gate.
3. With `M29_PAIR_EN=1` **at Mode29 entry**, and only for an actively injected fault, reject if FDI motor does not match the injection motor (safety supervision, NOT an estimator input). Reject inferred or injected loss > 70%; a full paired outage would leave too few independent actuators to meet F/Roll/Pitch.
4. Use the same thrust-domain softdrone static model to reduce the opposite motor by the FDI-inferred percentage. The original injection retains its **commanded** percentage; any mismatch is visible in the logs.
5. The yaw-free allocator solves `[F,Mx,My] = B*diag(eta)*f_command`, using separate effectiveness entries for *both* the detected and mirrored motor. The ordinary position loop and gain scheduler are unchanged.
6. Freeze the single-fault FDI identifier/severity while paired, because a two-motor residual no longer matches its one-motor signature model. This means automatic recovery of actual failed-motor effectiveness cannot be identified *during* mirroring; explicitly ending the injection releases mirroring and re-anchors the detector.
7. Permanently inhibit pairing for the remainder of this Mode29 run if the static thrust margin check fails, an injected motor/FDI mismatch occurs, estimated loss exceeds the envelope, the injection ends, or the active pair exceeds yaw-rate/position limits. On disengagement the synthetic second impairment is removed. The original latch keeps yaw released rather than abruptly commanding a fixed heading.

### Guard thresholds (source constants; not proof of safety)

- `M29_PAIR_EN`: 0 by default, 1 only in explicit experiment
- Allowed both injected and estimated loss at engagement: 60–70% (FDI's existing confirmation threshold is 60%). Additionally, absolute FDI-versus-injected loss bias must be <=8 percentage points; injected truth is consulted only as a safety gate, never as the estimate.
- Conservative static model margin: >=1.5, using measured-range thrust at nominal `w=80` (1800us) and max configured tilt
- Abort mirror if `abs(body_yaw_rate) > 4.0 rad/s`, horizontal position error >0.50m, or altitude error >0.40m, or AHRS position/rate is invalid
- Mode29 is constrained to its original takeoff+settle hover-gating conditions
- Arming with `M29_PAIR_EN=0` has the original one-motor behavior; the experiment flag is snapshotted on mode entry.

**Important:** With estimated loss 50% after an injected 60%, the existing FDI's 60% confirmation gate prevents pair activation. The prototype does not silently trust the oracle to bypass that problem. Check logs rather than lowering threshold in real flight.

## Observability (DataFlash BIN)

- `L1DG`: actual *software-commanded* injection motor, loss percentage, previous nominal commands and applied commands
- `L1FD`: FDI candidate/confirmed motor, estimated loss fraction (%), normalized signature-fit residual, observer matched moment
- `L1GS`: gain schedule mode, raw/filtered loss and active Kp/Kv
- `L1GA`: active KR, KO and tilt
- `L1PR`: `ena,act,lock,fail,opp,loss,margin,rate,xy,z,bias`
  - `loss`: frozen observer-selected mirror loss in %
  - `bias`: live `L1FD.loss - L1DG.loss` in percentage points (diagnostic *only*); while the pair is active the FDI value is intentionally frozen
  - `margin`: static model ratio, not an actual control-authority measurement
  - `rate`: measured body yaw rate in rad/s; sign indicates spin direction
  - `xyerr`, `zerr`: NED-origin hover error in meters
  - `lock=1`: disengaged/blocked for the rest of the Mode29 run

An injected loss percentage is a model target, not an independent force sensor measurement. Compare against motor test-stand calibration before claiming physical estimation accuracy.

## Next validation required before any propeller-on test

1. Confirm the CI build passes and the paired math unit tests pass (the unit tests are **not** a physical/SITL closed-loop test).
2. On a new SITL test or actuator-in-the-loop bench rig, use an independent kill/abort and verify M1<->M2 and M3<->M4 physical mapping.
3. Record `L1DG,L1FD,L1GS,L1PR`. Ensure a false FDI match does not engage the pair, and verify model consistency without propellers.
4. Simulate motor model errors, PWM saturation, external navigation glitches, and the yaw-rate guard; evaluate XY/Z overshoot and vertical thrust margin.
5. Re-evaluate the yaw-free geometric controller for high-spin tracking and actual moment dynamics. **Do not infer a guarantee of hover from symmetric thrust pairing alone.**

Do not deploy/test this new feature in free flight until these validation steps succeed. For older proven conditions keep `M29_PAIR_EN=0` and use the preserved 2026-10-06 milestone.
