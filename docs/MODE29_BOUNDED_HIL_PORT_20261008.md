# Mode29 Pixhawk6C HIL-only port: bounded allocation, progressive pairing and yaw recovery

Branch: `feature/mode29-bounded-hil-20261008`. Separate from all frozen
2026-10-06 flight firmware. **Code compilation does not constitute
flight-safety validation. NOT APPROVED for propeller-on fault injections.**

## Existing experimental workflow (interface unchanged)

The existing Mode29 entry validation, Arm -> Mode29 (NED hover target
`(0,0,-M29_TKOFF_ALT)`), takeoff/settle gate, Orange Pi's RC9..12
MAVLink single-motor injection/disable command, RC override watchdog and
motor ordering M1..M4 are retained. Only use with **motor-unpowered** HIL,
software-only injection, or propeller-free bench interfaces during validation.

## Experiment options (disabled by default)

- `M29_BALLOC=0`: original firmware allocator and pairing behavior.
- `M29_BALLOC=1, M29_PAIR_EN=0`: bounded collective-first, roll/pitch-second,
  yaw-last allocator, with blind FDI freeze on actuator saturation and staged
  injected-loss recovery.
- `M29_BALLOC=1, M29_PAIR_EN=1`: additionally enables the new experimental
  dynamic opposite-motor matching state machine (see below). This is **NOT**
  the original immediate-and-permanently-inhibited pairing logic.
- `M29_PAIR_EN=1, M29_BALLOC=0`: legacy experiment unchanged.

Only the real Softdrone motor-model build supports the new allocator.
No special loss percentage is coded: input is continuously valued
0..100% on any motor; physical feasibility determines achievable behavior.

## Progressive opposite derating and retry

- Blind onboard FDI identifies the original motor and estimates its loss.
  The known motor ID and severity from deliberate injected fault are only
  used as experiment safeguards; not substituted as estimator input during
  injection.
- For candidate opposite `(motor_id-1)^1`, check whether the *current*
  requested total thrust, roll and pitch can be attained with full target
  mirrored loss and all effective 0..100 motor boundaries.
- If target is feasible, hold feasibility for **150 ms**, then increase
  mirrored loss at **70 percentage points/s**. Recheck the exact
  F/Mx/My feasible interval at each step.
- If transient moments become infeasible, withdraw the synthetic opposite
  loss at **140 pp/s** and schedule another attempt after **300 ms**.
  Do NOT permanently inhibit for this ordinary wrench transient.
- Compare attainable yaw-braking torque under mirrored vs single loss
  whenever measured body yaw spin exceeds 0.5 rad/s. Mirroring is withheld
  if it worsens braking authority by more than 0.002 N*m. This explicitly
  fixes the mistaken assumption that equal fractional loss is always good
  for yaw braking.
- Wrong confirmed motor identification remains a hard supervised
  experiment block. Large position error or invalid navigation suppresses
  pairing. No hard-coded 80% or 90% exception.
- While an artificial second failure is applied, the one-fault FDI residual
  is not a valid severity update: it is held rather than misinterpreted.

## High-speed yaw and staged fault recovery

- In bounded/HIL fault and recovery modes, yaw controller requests a
  speed-damping torque `clamp(-0.045*gyro_z, ±0.15 N*m)`, but physical
  yaw moment is selected only from the residual nullspace after F/Mx/My.
  The allocator exposes `yaw_min_nm` and `yaw_max_nm`.
- `motor_bounded_yaw_unbrakeable` becomes true if the physically
  achievable moment cannot oppose the measured spin; a warning is
  emitted when spinning faster than 180 deg/s. **Neither the software yaw
  setpoint nor the measured gyro is artificially clipped. A hard yaw
  speed limit CANNOT be guaranteed if there is no braking torque.**
- When the operator ends a **software-injected** primary fault, the
  applied injector impairment is reduced at **100 pp/s** and the
  internal compensation tracks that same known effectiveness. The
  synthetic opposite impairment is withdrawn no slower than the primary.
  This avoids a one-tick 90%-to-zero injected-thrust effectiveness change.
- FDI adaptation is suspended during the staged injected release; a
  2-second re-anchoring cooldown follows. A previous yaw-free latch is
  retained to prevent grabbing an old heading while still spinning.
- **Crucial caveat:** This uses *known injected effectiveness on recovery*,
  so is **not** a validated solution for spontaneous mechanical recovery.
  Unknown recovery requires independent actuator-effectiveness sensing or
  observer verification. Fast yaw can also corrupt IMU/navigation tracking.
- Every path retains the old mode-exit, disarm, watchdog and finite-value
  actuator output checks. Hardware-in-loop protection remains to be tested.

## DataFlash records

- `L1BA`: `ena,sat,Freq,Fpred,Er,Ep,Freeze` — bounded allocation,
  requested/predicted collective, roll/pitch errors, frozen FDI updates.
- `L1PB`: `ena,mir,targ,retry,wait,ymn,ymx,spin,rec,inj` —
  actual mirror loss, FDI target, retry count, pending retry,
  attainable yaw-braking envelope, body yaw rate, staged recovery flag and
  applied **injected** fault loss.
- `L1DG,L1FD,L1PR,L1GS,L1GA` are retained for motor commands,
  FDI estimates, pair state, gain source and geometric-controller gains.
- `L1PB.ymn/ymx` are static **model predictions**. They are not direct
  force/torque measurements; calibration voltage/lag and geometry errors
  must be quantified separately.

## Required tests before any physical flight

1. Confirm GitHub Pixhawk6C toolchain compiles successfully.
2. Confirm timing budget at 400 Hz with `M29_BALLOC=1, M29_PAIR_EN=1`.
3. In an unpowered HIL vehicle, compare all four motor IDs and multiple
   fractional losses (not only 80/90) with the MuJoCo reference.
4. Check 150-ms feasibility hold, mirrored 70-pp/s ramp, transient
   retreat/retry and no false FDI reconfirmations.
5. Verify unavailable yaw braking is explicitly detected, and that no
   re-entry/exit path sacrifices collective or roll/pitch as a hidden
   price for yaw. Run sign-convention and actuator ordering tests.
6. End injection while high-speed yaw is simulated, check staged loss
   release, position bound, heading reacquisition and finite outputs.
7. Require full 36-second closed loop and **under 0.5 m maximum 3D
   position error** including post-fault recovery before even proposing
   propeller-on validation. Current 80%/90% MuJoCo results DO NOT pass.

## Build

GitHub workflow:
https://github.com/ssybh2/L1Quad-original/actions/workflows/build-mode29-bounded-hil.yml

Successful build artifact: `mode29-bounded-hil-only-pixhawk6c-firmware`,
containing `arducopter.apj`, `arducopter.bin`, and `BUILD_INFO.txt`.
The checks above are NOT implied by compilation success.
