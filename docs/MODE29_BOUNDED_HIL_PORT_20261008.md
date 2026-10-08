# Pixhawk 6C Mini — Mode29 bounded allocator, **bench/HIL-only** integration

## Scope and safety
This is a separately gated **research** firmware derived from
`feature/mode29-pixhawk-next-from-20261006`. It does **not** replace
`milestone/mode29-60pct-validated-20261006`, any older flyable image or
the Orange Pi branch. The MuJoCo 80%/90% tests still fail strict full-cycle
recovery safety conditions. **The binary is NOT validated for propeller-on
flight or intentional motor failures in flight.**

## Existing runtime protocol is retained
The existing sequence and controller entry are unchanged:
1. Arm using pilot safety procedures;
2. Mode29 enters, validates navigation/origin and tracks the fixed NED
   target `(0,0,-M29_TKOFF_ALT)` (usually `(0,0,-1)`);
3. Existing Orange Pi MAVLink RC9..RC12 commands can emulate a fault
   after `M29_TKOFF_T + M29_SETTLE_T`;
4. 500-ms derating command watchdog, Mode29/disarm cleanup and existing
   RC override release stay in place.

**These protocol steps are documented for interface compatibility;
do not perform step 3 with propellers installed using this HIL binary.**

## Migrated algorithm, and its limitations

- `M29_BALLOC=0` (default): keep all existing allocation paths.
- `M29_BALLOC=1` (disarmed setting for HIL only): use a static thrust-domain
  allocator constrained by the **actual motor effectiveness caps**.
  Exact priority: (1) collective F clamped only to achievable sum of caps;
  (2) roll and pitch projected onto a convex reachable moment polygon at
  that fixed F; (3) yaw receives only the leftover actuator nullspace.
- The allocator is a fixed-size C++ implementation (no heap allocations)
  using the existing real Softdrone 6S motor model and 0–100 command range.
  Fault loss is continuous 0–100%, any one motor 1–4.
- If the prior command was physically unattainable, already-confirmed
  blind FDI severity is frozen rather than interpreting a saturated
  allocation residual as a new failure. The recovered confidence can
  remain uncertain during prolonged saturation.
- FDI innovations inconsistent with the physical range of a single motor
  are not integrated into the severity estimate while this feature is
  enabled.
- New logger record `L1BA` fields:
  `ena` (opt-in status), `sat` (requested F/Mx/My unattainable),
  `Freq` (requested collective [N]), `Fpred` (bounded predicted
  collective [N]), `Er`/`Ep` (roll/pitch tracking errors [N·m]),
  `Freeze` (FDI frozen-cycle count).

## NOT yet migrated / reasons for blocking

- `M29_PAIR_EN=1` is **rejected at Mode29 entry** when `M29_BALLOC=1`.
  The existing firmware pairing algorithm still applies an instantaneous
  second artificial motor loss with a permanent inhibit latch.
  MuJoCo's re-entry timing, progressively ramped pair loss and physical
  yaw-authority optimization have NOT been implemented/validated in
  Pixhawk yet. Combining them would be unsafe.
- **Yaw limitation**: the current physical soft cap is not a guaranteed
  constraint when residual rotor moment cannot brake spin.
- **Recovery**: the high-loss MuJoCo simulations can maintain position
  during injection but lose position after restoring the failed motor.
  A separate recovery transition with actuator slew limits, status
  handover, fault persistence handling and navigation validation is needed.
- This code has not yet been compared against Softdrone thrust-stand data
  at every battery voltage, actual motor DSHOT/PWM actuator mapping, AHRS
  behavior at large yaw spin, or flight-control CPU performance.

## Bench/HIL checklist (propellers removed)

1. Capture the exact prior working firmware and all current parameters.
2. Flash the GitHub Actions `arducopter.apj` **only onto the intended
   Pixhawk 6C Mini test hardware** and retain the ability to restore.
3. Leave `M29_PAIR_EN=0`. With propellers removed, test
   `M29_BALLOC=0` first; only then set `M29_BALLOC=1` while disarmed.
4. Verify RC overrides, physical motor ordering M1..M4, actuator output
   limits, 500-ms watchdog, mode exit, disarm, navigation validity, and
   independent physical kill. Observe `L1BA,L1DG,L1FD,L1GS,L1PR`.
5. Exercise arbitrary fractional injected severities in a
   non-flight simulator or restrained **motor-unpowered** HIL setup;
   assert `Fpred` does not exceed physically reachable sum of caps.
6. Perform CPU loop timing regression, calibration mismatch tests,
   FDI false positive tests, and controlled recovery tests before
   considering a different propeller-on experimental firmware.

## Build location

GitHub Actions:
`https://github.com/ssybh2/L1Quad-original/actions/workflows/build-mode29-bounded-hil.yml`

Artifacts (only when CI compilation succeeds):
`mode29-bounded-hil-only-pixhawk6c-firmware` containing
`arducopter.apj`, `arducopter.bin`, `BUILD_INFO.txt`.

No claim of flight-safety or a successful flash is implied by compilation.
