# Mode29 fixed pre-fault PWM ceiling experiment — HIL ONLY (2026-10-09)

Branch: `feature/mode29-prefault-pwm-cap-hil-20261009`, based on the
successful v2 HIL commit `ef22ed8`. New experimental code is **NOT**
qualified for powered / propeller-on tests. Perform only unpowered,
closed-loop SITL/HIL replay and independently verify controller stability
before any real-flight consideration.

## Precise definition (changed from Runs 69–71)

Previous builds: `F_after(t) = (1-loss/100)*F_model(w_desired(t))`
with an inverse thrust model generating final PWM. The loss is a
**constant fraction of instantaneous desired model thrust**.

**NEW experimental branch (only when `M29_BALLOC=1`):**

- Use the last **actually applied** per-motor normalized PWM command
  `w_i(t0^-)` from the complete control tick just *before* the first
  valid injection of motor `i` during Mode29. The frozen snapshot
  contains all four motors. It is not recaptured while the fault runs.
- `w_cap_i(t) = w_i(t0^-) * (1 - loss_percent(t)/100)`.
- **Hard limit**: `w_i,applied(t) = clamp(w_i,allocated(t), 0, w_cap_i(t))`.
  The controller may command any value from 0 to that **fixed ceiling**.
  The same snapshot is used for every following tick, including during
  command response/feedback and staged release.
- `w` is a **normalized PWM increment**: PWM_us=1000+10*w.
  Do NOT multiply absolute microseconds by (1-loss)! E.g. pre-fault
  `w=40` (PWM=1400us), loss=80% -> upper bound **w=8 (1080us)**.
  This is NOT a commanded 20% of the pre-fault thrust: the non-linear
  calibrated motor model typically predicts far less force at w=8.
- Failure is a **PWM actuator saturation** rather than a proportional
  reduction of aerodynamic thrust. When command is below this ceiling,
  it is *not* further scaled down.

When `M29_BALLOC=0`, the old proportional-thrust software injection
code path remains unchanged. Existing #23/#25 firmware images and logs
are not modified, so runs MUST be compared with the correct semantics
and exact build commit.

## Internal allocation and experimental oracle involvement

The constrained allocator is updated to use
`F_cap_i=softdrone_thrust_from_w(w_cap_i)` and
`w_i=softdrone_w_from_thrust(f_i)` limited by the same PWM cap. No
`eta * F_nominal` or division by effectiveness is used in the bounded
allocation branch. The post-allocator actuator output independently
enforces the cap as a safety backstop.

**This HIL version uses injected motor identity and loss percentage in
the allocator's cap construction and in synthetic pairing admission.**
It is explicitly **oracle-assisted**, not fully blind FTC. The existing
L1 observer/FDI pipeline is retained as a *diagnostic* and receives no
direct injection loss in `update_auto_motor_fault_detector()`, but its
reported percentage estimates a different physical impairment and
MUST NOT be reported as correct PWM-cap loss estimation. `M29_GS_MODE=2`
still uses FDI severity for gain scheduling; prefer `M29_GS_MODE=0`
in isolated HIL protocol tests to avoid contaminated gain changes.

## Opposite motor / pair

When `M29_PAIR_EN=1`, the experiment uses the already-frozen
pre-primary-fault PWM snapshot of the opposite motor. It never samples
opposite PWM after the primary fault has changed commands.

- To avoid a step from the full 0..100 range to the *frozen hover PWM*
  at first admission, the opposite cap is interpolated continuously:
  `w_cap_opp=100 + (w_opp(t0^-)*(1-L_target/100)-100)
                        * (mirror_pct/L_target)`
  (bounded to 0..100 and `mirror_pct<=L_target`).
  It starts at **100** and reaches the user's required frozen snapshot
  ceiling at `mirror_pct=L_target`. The mirror percentage is the
  commanded progress toward the final hard cap, not an instantaneous
  multiplier on a desired thrust.
- The target mirror percentage is the *commanded primary PWM cap loss*
  for this supervised HIL experiment, not the L1-FDI projected thrust
  loss. The pair is **not** presented as blind autonomous fault control.
- Preserve existing 150ms full-wrench-feasibility hold,
  70 percentage-points/s mirror ramp, 140 pp/s withdrawal on infeasible
  wrench, 300ms retry and yaw braking authority comparison.
- Feasibility is now evaluated from `F_model(w_cap_i)`, not
  `(1-loss)*F_model(100)`. Pair may never engage for a deep cap.
- Release stores the exact capped primary PWM at command-off, then
  continuously expands its ceiling all the way to **100** as the
  injection-loss counter decreases at 100 percentage-points/s.
  Simply expanding to the old hover PWM and instantaneously jumping to
  100 at the end would be an unintended actuator discontinuity.
  The synthetic paired ceiling also expands continuously to 100 while
  withdrawn; persistent mechanical recovery is not inferred.

## DataFlash traces

- Existing `L1DG.c1..c4`: bounded allocator's final command
  `w_allocated`. Existing `L1DG.a1..a4`: post-hard-cap command sent to
  the FC output interface. These are PWM units, not motor thrust sensors.
- New `L1PC` (HIL normalized PWM):
  `valid,motor,base,cap,opp,bopp,copp,clip`.
  `valid=1` means snapshot latched, `motor` primary ID,
  `base` its frozen pre-fault PWM, `cap` its current max PWM,
  `opp` opposite motor ID, `bopp` its frozen baseline,
  `copp` its current max PWM, `clip` cumulative post-allocator
  clamps caused by unexpected allocator/physical cap disagreement.
- `L1BA.Freq/Fpred` are requested/predicted **model** collective,
  `L1BA.Er/Ep` are model moment residuals.
- `L1PB.inj/mir` are **PWM ceiling loss percentages** in this branch,
  NOT model thrust effectiveness loss percentages.
- Compare raw `RCOU` to `1000+10*L1DG.aN` by correct servo mapping.
  No data here directly measures physical motor thrust, RPM or ESC response.

## Mandatory before controlled simulation

1. Keep motors unpowered, remove propellers, or use SITL/software plant.
2. Confirm a stable and valid preceding hover sample, otherwise injection
   aborts. Inspect `L1PC.valid/base/cap`.
3. Verify all four possible failed motor IDs at 0, 10, 40, 60, 70,
   80, 90 and 100% commanded loss and at multiple before-fault PWMs.
4. Verify that neither the allocator nor final applied command exceeds
   the stored ceiling even when the requested control wrench changes.
5. Verify no false snapshot recapture and proper gradual cap release.
6. Verify paired motor snapshot/ramp/withdraw and that unreachable
   wrench requests are reported as saturated, never fabricated.
7. Re-evaluate FDI under a *cap* failure model before claiming blind
   identification, confirmation latency, or severity accuracy.
8. Observe CPU loop rate and position/attitude/sensor failsafes in
   full closed-loop test. CI green is **not** validation of this design.

IMPORTANT: 80% loss from w=40 leaves w<=8, close to the
Softdrone thrust-model dead zone (~4.48). This can be effectively
a near-total physical thrust outage, far more severe than the prior
80% thrust-effectiveness tests. NEVER retry this on the real powered
aircraft based only on successful compilation.
