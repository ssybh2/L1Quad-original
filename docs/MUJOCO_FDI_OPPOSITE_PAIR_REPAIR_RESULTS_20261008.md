# MuJoCo Mode29 FDI + opposite-pair integration: post-review fixes

Date: 2026-10-08. Scope: **simulation branch only** `feature/mujoco-opposite-pair-20261008`. The user's uploaded source and frozen 2026-10-06 milestone remain unchanged.

## Initial diagnosis

The earlier [FDI/pair bug report](../MUJOCO_FDI_OPPOSITE_PAIR_BUG_REPORT_2026-10-08.md) described:

1. FDI confirmed the **fault motor ID** after 60 ms but *loss magnitude* was not converged.
2. The pair bridge permanently inhibited itself on its first transient estimate-versus-injection disagreement.
3. Once the FDI compensator worked, its **innovation residual** was almost zero, leaving the *normalized innovation fit* extremely noisy. This was incorrectly used as a continuing identity gate; >5,000 samples were rejected even when the severity estimate had converged.
4. Pair enabled but not active disabled the original yaw-rate damping; recovery fallback masked the intended yaw-rate controller.
5. After a fault ended, the estimator could re-confirm a **healthy** motor due to stale residual/controller-topology transients.
6. Near 80% paired failure, requested full F/Mx/My sometimes exceeded physical actuator bounds; this previously raised a fatal exception.

## Code changes

- **Time-align FDI features:** filter the four motor fault signatures with the same temporal low-pass as the observed matched-moment residual, rather than fitting an old filtered residual against current motor thrust.
- **Preserve motor identity confidence:** latch `identity_fit_ratio` at confirmation. Subsequent near-zero compensated innovation residual ratios are logged for estimator diagnostics but are no longer used to disqualify already-confirmed motor identity.
- **Separate identity/severity confidence:** wait for 120 ms of sufficiently stable magnitude after it is compatible with the known experimental injection (8 percentage points). A temporary mismatch is *retryable* and no longer sets permanent `pair.inhibited`. The injector remains an **experimental safety veto**, not an FDI estimator input.
- **Preserve yaw physical damping** while a fault has been identified but opposite pairing is not yet active, and on pair fallback or recovery. No direct clipping of measured angular velocity is added.
- **Recovery cooldown:** re-anchor measured residual and moment baseline for 2 s after a confirmed detector recovery or after *active* synthetic pairing disengages. The fault motor estimate is reset only after paired topology was actually applied; if a newly proposed pair is rejected *before* its second impairment, single-fault FDI is retained.
- **Primary authority safety:** if `F/Mx/My` cannot be jointly allocated after attempting to mirror a motor, abandon the **artificial** second failure, log the reason, and fall back to the single-motor effectiveness-aware allocator. Never hide feasibility failure by silently sacrificing roll/pitch/height priorities.
- **Logs:** `fdi_loss_instant_pct`, `fdi_loss_rate_pp_s`, `fdi_cooldown_remaining_s`, `pair_retry_pending`, `pair_transient_reason`, `pair_severity_ready`, `allocator_effectiveness_loss_pct`, `schedule_allocator_mismatch_pp`. Use `python analyze_pair_log.py LOG.csv`.

## Headless baseline regression result (2026-10-08)

GitHub Actions run [37754398578](https://github.com/ssybh2/L1Quad-original/actions/runs/37754398578) succeeded with 36 s blind FDI fault + recovery experiments (M1, injection t=16..30s) and an independent oracle pairing comparison.

| Injected M1 loss | FDI first confirmed | Pair actually active | Pair active duration | Max actual yaw while paired | FDI fresh false re-confirmations | Final (36s) NED position error |
| --- | --- | --- | --- | --- | --- | --- |
| 60% | ~16.062s | 16.232s to 30s | 13.768s | 115.8 deg/s | 0 | 0.060 m |
| 70% | ~16.062s | 16.235s to 30s | 13.765s | 114.8 deg/s | 0 | 0.076 m |
| 80% | ~16.062s | **Rejected before second impairment** | 0 | not applicable | 0 | 0.296 m |

The 80% attempt at ~16.235s reported **`paired primary authority infeasible: primary thrust/roll/pitch wrench infeasible with rotor thrust bounds`**. Under current physical motor model and demanded transient wrench, forcing the second loss would be invalid. This is now an explicit, recoverable fallback, not a success claim for 80% paired flight.

- The 60/70% paired simulations stayed inside the configured **180 deg/s** yaw ceiling. They used a **117 deg/s nonzero self-spin target** with no direct gyro-rate clamp.
- All results are for the existing **motor thrust/reaction cubic curves, motor lag and static/noise seed**, with per-loss gain anchors as currently configured. They do not generalize to all loss severities, motors, disturbance seeds or physical quadrotors.
- Config uses `gain_schedule.mode="oracle"` for repeatable calibration of control gains, although **pair motor ID and severity are genuinely blind FDI** in these runs (`opposite_pair.source="fdi"`). Do not describe this as full estimator-driven gain scheduling.
- The controller intentionally allows **0%–100% arbitrary injection values**. A complete paired 100% loss is actuator-rank-deficient and must be reported as infeasible rather than forced.

## How to rerun

```powershell
git switch feature/mujoco-opposite-pair-20261008
git pull --ff-only origin feature/mujoco-opposite-pair-20261008
python -m pip install -r requirements.txt

# Blind: actually estimate motor/severity, then use FDI to engage opposite pair
.\run.ps1 -Pair -LossPercent 60 -Headless -NoRealtime
python .\analyze_pair_log.py .\logs\mode29_opposite_pair.csv

# Independent physical-allocation comparison; simulator-only truth-based pairing
.\run.ps1 -Pair -Oracle -LossPercent 60 -Headless -NoRealtime
```

The GitHub workflow now runs **60/70/80 full fault-and-recovery** regressions and explicit [`tools/check_pair_regression.py`](../tools/check_pair_regression.py) assertions to catch regressions where FDI identifies the motor but synthetic mirroring never engages (60/70), 80% crashes instead of safely refusing the pair, or recovered healthy motors are mis-confirmed.

## Remaining scientific and safety limitations

- Magnitude estimation is still a dynamic model-based proxy, not real measured thrust. The true injected fraction is used only to veto unsafe pairing in these controlled tests.
- The 80% fallback has substantially worse position error than the 60/70% paired cases. This requires separate study of available thrust, moment bounds, FDI and recovery dynamics, not only more aggressive gains.
- The 2-s cooldown trades repeated-fault detection responsiveness against false alarms. Consecutive real failures, unseen faults during cooldown, sustained large yaw, motor mismatch, and all four motor IDs require broader testing.
- A soft yaw-rate ceiling cannot promise physical angular rate remains below it when yaw control moment saturates. No free-flight or hardware-on-propeller approval is implied.
- All changes apply **only to this MuJoCo branch**, not to the Pixhawk firmware or Orange Pi working branches.
