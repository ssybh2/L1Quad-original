# MuJoCo universal fault-pair retry + position-first control (2026-10-08)

This is a **simulation experiment only**, on
`feature/mujoco-opposite-pair-20261008`.
It does NOT change the frozen 2026-10-06 baseline, the Pixhawk firmware,
or the Orange Pi branch. No special handling for 80% is coded; faults
0..100% remain continuous inputs and gain/yaw envelopes retain individual
0/10/20/.../100% tuning nodes.

## Root cause

A single transient (example: estimated M1 loss 83.96% at t=16.235 s)
could demand an impossible combination of total thrust, roll and pitch
moments after M2 was suddenly reduced to the same effectiveness.
Previously, the first unsatisfiable actuator-bound interval permanently
set `pair.inhibited=True`, preventing retry after the wrench returned
inside the feasible region.

A necessary static collective-thrust margin of 1.717 does not imply that
every dynamically demanded F/Mx/My vector is achievable.

## New behavior, general for every loss percentage and motor ID

1. Blind FDI still identifies the failed motor and settles the severity.
   Its experimental injection-truth agreement check is a **safety veto**,
   not an estimator input. Incorrect ID or invalid navigation can still
   trigger an appropriate hard safety inhibit.
2. On pair admission the synthetic opposite impairment starts at **0%**.
   With the original fault already applied, the program checks whether
   the **target** opposing effectiveness can realize the current three-axis
   wrench `[F,Mx,My]` inside `[0, Fmax_i]` for all four actual motors.
3. Full target pairing must remain feasible continuously for
   `feasible_hold_s` (default **0.15 s**) before synthetic derating begins.
   Transient F/Mx/My infeasibility does not trigger a permanent latch.
4. Increase opposite loss by at most `mirror_ramp_rate_pp_s * dt`
   (default **70 percentage points/s**); recheck the **proposed** loss each
   step. If demand grows or the feasible interval shrinks, withdraw as much
   of the artificial loss as needed to regain primary control authority.
5. If even **0%** synthetic loss is dynamically infeasible, withdraw the
   artificial loss, keep the FDI-identified real primary fault, apply
   single-motor control allocation, and retry after `retry_delay_s`
   (default **0.30 s**) once stability returns. No `inhibited=True`
   for this ordinary transient. FDI is retained across the temporary
   retry, instead of falsely resetting the known original failure.
6. Yaw uses **only remaining reaction-torque nullspace** after the
   primary wrench. It receives an optional slower nonzero spin target:
   default at most 65% of the original spin request, further reduced by
   XY position error and partial completion of the mirror ramp.
   If measured yaw approaches the individually scheduled yaw limit,
   withdraw the synthetic loss early to regain moment authority.
   An actual hard safety failure still withdraws the artificial impairment.
7. Once the injected original fault finishes, stop mirroring and let
   the normal FDI recovery/cooldown handling resume.

This algorithm is **not** guaranteed to make every actuator-bound wrench
feasible. It will sacrifice matching the opposite loss or self-spin,
rather than knowingly violate requested total thrust/Roll/Pitch constraints.
A complete 100%/100% opposite pair failure still lacks enough independent
actuators for arbitrary F/Mx/My.

## Edit parameters in `config_opposite_pair.toml`

```toml
[opposite_pair]
# ... existing FDI evidence and static margin settings ...
retry_delay_s = 0.30
feasible_hold_s = 0.15
mirror_ramp_rate_pp_s = 70.0
minimum_primary_reserve_n = 0.0
max_position_yaw_scale = 0.65
```

You can still individually customize every gain and yaw speed limit in
`[gain_schedule.loss_0]`, `loss_10`, ... `loss_100`.
A 37.4% loss interpolates neighboring calibration anchors; 80% is merely
one of the test points.

The source's per-loss `max_yaw_rate_deg_s` remains a **soft physical
control target**, not a virtual angular-velocity clamp. For position-first
operation you may reduce `max_position_yaw_scale`, increase the mirror
ramp duration, or require a longer feasibility hold, after MuJoCo testing.

## Logging and tests

In `logs/mode29_opposite_pair.csv`, compare:

- `pair_active`: **the second fault is actually being applied** (not
  just the presence of the pair controller)
- `pair_controller_armed`: pairing controller admitted and continuously
  evaluating the current wrench
- `pair_mirror_applied_pct`: real synthetic effectiveness loss on M2/M4
- `pair_mirror_target_pct`: original FDI estimated effectiveness loss
- `pair_full_wrench_feasible_stable`: target wrench continuously feasible
- `pair_retry_count`, `pair_retry_after_s`: temporary fallback/retries
- `pair_primary_interval_width_n`: available thrust nullspace width
- `pair_rollback_count`: ramp reversal to preserve primary control
- `pair_yaw_backoff_count`: yaw overspeed prevention by giving up mirroring
- `pair_position_yaw_scale`: current nonzero spin target sacrifice
- `pair_guard_reason`, `pair_transient_reason`: reason for pause/exit
- existing FDI estimate and position logs for input and resulting hover error

Run locally (Windows PowerShell):

```powershell
git switch feature/mujoco-opposite-pair-20261008
git pull --ff-only origin feature/mujoco-opposite-pair-20261008
.\run.ps1 -Pair -LossPercent 80 -Headless -NoRealtime
python .\analyze_pair_log.py .\logs\mode29_opposite_pair.csv
```

Use e.g. 20, 37.5, 65, 80, 90 as **test samples** rather than different
branches in the algorithm. Set `gain_schedule.mode="automatic"` to
evaluate FDI-based gain scheduling; `opposite_pair.source="fdi"` already
uses the observer for pair matching.

Validation uses `tests/test_pair_transition.py` (including an impossible
transient [12.194,-1.663,1.627] followed by a later feasible
[10.29,-0.083,0.007] demand), `tests/test_yaw_rate_schedule.py`,
and GitHub Actions 36 s closed-loop FDI-and-recovery simulations.

**No hardware/propeller-on test is approved** by these MuJoCo checks.
