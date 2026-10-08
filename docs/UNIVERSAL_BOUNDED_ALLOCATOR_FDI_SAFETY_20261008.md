# Universal bounded allocator and FDI saturation safeguards — 2026-10-08

Scope: ONLY `feature/mujoco-opposite-pair-20261008`; simulator changes,
NOT flight firmware or Orange Pi controller. These checks are research
diagnostics, NOT safe-flight certification.

## Failure repaired

The previous `Mixer.allocate_effectiveness_aware` solved an unconstrained
pseudoinverse and then clipped each independently negative nominal thrust
to zero. Under an infeasible F/Mx/My request, clipping can more than DOUBLE
collective thrust: an 11.625-N request was predicted as 24.66 N for the
reported high-degradation transient.

That code path is replaced by `bounded_allocator.bounded_allocate`, shared
by normal/recovery mode, yaw-free mode and FDI single-fault mode.
There is no special 90% branch or hard-coded severity threshold.

## Exact physical constraints

The effective rotor thrusts `f` always satisfy

```
0 <= f_i <= eta_i * thrust_at_command_100
eta_failed = 1 - loss_estimate_percent/100
eta_other = 1
```

The solver prioritizes:
1. Collective thrust: clamp the requested F to `[0, sum_i(max_actual_i)]`.
   Maintain the reachable collective exactly. No clipped pseudoinverse.
2. Roll/pitch: at that fixed collective, construct all vertices of the
   rotor-bound 4D polytope, project `[Mx,My]` to its **exact reachable
   convex 2D polygon**, then realize the projected moment.
3. Yaw: optimize reaction-torque via the remaining thrust nullspace,
   only after the collective and attainable Roll/Pitch are guaranteed.

If the whole F/Mx/My demand is feasible it is preserved; if only Roll/Pitch
is impossible, the result reports `alloc_primary_saturated=1` and the
realized moment errors. If total thrust is impossible due to insufficient
actual rotor capability, `alloc_collective_clamped=1` reports the
physically reachable value instead of inventing impossible lift.

## Detector confidence

A prior timestep's `alloc_primary_saturated` freezes an already
confirmed FDI severity: a saturated actuator residual cannot be treated
as a new independent motor fault. Implausible signed innovations outside
the single-rotor physical range are rejected, not integrated until the
estimate silently reaches 100%. Logs preserve both clipped trusted
`fdi_loss_instant_pct` and untrusted raw
`fdi_loss_unclipped_instant_pct` for diagnosing rejected outliers;
`fdi_estimate_frozen_samples` and
`fdi_implausible_innovation_count` are explicit.

**Limitation:** Holding severity through sustained saturation prevents
corrupt estimation but also leaves the previously estimated effectiveness
uncertain. It does not prove the FDI estimate is accurate.

## General safety mechanism

The simulator stops in a controlled manner if position error exceeds the
editable `safety.position_divergence_abort_m = 0.80` for
`safety.position_divergence_hold_s = 0.06`. The guard stays active after
the fault ends to cover the recovery transient. A stop is reported as
ABORTED / nonzero exit, NOT a successful recovery.

The **stricter** independent physical CI gate requires:
- requested duration completed (no premature stop),
- max **3D position error <=0.50 m** in every flight phase,
- bounded predicted rotor collective errors,
- physically bounded *trusted* FDI instantaneous and accumulated loss.

Tests use `tools/check_physical_safety.py`. A failing gate makes CI red
even if unit tests or basic identity/state-machine checks pass.
The 0.80m simulation abort is deliberately a last resort distinct
from the 0.50m regression acceptance criterion.

## How to reproduce

```powershell
git switch feature/mujoco-opposite-pair-20261008
git pull --ff-only origin feature/mujoco-opposite-pair-20261008
python -m unittest discover -s tests -p 'test_*.py' -v
.\run.ps1 -Pair -LossPercent 90
python .\tools\check_physical_safety.py .\logs\mode29_opposite_pair.csv --duration 36 --max-error 0.5
```

The default repository 90%-anchor gains are **not** identical to the
user's separately saved local gains. Reproducing that parameter set
requires explicitly merging it from the local TOML backup.

Safety remains unproven when max XY/Z error or recovery limits fail,
when available yaw braking torque is unachievable, or when the estimator
cannot observe a fault because of sustained actuator saturation.
No high-severity result should be called 'validated safe' merely because
the main simulation process returned successfully.
