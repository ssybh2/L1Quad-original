# Mode29 Position Gain Scheduling

## Frozen milestone

The pre-scheduling controller is frozen on branch:

`milestone/mode29-fdi-recovery-20261006`

at commit:

`ca4a5a44d21b7fa4efa0ad610edd53e470169ac9`

All gain-scheduling work starts from that exact controller state on:

`feature/mode29-position-gain-schedule`

## Scope

This experiment schedules **only** the translational geometric-controller gains:

- `GEOCTRL_KPX`, `GEOCTRL_KPY`, `GEOCTRL_KPZ`
- `GEOCTRL_KVX`, `GEOCTRL_KVY`, `GEOCTRL_KVZ`

The following remain unchanged by the scheduler:

- `GEOCTRL_KRX/KRY/KRZ`
- `GEOCTRL_KOX/KOY/KOZ`
- `ASV`, `ASOMEGA`
- `CTOFFQ1THRUST`, `CTOFFQ1MOMENT`, `CTOFFQ2MOMENT`
- the existing yaw-free and effectiveness-aware allocation logic

## Scheduler modes

`M29_GS_MODE` selects the scheduling source:

- `0`: disabled. The controller is equivalent to the frozen milestone for the position loop.
- `1`: oracle calibration. The scheduler uses the known injected loss percentage from the test injector.
- `2`: automatic. The scheduler uses the onboard blind-FDI loss estimate.

Mode 1 exists so that each loss anchor can be tuned without mixing gain-tuning error with FDI estimation error.

## Gain anchors

Six AP_Param anchor rows are available:

- 50%: `M29_G50_KPX ... M29_G50_KVZ`
- 60%: `M29_G60_KPX ... M29_G60_KVZ`
- 70%: `M29_G70_KPX ... M29_G70_KVZ`
- 80%: `M29_G80_KPX ... M29_G80_KVZ`
- 90%: `M29_G90_KPX ... M29_G90_KVZ`
- 100%: `M29_G100_KPX ... M29_G100_KVZ`

Between anchors the firmware uses piecewise-linear interpolation. Below 45% loss, the normal `GEOCTRL_KP*/KV*` values are retained. Between 45% and 50% the controller blends smoothly into the 50% anchor.

## Calibration sequence

Use `orangepi/configs/mode29_position_gain_schedule.yaml` with `M29_GS_MODE=1`.

Tune only one anchor row at a time. For example, while calibrating 60% loss, modify only:

`M29_G60_KPX/KPY/KPZ/KVX/KVY/KVZ`

Apply the YAML while DISARMED, then inject exactly 60% loss. The normal hover before the fault still uses the baseline `GEOCTRL_KP*/KV*` values.

After 50/60/70% anchors are established, test intermediate values such as 55% and 65% without adding new anchors. This validates whether interpolation generalizes rather than merely reproducing hand-tuned points.

## Automatic mode

After the oracle table is validated, set `M29_GS_MODE=2`.

The current onboard FDI remains blind to the injector. Its continuous loss estimate is accepted by the scheduler only when the motor-fault signature fit is credible. The scheduled loss is low-pass filtered and rate limited before gains are interpolated.

## Logging

`L1GS` records:

- `mode`
- `raw`: raw loss source
- `sched`: filtered/rate-limited loss used for interpolation
- `conf`: scheduler confidence
- `kpx,kpy,kpz,kvx,kvy,kvz`: actual gains used by the controller

Existing `L1FD` remains the FDI diagnostic log.
