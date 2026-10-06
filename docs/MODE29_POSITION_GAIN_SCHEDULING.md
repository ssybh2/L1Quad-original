# Mode29 Geometric Gain and Tilt Scheduling

## Frozen milestones

The original FDI/recovery controller remains frozen on:

`milestone/mode29-fdi-recovery-20261006`

at:

`ca4a5a44d21b7fa4efa0ad610edd53e470169ac9`

The first position-only gain-scheduler version is also frozen on:

`milestone/mode29-pos-gainsched-20261006`

at:

`e20ce0b81cd22cadf2bf0edae77a05ccc3d48b4f`

Current development continues on:

`feature/mode29-position-gain-schedule`

## Scheduled quantities

At each 50/60/70/80/90/100 percent motor-loss anchor the scheduler can now set:

- Position gains: `KPX/KPY/KPZ`
- Velocity gains: `KVX/KVY/KVZ`
- Attitude gains: `KRX/KRY/KRZ`
- Angular-rate gains: `KOX/KOY/KOZ`
- Maximum combined roll/pitch tilt: `TILT` in degrees

L1 parameters (`ASV`, `ASOMEGA`, `CTOFFQ*`) are deliberately not scheduled.

The existing yaw-free and effectiveness-aware allocation logic is unchanged.

## Scheduler modes

`M29_GS_MODE` selects the loss source:

- `0`: disabled. Use normal `GEOCTRL_*` values and `M29_MAX_TILT`.
- `1`: oracle calibration. Use the known injected loss percentage.
- `2`: automatic. Use the onboard blind-FDI loss estimate.

The scheduler low-pass filters and rate-limits the loss before interpolating parameters.

## Anchors and interpolation

Each loss anchor uses parameters named like:

`M29_G60_KPX`, `M29_G60_KVY`, `M29_G60_KRX`, `M29_G60_KOY`, `M29_G60_TILT`

The firmware performs piecewise-linear interpolation between neighboring anchors. Below 45% loss the normal controller is retained. From 45% to 50% it blends smoothly into the 50% anchor.

## Current calibrated values

The 50% anchor retains the previously validated position gains:

- KP = [4.0, 4.0, 10.0]
- KV = [4.0, 4.0, 2.0]

The 60% anchor currently stores:

- KP = [5.5, 5.5, 10.0]
- KV = [4.0, 4.0, 2.0]

Until separately tuned, KR/KO and tilt anchors start from the working baseline:

- KR = [1.0, 0.5, 0.25]
- KO = [0.1, 0.2, 0.1]
- tilt = 30 deg

## Calibration workflow

Use:

`orangepi/configs/mode29_position_gain_schedule.yaml`

with `M29_GS_MODE=1`.

Tune only the row matching the injected loss. For example, when calibrating 70% loss, change only the `M29_G70_*` parameters. Apply while DISARMED, inject exactly 70%, then evaluate the BIN log.

After neighboring anchors are validated, test intermediate losses such as 55% and 65% without adding anchors. This tests the interpolation itself.

## Logging

`L1GS` records scheduler mode, raw/scheduled loss, confidence and active KP/KV values.

`L1GA` records active KRX/KRY/KRZ, KOX/KOY/KOZ and scheduled maximum tilt.

`L1FD` remains the fault-detection/isolation diagnostic log.
