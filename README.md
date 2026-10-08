# Softdrone Mode29 MuJoCo v2 — disturbance tuning build

This version keeps the Mode29 geometric-controller / 6S motor-model baseline
and adds deliberately non-ideal plant/sensing conditions for tuning.

## New in v2

- configurable non-zero initial roll / pitch / yaw
- noisy position and velocity measurements
- noisy attitude and gyro measurements
- fixed configurable sensor biases
- motor first-order lag
- PWM-command jitter
- fixed per-motor thrust/reaction-torque mismatch
- motor thrust/reaction-torque output noise
- true state and measured state both logged to CSV
- final hover target visualized as a red sphere

The disturbance defaults are synthetic tuning values, not measured sensor or
ESC specifications.

## Target

With:

```toml
takeoff_altitude_m = 1.0
```

the final controller target remains:

```text
NED [0, 0, -1] m
```

The red sphere marks that final point in the MuJoCo Viewer.

## Requirements

- Python 3.11 or newer
- packages listed in `requirements.txt`

Install the Python dependencies from this directory:

```powershell
python -m pip install -r .\requirements.txt
```

## Initial attitude

Edit:

```toml
[initial_state]
attitude_rpy_deg = [3.0, -2.0, 5.0]
```

Format is `[roll, pitch, yaw]` in degrees.

## Tune disturbance strength

Sensor disturbance is under:

```toml
[sensor_noise]
```

Motor disturbance is under:

```toml
[motor_disturbance]
```

Set either `enabled = false` for idealized comparison runs.

## Tune Mode29 gains

Edit:

```toml
[gains]
```

Then save the file and rerun:

```powershell
python .\mode29_mujoco.py --config .\config.toml
```

Logs are written to:

```text
logs\mode29_sim.csv
```

The CSV now contains both true state (`x,y,z`, attitude) and noisy measured
state (`meas_x,meas_y,meas_z`, measured attitude), plus commanded and actual
motor outputs.

## Mode29 motor fault injection / FDI recovery

The simulator now ports the fault path from the repository's most advanced
non-main branch, `feature/mode29-position-gain-schedule` (reviewed at commit
`2c606d7`). The physical vehicle configuration is unchanged.

Default scenario in `config.toml`:

- smooth takeoff and settle first;
- inject 60% thrust-effectiveness loss on M1 at 16 s;
- keep the injector blind (it does not tell the controller which motor failed);
- isolate the motor from measured rotational dynamics;
- release fixed-yaw control after confirmation;
- allocate `[F, Mx, My]` with `B*diag(eta)` and preserve partial motor authority;
- use the remaining allocation null space for bounded yaw-rate damping while
  the fault is active, without recapturing a yaw angle;
- schedule geometric gains/tilt from the estimated loss;
- remove the injected fault at 24 s and recover with hysteresis;
- keep yaw angle free while damping yaw rate after recovery. The rate-only
  damping is needed because this ideal MuJoCo body has no aerodynamic yaw drag.

Motor order is unchanged:

1. M1 front-right, CCW
2. M2 rear-left, CCW
3. M3 front-left, CW
4. M4 rear-right, CW

### Run with the viewer

```powershell
.\run.ps1 -LossPercent 60
```

or:

```powershell
python .\mode29_mujoco.py `
  --config .\config.toml --loss-percent 60
```

`-LossPercent` / `--loss-percent` temporarily overrides
`fault_injection.loss_percent` for that run without rewriting `config.toml`.
If it is omitted, the value in `config.toml` is used.

L1 adaptive augmentation is enabled by default with `controller.l1enable = 1`.
Use `--l1` or `--no-l1` to override it for one run:

```powershell
python .\mode29_mujoco.py `
  --config .\config.toml --l1 --loss-percent 60
```

The port contains the L1Quad state predictor, piecewise-constant matched and
unmatched uncertainty estimator, one-stage thrust low-pass filter and two-stage
moment low-pass filter. The final actuator request is the geometric-controller
command plus the filtered L1 adaptive augmentation. L1 diagnostics are written
to the `l1_*` and `total_*` columns in the CSV log. At an FDI allocator-mode
transition, the predictor is re-seeded and held synchronized for the configured
`topology_transition_hold_s` interval; `l1_transition_hold` records that state.

The current validated L1 fault case is the default 60% M1 loss. No-fault and
60% runs complete the full 40 s scenario. Exploratory 50% and 70% cases require
separate FDI-threshold/gain-anchor tuning and are not yet validated profiles.

### Fast headless validation

```powershell
python .\mode29_mujoco.py `
  --config .\config.toml --headless --no-realtime
```

Use `--duration 20` to temporarily override only the run duration. The overrides
can be combined, for example `--duration 20 --loss-percent 75`.

### Change the injected fault

Edit only `[fault_injection]` in `config.toml`:

```toml
[fault_injection]
enabled = true
motor_id = 1          # 1..4
loss_percent = 60.0   # thrust-effectiveness loss, 0..100
start_time_s = 16.0
duration_s = 8.0      # <=0 means until the simulation ends
blind_fdi = true
request_yaw_free = false
```

`start_time_s` must not be earlier than `takeoff_time_s + settle_time_s`.
Set `enabled = false` for the original no-fault baseline.

Gain scheduler modes under `[gain_schedule]`:

- `"disabled"`: always use the original `[gains]` and `[safety]` values;
- `"oracle"`: use the known injected loss to calibrate the 50..100% anchors;
- `"automatic"`: use only the blind-FDI estimate (default experiment mode).

### Log interpretation

The main result is `logs/mode29_sim.csv`. Important fields:

- `fault_active`, `fault_motor`, `fault_loss_pct`: injected plant fault;
- `fdi_state`: 0 normal, 1 candidate, 2 confirmed;
- `fdi_motor`, `fdi_loss_estimate_pct`, `fdi_residual_ratio`: blind FDI output;
- `allocator_mode`: full-wrench, fault-aware, or recovery allocation;
- `yaw_free_active`: fixed heading has been released;
- `schedule_loss_pct`, `active_kp*`, `active_kv*`, `active_kr*`, `active_ko*`:
  active scheduled controller values;
- `l1_sigma_*`: estimated matched/unmatched uncertainty;
- `l1_uad_*`: filtered L1 adaptive augmentation;
- `total_*`: geometric baseline plus L1 command sent to the allocator;
- `wcmd*`: allocator request, `wnom*`: lagged nominal motor state, `w*`:
  command after fault injection.

## L1Quad notice

This software uses or is derived from the L1Quad software developed by the
Department of Mechanical Science and Engineering at the University of Illinois
Urbana-Champaign. Upstream source and research-use license:
<https://github.com/ssybh2/L1Quad-original>.
