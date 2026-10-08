# Mode29 MuJoCo: observer-gated opposite-motor spin / hover experiment

**This branch builds on your uploaded MuJoCo simulation without changing the upload branch.**

**Arbitrary motor-loss input:** `-LossPercent` accepts any finite percentage from
**0% through 100%**, including decimals such as 7.5%, 23.7%, and 92.3%.
The control scheme never treats 60% as a privileged failure magnitude.
0% means no injected failure; 100%+100% diagonal outage cannot provide three
independent F/Roll/Pitch inputs and is reported as infeasible, not ignored as
an invalid motor-loss command. Blind identification may fail for small faults
because the disturbance falls beneath the noise/observability floor.

Firmware research reference: `feature/mode29-pixhawk-next-from-20261006`
(base Oct 6 commit `2c606d7e`). This is a **simulation prototype**, not a
verified flight-control solution.

## Independently tune EACH 10% fault severity (including yaw-rate cap)

In `config_opposite_pair.toml`, every 0%, 10%, ..., 100% loss has an **independent**
`[gain_schedule.loss_N]` block. You can change `kp`, `kv`, `kr`,
`ko`, `max_tilt_deg`, and `max_yaw_rate_deg_s` separately at every
anchor without changing the others. None of the gain examples is a verified
optimum.

For example, the 90% tuning block may look like:

```toml
[gain_schedule.loss_90]
kp = [20.0, 20.0, 10.0]
kv = [8.0, 8.0, 2.0]
kr = [0.5, 0.5, 0.25]
ko = [1.5, 1.5, 0.1]
max_tilt_deg = 30.0
max_yaw_rate_deg_s = 180.0  # independently editable DEG/s soft limit
```

**The yaw limits belong to these exact same per-loss sections**, not a
single constant across all fault cases. Any intermediate loss gets
piecewise-linear interpolation. E.g. if the 30% and 40% speed limits are
80 and 120 deg/s, respectively, 37% uses 108 deg/s.
The gain interpolation is componentwise, e.g. `kp[0]` at 37% is
`0.3 * kp_30[0] + 0.7 * kp_40[0]`.

The default experiment config uses `[gain_schedule] mode="oracle"` for
repeatable **controller tuning** against a known injection. Later use
`mode="automatic"` to evaluate FDI-driven scheduling. This is a
different setting from `-Oracle` which uses ground truth to engage
the opposite-pair mechanism for simulation-only diagnosis.

The `[yaw_rate_schedule]` section tunes **nonzero spin target**
and anticipatory rotor-torque braking:
`target_spin_fraction`, `brake_start_fraction`,
`rate_gain_nm_per_rps`, `max_corrective_moment_nm`, and
`hard_abort_multiplier`. Example: at loss 90%, choose a ceiling of
180 deg/s and `target_spin_fraction=0.65` to **command about 117 deg/s
self-spin** (not a fixed heading). A larger rate drives accelerating motor
yaw torque, and overspeed drives braking torque *when available*.
Soft speed ceilings do **not guarantee** rate below the limit if motors
saturate or there is insufficient yaw torque authority. The allocator
preserves requested total thrust/Roll/Pitch where feasible before using
its remaining nullspace to regulate body yaw speed. The hard-abort envelope
ends synthetic mirroring if rate becomes much larger than its scheduled cap.
Neither mechanism artificially edits the simulated angular velocity.

**IMPORTANT:** The current Pixhawk6C **experimental firmware** does not yet
carry individually programmable AP_Param gains at 10/20/30/40%; the full
per-10% independent tuning interface in this branch applies to the
**MuJoCo configuration**. Do not apply these parameters to the real aircraft
until the firmware parameter interface and physical controller are separately
implemented and validated.

## Run the new experiment on Windows

From a checked-out copy of `feature/mujoco-opposite-pair-20261008`:

```powershell
python -m pip install -r requirements.txt
.\run.ps1 -Pair -LossPercent 60
```

Fast headless experiment:

```powershell
.\run.ps1 -Pair -LossPercent 60 -Headless -NoRealtime
```

**Important blind-estimator result:** Initial GitHub Actions smoke testing
finished successfully but `FDI confirmed at: not detected` and
`pair engaged at: never` for the 60% case. The first blind-mode run therefore
does **not** prove the new paired controller works. The observer estimate
was too weak or inconsistent to satisfy the original FDI evidence criteria.
The updated optional experiment removes any fixed percentage threshold,
though this does not automatically make the estimator accurate.

To test the *paired allocation and spinning physics separately*, you can
explicitly enable the SIMULATION-ONLY oracle comparison:

```powershell
.\run.ps1 -Pair -Oracle -LossPercent 60 -Headless -NoRealtime
```

The `-Oracle` switch substitutes the **known injected fault ID and percentage
for the paired controller only**. It is *not* blind identification, does not
validate the FDI and does not exist in the flight firmware. `-Pair` without
`-Oracle` keeps blind FDI as the only possible activation source.

To compare against your original single-motor recovery behavior, omit `-Pair`:

```powershell
.\run.ps1 -LossPercent 60
```

These run **different configs and different log CSV files**:
- `config.toml` defaults to `[opposite_pair] enabled=false` (your old behavior);
- `config_opposite_pair.toml` explicitly enables observer-gated pairing, retains
  pre-existing 0..100% gain interpolation anchors, and uses residual/moment
  observability plus persistence instead of a special FDI percentage threshold,
  releases yaw without null-space damping, and disables artificial yaw-rate clipping.

The experiment still runs the position loop against NED `(0,0,-1)` after
the normal takeoff and settle interval. The FDI first identifies the motor
and estimates its loss from measured motion, without reading injector truth.
It then reduces the opposite motor's thrust effectiveness by the **estimated**
percentage and allocates `F,Mx,My` using a dual-effectiveness matrix.

Motor map: `M1 <-> M2`, `M3 <-> M4`. Original injected loss is applied to
the selected motor; a separate synthetic derating is applied to the opposite motor.
Both are applied in **thrust space** after motor lag, not as a PWM fraction.

Important research limitations:
- Only engages after confirmed single-motor FDI and an 8 percentage-point
  controlled-experiment agreement gate. No artificial percentage ceiling;
  instead checks three-axis allocation rank/conditioning and thrust margin;
- Checks position error, yaw rate and a conservative static thrust margin;
- Freezes the single-fault estimate during the paired impairment (a
  single-fault signature is invalid once the second motor is derated);
- Does **not** guarantee hover or safe yaw under arbitrary fault severity.
  On a guard failure it disables artificial mirroring and records the reason,
  but the underlying injected failure can remain active.
- No Oct 7 Yaw-damping firmware parameters were copied into this design.
  Existing original simulation damping remains available only in baseline mode.

Diagnostic log: `logs/mode29_opposite_pair.csv`. Summarize it with:

```powershell
python .\analyze_pair_log.py .\logs\mode29_opposite_pair.csv
```

The summary reports FDI confirmation, paired time, guard exit reason,
position error and injection-relative FDI bias.

Diagnostic log: `logs/mode29_opposite_pair.csv`. New columns include
`pair_active`, `pair_inhibited`, `pair_failed_motor`,
`pair_opposite_motor`, `pair_loss_estimate_pct`,
`pair_static_margin`, `pair_estimate_bias_pp`,
`pair_xy_error_m`, and `pair_z_error_m`. Compare to the original
`fault_loss_pct`, `fdi_loss_estimate_pct`, `yaw_rate_rps`,
`allocator_mode`, `x,y,z`, `roll_deg`, and `pitch_deg`.

Validation:

```powershell
python -m unittest discover -s tests -p "test_*.py" -v
```

GitHub Actions runs syntax checks, mathematical unit tests and one
headless smoke simulation. That does not certify closed-loop stability.

---

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
