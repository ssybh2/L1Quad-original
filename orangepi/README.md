# Orange Pi motor-derating controller

This directory is the Orange Pi side of the Softdrone Mode 29 motor-degradation experiment.

## Link assignment

- Pixhawk USB `if02`: existing NOKOV -> MAVLink ODOMETRY bridge.
- Pixhawk USB `if00`: this motor-derating controller.
- QGroundControl: preferably over the Wi-Fi telemetry link.
- Physical RC: arming/disarming and flight-mode selection.

The controller deliberately overrides only RC channels 9..12 through MAVLink2 `RC_CHANNELS_OVERRIDE`. Channels 1..8 remain untouched.

| Channel | Meaning | Encoding |
| --- | --- | --- |
| RC9 | derating enable | 1000=off, 2000=on |
| RC10 | motor id | M1=1125, M2=1375, M3=1625, M4=1875 |
| RC11 | thrust-effectiveness loss | 1000=0%, 2000=100% |
| RC12 | yaw policy | 1000=keep yaw, 2000=yaw-free |

The matching **2026-10-08 experimental firmware** branch is:

`feature/mode29-pixhawk-next-from-20261006`

The immutable 2026-10-06 tested baseline remains:
`milestone/orangepi-mode29-20261006-no-yaw-protection`
(commit `2c606d7e532c54416fc850995449b9acfd6f6a1b`).

## Opposite-motor matching experiment (simulator / propellers removed first)

This experimental feature is disabled in both existing ordinary YAML profiles:
`M29_PAIR_EN: 0`. It requires the matching new Pixhawk firmware.

A **separate**, explicitly opt-in profile is:
`orangepi/configs/mode29_opposite_pair_experiment.yaml`.
It sets `M29_PAIR_EN: 1` and retains the 2026-10-06 60% gain
anchor. The mode setting `M29_GS_MODE: 1` (injected-loss oracle
for **gain scheduling only**) keeps the FDI motor and percentage identification
fully independent; switch to `2` only in later validated simulations.
The fault detector must still confirm the fault before any opposite motor
is derated.

Review the changes **while disarmed and propellers removed**:

```bash
python3 orangepi/apply_flight_config.py \
  orangepi/configs/mode29_opposite_pair_experiment.yaml --dry-run
```

Do NOT apply this experimental profile to a free-flying aircraft based
only on the repository build. Pair activation is guarded and can
abort, but loss of position and uncontrolled spin remain possible.
The normal/validated profile explicitly sets `M29_PAIR_EN=0`.

Diagnose test logs `L1DG` (injection truth), `L1FD` (observer),
`L1GS` (scheduled gain), and `L1PR` (pair state and position/yaw
errors). This experiment does not include any of the October 7
`M29_YAW_KD`, `M29_YAW_RMAX`, or `M29_YAW_MMAX` changes.



The runtime motor-loss command accepts **any finite value from 0% to
100%**, including decimals such as 7.5%, 23.7% and 91.3%. RC11 maps
linearly across that full range; 5% -> 1050 us, 50% -> 1500 us,
100% -> 2000 us. The percentage is a modeled thrust-effectiveness loss, not a
raw PWM percentage. **60% has no special status in the opt-in opposite-pair
experiment.** The 50/60/.../100 gain anchors are interpolation points, not
a list of permitted injections. The paired feature may reject activation
when observer confidence or effective actuator control rank is insufficient;
in particular, 100% paired loss leaves only two effective motors and cannot
independently realize total thrust, roll and pitch.

The original non-paired `M29_PAIR_EN=0` detector keeps the 2026-10-06
flight-test thresholds. Do not assume detecting arbitrarily small faults
is possible with a noisy IMU, nor interpret 0% injection as a motor failure.

## Install

```bash
python3 -m pip install --user -r orangepi/requirements.txt
```

## Required Pixhawk setup

Before enabling derating, set:

```text
TRAJINDEX = 0
LANDFLAG = 0
RC_OVERRIDE_TIME = 0.5

RC9_OPTION  = 0
RC10_OPTION = 0
RC11_OPTION = 0
RC12_OPTION = 0
```

Also keep `FLTMODE_CH` away from RC9..RC12; RC5 is the intended flight-mode channel in the example below.

`RC_OVERRIDE_TIME=0.5` makes the override disappear about 500 ms after the Orange Pi stops sending.

Do not assign RC9..RC12 to other flight-critical functions for this experiment.

## Check the link and relevant parameters

```bash
python3 orangepi/motor_derating_control.py status
```

Expected items include:

```text
RC_OVERRIDE_TIME=0.5
TRAJINDEX=0
LANDFLAG=0
L1ENABLE=...
```

## Runtime controller tuning from the Orange Pi

Both controller families can now be changed through MAVLink `AP_Param` values
without rebuilding or reflashing the firmware.

There are two distinct control stacks:

- ArduCopter `ATC_*` parameters tune the normal ArduCopter attitude/rate
  controller used by STABILIZE and related normal flight modes.
- `GEOCTRL_*` and the L1 parameters tune the custom Mode 29 controller.

Mode 29 directly computes geometric-control moments and performs its own motor
allocation, so its primary controller does **not** reuse
`ATC_RAT_*` / `ATC_ANG_*` gains. The unified Orange Pi tool supports both
families in one workflow because it is useful to version, compare, back up, and
restore them together.

The unified tool is:

```text
orangepi/apply_flight_config.py
```

It intentionally:

- refuses to write while the vehicle is ARMED;
- accepts only an explicit controller-parameter whitelist;
- rejects NaN/Inf values;
- reads all current values before changing anything;
- saves an automatic pre-change backup under `~/flight_param_backups/`;
- writes one parameter at a time and verifies the Pixhawk readback.

### Current STABILIZE/manual baseline

The 2026-10-02 values captured from the aircraft are stored in:

```text
orangepi/configs/stabilize_pid_baseline.yaml
```

To make an experiment profile:

```bash
cp orangepi/configs/stabilize_pid_baseline.yaml \
   orangepi/configs/stabilize_pid_tuning_v1.yaml
```

Edit only the parameter(s) being tested, preview the changes:

```bash
python3 orangepi/apply_flight_config.py \
  orangepi/configs/stabilize_pid_tuning_v1.yaml \
  --dry-run
```

Then apply them while **DISARMED**:

```bash
python3 orangepi/apply_flight_config.py \
  orangepi/configs/stabilize_pid_tuning_v1.yaml
```

The supported normal-flight controller parameters include the Roll/Pitch/Yaw
angle gains, Roll/Pitch/Yaw rate P/I/D/FF gains, and `ATC_INPUT_TC`.

### Mode 29 runtime tuning

There is exactly one version-controlled Mode 29 configuration file:

```text
orangepi/configs/mode29_tuning.yaml
```

Edit this file for all Mode 29 runtime tuning. Do not create separate
`mode29_baseline.yaml`, `mode29_tuning_v1.yaml`, or other duplicate Mode 29
profiles during normal testing.

Preview changes while the Pixhawk is DISARMED:

```bash
python3 orangepi/apply_flight_config.py \
  orangepi/configs/mode29_tuning.yaml \
  --dry-run
```

Apply the profile while DISARMED:

```bash
python3 orangepi/apply_flight_config.py \
  orangepi/configs/mode29_tuning.yaml
```

Mode 29 parameters supported by the unified tool include
`GEOCTRL_KP*`, `GEOCTRL_KV*`, `GEOCTRL_KR*`, `GEOCTRL_KO*`,
`L1ENABLE`, `ASV`, `ASOMEGA`, the L1 cutoff parameters, and the runtime
trajectory parameters `M29_TKOFF_ALT`, `M29_TKOFF_T`, and
`M29_SETTLE_T`.

The takeoff trajectory uses a normalized seventh-order smoothstep scaled by
`M29_TKOFF_ALT` and `M29_TKOFF_T`. `M29_SETTLE_T` controls the extra
hover time before motor-degradation injection becomes eligible.

These values are runtime `AP_Param` values. Changing them through the Orange
Pi does **not** require rebuilding or reflashing the firmware. The C++ values
in `L1AC_customization/ArduCopter/config.h` are firmware defaults rather than
the normal tuning workflow.

## Flight sequence

1. Start the existing NOKOV bridge on `if02`.
2. Confirm ExternalNav/EKF is healthy.
3. Put the aircraft physically near the mocap `(0,0,0)` origin.
4. Arm with the normal RC procedure.
5. Use the RC flight-mode switch to enter Mode 29.
6. Mode 29 performs a smooth takeoff using `M29_TKOFF_ALT` and `M29_TKOFF_T`.
7. `TRAJINDEX=0` then holds `z=-M29_TKOFF_ALT` at the NED origin.
8. Motor derating is refused until `M29_TKOFF_T + M29_SETTLE_T` has elapsed.
9. Start the Orange Pi fault command only after the aircraft has visibly settled.

Example: Motor 1 has a 20% thrust-effectiveness loss for 5 s and yaw is released:

```bash
python3 orangepi/motor_derating_control.py enable \
  --motor 1 \
  --loss 20 \
  --duration 5
```

Keep fixed-yaw control instead:

```bash
python3 orangepi/motor_derating_control.py enable \
  --motor 1 \
  --loss 20 \
  --duration 5 \
  --keep-yaw
```

Blind onboard-FDI experiment: the injector still applies the selected motor-loss
ground truth, but it does **not** request yaw-free. The Mode29 detector must
identify a severe motor fault from the L1 matched-moment estimate and switch to
automatic yaw-free/fault-aware allocation itself:

```bash
python3 orangepi/motor_derating_control.py enable \
  --motor 1 \
  --loss 80 \
  --duration 3 \
  --blind
```

The current onboard detector is intentionally conservative: it is aimed at
severe loss and requires a consistent motor signature for about 60 ms before
confirmation. After confirmation, Mode29 no longer forces the isolated motor
to zero unless the estimated loss is actually 100%. Instead, the reduced-
attitude allocator uses the estimated effectiveness of that motor together
with the three healthy motors.

The confirmed loss estimate continues adapting online. If the actuator
recovers and the estimated loss remains below 35% for about 100 ms, that motor
automatically rejoins the normal four-motor yaw-free allocator. Yaw remains
released for the rest of the current Mode29 run to avoid an abrupt heading
recapture while the vehicle may still be spinning; exiting Mode29 or disarming
clears that latch.

DataFlash `L1FD` records detector state, isolated motor, candidate,
confirmation count, recovery count, estimated loss, residual score, and the
three matched-moment estimates.

Immediately release RC9..RC12 overrides:

```bash
python3 orangepi/motor_derating_control.py disable
```

The script also releases all four override channels when it exits normally, on Ctrl+C, or after its requested duration.

## RC Mode 29 switch

The current Softdrone firmware already exposes flight mode `29: ADAPTIVE` through `FLTMODE1..FLTMODE6`.

For a common three-position switch on RC5:

```text
FLTMODE_CH = 5

LOW  (~1000 us) -> FLTMODE1
MID  (~1500 us) -> FLTMODE4
HIGH (~2000 us) -> FLTMODE6
```

One useful setup is:

```text
FLTMODE1 = 0    # Stabilize
FLTMODE4 = 5    # Loiter
FLTMODE6 = 29   # Adaptive
```

Verify the actual RC5 PWM values in QGC before relying on these positions.

## Safety / first test

For the first validation keep props removed. Confirm:

- RC mode switch really reaches custom mode 29.
- `if02` ODOMETRY remains healthy while `if00` runs this script.
- RC9..RC12 appear only while the script is active.
- Motor ID 1..4 matches the physical Mode 29 motor order.
- Leaving Mode 29 or stopping the script removes the derating command.
- Validate each new loss level progressively and review the DataFlash L1DG record before increasing it further.
