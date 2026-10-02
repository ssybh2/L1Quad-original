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
| RC11 | thrust-effectiveness loss | 1000=0%, 2000=30% |
| RC12 | yaw policy | 1000=keep yaw, 2000=yaw-free |

The matching firmware branch is:

`feature/softdrone-motor-degradation-yaw-free`

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

The existing Mode 29 baseline remains:

```text
orangepi/configs/mode29_baseline.yaml
```

The legacy Mode-29-only command is still supported:

```bash
python3 orangepi/apply_mode29_config.py \
  orangepi/configs/mode29_baseline.yaml \
  --dry-run
```

For new work, the unified tool can apply the same profile:

```bash
python3 orangepi/apply_flight_config.py \
  orangepi/configs/mode29_baseline.yaml \
  --dry-run
```

Mode 29 parameters currently supported by the unified tool are the
`GEOCTRL_KP*`, `GEOCTRL_KV*`, `GEOCTRL_KR*`, `GEOCTRL_KO*`,
`L1ENABLE`, `ASV`, `ASOMEGA`, and L1 cutoff parameters.

### Recommended local working files

Keep the two version-controlled baselines unchanged as recovery references:

```text
orangepi/configs/stabilize_pid_baseline.yaml
orangepi/configs/mode29_baseline.yaml
```

Create one working copy for each controller family when tuning:

```bash
cp orangepi/configs/stabilize_pid_baseline.yaml \
   orangepi/configs/stabilize_pid_tuning_v1.yaml

cp orangepi/configs/mode29_baseline.yaml \
   orangepi/configs/mode29_tuning_v1.yaml
```

This keeps the workflow simple: one file for normal ArduCopter/STABILIZE
tuning and one file for Mode 29 tuning. A combined duplicate profile is not
needed.

All of these values are runtime `AP_Param` values. Changing them through the
Orange Pi does **not** require a new firmware build or a new flash. The C++
values in `L1AC_customization/ArduCopter/config.h` remain firmware defaults,
not the normal tuning workflow.

## Flight sequence

1. Start the existing NOKOV bridge on `if02`.
2. Confirm ExternalNav/EKF is healthy.
3. Put the aircraft physically near the mocap `(0,0,0)` origin.
4. Arm with the normal RC procedure.
5. Use the RC flight-mode switch to enter Mode 29.
6. Mode 29 performs its existing 2 s takeoff from `(0,0,0)` to `(0,0,-1)`.
7. `TRAJINDEX=0` then holds `(0,0,-1)`.
8. The firmware refuses motor derating until Mode 29 has been active for at least 3 s.
9. Start the Orange Pi command.

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
- Start real-flight tests with a small loss, such as 5%, before larger values.
