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
python3 -m pip install --user pymavlink pyserial
```

## Required Pixhawk setup

Before enabling derating, set:

```text
TRAJINDEX = 0
LANDFLAG = 0
RC_OVERRIDE_TIME = 0.5
```

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
