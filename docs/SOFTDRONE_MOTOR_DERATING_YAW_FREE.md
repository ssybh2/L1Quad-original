# Softdrone Mode 29 motor derating + yaw-free recovery

This branch is based on `softdrone` and adds a runtime motor-derating experiment path without changing the existing NOKOV/ExternalNav setup.

Branch: `feature/softdrone-motor-degradation-yaw-free`

## Mode 29 behavior

- `TRAJINDEX=0` is now an explicit 1 m hover target.
- On Mode 29 entry, the existing takeoff trajectory is preserved: `(0,0,0) -> (0,0,-1)` over the first 2 s.
- After 2 s, the target stays at `(0,0,-1)` with zero target velocity/acceleration.
- Motor derating cannot become active before 3 s in Mode 29.

## Orange Pi runtime command

The firmware reads MAVLink2 `RC_CHANNELS_OVERRIDE` on RC9..RC12. The normal pilot channels 1..8 are not overridden.

| Channel | Meaning | Encoding |
| --- | --- | --- |
| RC9 | enable | 1000=off, 2000=on |
| RC10 | motor | <1250=M1, 1250..1499=M2, 1500..1749=M3, >=1750=M4 |
| RC11 | thrust loss | 1000=0%, 2000=30% |
| RC12 | yaw policy | 1000=fixed yaw, 2000=yaw-free |

All four channels must have an active MAVLink override. If any override disappears, Mode 29 clears the derating state.

Set `RC_OVERRIDE_TIME=0.5` so a stopped Orange Pi stream expires in about 500 ms.

## Derating model

The injected percentage is treated as a thrust-effectiveness loss, not a raw PWM percentage.

For the real Softdrone motor fit:

`F(w) = 0.000968094*w^2 + 0.004763730*w`

with `w=(PWM_us-1000)/10`.

For example, a 20% loss requests an applied thrust of `0.8 * F_nominal` and inverts the fitted thrust curve to obtain the lower actuator command.

## Yaw-free mode

When RC12 requests yaw-free mode and the derating command is active:

- the position target remains `(0,0,-1)`;
- yaw is still measured by EKF and logged;
- the desired heading is continuously aligned to the measured heading instead of forcing yaw back to zero;
- geometric-controller yaw torque is forced to zero;
- L1 adaptive yaw torque is forced to zero;
- a reduced-attitude allocator controls only total thrust, roll moment and pitch moment;
- no yaw-moment equation is imposed, so yaw is allowed to drift/spin while the controller prioritizes position and thrust-vector control.

The yaw-free allocator uses the three constraints:

`F  = f1 + f2 + f3 + f4`

`Mx = L/2 * (-f1 + f2 + f3 - f4)`

`My = D/2 * ( f1 - f2 + f3 - f4)`

and maps the resulting individual thrusts back through the motor thrust curve.

## Logging

`L1DG` records:

- active state;
- yaw-free state;
- selected motor;
- requested loss percentage;
- command age;
- commanded motor values `c1..c4`;
- applied motor values `a1..a4` after derating.

Existing `L1AC`, `L1AB`, `L1AD`, and `L1AE` logs remain available for position, baseline control, predictor state and L1 adaptive terms.

## Build

After pulling the branch, apply the customized ArduCopter files to the pinned ArduPilot submodule using the same customization copy step already used by this repository, then build for Pixhawk6C:

```bash
git switch feature/softdrone-motor-degradation-yaw-free
git submodule update --init --recursive

cp L1AC_customization/ArduCopter/ACRL_trajectories.cpp \
   L1AC_customization/ArduCopter/ACRL_trajectories.h \
   L1AC_customization/ArduCopter/mode_adaptive.cpp \
   L1AC_customization/ArduCopter/Copter.h \
   L1AC_customization/ArduCopter/Parameters.cpp \
   L1AC_customization/ArduCopter/Parameters.h \
   L1AC_customization/ArduCopter/config.h \
   L1AC_customization/ArduCopter/mode.cpp \
   L1AC_customization/ArduCopter/mode.h \
   L1AC_customization/ArduCopter/motors.cpp \
   ardupilot/ArduCopter/

cd ardupilot
./waf configure --board Pixhawk6C
./waf copter
```

## RC Mode 29 switch

The existing firmware parameter metadata already includes `29: ADAPTIVE` in `FLTMODE1..FLTMODE6`.

For a typical 3-position flight-mode switch on RC5:

```text
FLTMODE_CH = 5
FLTMODE1   = 0     # Stabilize, ~1000 us
FLTMODE4   = 5     # Loiter,    ~1500 us
FLTMODE6   = 29    # Adaptive,  ~2000 us
```

Verify the actual switch PWM values in QGC before flight.

## First validation

Do the first validation with props removed. Confirm the actual physical mapping of Mode 29 motor IDs 1..4 before any powered degradation test.
