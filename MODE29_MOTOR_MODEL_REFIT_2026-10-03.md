# Mode29 Motor Model Refit — 2026-10-03

## Current firmware model: 6S PWM calibration

The current real-airframe Mode29 motor model is fitted from the newly supplied
**6S** single-motor thrust-stand staircase data recorded on 2026-10-03.

Hardware and command mode reported by the test stand:

- motor: 1405
- propeller: 3 inch
- ESC: 30 A
- throttle mode: **50 Hz PWM**
- command range: approximately **1000–1800 us**
- measured pack voltage across the two valid sweeps: **21.95–24.69 V**
- mean measured pack voltage across the fitted staircase points: **23.77 V**
- measured outputs: thrust `F(KG)` and reaction torque `T(N.M)`

`F(KG)` is converted to newtons with `9.80665 N/kgf`.

Two non-empty staircase exports are fitted together. Two additional staircase
exports in the archive contain headers but no samples and are not used.

For thrust, the near-zero samples are retained to identify the ESC/motor onset.
For reaction torque, one 1000-us sample reports a nonzero torque while RPM is
exactly zero; that zero-RPM sensor-offset point is excluded from the torque fit.

## 6S dead-zone cubic model

Mode29 uses

```text
w = (PWM_us - 1000) / 10

xF = max(0, w - 4.47703190)

F(w) = -2.62683159e-5 xF^3
       +4.01680390e-3 xF^2
       +4.05756758e-8 xF             [N]

xM = max(0, w - 6.24726216)

M(w) = -2.50608401e-7 xM^3
       +3.84208303e-5 xM^2
       +5.42809055e-4 xM             [N*m]
```

Fit quality over the supplied staircase data:

- thrust RMSE: **0.07037 N**
- thrust R²: **0.999679**
- reaction-torque RMSE over running-motor samples: **0.000898 N·m**
- reaction-torque R² over running-motor samples: **0.999660**

The fitted thrust and reaction-torque curves remain monotonic throughout the
Mode29 command interval `w=0..100`.

## Hover consistency check

For the current hard-coded vehicle mass `1.3854 kg`, equal-motor static hover
requires approximately

```text
F_per_motor = m*g/4 = 3.3965 N
w_hover     = 37.2884
PWM_hover   = 1372.9 us equivalent
```

This is consistent with the approximately 1390-us-equivalent motor command
observed in the recent 6S flight log, unlike the earlier 4S calibration.

## Firmware integration

The same 6S real-airframe model is shared by:

1. the normal four-channel F/Mx/My/Mz allocator;
2. the yaw-free F/Mx/My allocator used during motor-degradation tests;
3. the motor-degradation effectiveness mapping and inverse thrust lookup.

The normal mixer linearizes the nonlinear thrust and reaction-torque curves
about each motor command and performs two Newton-style refinement steps.

The inverse thrust map uses deterministic bisection. Although the thrust-stand
sweep is measured through about 1800 us, Mode29 keeps its existing 0..100
actuator command range and monotonically extrapolates the fitted curve above
the measured range when necessary.

At Mode29 entry the firmware emits:

```text
Mode29 motor model: 6S PWM fit 2026-10-03
```

so the flashed build can be distinguished from the earlier 4S model.

## Previous 4S calibration

The immediately preceding firmware commit used the 2026-09-26 4S data:

```text
xF = max(0, w - 4.75)
F(w) = -7.02276361e-6 xF^3
       +1.65815719e-3 xF^2
       +7.27527105e-5 xF

xM = max(0, w - 6.20)
M(w) = -6.18352630e-8 xM^3
       +1.56009927e-5 xM^2
       +3.50634715e-4 xM
```

That 4S model predicted an equal-motor hover point near 1558 us and therefore
must not be used as the direct static command-to-thrust model for the current
6S flight configuration.

## Limitation

This remains a static PWM-command model. Battery state, ESC temperature,
propeller inflow, dynamic flight conditions, and motor-to-motor variation are
not separately modeled. The 2026-10-03 staircase itself spans substantial
battery sag, so the fitted curve is representative of that particular 6S
test sequence rather than a voltage-normalized motor constant.
