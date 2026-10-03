# Mode29 Motor Model Refit — 2026-10-03

## Source data

The refit uses the two repeated staircase exports supplied from the single-motor thrust stand on 2026-09-26:

- motor: 1405
- propeller: 3 inch
- ESC: 30 A
- throttle mode recorded by the stand: **50 Hz PWM**
- fitted range: approximately **1050–1800 us**
- measured outputs: thrust `F(KG)` and reaction torque `T(N.M)`

`F(KG)` was converted to newtons using `9.80665 N/kgf`.

The two staircase repeats were fitted together.  The near-zero points are retained to estimate the motor/ESC dead-zone.

## Previous model

The previous real-airframe model was

```text
w = (PWM_us - 1000) / 10

F(w) = 0.000968094 w^2 + 0.004763730 w
M(w) = 0.0000107130307 w^2 + 0.000243044484 w
```

Re-fitting the supplied data with the **same zero-intercept quadratic structure reproduces those coefficients essentially exactly**.  Therefore simply running the old quadratic fit again would not change the firmware.

Across the two staircase repeats (1050–1800 us):

- thrust RMSE: **0.09566 N**, R² **0.997886**
- reaction-torque RMSE: **0.001341 N·m**, R² **0.997619**

## Updated dead-zone cubic model

To improve the static fit while keeping zero output below the motor/ESC onset, Mode29 now uses

```text
w = (PWM_us - 1000) / 10

xF = max(0, w - 4.75)
F(w) = -7.02276361e-6 xF^3
       +1.65815719e-3 xF^2
       +7.27527105e-5 xF

xM = max(0, w - 6.20)
M(w) = -6.18352630e-8 xM^3
       +1.56009927e-5 xM^2
       +3.50634715e-4 xM
```

Across the same two staircase repeats:

- thrust RMSE: **0.03079 N**, R² **0.999781**
- reaction-torque RMSE: **0.000573 N·m**, R² **0.999566**

The fitted curves are monotonic over the validated operating range.

For the current hard-coded vehicle mass `1.3854 kg`, the static equal-motor hover solution changes from approximately

```text
old quadratic: w = 56.80  -> 1568 us equivalent
new cubic:     w = 55.83  -> 1558 us equivalent
```

before attitude/moment corrections.

## Firmware integration

The same real-airframe model is now used consistently by:

1. the normal four-channel F/Mx/My/Mz allocator;
2. the yaw-free F/Mx/My allocator used during motor-degradation tests;
3. the degradation effectiveness mapping.

The full mixer linearizes the cubic thrust and torque curves about each motor command and performs two Newton-style allocation refinements, matching the previous iterative architecture.

## Important limitation

The thrust-stand files explicitly report **50 Hz PWM**.  If the aircraft is flown with DShot, this fit is a model of the tested PWM-command operating point, not a direct DShot thrust calibration.  A dedicated DShot600 thrust-stand sweep would be the correct next calibration if command-to-thrust mismatch remains in flight.
