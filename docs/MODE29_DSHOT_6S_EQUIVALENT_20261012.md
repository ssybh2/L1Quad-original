# Mode29 6S DShot-equivalent model — 2026-10-12 (DIAGNOSTIC ONLY)

> **NO PHYSICAL DSHOT THRUST DATA. NOT VERIFIED AT 4S. NO PROPELLER-ON FLIGHT OR MOTOR-FAULT INJECTION.**
>
> This change does **not** turn a 50-Hz PWM thrust dataset into a DShot experiment. It simply supplies an exact mathematical re-expression of the established *6S PWM* model using the ArduPilot 4.4 DShot command coordinate.

## Parent and unchanged control

- Parent: `feature/mode29-apm80-recover-bdshot-rpm-20261012` at `e23cc9da2dee5bf91c3665916aba5b5d475e148b`, descended from original 80% APM recover release BUILD_INFO commit `f3f12d901e82c779cd842246556091ba21492838`.
- Mode29 125ms FDI anti-windup settling, 30pp/s rise cap, bounded allocation, L1 and software-injected recovery are **not changed**. Actual PWM/DShot switching remains governed by the user's runtime `MOT_PWM_TYPE`.
- All original `F(w)` and `M(w)` coefficients, forward model, inverse model and control allocation are retained, with **no change to the actuator command**. This is deliberate: unvalidated DShot data must not be silently treated as measured thrust.

## Bench source data

- Source: `soft(3).zip`: two valid 2026-10-03 *50Hz-PWM* staircase sweeps, 17 distinct throttle points in each, 34 observations.
- Data includes PWM, RPM, thrust (kgf), reaction moment (N·m), pack voltage and current. The motor/prop/ESC names in archive filenames are inaccurate; **do not use them as hardware identification**.
- Tested voltage range: **21.95–24.69 V (6S)**; mean about **23.77 V**. NOT valid for the intended 4S power supply without another test.
- Tested PWM: **1000–1800 µs**. This covers ArduPilot equivalent DShot throttle **D=0–1648**, not D=2047. The `Ex` mask in logs exposes predictions beyond the actual measurement range.
- Fitting the existing underlying cubic to the 34 measurements yields R²=0.999679 and RMSE=0.0704N for thrust. Excluding the physically invalid 1000-us stationary torque offset, reaction moment yields R²=0.999660 and RMSE=0.000898 N·m.
- High fit R² demonstrates a good match under original PWM 6S test conditions **only**. It says nothing about DShot matching, 4S behavior, response delay, rotor lag or in-flight thrust.

## ArduPilot 4.4 non-3D DShot mapping

Current Mode29 issues `virtual_pwm = 1000+10*w`, with `w in [0,100]`. Its DShot driver applies integer PWM quantization and for armed non-3D outputs:

```
D = 0                               if virtual_pwm == 1000
D = min(2047, 48+2*(virtual_pwm-1000))   otherwise
w_ideal = (D-48)/20                 for D>=48
```

In continuous DShot coordinates, the original model transforms exactly to:

```
XF = max(0, D - 137.5406380)
F_proxy(D) = max(0, -3.2835394875e-9*XF^3
                     +1.0042009750e-5*XF^2
                     +2.0287837900e-9*XF)      [N]

XM = max(0, D - 172.9452432)
M_proxy(D) = max(0, -3.1326050125e-11*XM^3
                     +9.6052075750e-8*XM^2
                     +2.7140452750e-5*XM)      [N*m]
```

These are coordinate-transformed 6S PWM polynomials, **not measured DShot coefficients**. They ignore battery-voltage dependence because the staircase loads and voltage sag are correlated: those data cannot independently identify a voltage effect. Never extrapolate DShot >1648 as a verified model.

## New diagnostic logging

- `L1RP`: existing 20Hz measured RPM from physical FMU OUT 1–4; original schema unchanged.
- `L1DM`: new 20Hz **predicted** DShot-equivalent output diagnostics:
  `Ms,D1,D2,D3,D4,F1,F2,F3,F4,T1,T2,T3,T4,Ex`.
- `D1..D4` are *pre-ESC throttle request values* in **logical Mode29 motor order** (M1..M4), calculated from `motorPWM[]` and the ArduPilot DShot conversion. They do NOT guarantee actual wire frames or confirm `MOT_PWM_TYPE` is set to DShot.
- `F1..F4` and `T1..T4` are **model-predicted** thrust / reaction-torque magnitudes, not ESC measurements. `Ex` is the bitmask of motors with proxy throttle D>1648 (outside the measured envelope).
- Real RPM in `L1RP` is in **physical FMU-port order**. Current parameter mapping is SERVO9=36(M4), SERVO10=34(M2), SERVO11=35(M3), SERVO12=33(M1); align carefully before plots.
- No model-predicted F or T is used by FDI, actuator allocator or L1, so missing telemetry does not change motor output.

## Bench-only Mission Planner settings

- Back up original parameters, original 80% APM firmware and existing PWM fit.
- Keep existing FMU OUT1–4 cabling and `SERVO9/10/11/12_FUNCTION=36/34/35/33`.
- In propeller-removed testing only, set `MOT_PWM_TYPE=5` (DShot300), `SERVO_BLH_BDMASK=3840`, `SERVO_DSHOT_ESC=1`, and verify the motor has the actual `SERVO_BLH_POLES` number (do not assume 14). Restart and verify DShot activation plus four RPM streams.
- Existing `SERVO_BLH_RVMASK=1024` affects physical FMU OUT3 / logical M3. Verify motor direction and safe stop before applying any throttle, and distinguish 3D Bidirectional ESC *motor direction mode* from bidirectional DShot *RPM feedback*.
- For normal PWM tests, retain `MOT_PWM_TYPE=0` and old thrust calibration; with PWM, no bidirectional DShot RPM feedback is expected.
- Obtain actual **4S DShot300** stepped measurements including ESC RPM, thrust, reaction torque, voltage and settling time, then replace the proxy with a directly measured DShot model and revalidate fault control in HIL before considering flight.

Build workflow `.github/workflows/build-mode29-apm80-dshot-proxy.yml` runs original APM80 test contracts plus strict numeric coordinate-equivalence checks. A green build confirms compilation and arithmetic consistency, NOT physical DShot thrust calibration.
