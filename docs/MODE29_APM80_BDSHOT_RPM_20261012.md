# Mode29 APM 80% recovery: exact-source bidirectional DShot RPM port (bench only)

**NOT FLIGHT QUALIFIED: propellers removed / unpowered HIL only.**

## Original release provenance

- User-designated release: [80% APM recover the nomianl cmd](https://github.com/ssybh2/L1Quad-original/releases/tag/pair).
- Exact source branch found inside the user's original firmware `BUILD_INFO.txt`: `feature/mode29-fdi-antiwindup-hil-20261008`.
- **Exact original source commit**: `f3f12d901e82c779cd842246556091ba21492838`.
- This RPM development branch starts from that exact commit. The older prototype branch started from `f31362c`, which is NOT the same release source and is superseded by this branch.
- The GitHub release tag itself points to `main`, so trace source origin from the firmware ZIP's BUILD_INFO rather than its tag.

## What is retained

Original geometric controller, L1 adaptive compensation, FDI, bounded allocator, optional opposite-pair derating, staged *software-injected* failure release, ESC motor command path `motors->rc_write()`, and safety protections are unchanged.

In particular, the original Mode29 FDI anti-windup keeps **125ms observation settling after confirmation**, gate checks for saturated allocation / weak excitation / poor fit, **maximum +30 percentage points/s loss-estimate rise**, and the `L1FI` fault diagnostic log. The firmware's staged release is *not validated for naturally recovered real-world physical failures*.

## Added code

- Compile the existing ArduPilot submodule as `Pixhawk6C-bdshot` instead of the original `Pixhawk6C`. This enables the bidirectional-DShot build variant, but the user's saved runtime parameters still need to be changed and validated separately.
- Add a **read-only** `L1RP` log in Mode29 at approximately 20Hz, format `Ms,R1,R2,R3,R4,Val`. `R1..R4` correspond to the **physical FMU PWM OUT 1–4** ports, not the logical Mode29 motor order. If fresh RPM unavailable, value=-1; `Val` bitmask encodes valid channels (bits 0..3).
- No RPM enters FDI, L1, motor allocation or fault recovery. The logger will never shut down a motor when telemetry is missing.
- A 20Hz DataFlash diagnostic log is NOT proof of full-rate RPM data suitability for FDI; measure latency and lost frames separately.

## User's existing physical wiring and motor allocation: DO NOT REORDER

| Physical FMU output | ArduPilot output | SERVO function | Logical motor |
|---|---|---|---|
| FMU1 | SERVO9 | 36 | M4 |
| FMU2 | SERVO10 | 34 | M2 |
| FMU3 | SERVO11 | 35 | M3 |
| FMU4 | SERVO12 | 33 | M1 |

Thus, with the unchanged mapping: `L1RP.R1=M4`, `R2=M2`, `R3=M3`, `R4=M1`. Confirm with no propellers before interpreting logs.

## No-propeller test parameters

1. Back up all Mission Planner parameters and retain the release's original known-good binary.
2. Keep all four ESC signal wires at FMU PWM OUT1–4; DO NOT move to I/O PWM OUT.
3. Flash the new firmware artifact only after GitHub Actions succeeds; confirm that `SERVO_BLH_BDMASK` appears after firmware change.
4. With a physically safe no-propeller bench and motor-order validation, test `MOT_PWM_TYPE=5` (DShot300), `SERVO_BLH_BDMASK=3840` (SERVO9–12), `SERVO_DSHOT_ESC=1` (BLHeli_32). `SERVO_BLH_AUTO=1` is optional. `SERVO_BLH_POLES` must match the actual motor pole count; 14 is only a default.
5. Existing `SERVO_BLH_RVMASK=1024` refers to physical FMU OUT3 (M3). **Switching from PWM to DShot changes the direction command path**. Verify motor direction, stop/disarm behavior and output ordering with props removed. `SERVO_BLH_3DMASK` is for reversible ESC control, not required merely to get RPM.
6. Read standard `ESCn` and custom `L1RP` logs. Confirm `Val=15` when all four RPM streams are received and each R value matches the correct physical motor.

## Critical calibration warning

The original Mode29 thrust and reaction-torque fit `F(w),M(w)` used a previous PWM ESC protocol. DShot input command may produce a different RPM/thrust curve. The earlier 80% recovery result **does not transfer automatically** to DShot300. Repeat motor command/RPM/thrust calibration, 400Hz timing audit, and HIL fault/recovery tests before any flight with propellers attached.

## Build

New isolated workflow: `.github/workflows/build-mode29-apm80-rpm-bdshot.yml`. It runs the inherited anti-windup regression tests and generates an `arducopter.apj`, `arducopter.bin`, `BUILD_INFO.txt` package on successful compilation. A passing CI run is not a flight safety clearance.
