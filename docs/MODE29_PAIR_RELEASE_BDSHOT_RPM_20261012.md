# Mode29 paired-recovery baseline: Pixhawk6C Mini BLHeli_32 bidirectional DShot RPM bench port

**BENCH / UNPOWERED HIL ONLY. NOT FLIGHT VALIDATED. NEVER power propellers based solely on a successful compile.**

## Exact source provenance and release limitation

- User-named release: [80% APM recover the nomianl cmd](https://github.com/ssybh2/L1Quad-original/releases/tag/pair) (tag `pair`).
- Asset: `mode29-bounded-hil-only-pixhawk6c-firmware.3.zip`, digest SHA256 `6398ce78762ddf04d8504c280ad16253646de3d9d289c5533653f7f6925445f9`.
- Its release tag `pair` points to `18664bffabf95a770a40c13833228d66c1ada00d` on `main`, not an identifiable build-source commit. **Thus the exact source SHA of the release asset remains unverified.**
- This development branch was deliberately created from matching artifact-producing workflow source branch `feature/mode29-bounded-hil-20261008`, at commit `f31362c115739def9f3062a003d67e9c2f7d83b9`.
- The source's workflow builds an artifact named `mode29-bounded-hil-only-pixhawk6c-firmware` and explicitly contains bounded allocation, opt-in opposite-pair motor derating, and staged recovery. This establishes a strong *matching family*, NOT binary identity with the downloadable release.
- To establish exact identity, read `BUILD_INFO.txt` **inside the original release ZIP**, and compare its `git_commit` with the base commit above. If different, rebase/cherry-pick onto the original identified commit before judging 80% recovery equivalence.

## What this change does and does NOT do

- Adds a second GitHub Actions workflow to compile the existing custom Mode29 controller using the existing ArduPilot 4.4.4 submodule's `Pixhawk6C-bdshot` board definition. It does **not** upgrade ArduPilot or alter `ardupilot` submodule SHA.
- Adds **read-only** `L1RP` DataFlash log at approximately 20 Hz using `AP::esc_telem().get_raw_rpm(0..3)`. Each record has `Ms,R1,R2,R3,R4,Val`; R1..R4 are **FMU PWM OUT / AUX 1..4 physical channels**, in physical mechanical RPM. `Val` is bitmask (bit 0=FMU1 through bit 3=FMU4); invalid/stale reads are -1.
- Keeps `ModeAdaptive::run()` motor command path, L1, original FDI, paired-derating and staged recovery logic unchanged. No RPM goes into FDI or motor allocation yet.
- Retains baseline workflow `build-mode29-bounded-hil.yml` unchanged.

## Actual user motor wiring and mapping — do not reassign without tracing motor direction

| FMU connector | ArduPilot output | SERVOx_FUNCTION | Motor identity |
|---|---|---|---|
| FMU OUT 1 (AUX1) | SERVO9 | 36 | Motor4 |
| FMU OUT 2 (AUX2) | SERVO10 | 34 | Motor2 |
| FMU OUT 3 (AUX3) | SERVO11 | 35 | Motor3 |
| FMU OUT 4 (AUX4) | SERVO12 | 33 | Motor1 |

Therefore `L1RP.R1=M4`, `R2=M2`, `R3=M3`, `R4=M1`, **only while these user-provided SERVO function values remain unchanged**. All values must be independently checked with physical motor tests before use.

## Configuration for PROPELLER-REMOVED bench validation only

1. Back up full current parameters in Mission Planner and retain the original working firmware ZIP.
2. Leave FMU/AUX 1..4 signal wires in place; do not move them to I/O / MAIN.
3. Install only the clearly labeled experimental `Pixhawk6C-bdshot` artifact from this branch after CI succeeds.
4. Confirm `SERVO_BLH_BDMASK` exists after flashing, and ensure correct `SERVO9_FUNCTION=36`, `SERVO10_FUNCTION=34`, `SERVO11_FUNCTION=35`, `SERVO12_FUNCTION=33` under the user's verified wiring. Review the independent MAIN-output functions to prevent duplicate motor outputs.
5. **Only with propellers removed**: test `MOT_PWM_TYPE=5` (DShot300), `SERVO_BLH_BDMASK=3840` (FMU AUX1..4), `SERVO_DSHOT_ESC=1` (BLHeli_32), and `SERVO_BLH_AUTO=1`. `SERVO_BLH_POLES=14` ONLY if the motor really has 14 magnetic poles. Avoid setting `SERVO_BLH_3DMASK` for RPM telemetry.
6. **Important:** existing `SERVO_BLH_RVMASK=1024` applies DShot reverse to physical FMU OUT 3 (the user's Motor3). Verify ESC direction and conventional motor spin before any DShot motor test. Do not overwrite this mask blindly.
7. With no propellers, independently test one motor at a time in a constrained setup and inspect DataFlash `L1RP` and standard `ESCn` records. Require `Val=15` when all four streams are fresh, and check whether `R1..R4` match the observed motor, not just a numerically plausible RPM.
8. Verify RPM telemetry dropout, >1s stale-data invalidation, eRPM-to-RPM pole count conversion, and no unexpected direction or throttle remapping. If any check fails, stop the experiment.

### Critical motor-model warning

The inherited Softdrone model `F(w)` and `M(w)` were fitted against a previous 50 Hz PWM ESC setup. **Switching to DShot300 changes the actuator command representation and may change the thrust-vs-command curve.** A previously working 80% loss/recovery run is NOT automatically equivalent on DShot. Recalibrate the DShot command-to-RPM-to-thrust relation on a thrust stand and test the L1/FDI compensation in HIL before even considering flight.

### RPM telemetry failure behavior

`L1RP` is strictly diagnostic. A missing response generates -1 and clears validity; neither event is interpreted as a failed motor and neither changes the controller's PWM. RPM does not imply thrust without a calibrated rotor/ESC model.

## Firmware build

Workflow: `.github/workflows/build-mode29-pair-rpm-bdshot.yml`.

It compiles an `arducopter.apj` and `arducopter.bin` **only if GitHub Actions succeeds**; do not claim the branch is build-verified before inspecting the workflow result.
