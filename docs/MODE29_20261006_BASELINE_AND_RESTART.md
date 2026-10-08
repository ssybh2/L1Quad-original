# Mode29 — 2026-10-06 Flight-Test Baseline and Clean Restart

> Status: historical, user-preferred flight-test baseline. This is a **versioned snapshot**, not a certification of safety or a guarantee of recovery under every 60% motor-loss condition.

## Source of truth (both Pixhawk and Orange Pi)

- Repository: `ssybh2/L1Quad-original`
- Exact source commit: `2c606d7e532c54416fc850995449b9acfd6f6a1b`
- Commit date: 2026-10-06 11:56:53 UTC (19:56:53 UTC+8)
- Original build branch: `feature/mode29-position-gain-schedule`
- Frozen snapshot branch: `milestone/mode29-60pct-validated-20261006`
- New **ongoing development** branch: `feature/mode29-from-60pct-20261006` (forked directly from the exact source commit)
- Older 2026-10-07 experiments **archived** at: `archive/mode29-20261007-experimental` -> `005e2d8df99adb9909dfff9f099675eab52783e0`. **Do not merge/cherry-pick these changes by default**.

The frozen snapshot branch must remain pointed at the original source commit. The new development branch is the only place to implement subsequent changes. Historical `feature/mode29-position-gain-schedule` is not the continuation branch, since it includes later 2026-10-07 modifications.

## Exact GitHub Actions / Release ZIP provenance

- Original build run: https://github.com/ssybh2/L1Quad-original/actions/runs/37459816936
- Build artifact ID: `11412336925`
- Artifact name: `mode29-position-gain-schedule-pixhawk6c-firmware`
- Build completed/uploaded: 2026-10-06 12:06:29 UTC
- Release: https://github.com/ssybh2/L1Quad-original/releases/tag/APM
- Release ZIP: https://github.com/ssybh2/L1Quad-original/releases/download/APM/mode29-position-gain-schedule-pixhawk6c-firmware.zip
- ZIP SHA-256: `5c3963207563505fce9671a4a4d709b39eb20ca790baa4d8a54b4c630a2b3c6e`
- ZIP bytes: `2486737`
- Release uploaded 2026-10-07; its ZIP has the same SHA-256 as the 2026-10-06 workflow artifact. Note: Release tag `APM` itself refers to an older `main` commit and must not be used to select the baseline sources.

## Dedicated frozen Orange Pi milestone (no 2026-10-07 yaw tuning)

- Frozen Orange Pi branch: `milestone/orangepi-mode29-20261006-no-yaw-protection`
- **Exact** paired commit: `2c606d7e532c54416fc850995449b9acfd6f6a1b`
- This branch deliberately points to the **same repository snapshot** as the Pixhawk milestone, so `orangepi/` cannot drift out of alignment with the known firmware source.
- The three **2026-10-07-only** yaw tuning parameters `M29_YAW_KD`, `M29_YAW_RMAX`, and `M29_YAW_MMAX` are **absent** from all files in the frozen `orangepi/` directory.
- The new `feature/mode29-from-60pct-20261006` development branch also excludes those parameter declarations and YAML values; its YAML files differ from the frozen snapshot only by changing the informational `firmware_branch` metadata.
- Do not copy these October 7 yaw parameters or related candidate/reduced-attitude changes into the new work without an explicit decision and separate verification.
- Changes to a GitHub branch do not automatically change any scripts deployed on the physical Orange Pi or saved parameters on the Pixhawk.

## Paired Orange Pi source (same exact commit)

The Orange Pi Python scripts and YAML live **within the same repository and commit** under `orangepi/`. No separate Orange Pi repository revision is needed.

- `orangepi/motor_derating_control.py` — MAVLink motor derating / RC9–RC12 control
- `orangepi/apply_flight_config.py` — runtime flight-parameter profile tool (requires DISARMED, supports dry-run and backup)
- `orangepi/configs/mode29_position_gain_schedule.yaml` — loss-specific gain/tilt anchors; `M29_GS_MODE: 1` for oracle calibration
- `orangepi/configs/mode29_tuning.yaml` — ordinary Mode29 profile; `M29_GS_MODE: 0` (gain scheduling disabled)
- `orangepi/configs/stabilize_pid_baseline.yaml` — separate normal/STABILIZE controller configuration
- `orangepi/README.md` — connection, operation and safety notes (some old textual branch references may refer to earlier development; pin the commit instead)

In the **versioned YAML file**, the 60% anchor contains:
- `M29_G60_KPX/KPY/KPZ = 5.5, 5.5, 10.0`
- `M29_G60_KVX/KVY/KVZ = 4.0, 4.0, 2.0`
- `M29_G60_KRX/KRY/KRZ = 1.0, 0.5, 0.25`
- `M29_G60_KOX/KOY/KOZ = 0.1, 0.2, 0.1`
- `M29_G60_TILT = 30.0`

These are **repository file values**; actual parameters stored on a Pixhawk cannot be inferred solely from the YAML. Back up and read back current parameter values before attempting to reproduce the flight result.

## Recommended workflows

Restore exact frozen source without changing an existing working directory:

```bash
git clone https://github.com/ssybh2/L1Quad-original.git L1Quad-mode29-baseline
cd L1Quad-mode29-baseline
git checkout --detach 2c606d7e532c54416fc850995449b9acfd6f6a1b
git submodule update --init --recursive
git rev-parse HEAD
```

Continue all new development from the explicitly new branch:

```bash
git clone -b feature/mode29-from-60pct-20261006 https://github.com/ssybh2/L1Quad-original.git L1Quad-mode29-next
cd L1Quad-mode29-next
git submodule update --init --recursive
git status -sb
```

Inspect the **historical** Orange Pi 60% calibration profile without applying:

```bash
python3 orangepi/apply_flight_config.py orangepi/configs/mode29_position_gain_schedule.yaml --dry-run
```

The parameter tool still connects to the Pixhawk for checks even in dry-run. Do this only with props removed and disarmed. Do not apply a gain schedule or inject a fault just because it worked in one prior flight: first verify the motor order, model, configuration, external navigation, actuator margins, logs and fail-safe behavior.

## Version-management rule

1. Treat `milestone/mode29-60pct-validated-20261006` as read-only; do not push commits or move its ref.
2. Build/test new improvements on `feature/mode29-from-60pct-20261006`.
3. Keep `archive/mode29-20261007-experimental` for comparison only.
4. Record firmware/Orange Pi commit ID, `M29_GS_MODE`, the full Pixhawk parameter dump, motor props/battery and test log for every new flight.
5. Before any new real-aircraft test, use props-off tests followed by staged safe validation with an independent disarm/kill mechanism.
