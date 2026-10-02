# Softdrone Progress Log — 2026-10-02

> Scope: real-airframe Mode 29 / NOKOV / Orange Pi motor-degradation experiment.
>
> This log freezes the current state before any further controller-code changes.

## 1. Current repository state

Repository:

```text
ssybh2/L1Quad-original
```

Primary Pixhawk firmware branch:

```text
feature/softdrone-motor-degradation-yaw-free
HEAD: f66904aa8a4e205d387b25a185aa9082e94fc911
```

Orange Pi experiment branch:

```text
feature/orangepi-motor-fault-injector
HEAD: 350fe088415075f836919bac6905f07539b81749
```

Pinned ArduPilot submodule:

```text
ArduCopter 4.4.4
5d7fb310fb2e1e89bf34e0b0845d146ff8abf2d0
```

Important backup branches already exist:

```text
backup/softdrone-yaw-free-pre-nan-fix-20261001
backup/softdrone-pre-runtime-takeoff-params-20261002
backup/orangepi-pre-unified-flight-tuning-20261002
backup/orangepi-pre-runtime-takeoff-params-20261002
```

### Important distinction: repository HEAD vs firmware currently on the aircraft

The aircraft was confirmed flashed with the Mode 29 NaN/safety-fix firmware derived from:

```text
0189b3b3c60c8667bc23bf38c43fd80819f00157
Fix Mode29 NaN propagation and add safe fallback guards
```

The remote firmware branch has since advanced to `f66904aa...` and now contains runtime-configurable takeoff trajectory parameters. Unless a newer APJ has been built and flashed after those commits, the aircraft should still be treated as running the earlier `0189b3b3...` safety-fix firmware.

Do not assume `M29_TKOFF_ALT`, `M29_TKOFF_T`, or `M29_SETTLE_T` are active on the aircraft until the newer firmware is compiled, flashed, and read back successfully.

---

## 2. Hardware / runtime architecture

Current experiment chain:

```text
NOKOV motion capture
    ↓
vrpn_client_ros
    ↓
/softdrone/pose
    ↓
~/nokov_ws/softdrone_mocap_to_pixhawk.py
    ↓ MAVLink ODOMETRY
Pixhawk 6C Mini / EKF3 ExternalNav
    ↓
Mode 29 geometric controller
    ↓
Mode 29 custom motor allocation
    ↓
SERVO9..12
    ↓
ESC / motors
```

Orange Pi 5 Max runs NOKOV/ROS bridge, runtime parameter tools, and motor-degradation commands.

Pixhawk USB usage:

```text
if02 → NOKOV/ODOMETRY bridge
if00 → diagnostics / PARAM_SET / motor_derating_control.py
```

Do not replace the stable `/dev/serial/by-id/...` paths with permanent `ttyACM0/1` assumptions.

---

## 3. Mode 29 motor mapping

Current logical motor layout used by Mode 29:

```text
             FRONT

M3 CW                    M1 CCW

M2 CCW                   M4 CW
```

Current ArduPilot output mapping:

```text
SERVO9_FUNCTION  = 36  → M4 rear-right
SERVO10_FUNCTION = 34  → M2 rear-left
SERVO11_FUNCTION = 35  → M3 front-left
SERVO12_FUNCTION = 33  → M1 front-right
```

Motor numbering was previously corrected and manual motor identification was confirmed. Rotation direction still needs to remain consistent with the physical aircraft.

---

## 4. External navigation configuration

Current relevant parameters read from Pixhawk:

```text
AHRS_ORIENTATION = 8
VISO_TYPE        = 1
VISO_ORIENT      = 0
VISO_SCALE       = 1
VISO_DELAY_MS    = 10

EK3_SRC1_POSXY   = 6
EK3_SRC1_POSZ    = 6
EK3_SRC1_VELXY   = 0
EK3_SRC1_VELZ    = 0
EK3_SRC1_YAW     = 6

TRAJINDEX        = 0
L1ENABLE         = 0

SERVO9_FUNCTION  = 36
SERVO10_FUNCTION = 34
SERVO11_FUNCTION = 35
SERVO12_FUNCTION = 33
```

`AHRS_ORIENTATION=8` is ArduPilot `ROTATION_ROLL_180`. This is only correct if the Pixhawk installation is physically consistent with that board orientation. STABILIZE manual flight has been working, so this is not currently the leading suspect, but it still deserves a physical cross-check.

---

## 5. NOKOV → ArduPilot world-frame mapping: experimentally verified

Bridge code uses:

```python
x_ap = +y_nokov
y_ap = +x_nokov
z_ap = -z_nokov
```

The following real-airframe hand-movement tests were completed on 2026-10-02.

### Z test

Aircraft was lifted upward in NOKOV `+Z`.

Observed ArduPilot position moved toward negative NED Z, for example:

```text
AP pos ≈ (+0.134, +0.202, -0.688)
```

Result:

```text
NOKOV +Z → AP -Z   PASS
```

### NOKOV +Y test

Starting from approximately:

```text
AP pos ≈ (+0.134, +0.202, -0.688)
```

After moving mainly along NOKOV `+Y`:

```text
AP pos ≈ (+1.173, +0.202, -0.641)
```

Result:

```text
NOKOV +Y → AP +X   PASS
```

### NOKOV +X test

After then moving mainly along NOKOV `+X`:

```text
AP pos ≈ (+1.042, +1.620, -0.649)
```

Result:

```text
NOKOV +X → AP +Y   PASS
```

Therefore the translational world-frame mapping currently appears correct:

```text
NOKOV +Y → AP +X   PASS
NOKOV +X → AP +Y   PASS
NOKOV +Z → AP -Z   PASS
```

This substantially reduces the likelihood that the Mode 29 flip tendency is caused by a simple XYZ world-frame swap/sign error.

---

## 6. Remaining attitude-frame test

The static quaternion looked reasonable near level, but a zero attitude alone does not prove the roll/pitch signs are correct.

Still required with props OFF and vehicle DISARMED:

```text
right side DOWN → Pixhawk roll must become POSITIVE
nose UP         → Pixhawk pitch must become POSITIVE
yaw RIGHT       → Pixhawk yaw must increase
```

Run:

```bash
python3 ~/mode29_frame_audit.py
```

Compare the Pixhawk `ATTITUDE` output with the bridge `AP rpy` output.

Until these sign tests pass, do not use a real flight to diagnose Mode 29 attitude behavior.

---

## 7. Mode 29 NaN / re-arm failure: root cause and fix

A previous real test produced:

```text
PreArm: Internal errors 0x400 ... cnstring_nan
Arm:    Internal errors 0x400 ... cnstring_nan
```

The fault was traced to a non-finite value reaching:

```cpp
constrain_float(motorPWMCommanded[i], 0.0f, 100.0f)
```

in Mode 29.

ArduPilot records `constraining_nan` as an internal error; after the emergency disarm, that latched error prevented the next ARM until reboot.

Commit:

```text
0189b3b3c60c8667bc23bf38c43fd80819f00157
Fix Mode29 NaN propagation and add safe fallback guards
```

added the main containment fixes:

- L1 disabled now means a real hard bypass of the L1 computation.
- L1 historical states are explicitly initialized/cleared.
- Mode 29 custom controller math returns immediately while DISARMED.
- trajectory, geometric-controller, L1, and motor-allocation outputs are checked for NaN/Inf.
- non-finite control output aborts Mode 29 instead of entering `constrain_float()`.
- abnormal Mode 29 execution attempts to return to STABILIZE.
- an origin-entry guard rejects Mode 29 when too far from the expected local origin.
- current threshold is approximately `XY <= 0.50 m`, `|Z| <= 0.50 m`.

This safety-fix firmware has been flashed to the aircraft.

---

## 8. Current Mode 29 behavior under investigation

Two distinct symptoms have been seen.

### 8.1 Strong tilt / apparent tendency to flip after entering Mode 29

At one diagnostic point the real Pixhawk EKF state was approximately:

```text
LOCAL_POSITION_NED:
x = +0.102 m
y = +0.734 m
z = -0.161 m
```

This was not actually near the Mode 29 world origin. A large horizontal position error can immediately request a large tilt from the geometric controller.

After hand-movement tests, the translational NOKOV/AP frame mapping appears correct. The remaining leading suspects are now:

1. roll/pitch attitude sign mismatch;
2. Mode 29 `Mx/My → motor` feedback direction;
3. physical motor/output mapping inconsistency despite correct logical numbering;
4. entry-position / origin mismatch at the moment of Mode 29 transition.

### Required props-off mixer feedback test

With aircraft fixed, props OFF, and Mode 29 entered only for a short bench test:

For **right side DOWN** / positive roll, the corrective output should increase the right-side motors:

```text
SERVO9  / M4 rear-right   ↑
SERVO12 / M1 front-right  ↑
SERVO10 / M2 rear-left    ↓
SERVO11 / M3 front-left   ↓
```

For **nose UP** / positive pitch, the corrective output should increase the rear motors:

```text
SERVO9  / M4 rear-right   ↑
SERVO10 / M2 rear-left    ↑
SERVO11 / M3 front-left   ↓
SERVO12 / M1 front-right  ↓
```

If either test is reversed, that axis is acting as positive feedback and can explain the apparent flip tendency.

---

## 9. Aggressive vertical Mode 29 takeoff

The firmware that was already flashed used the original hardcoded takeoff behavior:

```text
(0,0,0) → (0,0,-1 m)
duration = 2 s
```

The old `ACRL_trajectory_takeoff()` trajectory therefore contained a fixed altitude and fixed timing independent of the YAML gains.

This is why simply reducing `GEOCTRL_KPZ/KVZ` is not the correct architectural solution to an overly aggressive takeoff. The reference trajectory itself must be adjustable.

### User-reported local tuning during the test

The latest locally reported Mode 29 gains were:

```text
GEOCTRL_KPX = 2.0
GEOCTRL_KPY = 2.0
GEOCTRL_KPZ = 1.0

GEOCTRL_KVX = 1.5
GEOCTRL_KVY = 0.9
GEOCTRL_KVZ = 0.2

GEOCTRL_KRX = 0.55
GEOCTRL_KRY = 0.35
GEOCTRL_KRZ = 0.15

GEOCTRL_KOX = 0.035
GEOCTRL_KOY = 0.030
GEOCTRL_KOZ = 0.004

L1ENABLE = 0
```

Important: the remote version-controlled `orangepi/configs/mode29_baseline.yaml` currently still contains the original higher baseline gains:

```text
KPX/KPY/KPZ = 14 / 15 / 15
KVX/KVY/KVZ = 1.5 / 0.9 / 1.1
```

This local-vs-remote tuning difference must be reconciled before the next experiment so the exact parameters used in flight are unambiguous.

---

## 10. Runtime takeoff parameterization now present on the remote branches

Although the aircraft is still assumed to be running the previously flashed safety-fix build, the current remote firmware branch has already advanced and now contains runtime-configurable Mode 29 takeoff parameters.

Current defaults in the remote firmware:

```text
M29_TKOFF_ALT = 1.0 m
M29_TKOFF_T   = 4.0 s
M29_SETTLE_T  = 1.5 s
```

Relevant firmware commits after `0189b3b3...` include:

```text
4830699  Mode29: add runtime takeoff parameter storage
b84e6a5  Mode29: define runtime takeoff defaults
3d84111  Mode29: register runtime takeoff AP_Param values
8b4a6aa  Mode29: parameterize trajectory altitude and takeoff timing
4da0586  Mode29: snapshot runtime takeoff configuration
9db41f3  Mode29: replace hardcoded takeoff with runtime smooth trajectory
f66904a  Mode29: use runtime takeoff altitude duration and settle time
```

The remote code now:

- uses a runtime-configured takeoff altitude;
- uses a runtime-configured takeoff duration;
- freezes those values on Mode 29 entry for that run;
- keeps hover altitude consistent with the configured takeoff altitude;
- replaces the fixed 2-second takeoff reference with a parameterized smooth trajectory;
- delays motor-degradation activation until `takeoffTime + settleTime`;
- propagates the configured altitude into the other trajectory paths.

The Orange Pi branch has corresponding support:

```text
M29_TKOFF_ALT
M29_TKOFF_T
M29_SETTLE_T
```

in:

```text
orangepi/apply_flight_config.py
orangepi/apply_mode29_config.py
orangepi/configs/mode29_baseline.yaml
orangepi/motor_derating_control.py
orangepi/README.md
```

Current Orange Pi remote defaults are also:

```text
M29_TKOFF_ALT = 1.0
M29_TKOFF_T   = 4.0
M29_SETTLE_T  = 1.5
```

Again: these new parameters become usable on the real aircraft only after the matching newer firmware is compiled and flashed.

---

## 11. Orange Pi tuning workflow

Current recommended configuration layout is intentionally simple:

```text
orangepi/configs/
├── mode29_baseline.yaml
├── mode29_tuning_v1.yaml          # local working copy
├── stabilize_pid_baseline.yaml
└── stabilize_pid_tuning_v1.yaml   # local working copy
```

Combined duplicate `flight_tuning_*.yaml` files were removed from the recommended workflow.

Use:

```bash
python3 orangepi/apply_flight_config.py \
  orangepi/configs/stabilize_pid_tuning_v1.yaml \
  --dry-run
```

for normal ArduCopter/STABILIZE tuning, and:

```bash
python3 orangepi/apply_flight_config.py \
  orangepi/configs/mode29_tuning_v1.yaml \
  --dry-run
```

for Mode 29.

Apply only while DISARMED.

`apply_flight_config.py` now supports both:

```text
ATC_*                         normal ArduCopter attitude/rate controller
GEOCTRL_* / L1* / M29_*      custom Mode 29 controller / trajectory
```

These are separate control stacks even though the same Orange Pi tool can manage both.

---

## 12. Manual/STABILIZE tuning

The initial captured ArduCopter attitude baseline was:

```text
ATC_ANG_RLL_P = 4.5
ATC_ANG_PIT_P = 4.5
ATC_ANG_YAW_P = 4.5

ATC_RAT_RLL_P = 0.135
ATC_RAT_RLL_I = 0.135
ATC_RAT_RLL_D = 0.0036
ATC_RAT_RLL_FF = 0

ATC_RAT_PIT_P = 0.135
ATC_RAT_PIT_I = 0.135
ATC_RAT_PIT_D = 0.0036
ATC_RAT_PIT_FF = 0

ATC_RAT_YAW_P = 0.18
ATC_RAT_YAW_I = 0.018
ATC_RAT_YAW_D = 0
ATC_RAT_YAW_FF = 0

ATC_INPUT_TC = 0.15
```

The user subsequently tuned manual flight and reported that the manual PID was satisfactory. The exact final tuned values were not captured in this log, so the next session should read them back from the Pixhawk or the local `stabilize_pid_tuning_v1.yaml` before changing anything else.

Mode 29 does **not** use the `ATC_RAT_*` / `ATC_ANG_*` gains as its primary controller.

---

## 13. Motor degradation workflow

Runtime motor fault injection remains on the Orange Pi through RC9–RC12 overrides.

Example indefinite fault:

```bash
cd ~/L1Quad-original

python3 orangepi/motor_derating_control.py enable \
  --motor 1 \
  --loss 20 \
  --duration 0
```

`--duration 0` means keep the injected loss active until explicitly stopped, Mode 29 is exited, or the vehicle is disarmed.

Cancel:

```bash
python3 orangepi/motor_derating_control.py disable
```

Fault testing must not resume until baseline Mode 29 takeoff/hover and the attitude/mixer sign tests are proven stable.

---

## 14. Current blockers

At the end of 2026-10-02, the project is **not yet ready for the motor-degradation flight series**.

The remaining blockers are:

1. verify roll and pitch attitude signs with hand-tilt tests;
2. verify Mode 29 roll/pitch corrective motor directions with props OFF;
3. reconcile local Mode 29 tuning values with the version-controlled YAML;
4. confirm which exact firmware commit is physically installed;
5. compile and validate the current runtime-takeoff branch head;
6. flash the runtime-takeoff build only after the bench checks are satisfactory;
7. verify `M29_TKOFF_ALT/T/SETTLE_T` readback on the aircraft;
8. perform a no-fault Mode 29 takeoff/hover test before any motor loss is injected.

---

## 15. Recommended next-session sequence

### Step 1 — Freeze controller code

Do not make additional controller changes until the existing coordinate/mixer tests are complete.

### Step 2 — Read the actual aircraft state

With props OFF:

```bash
python3 ~/mode29_frame_audit.py
```

Confirm:

```text
right side DOWN → roll > 0
nose UP         → pitch > 0
yaw RIGHT       → yaw increases
```

### Step 3 — Mode 29 corrective-output bench test

Props OFF, aircraft restrained, no motor degradation, `L1ENABLE=0`.

Check the expected SERVO9–12 changes for positive roll and positive pitch as listed in Section 8.

If the output is reversed, stop and repair the frame/mixer mapping before flight.

### Step 4 — Verify firmware generation

The currently flashed aircraft should be treated as running the `0189b3b3...` safety build until proven otherwise.

Before using runtime takeoff parameters, build the current firmware branch and verify that the APJ corresponds to:

```text
f66904aa8a4e205d387b25a185aa9082e94fc911
```

### Step 5 — Update Orange Pi branch

```bash
cd ~/L1Quad-original
git checkout feature/orangepi-motor-fault-injector
git pull --ff-only origin feature/orangepi-motor-fault-injector
```

### Step 6 — Reconcile tuning files

Read the current Pixhawk values and compare against the local tuning YAML.

Do not overwrite the working local values with the remote baseline without first checking the diff.

### Step 7 — After the newer firmware is flashed

Verify:

```bash
python3 orangepi/motor_derating_control.py status
```

Expected new fields:

```text
M29_TKOFF_ALT
M29_TKOFF_T
M29_SETTLE_T
```

Initial conservative trajectory target:

```text
M29_TKOFF_ALT = 1.0 m
M29_TKOFF_T   = 4.0 s
M29_SETTLE_T  = 1.5 s
```

### Step 8 — No-fault flight first

With the aircraft physically near the intended EKF/NOKOV origin:

```text
ARM in STABILIZE
→ verify normal response
→ enter Mode 29
→ no fault injection
→ observe smooth takeoff and hover
→ exit back to STABILIZE
→ land
```

Only after repeated stable no-fault Mode 29 tests should degradation testing resume.

### Step 9 — Reintroduce faults gradually

Recommended sequence:

```text
5% → 10% → 15% → 20%
```

Use `--duration 0` only after a small-loss test is already demonstrated stable and an immediate recovery path is ready.

---

## 16. End-of-day state

What is working:

- manual STABILIZE flight is functional and has been tuned;
- NOKOV → bridge → Pixhawk ExternalNav is functioning;
- translational XYZ coordinate mapping has been experimentally verified;
- Mode 29 motor numbering/output routing has been corrected;
- Orange Pi runtime tuning workflow exists;
- Orange Pi runtime motor degradation exists;
- yaw-free degradation support exists;
- the previous Mode 29 NaN/re-arm failure has a dedicated firmware safety fix;
- the remote branches now contain runtime-configurable Mode 29 takeoff altitude/time/settle parameters.

What is not yet proven:

- roll/pitch attitude sign mapping under manual tilt;
- Mode 29 roll/pitch motor feedback direction;
- latest remote runtime-takeoff firmware on real hardware;
- stable no-fault Mode 29 takeoff and hover with the new trajectory settings;
- stable motor-degradation flight after the above changes.

The immediate priority is **not more controller tuning**. The immediate priority is to finish the props-off attitude/mixer sign audit, establish an exact firmware/parameter baseline, and only then resume real Mode 29 flight testing.
