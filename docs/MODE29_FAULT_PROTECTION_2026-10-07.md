# Mode29 fault-protection update — 2026-10-07

This update addresses two coupled structural risks in Mode29:

1. A severe motor-fault candidate previously waited for full FDI confirmation before
   releasing yaw and entering reduced allocation.
2. After confirmation, delayed L1 residuals could be integrated again and drive a
   real ~90% loss estimate toward a false 100% saturation.

## Control-state behaviour

### Healthy

The normal full geometric controller and full F/Mx/My/Mz allocator are unchanged.

### FDI candidate

As soon as a severe candidate exists, Mode29 now:

- releases heading immediately;
- switches to true reduced-attitude control based on desired thrust direction;
- controls F/Mx/My as the primary wrench;
- requests only secondary yaw-rate damping;
- uses the yaw-free primary allocator while motor identity is still unconfirmed.

The candidate stage deliberately does **not** trust the unconfirmed continuous
severity enough to use effectiveness-aware compensation. This preserves the
independent FDI confirmation path while removing the dangerous yaw objective
immediately.

### Confirmed fault

After motor identity is confirmed, Mode29 uses the effectiveness-aware
F/Mx/My allocator. Yaw-rate damping is applied only in the primary allocator's
null space. Therefore yaw damping cannot intentionally steal modeled F/Mx/My
authority. At a true 100% single-motor failure, useful yaw null-space authority
vanishes naturally and F/Mx/My remain the priority.

## True reduced attitude

Fault protection no longer computes a full yaw-referenced desired rotation and
then zeros Mz afterward. It directly controls the body-Z/thrust direction:

- desired thrust direction comes from the position/velocity controller;
- roll/pitch correction is based on the thrust-direction error;
- yaw heading is not controlled;
- rigid-body gyroscopic compensation is retained;
- body-z rate receives only bounded damping.

This removes yaw-heading error from the roll/pitch attitude objective while
retaining real rotational dynamics.

## New yaw-protection parameters

- `M29_YAW_KD`: free-yaw body-z rate damping gain, default 0.02 N*m/(rad/s)
- `M29_YAW_RMAX`: soft yaw-rate envelope, default 360 deg/s
- `M29_YAW_MMAX`: maximum secondary yaw damping request, default 0.12 N*m

`M29_YAW_RMAX` is not a hard guaranteed rate cap. With one actuator completely
lost, only three effective actuator DOFs remain and F/Mx/My take priority.

The Orange Pi parameter whitelist and `mode29_tuning.yaml` include these values.

## FDI severity anti-windup

Motor identity confirmation and continuous severity estimation now run on
different time scales.

After confirmation:

- severity integration pauses for ~100 ms;
- the pre-confirmation filtered disturbance is flushed before rebuilding under
  the effectiveness-aware allocator;
- severity uses roll/pitch fault signature only, avoiding free-yaw contamination;
- the degraded motor command must be high enough for severity observability;
- the fault signature norm must exceed a minimum threshold;
- the old small-residual OR bypass is removed;
- residual direction must fit the isolated motor signature;
- projected residual is bounded;
- a residual deadband suppresses noise-driven updates;
- correction direction must remain consistent for several samples;
- loss increase is rate-limited to 25 percentage-points/s;
- loss decrease is rate-limited to 75 percentage-points/s;
- a confirmed motor ID no longer forces gain-schedule confidence to 1.0.

The estimator still permits convergence to a real 100% failure. There is no
hard 90% cap.

## Logs

Existing `L1FD` remains compatible.

New `L1FQ` records:

- `obs`: whether severity is currently observable
- `settle`: post-confirmation update hold samples remaining
- `dir`: current severity correction direction
- `dcnt`: consecutive correction-direction samples
- `conf`: gain-scheduler confidence

New `L1RA` records reduced-attitude tilt errors, body-z rate and yaw damping
moment request.

## Validation sequence

Do not start with a 90–100% free-flight test.

Recommended order:

1. firmware CI build;
2. SITL/HIL regression;
3. propellers removed: parameter/configuration and state-transition checks;
4. fixed thrust stand / restrained airframe;
5. progressively test 80%, 90%, 95%, 100% injection;
6. recovery tests;
7. repeat all motor IDs and different thrust levels;
8. add voltage sag, actuator lag/model mismatch and IMU vibration/noise;
9. only after those tests, proceed to conservative free-flight tests.

Acceptance criteria include:

- a 90% fault does not remain falsely saturated at 100%;
- a 100% fault can still converge near 100%;
- low-command residuals do not produce severity jumps;
- allocator switching does not cause a second integration of stale residual;
- recovery can drive severity down and release the confirmed fault;
- gain scheduling does not jump to a high-confidence wrong anchor;
- candidate protection starts before formal identity confirmation;
- fault motor identification remains stable.
