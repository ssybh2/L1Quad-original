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
- immediately uses the candidate motor ID in the effectiveness-aware reduced allocator;
- deliberately uses a fixed **60% provisional loss** during Candidate instead of
  feeding the instantaneous blind severity directly into the allocator.

Motor identity and severity are therefore separated. The first severe-candidate
cycle is actionable for protection, but a one-cycle 90..100% severity spike cannot
make a real ~60% fault look like a nearly dead motor to the allocator.

Candidate entry remains at 60% loss evidence. An already-active Candidate is held
while same-motor evidence remains above 50%, and weak/contradictory evidence must
persist for about 30 ms before Candidate protection is released. This 60%/50%
hysteresis prevents control-structure chatter around the 60% boundary.

The raw Candidate severity is logged separately and low-pass filtered only for
initialising the continuous severity estimate after formal confirmation. The
Candidate allocator itself remains fixed at the 60% provisional protection level.

After Candidate protection starts, FDI changes interpretation of the residual.
The effectiveness-aware allocator has already modelled a 60% loss, so the
remaining L1 residual is treated as a correction around that model:

`absolute loss ~= 60% provisional loss + residual correction`.

The uncompensated pre-Candidate residual is flushed when Candidate protection is
entered so the same transient is not counted twice. This prevents a correctly
compensated 60% fault from appearing to "disappear" merely because the allocator
has removed most of its residual. Candidate confirmation is also gated on minimum
actuator command/signature observability.

Formal confirmation still latches the diagnosis after the existing confirmation
window; it no longer gates the start of protective control allocation.

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
- the degraded motor command must be high enough for severity observability (current guard: command >= 13, where w=(PWM-1000)/10);
- the roll/pitch fault-signature norm must exceed a minimum threshold;
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

New `L1FC` records Candidate-specific diagnostics:

- `release`: consecutive weak-evidence samples toward Candidate release
- `raw`: instantaneous blind Candidate severity
- `filt`: filtered Candidate severity used only to initialise Confirmed severity
- `protect`: provisional allocator severity (60%)

New `L1RA` records reduced-attitude tilt errors, body-z rate and yaw damping
moment request.

## Validation sequence

Do not start with a 90–100% free-flight test.

Recommended order:

1. firmware CI build;
2. SITL/HIL regression;
3. propellers removed: parameter/configuration and state-transition checks;
4. fixed thrust stand / restrained airframe;
5. progressively test 60%, 70%, 80%, 90%, 95%, 100% injection;
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
