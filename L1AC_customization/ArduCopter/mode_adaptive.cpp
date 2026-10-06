// mode_adaptive.cpp
// This file contains the main code for running ACRL's adaptive flight mode.
// Copyright 2021 Sheng Cheng, all rights reserved.

// Dr. Sheng Cheng, Nov. 2021
// Email: chengs@illinois.edu
// Advanced Controls Research Laboratory
// Department of Mechanical Science and Engineering
// University of Illinois Urbana-Champaign
// Urbana, IL 61821, USA

#include "Copter.h"
#include <AP_HAL/AP_HAL.h>
#include <AP_Motors/AP_Motors_Class.h> // for sending motor speed
#include "ACRL_trajectories.h"         // small libraries for trajectories at ACRL

#if MODE_ADAPTIVE_ENABLED == ENABLED

namespace {

constexpr float MODE29_ENTRY_MAX_XY_M = 0.50f;
constexpr float MODE29_ENTRY_MAX_Z_M = 0.50f;

#if REAL_OR_SITL
// Softdrone single-motor 6S static model refit from the two valid 2026-10-03
// staircase datasets (1405 motor, 3-inch prop, 30 A ESC, 50 Hz PWM).
//
// Measured pack voltage across the fitted sweeps: 21.95..24.69 V
// (mean 23.77 V). Command variable:
//     w = (PWM_us - 1000) / 10
//
// The dead-zone cubic preserves F(0)=M(0)=0 and remains monotonic throughout
// Mode29's 0..100 command range. The thrust fit retains the near-zero samples
// to identify the ESC/motor onset. For reaction torque, the physically invalid
// zero-RPM torque-offset sample from one repeat is excluded from the fit.
//
// xF = max(0, w - SOFTDRONE_F_W_DEAD)
// F  = F_C3*xF^3 + F_C2*xF^2 + F_C1*xF      [N]
//
// xM = max(0, w - SOFTDRONE_M_W_DEAD)
// M  = M_C3*xM^3 + M_C2*xM^2 + M_C1*xM      [N*m]
constexpr float SOFTDRONE_F_W_DEAD = 4.47703190f;
constexpr float SOFTDRONE_F_C3 = -2.62683159e-05f;
constexpr float SOFTDRONE_F_C2 =  4.01680390e-03f;
constexpr float SOFTDRONE_F_C1 =  4.05756758e-08f;

constexpr float SOFTDRONE_M_W_DEAD = 6.24726216f;
constexpr float SOFTDRONE_M_C3 = -2.50608401e-07f;
constexpr float SOFTDRONE_M_C2 =  3.84208303e-05f;
constexpr float SOFTDRONE_M_C1 =  5.42809055e-04f;

float softdrone_poly_eval(float w,
                          float w_dead,
                          float c3,
                          float c2,
                          float c1)
{
    const float x = MAX(0.0f, w - w_dead);
    return MAX(0.0f, ((c3 * x + c2) * x + c1) * x);
}

float softdrone_poly_slope(float w,
                           float w_dead,
                           float c3,
                           float c2,
                           float c1)
{
    // Use the right-hand slope at the dead-zone boundary so the local
    // allocator remains non-singular even if a motor command is very small.
    const float x = MAX(0.0f, w - w_dead);
    return MAX(1.0e-6f, (3.0f * c3 * x + 2.0f * c2) * x + c1);
}

float softdrone_thrust_from_w(float w)
{
    return softdrone_poly_eval(w,
                               SOFTDRONE_F_W_DEAD,
                               SOFTDRONE_F_C3,
                               SOFTDRONE_F_C2,
                               SOFTDRONE_F_C1);
}

float softdrone_thrust_slope_from_w(float w)
{
    return softdrone_poly_slope(w,
                                SOFTDRONE_F_W_DEAD,
                                SOFTDRONE_F_C3,
                                SOFTDRONE_F_C2,
                                SOFTDRONE_F_C1);
}

float softdrone_moment_from_w(float w)
{
    return softdrone_poly_eval(w,
                               SOFTDRONE_M_W_DEAD,
                               SOFTDRONE_M_C3,
                               SOFTDRONE_M_C2,
                               SOFTDRONE_M_C1);
}

float softdrone_moment_slope_from_w(float w)
{
    return softdrone_poly_slope(w,
                                SOFTDRONE_M_W_DEAD,
                                SOFTDRONE_M_C3,
                                SOFTDRONE_M_C2,
                                SOFTDRONE_M_C1);
}

float softdrone_w_from_thrust(float thrust_n)
{
    if (!isfinite(thrust_n) || thrust_n <= 0.0f) {
        return 0.0f;
    }

    // The thrust stand was validated through w~=80 (1800 us), but Mode29's
    // actuator range is 0..100.  Continue the monotonic polynomial up to the
    // normal actuator limit while never asking the inverse to leave that range.
    float lo = SOFTDRONE_F_W_DEAD;
    float hi = 100.0f;

    if (thrust_n >= softdrone_thrust_from_w(hi)) {
        return hi;
    }

    // Deterministic bisection avoids fragile closed-form cubic roots and is
    // cheap relative to the rest of the Mode29 geometric controller.
    for (uint8_t i = 0; i < 24; i++) {
        const float mid = 0.5f * (lo + hi);
        if (softdrone_thrust_from_w(mid) < thrust_n) {
            lo = mid;
        } else {
            hi = mid;
        }
    }
    return 0.5f * (lo + hi);
}
#endif

bool mode29_finite(const Vector2f &v)
{
    return isfinite(v.x) && isfinite(v.y);
}

bool mode29_finite(const Vector3f &v)
{
    return isfinite(v.x) && isfinite(v.y) && isfinite(v.z);
}

bool mode29_finite(const VectorN<float, 4> &v)
{
    for (uint8_t i = 0; i < 4; i++) {
        if (!isfinite(v[i])) {
            return false;
        }
    }
    return true;
}

void mode29_zero(VectorN<float, 4> &v)
{
    for (uint8_t i = 0; i < 4; i++) {
        v[i] = 0.0f;
    }
}

} // namespace

/*
 * Init and run calls for adaptive flight mode (copied from stabilize)
 */

// Init function: this function will be called everytime the FC enters the adaptive mode.
bool ModeAdaptive::init(bool ignore_checks)
{
    (void)ignore_checks;

    // Never permit direct motor output until all Mode 29 entry checks pass.
    motorEnable = 0;
    landingComplete = 0;
    landingTriggered = 0;
    clear_motor_degradation_command();
    clear_auto_motor_fault();

    if (!ahrs.have_inertial_nav()) {
        GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL,
                      "Mode29 rejected: inertial navigation inactive");
        return false;
    }

    if (!ahrs.get_velocity_NED(v_hat_prev) || !mode29_finite(v_hat_prev)) {
        GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL,
                      "Mode29 rejected: invalid NED velocity");
        return false;
    }
    v_prev = v_hat_prev;

    omega_hat_prev = AP::ahrs().get_gyro();
    if (!mode29_finite(omega_hat_prev)) {
        GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL,
                      "Mode29 rejected: invalid gyro state");
        return false;
    }
    omega_prev = omega_hat_prev;

    Vector3f entry_position;
    if (!ahrs.get_relative_position_NED_origin(entry_position) ||
        !mode29_finite(entry_position)) {
        GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL,
                      "Mode29 rejected: invalid local position");
        return false;
    }

    // This experiment intentionally uses the fixed NED-origin trajectory
    // (0,0,0) -> (0,0,-1). Refuse Mode 29 if the aircraft is not physically
    // near the mocap/EKF origin so it cannot suddenly fly back to the origin.
    const float entry_xy =
        sqrtf(entry_position.x * entry_position.x +
              entry_position.y * entry_position.y);
    if (entry_xy > MODE29_ENTRY_MAX_XY_M ||
        fabsf(entry_position.z) > MODE29_ENTRY_MAX_Z_M) {
        GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL,
                      "Mode29 rejected: origin offset xy=%.2fm z=%.2fm",
                      (double)entry_xy,
                      (double)entry_position.z);
        return false;
    }

    Quaternion q;
    ahrs.get_quat_body_to_ned(q);
    if (!isfinite(q.q1) || !isfinite(q.q2) ||
        !isfinite(q.q3) || !isfinite(q.q4)) {
        GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL,
                      "Mode29 rejected: invalid attitude quaternion");
        return false;
    }
    q.rotation_matrix(R_prev);

    // Explicitly initialise every L1 state. Multiplying stale values by zero
    // is unsafe because IEEE NaN * 0 is still NaN.
    mode29_zero(u_b_prev);
    mode29_zero(u_ad_prev);
    mode29_zero(sigma_m_hat_prev);
    mode29_zero(lpf1_prev);
    mode29_zero(lpf2_prev);
    sigma_um_hat_prev[0] = 0.0f;
    sigma_um_hat_prev[1] = 0.0f;

    trajIndex = g.trajIndex;
    radiusX = g.circRadiusX;
    radiusY = g.circRadiusY;
    targetSpeed = g.circSpeed;

    const float configured_takeoff_alt = (float)g.m29_takeoff_alt;
    const float configured_takeoff_time = (float)g.m29_takeoff_time;
    const float configured_settle_time = (float)g.m29_settle_time;
    const float configured_max_tilt = (float)g.m29_max_tilt;

    if (!isfinite(configured_takeoff_alt) ||
        configured_takeoff_alt < 0.2f ||
        configured_takeoff_alt > 5.0f) {
        GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL,
                      "Mode29 rejected: M29_TKOFF_ALT invalid");
        return false;
    }

    if (!isfinite(configured_takeoff_time) ||
        configured_takeoff_time < 1.0f ||
        configured_takeoff_time > 15.0f) {
        GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL,
                      "Mode29 rejected: M29_TKOFF_T invalid");
        return false;
    }

    if (!isfinite(configured_settle_time) ||
        configured_settle_time < 0.0f ||
        configured_settle_time > 15.0f) {
        GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL,
                      "Mode29 rejected: M29_SETTLE_T invalid");
        return false;
    }

    if (!isfinite(configured_max_tilt) ||
        configured_max_tilt < 5.0f ||
        configured_max_tilt > 60.0f) {
        GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL,
                      "Mode29 rejected: M29_MAX_TILT invalid");
        return false;
    }

    // Freeze the trajectory configuration for this Mode29 run. The Orange Pi
    // tool only writes these values while DISARMED, so a flight cannot change
    // its reference trajectory halfway through the run.
    takeoffAlt = configured_takeoff_alt;
    takeoffTime = configured_takeoff_time;
    settleTime = configured_settle_time;
    maxTiltDeg = configured_max_tilt;
    reset_gain_schedule();

    motorEnable = 1;

    GCS_SEND_TEXT(MAV_SEVERITY_INFO,
                  "Adaptive mode ready: origin xy=%.2fm z=%.2fm L1=%d",
                  (double)entry_xy,
                  (double)entry_position.z,
                  (int)g.l1enable);
    GCS_SEND_TEXT(MAV_SEVERITY_INFO,
                  "Mode29 takeoff: H=%.2fm T=%.2fs settle=%.2fs tilt=%.1fdeg",
                  (double)takeoffAlt,
                  (double)takeoffTime,
                  (double)settleTime,
                  (double)maxTiltDeg);
#if REAL_OR_SITL
    GCS_SEND_TEXT(MAV_SEVERITY_INFO,
                  "Mode29 motor model: 6S PWM fit 2026-10-03");
#endif
    return true;
}

void ModeAdaptive::exit()
{
    // Leaving Mode 29 immediately clears both injected and detected faults.
    clear_motor_degradation_command();
    clear_auto_motor_fault();
    reset_gain_schedule();
}

void ModeAdaptive::set_motor_degradation_command(bool enable, uint8_t motor_id, float loss_pct, bool yaw_free)
{
    if (!enable) {
        clear_motor_degradation_command();
        return;
    }

    if (motor_id < 1 || motor_id > 4) {
        return;
    }

    motor_degradation_enabled = true;
    motor_degradation_motor_id = motor_id;
    motor_degradation_loss_pct = constrain_float(loss_pct, 0.0f, MOTOR_DEG_MAX_LOSS_PCT);
    motor_degradation_yaw_free = yaw_free;
    motor_degradation_last_rx_ms = AP_HAL::millis();
}

void ModeAdaptive::clear_motor_degradation_command()
{
    motor_degradation_enabled = false;
    motor_degradation_yaw_free = true;
    motor_degradation_motor_id = 0;
    motor_degradation_loss_pct = 0.0f;
    motor_degradation_last_rx_ms = 0U;
}

bool ModeAdaptive::motor_degradation_command_fresh(uint32_t now_ms) const
{
    return motor_degradation_enabled &&
           motor_degradation_last_rx_ms != 0U &&
           (now_ms - motor_degradation_last_rx_ms) <= MOTOR_DEG_WATCHDOG_MS;
}

void ModeAdaptive::clear_auto_motor_fault()
{
    motor_fault_confirmed = false;
    motor_fault_yaw_free_latched = false;
    motor_fault_detected_id = 0;
    motor_fault_candidate_id = 0;
    motor_fault_confirm_count = 0;
    motor_fault_recovery_count = 0;
    motor_fault_loss_estimate_pct = 0.0f;
    motor_fault_residual_ratio = 1.0f;
    motor_fault_sigma_filtered.x = 0.0f;
    motor_fault_sigma_filtered.y = 0.0f;
    motor_fault_sigma_filtered.z = 0.0f;
    motor_fault_sigma_baseline.x = 0.0f;
    motor_fault_sigma_baseline.y = 0.0f;
    motor_fault_sigma_baseline.z = 0.0f;
    motor_fault_sigma_valid = false;
    mode29_zero(motor_fault_nominal_prev);
    motor_fault_nominal_prev_valid = false;
}


void ModeAdaptive::reset_gain_schedule()
{
    gain_schedule_loss_raw_pct = 0.0f;
    gain_schedule_loss_sched_pct = 0.0f;
    gain_schedule_confidence = 0.0f;

    gain_schedule_active.kpx = g.GeoCtrl_Kpx;
    gain_schedule_active.kpy = g.GeoCtrl_Kpy;
    gain_schedule_active.kpz = g.GeoCtrl_Kpz;
    gain_schedule_active.kvx = g.GeoCtrl_Kvx;
    gain_schedule_active.kvy = g.GeoCtrl_Kvy;
    gain_schedule_active.kvz = g.GeoCtrl_Kvz;
    gain_schedule_active.krx = g.GeoCtrl_KRx;
    gain_schedule_active.kry = g.GeoCtrl_KRy;
    gain_schedule_active.krz = g.GeoCtrl_KRz;
    gain_schedule_active.kox = g.GeoCtrl_KOx;
    gain_schedule_active.koy = g.GeoCtrl_KOy;
    gain_schedule_active.koz = g.GeoCtrl_KOz;
    gain_schedule_active.max_tilt_deg =
        constrain_float(maxTiltDeg, 5.0f, 60.0f);
}

ModeAdaptive::GainScheduleSet ModeAdaptive::gain_schedule_at_loss(float loss_pct) const
{
    const GainScheduleSet base = {
        (float)g.GeoCtrl_Kpx,
        (float)g.GeoCtrl_Kpy,
        (float)g.GeoCtrl_Kpz,
        (float)g.GeoCtrl_Kvx,
        (float)g.GeoCtrl_Kvy,
        (float)g.GeoCtrl_Kvz,
        (float)g.GeoCtrl_KRx,
        (float)g.GeoCtrl_KRy,
        (float)g.GeoCtrl_KRz,
        (float)g.GeoCtrl_KOx,
        (float)g.GeoCtrl_KOy,
        (float)g.GeoCtrl_KOz,
        constrain_float(maxTiltDeg, 5.0f, 60.0f)
    };

    const GainScheduleSet anchors[6] = {
        {(float)g.m29_g50_kpx,  (float)g.m29_g50_kpy,  (float)g.m29_g50_kpz,
         (float)g.m29_g50_kvx,  (float)g.m29_g50_kvy,  (float)g.m29_g50_kvz,
         (float)g.m29_g50_krx,  (float)g.m29_g50_kry,  (float)g.m29_g50_krz,
         (float)g.m29_g50_kox,  (float)g.m29_g50_koy,  (float)g.m29_g50_koz,
         (float)g.m29_g50_tilt},
        {(float)g.m29_g60_kpx,  (float)g.m29_g60_kpy,  (float)g.m29_g60_kpz,
         (float)g.m29_g60_kvx,  (float)g.m29_g60_kvy,  (float)g.m29_g60_kvz,
         (float)g.m29_g60_krx,  (float)g.m29_g60_kry,  (float)g.m29_g60_krz,
         (float)g.m29_g60_kox,  (float)g.m29_g60_koy,  (float)g.m29_g60_koz,
         (float)g.m29_g60_tilt},
        {(float)g.m29_g70_kpx,  (float)g.m29_g70_kpy,  (float)g.m29_g70_kpz,
         (float)g.m29_g70_kvx,  (float)g.m29_g70_kvy,  (float)g.m29_g70_kvz,
         (float)g.m29_g70_krx,  (float)g.m29_g70_kry,  (float)g.m29_g70_krz,
         (float)g.m29_g70_kox,  (float)g.m29_g70_koy,  (float)g.m29_g70_koz,
         (float)g.m29_g70_tilt},
        {(float)g.m29_g80_kpx,  (float)g.m29_g80_kpy,  (float)g.m29_g80_kpz,
         (float)g.m29_g80_kvx,  (float)g.m29_g80_kvy,  (float)g.m29_g80_kvz,
         (float)g.m29_g80_krx,  (float)g.m29_g80_kry,  (float)g.m29_g80_krz,
         (float)g.m29_g80_kox,  (float)g.m29_g80_koy,  (float)g.m29_g80_koz,
         (float)g.m29_g80_tilt},
        {(float)g.m29_g90_kpx,  (float)g.m29_g90_kpy,  (float)g.m29_g90_kpz,
         (float)g.m29_g90_kvx,  (float)g.m29_g90_kvy,  (float)g.m29_g90_kvz,
         (float)g.m29_g90_krx,  (float)g.m29_g90_kry,  (float)g.m29_g90_krz,
         (float)g.m29_g90_kox,  (float)g.m29_g90_koy,  (float)g.m29_g90_koz,
         (float)g.m29_g90_tilt},
        {(float)g.m29_g100_kpx, (float)g.m29_g100_kpy, (float)g.m29_g100_kpz,
         (float)g.m29_g100_kvx, (float)g.m29_g100_kvy, (float)g.m29_g100_kvz,
         (float)g.m29_g100_krx, (float)g.m29_g100_kry, (float)g.m29_g100_krz,
         (float)g.m29_g100_kox, (float)g.m29_g100_koy, (float)g.m29_g100_koz,
         (float)g.m29_g100_tilt}
    };

    const auto interpolate = [](const GainScheduleSet &a,
                                const GainScheduleSet &b,
                                float t) {
        t = constrain_float(t, 0.0f, 1.0f);
        GainScheduleSet out;
        out.kpx = a.kpx + (b.kpx - a.kpx) * t;
        out.kpy = a.kpy + (b.kpy - a.kpy) * t;
        out.kpz = a.kpz + (b.kpz - a.kpz) * t;
        out.kvx = a.kvx + (b.kvx - a.kvx) * t;
        out.kvy = a.kvy + (b.kvy - a.kvy) * t;
        out.kvz = a.kvz + (b.kvz - a.kvz) * t;
        out.krx = a.krx + (b.krx - a.krx) * t;
        out.kry = a.kry + (b.kry - a.kry) * t;
        out.krz = a.krz + (b.krz - a.krz) * t;
        out.kox = a.kox + (b.kox - a.kox) * t;
        out.koy = a.koy + (b.koy - a.koy) * t;
        out.koz = a.koz + (b.koz - a.koz) * t;
        out.max_tilt_deg =
            constrain_float(a.max_tilt_deg + (b.max_tilt_deg - a.max_tilt_deg) * t,
                            5.0f,
                            60.0f);
        return out;
    };

    const float loss = constrain_float(loss_pct, 0.0f, 100.0f);

    // Preserve the proven normal controller below 45% loss. Blend smoothly
    // into the first 50% calibrated anchor over 45..50%.
    if (loss <= 45.0f) {
        return base;
    }
    if (loss < 50.0f) {
        return interpolate(base, anchors[0], (loss - 45.0f) / 5.0f);
    }

    if (loss >= 100.0f) {
        return anchors[5];
    }

    const uint8_t lower = (uint8_t)constrain_int16(
        (int16_t)((loss - 50.0f) / 10.0f),
        0,
        4
    );
    const float lower_loss = 50.0f + 10.0f * lower;
    const float t = (loss - lower_loss) / 10.0f;
    return interpolate(anchors[lower], anchors[lower + 1U], t);
}

void ModeAdaptive::update_gain_schedule(bool motor_degradation_active)
{
    const int8_t mode = constrain_int16((int16_t)g.m29_gs_mode, 0, 2);
    float raw_loss = 0.0f;
    float confidence = 0.0f;

    if (mode == 1) {
        // Oracle/calibration mode: use the known injected loss only for
        // scheduling. This bypasses FDI so each anchor can be tuned
        // independently and repeatably.
        if (motor_degradation_active) {
            raw_loss = constrain_float(motor_degradation_loss_pct, 0.0f, 100.0f);
            confidence = 1.0f;
        }
    } else if (mode == 2) {
        // Automatic mode: use the onboard blind-FDI severity estimate.
        const float fit_limit = 0.55f;
        const float fit_confidence = constrain_float(
            1.0f - motor_fault_residual_ratio / fit_limit,
            0.0f,
            1.0f
        );
        const bool estimate_valid =
            isfinite(motor_fault_loss_estimate_pct) &&
            isfinite(motor_fault_residual_ratio) &&
            motor_fault_loss_estimate_pct >= 40.0f &&
            (motor_fault_confirmed || motor_fault_residual_ratio <= fit_limit);

        if (estimate_valid) {
            raw_loss = constrain_float(motor_fault_loss_estimate_pct, 0.0f, 100.0f);
            confidence = motor_fault_confirmed ? 1.0f : fit_confidence;
        }
    }

    gain_schedule_loss_raw_pct = raw_loss;
    gain_schedule_confidence = confidence;

    if (mode == 0) {
        gain_schedule_loss_sched_pct = 0.0f;
        gain_schedule_active = gain_schedule_at_loss(0.0f);
        return;
    }

    // Confidence gate, LPF and rate limiter prevent gain/tilt chatter when
    // automatic severity estimation is noisy. Oracle mode passes through the
    // same smooth transition so the fault step does not create a parameter step.
    const float target_loss = confidence >= 0.20f ? raw_loss : 0.0f;
    const float lpf_target =
        gain_schedule_loss_sched_pct +
        0.08f * (target_loss - gain_schedule_loss_sched_pct);
    const float delta = constrain_float(
        lpf_target - gain_schedule_loss_sched_pct,
        -2.5f,
        2.5f
    );
    gain_schedule_loss_sched_pct =
        constrain_float(gain_schedule_loss_sched_pct + delta, 0.0f, 100.0f);

    gain_schedule_active =
        gain_schedule_at_loss(gain_schedule_loss_sched_pct);
}

void ModeAdaptive::update_auto_motor_fault_detector(float time_in_this_run)
{
    // IMPORTANT: this detector is intentionally blind to the fault injector.
    // It never reads motor_degradation_motor_id or motor_degradation_loss_pct.
    // Before confirmation it identifies the motor from the L1 matched-moment
    // estimate. After confirmation it keeps estimating that motor's remaining
    // effectiveness from the residual created by the effectiveness-aware
    // allocator. This also provides a blind recovery path when the actuator
    // returns to normal.
    Vector3f sigma_now = {
        sigma_m_hat_prev[1],
        sigma_m_hat_prev[2],
        sigma_m_hat_prev[3]
    };

    if (!mode29_finite(sigma_now)) {
        motor_fault_candidate_id = 0;
        motor_fault_confirm_count = 0;
        motor_fault_recovery_count = 0;
        return;
    }

    if (!motor_fault_sigma_valid) {
        motor_fault_sigma_filtered = sigma_now;
        motor_fault_sigma_baseline = sigma_now;
        motor_fault_sigma_valid = true;
        return;
    }

    constexpr float sigma_alpha = 0.05f;
    constexpr float baseline_alpha_fast = 0.02f;
    constexpr float baseline_alpha_slow = 0.0005f;
    constexpr float confirmed_loss_update_gain = 0.05f;

    motor_fault_sigma_filtered =
        motor_fault_sigma_filtered +
        (sigma_now - motor_fault_sigma_filtered) * sigma_alpha;

    const bool detector_gate =
        motors->armed() &&
        trajIndex == 0 &&
        !g.LandFlag &&
        g.l1enable != 0 &&
        time_in_this_run >= (takeoffTime + settleTime) &&
        motor_fault_nominal_prev_valid;

    if (!detector_gate) {
        motor_fault_sigma_baseline =
            motor_fault_sigma_baseline +
            (motor_fault_sigma_filtered - motor_fault_sigma_baseline) *
                baseline_alpha_fast;
        motor_fault_candidate_id = 0;
        motor_fault_confirm_count = 0;
        motor_fault_recovery_count = 0;
        motor_fault_loss_estimate_pct = 0.0f;
        motor_fault_residual_ratio = 1.0f;
        return;
    }

    const Vector3f observed =
        motor_fault_sigma_filtered - motor_fault_sigma_baseline;
    const float observed_rp =
        sqrtf(observed.x * observed.x + observed.y * observed.y);
    const float observed_norm = observed.length();

#if (!REAL_OR_SITL)
    const float detector_L = 0.25f;
    const float detector_D = 0.25f;
    const float detector_a_F = 0.0014597f;
    const float detector_b_F = 0.043693f;
    const float detector_a_M = 0.000011667f;
    const float detector_b_M = 0.0059137f;
#elif (REAL_OR_SITL)
    const float detector_L = 0.28f;
    const float detector_D = 0.28f;
#endif

    const auto motor_loss_signature = [&](uint8_t motor_index, float w) {
#if (!REAL_OR_SITL)
        const float f_nom = detector_a_F * w * w + detector_b_F * w;
        const float m_nom = detector_a_M * w * w + detector_b_M * w;
#elif (REAL_OR_SITL)
        const float f_nom = softdrone_thrust_from_w(w);
        const float m_nom = softdrone_moment_from_w(w);
#endif

        Vector3f signature;
        switch (motor_index) {
        case 0: // M1 front-right, CCW
            signature.x = +0.5f * detector_L * f_nom;
            signature.y = -0.5f * detector_D * f_nom;
            signature.z = -m_nom;
            break;
        case 1: // M2 rear-left, CCW
            signature.x = -0.5f * detector_L * f_nom;
            signature.y = +0.5f * detector_D * f_nom;
            signature.z = -m_nom;
            break;
        case 2: // M3 front-left, CW
            signature.x = -0.5f * detector_L * f_nom;
            signature.y = -0.5f * detector_D * f_nom;
            signature.z = +m_nom;
            break;
        default: // M4 rear-right, CW
            signature.x = +0.5f * detector_L * f_nom;
            signature.y = +0.5f * detector_D * f_nom;
            signature.z = +m_nom;
            break;
        }
        return signature;
    };

    // Once a severe fault is confirmed, keep the isolated motor id but continue
    // estimating the loss percentage.  With an effectiveness-aware allocator,
    // a correctly estimated loss produces near-zero residual.  If the motor
    // recovers, the allocator temporarily over-compensates and the projection
    // becomes negative, which drives the loss estimate back toward zero.
    if (motor_fault_confirmed) {
        if (motor_fault_detected_id < 1 || motor_fault_detected_id > 4) {
            clear_auto_motor_fault();
            return;
        }

        const uint8_t idx = motor_fault_detected_id - 1U;
        const float w =
            constrain_float(motor_fault_nominal_prev[idx], 0.0f, 100.0f);
        const Vector3f signature = motor_loss_signature(idx, w);
        const float signature_norm_sq = signature * signature;

        if (!isfinite(signature_norm_sq) || signature_norm_sq < 1.0e-6f ||
            !isfinite(observed_norm)) {
            motor_fault_recovery_count = 0;
            return;
        }

        const float signed_residual_fraction =
            (observed * signature) / signature_norm_sq;
        const Vector3f fit_error =
            observed - signature * signed_residual_fraction;
        const float residual_ratio =
            fit_error.length() / MAX(observed_norm, 1.0e-4f);

        motor_fault_residual_ratio =
            isfinite(residual_ratio) ? residual_ratio : 1.0f;

        // Only adapt the effectiveness estimate when the residual still looks
        // like the isolated motor's signature.  This prevents unrelated motion
        // or mocap transients from walking the estimate.
        if (isfinite(signed_residual_fraction) &&
            (observed_norm < 1.0e-4f ||
             motor_fault_residual_ratio <= 0.55f)) {
            float loss_fraction =
                constrain_float(motor_fault_loss_estimate_pct * 0.01f,
                                0.0f,
                                1.0f);
            loss_fraction = constrain_float(
                loss_fraction +
                    confirmed_loss_update_gain * signed_residual_fraction,
                0.0f,
                1.0f
            );
            motor_fault_loss_estimate_pct = 100.0f * loss_fraction;
        }

        const bool recovered =
            motor_fault_loss_estimate_pct <=
                100.0f * MOTOR_FDI_RELEASE_LOSS_FRACTION &&
            (observed_norm < MOTOR_FDI_MIN_RP_MOMENT ||
             motor_fault_residual_ratio <= 0.55f);

        if (recovered) {
            if (motor_fault_recovery_count < 65535U) {
                motor_fault_recovery_count++;
            }
        } else {
            motor_fault_recovery_count = 0;
        }

        if (motor_fault_recovery_count >= MOTOR_FDI_RECOVER_SAMPLES) {
            const uint8_t recovered_motor = motor_fault_detected_id;
            GCS_SEND_TEXT(MAV_SEVERITY_INFO,
                          "Mode29 FDI: M%u recovered, loss %.0f%%",
                          (unsigned)recovered_motor,
                          (double)motor_fault_loss_estimate_pct);

            // Re-anchor the healthy baseline at the current observer state so
            // the recovery transient does not immediately retrigger the FDI.
            motor_fault_confirmed = false;
            motor_fault_detected_id = 0;
            motor_fault_candidate_id = 0;
            motor_fault_confirm_count = 0;
            motor_fault_recovery_count = 0;
            motor_fault_loss_estimate_pct = 0.0f;
            motor_fault_residual_ratio = 1.0f;
            motor_fault_sigma_baseline = motor_fault_sigma_filtered;
        }
        return;
    }

    if (!isfinite(observed_norm) ||
        observed_rp < MOTOR_FDI_MIN_RP_MOMENT) {
        motor_fault_sigma_baseline =
            motor_fault_sigma_baseline +
            (motor_fault_sigma_filtered - motor_fault_sigma_baseline) *
                baseline_alpha_slow;
        motor_fault_candidate_id = 0;
        motor_fault_confirm_count = 0;
        motor_fault_recovery_count = 0;
        motor_fault_loss_estimate_pct = 0.0f;
        motor_fault_residual_ratio = 1.0f;
        return;
    }

    uint8_t best_motor = 0;
    float best_loss = 0.0f;
    float best_ratio = 999.0f;

    for (uint8_t i = 0; i < 4; i++) {
        const float w =
            constrain_float(motor_fault_nominal_prev[i], 0.0f, 100.0f);
        const Vector3f signature = motor_loss_signature(i, w);
        const float signature_norm_sq = signature * signature;

        if (!isfinite(signature_norm_sq) || signature_norm_sq < 1.0e-6f) {
            continue;
        }

        const float loss_fraction = constrain_float(
            (observed * signature) / signature_norm_sq,
            0.0f,
            1.0f
        );
        const Vector3f fit_error =
            observed - signature * loss_fraction;
        const float residual_ratio =
            fit_error.length() / MAX(observed_norm, 1.0e-4f);

        if (isfinite(residual_ratio) && residual_ratio < best_ratio) {
            best_ratio = residual_ratio;
            best_loss = loss_fraction;
            best_motor = i + 1U;
        }
    }

    motor_fault_loss_estimate_pct = 100.0f * best_loss;
    motor_fault_residual_ratio = best_ratio;

    const bool severe_candidate =
        best_motor != 0 &&
        best_loss >= MOTOR_FDI_MIN_LOSS_FRACTION &&
        best_ratio <= MOTOR_FDI_MAX_RESIDUAL_RATIO;

    if (!severe_candidate) {
        motor_fault_sigma_baseline =
            motor_fault_sigma_baseline +
            (motor_fault_sigma_filtered - motor_fault_sigma_baseline) *
                baseline_alpha_slow;
        motor_fault_candidate_id = 0;
        motor_fault_confirm_count = 0;
        motor_fault_recovery_count = 0;
        return;
    }

    if (motor_fault_candidate_id == best_motor) {
        if (motor_fault_confirm_count < 65535U) {
            motor_fault_confirm_count++;
        }
    } else {
        motor_fault_candidate_id = best_motor;
        motor_fault_confirm_count = 1;
    }

    if (motor_fault_confirm_count >= MOTOR_FDI_CONFIRM_SAMPLES) {
        motor_fault_confirmed = true;
        motor_fault_yaw_free_latched = true;
        motor_fault_detected_id = best_motor;
        motor_fault_recovery_count = 0;
        GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL,
                      "Mode29 FDI: M%u severe loss %.0f%%",
                      (unsigned)motor_fault_detected_id,
                      (double)motor_fault_loss_estimate_pct);
    }
}

void ModeAdaptive::run()
{
    static uint32_t initialTime = 0; // store previous system time

    if (!motors->armed())
    {
        // Motors should be Stopped
        motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::SHUT_DOWN);
    }
    else if (copter.ap.throttle_zero)
    {
        // Attempting to Land
        motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::GROUND_IDLE);
    }
    else
    {
        motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);
    }

    switch (motors->get_spool_state())
    {
    case AP_Motors::SpoolState::SHUT_DOWN:
        // Motors Stopped
        attitude_control->reset_yaw_target_and_rate();
        attitude_control->reset_rate_controller_I_terms();
        break;

    case AP_Motors::SpoolState::GROUND_IDLE:
        // Landed
        attitude_control->reset_yaw_target_and_rate();
        attitude_control->reset_rate_controller_I_terms_smoothly();
        break;

    case AP_Motors::SpoolState::THROTTLE_UNLIMITED:
        // clear landing flag above zero throttle
        if (!motors->limit.throttle_lower)
        {
            set_land_complete(false);
        }
        break;

    case AP_Motors::SpoolState::SPOOLING_UP:
    case AP_Motors::SpoolState::SPOOLING_DOWN:
        // do nothing
        break;
    }

    // Do not execute any custom trajectory/controller/mixer math while
    // disarmed. This prevents stale or invalid controller state from creating
    // a latched ArduPilot internal error after an emergency disarm.
    if (!motors->armed()) {
        clear_motor_degradation_command();
        clear_auto_motor_fault();
        reset_gain_schedule();
        return;
    }

    const auto abort_mode29 = [this](const char *reason) {
        GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL, "Mode29 abort: %s", reason);
        clear_motor_degradation_command();
        motorEnable = 0;

        // Preserve ARM state and hand control back to the normal ArduCopter
        // controller. If that mode transition ever fails, command minimum
        // output as the final containment action.
        if (!copter.set_mode(Mode::Number::STABILIZE, ModeReason::UNKNOWN)) {
            for (uint8_t i = 0; i < 4; i++) {
                motors->rc_write(i, 1000);
            }
        }
    };

    // ===================================================
    // start custom code by ACRL

    // load current time
    uint32_t tnow = 0;
    float currentTime = 0;              // This is the duration since the first time the Adaptive flight mode is entered.
    static float currentTimeLast = 0;   // This variable stores the previous value of currentTime.
    float timeInThisRun = 0;            // This is the duration since the Adaptive flight mode is entered most recently.
    static float timeBiasInThisRun = 0; // This is the most recent time that the Adaptive mode is entered
    if (initialTime == 0)
    {
        initialTime = AP_HAL::micros();
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Entering Adaptive mode for the first time.");
    }
    else
    {
        tnow = AP_HAL::micros();
        currentTime = 0.000001f * (tnow - initialTime);
        if (currentTime - currentTimeLast <= 0.1) // this means the Adaptive mode hasn't been changed.
        {
            timeInThisRun = currentTime - timeBiasInThisRun;
        }
        else // reset timeInThisRun
        {
            timeBiasInThisRun = currentTime;
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Start time for this run: %f s.", currentTime);
        }
    }

    Vector3f targetPos;
    Vector3f targetVel;
    Vector3f targetAcc;
    Vector3f targetJerk;
    Vector3f targetSnap;
    Vector2f targetYaw;
    Vector2f targetYaw_dot;
    Vector2f targetYaw_ddot;

    // Evaluate trajectories. The base altitude and takeoff duration are
    // runtime AP_Param values rather than firmware constants.
    if (timeInThisRun < takeoffTime)
    {
        ACRL_trajectory_takeoff(timeInThisRun,
                                takeoffAlt,
                                takeoffTime,
                                &targetPos,
                                &targetVel,
                                &targetAcc,
                                &targetJerk,
                                &targetSnap,
                                &targetYaw,
                                &targetYaw_dot,
                                &targetYaw_ddot);
    }
    else
    {
        switch (trajIndex)
        {
        case 0: // hover at the runtime-configured altitude above NED origin
        {
            targetPos = (Vector3f){0, 0, -takeoffAlt};
            targetVel = (Vector3f){0, 0, 0};
            targetAcc = (Vector3f){0, 0, 0};
            targetJerk = (Vector3f){0, 0, 0};
            targetSnap = (Vector3f){0, 0, 0};
            targetYaw = (Vector2f){1, 0};
            targetYaw_dot = (Vector2f){0, 0};
            targetYaw_ddot = (Vector2f){0, 0};
            break;
        }
        case 1: // circular trajectory with variable yaw
        {
            #if (!REAL_OR_SITL) // SITL
                const float timeOffset = takeoffTime;
                ACRL_trajectory_circle_variable_yaw(timeInThisRun, radiusX, takeoffAlt, timeOffset, targetSpeed, &targetPos, &targetVel, &targetAcc, &targetJerk, &targetSnap, &targetYaw, &targetYaw_dot, &targetYaw_ddot);
            #elif (REAL_OR_SITL) // Real
                const float transitionDuration = 2.0f;
                if (timeInThisRun < takeoffTime + transitionDuration)
                {
                    // Transition horizontally from the takeoff point to the
                    // circle start while preserving the configured altitude.
                    const float timeOffset = takeoffTime;
                    ACRL_trajectory_transition_to_start(timeInThisRun, radiusX, takeoffAlt, timeOffset, &targetPos, &targetVel, &targetAcc, &targetJerk, &targetSnap, &targetYaw, &targetYaw_dot, &targetYaw_ddot);
                }
                else
                {
                    const float timeOffset = takeoffTime + transitionDuration;
                    ACRL_trajectory_circle_variable_yaw(timeInThisRun, radiusX, takeoffAlt, timeOffset, targetSpeed, &targetPos, &targetVel, &targetAcc, &targetJerk, &targetSnap, &targetYaw, &targetYaw_dot, &targetYaw_ddot);
                }
            #endif
            break;
        }
        case 2: // circular trajectory with fixed yaw
        {
            #if (!REAL_OR_SITL) // SITL
                const float timeOffset = takeoffTime;
                ACRL_trajectory_circle_fixed_yaw(timeInThisRun, radiusX, takeoffAlt, timeOffset, targetSpeed, &targetPos, &targetVel, &targetAcc, &targetJerk, &targetSnap, &targetYaw, &targetYaw_dot, &targetYaw_ddot);
            #elif (REAL_OR_SITL) // Real
                const float transitionDuration = 2.0f;
                if (timeInThisRun < takeoffTime + transitionDuration)
                {
                    const float timeOffset = takeoffTime;
                    ACRL_trajectory_transition_to_start(timeInThisRun, radiusX, takeoffAlt, timeOffset, &targetPos, &targetVel, &targetAcc, &targetJerk, &targetSnap, &targetYaw, &targetYaw_dot, &targetYaw_ddot);
                }
                else
                {
                    const float timeOffset = takeoffTime + transitionDuration;
                    ACRL_trajectory_circle_fixed_yaw(timeInThisRun, radiusX, takeoffAlt, timeOffset, targetSpeed, &targetPos, &targetVel, &targetAcc, &targetJerk, &targetSnap, &targetYaw, &targetYaw_dot, &targetYaw_ddot);
                }
            #endif
            break;
        }
        case 3: // figure8 trajectory with fixed yaw
        {
            ACRL_trajectory_figure8_fixed_yaw(timeInThisRun, radiusX, radiusY, takeoffAlt, takeoffTime, targetSpeed, &targetPos, &targetVel, &targetAcc, &targetJerk, &targetSnap, &targetYaw, &targetYaw_dot, &targetYaw_ddot);
            break;
        }
        case 4: // figure8 trajectory with tilted altitude
        {
            ACRL_trajectory_figure8_tilted(timeInThisRun, radiusX, radiusY, takeoffAlt, takeoffTime, targetSpeed, &targetPos, &targetVel, &targetAcc, &targetJerk, &targetSnap, &targetYaw, &targetYaw_dot, &targetYaw_ddot);
            break;
        }
        default:
        {
            GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "Wrong trajectory index. Drone will hover.");
            targetPos = (Vector3f){0, 0, -takeoffAlt};
            targetVel = (Vector3f){0, 0, 0};
            targetAcc = (Vector3f){0, 0, 0};
            targetJerk = (Vector3f){0, 0, 0};
            targetSnap = (Vector3f){0, 0, 0};
            targetYaw = (Vector2f){1, 0};
            targetYaw_dot = (Vector2f){0, 0};
            targetYaw_ddot = (Vector2f){0, 0};
            break;
        }
        }
    }

    // Reject any non-finite trajectory before it reaches the controller.
    if (!mode29_finite(targetPos) ||
        !mode29_finite(targetVel) ||
        !mode29_finite(targetAcc) ||
        !mode29_finite(targetJerk) ||
        !mode29_finite(targetSnap) ||
        !mode29_finite(targetYaw) ||
        !mode29_finite(targetYaw_dot) ||
        !mode29_finite(targetYaw_ddot)) {
        abort_mode29("non-finite trajectory");
        return;
    }

    // initialize for landing mode
    if (g.LandFlag && !landingTriggered) 
    {
        landingTriggered = 1; // set landingTriggered to 1
        landingTimeOffset = timeInThisRun; // store the time offset
    }

    // executing landing mode
    if (g.LandFlag && landingTriggered) // switch to landing mode
    {   
        if(ahrs.get_relative_position_NED_origin(currentPosition)){;}// save current position
        if(ahrs.get_velocity_NED(currentVelocity)){;}
        currentYaw = ahrs.get_yaw(); // save current yaw
        if (currentPosition[2] >= -0.3) // if the initial altitude upon entering land mode is within 30 cm, then set landComplete to 1 to overwrite the motor throttle to 1.
        {   
            if (!landingComplete)
            {
                landingComplete = 1;
                gcs().send_text(MAV_SEVERITY_INFO, "Quadrotor is on the ground. Motor commands set to minimum.");
            }       
        }
        else 
        {
            float decRate = 1; // 1m/s^2
            landingComplete = ACRL_trajectory_land(timeInThisRun - landingTimeOffset, currentPosition, currentVelocity, currentYaw, decRate, &targetPos, &targetVel, &targetAcc, &targetJerk, &targetSnap, &targetYaw, &targetYaw_dot, &targetYaw_ddot);
        }   
    }

    // Orange Pi runtime control uses standard MAVLink RC_CHANNELS_OVERRIDE
    // on channels 9..12, leaving the pilot's normal flight-mode/arm channels
    // untouched:
    //   RC9  : <=1200 disable, >=1800 enable
    //   RC10 : motor selector (1..4)
    //   RC11 : 1000..2000 => 0..100% thrust-effectiveness loss
    //   RC12 : <=1200 keep yaw control, >=1800 yaw-free mode
    RC_Channel *deg_enable_ch = rc().channel(8);
    RC_Channel *deg_motor_ch = rc().channel(9);
    RC_Channel *deg_loss_ch = rc().channel(10);
    RC_Channel *deg_yaw_ch = rc().channel(11);

    const bool deg_override_valid =
        deg_enable_ch != nullptr && deg_enable_ch->has_override() &&
        deg_motor_ch != nullptr && deg_motor_ch->has_override() &&
        deg_loss_ch != nullptr && deg_loss_ch->has_override() &&
        deg_yaw_ch != nullptr && deg_yaw_ch->has_override();

    if (deg_override_valid) {
        const bool enable = deg_enable_ch->get_radio_in() >= 1800;

        uint8_t motor_id = 1;
        const uint16_t motor_pwm = deg_motor_ch->get_radio_in();
        if (motor_pwm >= 1750) {
            motor_id = 4;
        } else if (motor_pwm >= 1500) {
            motor_id = 3;
        } else if (motor_pwm >= 1250) {
            motor_id = 2;
        }

        const float loss_pct = constrain_float(
            (deg_loss_ch->get_radio_in() - 1000) * (MOTOR_DEG_MAX_LOSS_PCT / 1000.0f),
            0.0f,
            MOTOR_DEG_MAX_LOSS_PCT
        );
        const bool yaw_free = deg_yaw_ch->get_radio_in() >= 1800;

        set_motor_degradation_command(enable, motor_id, loss_pct, yaw_free);
    } else {
        clear_motor_degradation_command();
    }

    const uint32_t motor_deg_now_ms = AP_HAL::millis();
    const uint32_t motor_deg_age_ms =
        motor_degradation_last_rx_ms == 0U ? 999999U : motor_deg_now_ms - motor_degradation_last_rx_ms;

    // Motor derating is only permitted for the hover experiment after the
    // runtime-configured takeoff and settle intervals have both completed.
    const bool motor_degradation_active =
        motors->armed() &&
        trajIndex == 0 &&
        !g.LandFlag &&
        timeInThisRun >= (takeoffTime + settleTime) &&
        motor_degradation_loss_pct > 0.0f &&
        motor_degradation_command_fresh(motor_deg_now_ms);

    // Blind FDI uses the previous cycle's L1 matched-moment estimate and
    // nominal actuator commands. A confirmed severe fault automatically
    // releases yaw even when the injector itself requested keep-yaw.
    update_auto_motor_fault_detector(timeInThisRun);
    update_gain_schedule(motor_degradation_active);

    AP::logger().Write("L1GS",
                       "mode,raw,sched,conf,kpx,kpy,kpz,kvx,kvy,kvz",
                       "Bfffffffff",
                       (uint8_t)constrain_int16((int16_t)g.m29_gs_mode, 0, 2),
                       (double)gain_schedule_loss_raw_pct,
                       (double)gain_schedule_loss_sched_pct,
                       (double)gain_schedule_confidence,
                       (double)gain_schedule_active.kpx,
                       (double)gain_schedule_active.kpy,
                       (double)gain_schedule_active.kpz,
                       (double)gain_schedule_active.kvx,
                       (double)gain_schedule_active.kvy,
                       (double)gain_schedule_active.kvz);

    AP::logger().Write("L1GA",
                       "krx,kry,krz,kox,koy,koz,tilt",
                       "fffffff",
                       (double)gain_schedule_active.krx,
                       (double)gain_schedule_active.kry,
                       (double)gain_schedule_active.krz,
                       (double)gain_schedule_active.kox,
                       (double)gain_schedule_active.koy,
                       (double)gain_schedule_active.koz,
                       (double)gain_schedule_active.max_tilt_deg);

    const bool yaw_free_active =
        motor_fault_yaw_free_latched ||
        (motor_degradation_active && motor_degradation_yaw_free);

    if (yaw_free_active) {
        // Keep estimating yaw, but stop asking the aircraft to return to a
        // fixed heading. The explicit yaw torque is removed below.
        const float yaw_now = ahrs.get_yaw();
        targetYaw = (Vector2f){cosf(yaw_now), sinf(yaw_now)};
        targetYaw_dot = (Vector2f){0, 0};
        targetYaw_ddot = (Vector2f){0, 0};
    }

    VectorN<float, 4> thrustMomentCmd;
    thrustMomentCmd = geometricController(targetPos, targetVel, targetAcc, targetJerk, targetSnap, targetYaw, targetYaw_dot, targetYaw_ddot);

    if (!mode29_finite(thrustMomentCmd) || thrustMomentCmd[0] <= 0.0f) {
        abort_mode29("invalid geometric-controller output");
        return;
    }

    if (yaw_free_active) {
        thrustMomentCmd[3] = 0.0f;
    }

    uint8_t LandFlag = 0;
    LandFlag = g.LandFlag;
    AP::logger().Write("L1AB", "thrust,mx,my,mz,landflag,landtrig,landcomp", "ffffBBB",
                       (double)thrustMomentCmd[0],
                       (double)thrustMomentCmd[1],
                       (double)thrustMomentCmd[2],
                       (double)thrustMomentCmd[3],
                       LandFlag,
                       landingTriggered,
                       landingComplete);

    // L1 adaptive augmentation. When L1ENABLE=0, bypass the entire adaptive
    // computation instead of calculating it and multiplying by zero later:
    // IEEE NaN * 0 is still NaN and was the source of a Mode 29 flyaway.
    VectorN<float, 4> L1thrustMomentCmd;
    mode29_zero(L1thrustMomentCmd);

    if (g.l1enable != 0) {
        L1thrustMomentCmd =
            L1AdaptiveAugmentation(thrustMomentCmd, yaw_free_active);

        if (!mode29_finite(L1thrustMomentCmd)) {
            abort_mode29("non-finite L1 output");
            return;
        }
    } else {
        // Keep the dormant predictor state finite and synchronised so a later
        // disarmed configuration change cannot revive stale NaN state.
        Vector3f v_now;
        if (ahrs.get_velocity_NED(v_now) && mode29_finite(v_now)) {
            v_prev = v_now;
            v_hat_prev = v_now;
        }

        const Vector3f omega_now = AP::ahrs().get_gyro();
        if (mode29_finite(omega_now)) {
            omega_prev = omega_now;
            omega_hat_prev = omega_now;
        }

        Quaternion q_now;
        ahrs.get_quat_body_to_ned(q_now);
        if (isfinite(q_now.q1) && isfinite(q_now.q2) &&
            isfinite(q_now.q3) && isfinite(q_now.q4)) {
            q_now.rotation_matrix(R_prev);
        }

        u_b_prev = thrustMomentCmd;
        mode29_zero(u_ad_prev);
        mode29_zero(sigma_m_hat_prev);
        mode29_zero(lpf1_prev);
        mode29_zero(lpf2_prev);
        sigma_um_hat_prev[0] = 0.0f;
        sigma_um_hat_prev[1] = 0.0f;
    }

    // uncomment the lines below if you want to inject uncertainty to the control channels
    // thrustMomentCmd[0] = thrustMomentCmd[0] + 5 * sinf(0.5 * currentTime);
    // thrustMomentCmd[1] = thrustMomentCmd[1] + 0.1 * sinf( currentTime);
    // thrustMomentCmd[2] = thrustMomentCmd[2] + 0.05 * sinf( 2 * currentTime);

    // Motor allocation. Normal Mode 29 keeps the original full F/Mx/My/Mz
    // mixer. Fault mode intentionally drops the yaw-moment objective so the
    // remaining actuator authority is spent on thrust, roll and pitch.
    VectorN<float, 4> motorPWMCommanded;
    if (motor_fault_confirmed) {
        motorPWMCommanded =
            motorMixingYawFreeEffectivenessAware(
                thrustMomentCmd + L1thrustMomentCmd,
                motor_fault_detected_id,
                motor_fault_loss_estimate_pct);
    } else if (yaw_free_active) {
        motorPWMCommanded =
            motorMixingYawFree(thrustMomentCmd + L1thrustMomentCmd);
    } else {
        motorPWMCommanded =
            motorMixing(thrustMomentCmd + L1thrustMomentCmd);
    }

    // Never pass NaN/Inf into constrain_float(). ArduPilot intentionally
    // latches constraining_nan as an internal error, which then blocks future
    // arming until reboot.
    if (!mode29_finite(motorPWMCommanded)) {
        abort_mode29("non-finite motor allocation");
        return;
    }

    // Saturate the nominal actuator request before the injected effectiveness
    // loss. Therefore an 80%-effective motor can never recover 100% nominal
    // thrust by asking for more than the normal actuator limit.
    for (uint8_t i = 0; i < 4; i++) {
        motorPWMCommanded[i] = constrain_float(motorPWMCommanded[i], 0.0f, 100.0f);
    }

    motor_fault_nominal_prev = motorPWMCommanded;
    motor_fault_nominal_prev_valid = true;

    VectorN<float, 4> motorPWM = motorPWMCommanded;

    if (motor_degradation_active) {
        const uint8_t motor_index = motor_degradation_motor_id - 1;
        const float effectiveness = 1.0f - motor_degradation_loss_pct * 0.01f;

        const float w_nom = motorPWMCommanded[motor_index];
#if (!REAL_OR_SITL)
        const float deg_a_F = 0.0014597f;
        const float deg_b_F = 0.043693f;
        const float thrust_nom = deg_a_F * w_nom * w_nom + deg_b_F * w_nom;
        const float thrust_applied = MAX(0.0f, effectiveness * thrust_nom);
        const float disc = deg_b_F * deg_b_F + 4.0f * deg_a_F * thrust_applied;
        motorPWM[motor_index] =
            (-deg_b_F + sqrtF(MAX(0.0f, disc))) / (2.0f * deg_a_F);
#elif (REAL_OR_SITL)
        const float thrust_nom = softdrone_thrust_from_w(w_nom);
        const float thrust_applied = MAX(0.0f, effectiveness * thrust_nom);
        motorPWM[motor_index] = softdrone_w_from_thrust(thrust_applied);
#endif
        motorPWM[motor_index] = constrain_float(motorPWM[motor_index], 0.0f, 100.0f);
    }

    // disarm the vehicle by setting PWM to 1 when landing is completed
    if (landingComplete)
    {
        motorPWM[0] = 1;
        motorPWM[1] = 1;
        motorPWM[2] = 1;
        motorPWM[3] = 1;
    }

    AP::logger().Write("L1DG",
                       "active,yawfree,motor,loss,age,c1,c2,c3,c4,a1,a2,a3,a4",
                       "BBBfIffffffff",
                       (uint8_t)motor_degradation_active,
                       (uint8_t)yaw_free_active,
                       motor_degradation_motor_id,
                       (double)motor_degradation_loss_pct,
                       motor_deg_age_ms,
                       (double)motorPWMCommanded[0],
                       (double)motorPWMCommanded[1],
                       (double)motorPWMCommanded[2],
                       (double)motorPWMCommanded[3],
                       (double)motorPWM[0],
                       (double)motorPWM[1],
                       (double)motorPWM[2],
                       (double)motorPWM[3]);

    const uint8_t fdi_state =
        motor_fault_confirmed ? 2U :
        (motor_fault_candidate_id != 0 ? 1U : 0U);
    AP::logger().Write("L1FD",
                       "state,motor,cand,count,rcnt,loss,res,smx,smy,smz",
                       "BBBHHfffff",
                       fdi_state,
                       motor_fault_detected_id,
                       motor_fault_candidate_id,
                       motor_fault_confirm_count,
                       motor_fault_recovery_count,
                       (double)motor_fault_loss_estimate_pct,
                       (double)motor_fault_residual_ratio,
                       (double)(sigma_m_hat_prev[1]),
                       (double)(sigma_m_hat_prev[2]),
                       (double)(sigma_m_hat_prev[3]));

    if (motors->armed()) // only command the motor PWM when the vehicle is armed.
    {
        motors->rc_write(0, 1000 + motorEnable * 10 * motorPWM[0]); // manual set motor speed: PWM_MIN/MAX has been forced to 1000/2000
        motors->rc_write(1, 1000 + motorEnable * 10 * motorPWM[1]); // rc_write is called from <AP_Motors/AP_Motors_Class.h>
        motors->rc_write(2, 1000 + motorEnable * 10 * motorPWM[2]);
        motors->rc_write(3, 1000 + motorEnable * 10 * motorPWM[3]);
    }
    else 
    {
        GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "Vehicle not armed.");
        motorEnable = 0; // if the vehicle is not armed, disable the flight.
    }
    currentTimeLast = currentTime; // store the value of currentTime

    // logging
    Vector3f statePos;

    int locAvailable = ahrs.get_relative_position_NED_origin(statePos);
    if (!locAvailable)
    {
        gcs().send_text(MAV_SEVERITY_CRITICAL, "Location unavailable.");
    }

    AP::logger().Write("L1AC", "currentT,thisRunT,xxd,yyd,zzd,xx,yy,zz,m1,m2,m3,m4", "ffffffffffff",
                       (double)currentTime,
                       (double)timeInThisRun,
                       (double)targetPos.x,
                       (double)targetPos.y,
                       (double)targetPos.z,
                       (double)statePos.x,
                       (double)statePos.y,
                       (double)statePos.z,
                       (double)motorPWM[0],
                       (double)motorPWM[1],
                       (double)motorPWM[2],
                       (double)motorPWM[3]);
    // end custom code by ACRL
    // ===================================================
}

VectorN<float, 4> ModeAdaptive::geometricController(Vector3f targetPos,
                                                    Vector3f targetVel,
                                                    Vector3f targetAcc,
                                                    Vector3f targetJerk,
                                                    Vector3f targetSnap,
                                                    Vector2f targetYaw,
                                                    Vector2f targetYaw_dot,
                                                    Vector2f targetYaw_ddot)
{
    Vector3f r_error;
    Vector3f v_error;
    Vector3f target_force;
    Vector3f z_axis;
    Vector3f x_axis_desired;
    Vector3f y_axis_desired;
    Vector3f x_c_des;
    Vector3f eR, ew, M;
    Vector3f e3 = {0, 0, 1};

    Vector3f statePos;
    Vector3f stateVel;

    Vector2f positionNE;

    int locAvailable = ahrs.get_relative_position_NED_origin(statePos);
    if (!locAvailable)
    {
        gcs().send_text(MAV_SEVERITY_CRITICAL, "location unavailable.");
    }

    // Ground velocity in meters/second, North/East/Down
    // order. Check if have_inertial_nav() is true before assigning values to stateVel.
    if (ahrs.have_inertial_nav())
    {
        if(ahrs.get_velocity_NED(stateVel)){;}
    }
    else
    {
        gcs().send_text(MAV_SEVERITY_CRITICAL, "inertial navigation is inactive");
    }

    // Position Error (ep)
    r_error = statePos - targetPos;

    // Velocity Error (ev)
    v_error = stateVel - targetVel;

    // Scheduled geometric-controller gains. L1 adaptation parameters remain
    // fixed; only the baseline geometric controller and its tilt envelope vary.
    const GainScheduleSet &gs = gain_schedule_active;
    target_force.x = kg_vehicleMass * targetAcc.x - gs.kpx * r_error.x - gs.kvx * v_error.x;
    target_force.y = kg_vehicleMass * targetAcc.y - gs.kpy * r_error.y - gs.kvy * v_error.y;
    target_force.z = kg_vehicleMass * (targetAcc.z - GRAVITY_MAGNITUDE) - gs.kpz * r_error.z - gs.kvz * v_error.z;

    // Safety-project the desired thrust vector into an upright cone.
    //
    // In NED, a normal upright vehicle has body +Z pointing down and thrust
    // acts along -body-Z. Therefore -target_force is the desired body +Z
    // direction. Without this guard, a large altitude overshoot can make the
    // raw target_force point downward, which asks the geometric controller to
    // rotate the aircraft through 90 deg and eventually fly inverted.
    //
    // The projection does two things:
    //   1. keeps a small positive upright component so inverted thrust is
    //      never requested; gravity can still provide downward acceleration;
    //   2. limits the horizontal/vertical ratio using the active scheduled
    //      tilt anchor (or M29_MAX_TILT when scheduling is disabled).
    bool thrust_vector_limited = false;
    Vector3f desired_body_z_force = -target_force;
    const float min_upright_force =
        0.05f * kg_vehicleMass * GRAVITY_MAGNITUDE;

    if (desired_body_z_force.z < min_upright_force) {
        desired_body_z_force.z = min_upright_force;
        thrust_vector_limited = true;
    }

    const float max_tilt_rad =
        constrain_float(gs.max_tilt_deg, 5.0f, 60.0f) * DEG_TO_RAD;
    const float horizontal_force =
        sqrtF(desired_body_z_force.x * desired_body_z_force.x +
              desired_body_z_force.y * desired_body_z_force.y);
    const float max_horizontal_force =
        desired_body_z_force.z * tanf(max_tilt_rad);

    if (horizontal_force > max_horizontal_force &&
        horizontal_force > 1.0e-6f) {
        const float horizontal_scale =
            max_horizontal_force / horizontal_force;
        desired_body_z_force.x *= horizontal_scale;
        desired_body_z_force.y *= horizontal_scale;
        thrust_vector_limited = true;
    }

    target_force = -desired_body_z_force;

    // Z-Axis [zB]
    Quaternion q;
    ahrs.get_quat_body_to_ned(q);

    Matrix3f R;
    q.rotation_matrix(R); // transforming the quaternion q to rotation matrix R

    z_axis = R.colz();

    // target thrust [F]
    float target_thrust = -target_force * z_axis;

    // Calculate axis [zB_des]
    Vector3f z_axis_desired = -target_force;
    z_axis_desired.normalize();

    // [xC_des]
    // x_axis_desired = z_axis_desired x [cos(yaw), sin(yaw), 0]^T
    x_c_des[0] = targetYaw[0]; // x
    x_c_des[1] = targetYaw[1]; // y
    x_c_des[2] = 0;            // z

    Vector3f x_c_des_dot = {targetYaw_dot, 0};   // time derivative of x_c_des
    Vector3f x_c_des_ddot = {targetYaw_ddot, 0}; // time derivative of x_c_des_dot

    // [yB_des]
    y_axis_desired = (z_axis_desired % x_c_des);
    y_axis_desired.normalize();
    // [xB_des]
    x_axis_desired = y_axis_desired % z_axis_desired;

    // [eR]
    Matrix3f Rdes(Vector3f(x_axis_desired.x, y_axis_desired.x, z_axis_desired.x),
                  Vector3f(x_axis_desired.y, y_axis_desired.y, z_axis_desired.y),
                  Vector3f(x_axis_desired.z, y_axis_desired.z, z_axis_desired.z));

    Matrix3f eRM = (Rdes.transposed() * R - R.transposed() * Rdes) / 2;
    eR = veeOperator(eRM);

    Vector3f Omega = AP::ahrs().get_gyro();

    // compute Omegad: this comes from Appendix F in https://arxiv.org/pdf/1003.2005v3.pdf
    Vector3f a_error; // error on acceleration
    a_error = e3 * GRAVITY_MAGNITUDE - R.colz() * target_thrust / kg_vehicleMass - targetAcc;

    Vector3f target_force_dot; // derivative of target_force
    target_force_dot.x = -gs.kpx * v_error.x - gs.kvx * a_error.x + kg_vehicleMass * targetJerk.x;
    target_force_dot.y = -gs.kpy * v_error.y - gs.kvy * a_error.y + kg_vehicleMass * targetJerk.y;
    target_force_dot.z = -gs.kpz * v_error.z - gs.kvz * a_error.z + kg_vehicleMass * targetJerk.z;

    // Once the safety projection is active, the analytical derivative of the
    // unconstrained force no longer represents the constrained command.
    // Zeroing its feed-forward derivatives avoids injecting a spurious desired
    // angular-rate command while the safety limiter is protecting the vehicle.
    if (thrust_vector_limited) {
        target_force_dot = (Vector3f){0, 0, 0};
    }

    Vector3f b3_dot = R * hatOperator(Omega) * e3;

    float target_thrust_dot = -target_force_dot * R.colz() - target_force * b3_dot;

    Vector3f j_error; // error on jerk
    j_error = -R.colz() * target_thrust_dot / kg_vehicleMass - b3_dot * target_thrust / kg_vehicleMass - targetJerk;

    Vector3f target_force_ddot; // derivative of target_force_dot
    target_force_ddot.x = -gs.kpx * a_error.x - gs.kvx * j_error.x + kg_vehicleMass * targetSnap.x;
    target_force_ddot.y = -gs.kpy * a_error.y - gs.kvy * j_error.y + kg_vehicleMass * targetSnap.y;
    target_force_ddot.z = -gs.kpz * a_error.z - gs.kvz * j_error.z + kg_vehicleMass * targetSnap.z;

    if (thrust_vector_limited) {
        target_force_ddot = (Vector3f){0, 0, 0};
    }

    VectorN<float, 9> b3cCollection;                                                // collection of three three-dimensional vectors b3c, b3c_dot, b3c_ddot
    b3cCollection = unit_vec(-target_force, -target_force_dot, -target_force_ddot); // unit_vec function is from geometric controller's git repo: https://github.com/fdcl-gwu/uav_geometric_control/blob/master/matlab/aux_functions/deriv_unit_vector.m

    Vector3f b3c;
    Vector3f b3c_dot;
    Vector3f b3c_ddot;

    b3c[0] = b3cCollection[0];
    b3c[1] = b3cCollection[1];
    b3c[2] = b3cCollection[2];

    b3c_dot[0] = b3cCollection[3];
    b3c_dot[1] = b3cCollection[4];
    b3c_dot[2] = b3cCollection[5];

    b3c_ddot[0] = b3cCollection[6];
    b3c_ddot[1] = b3cCollection[7];
    b3c_ddot[2] = b3cCollection[8];

    Vector3f A2 = -hatOperator(x_c_des) * b3c;
    Vector3f A2_dot = -hatOperator(x_c_des_dot) * b3c - hatOperator(x_c_des) * b3c_dot;
    Vector3f A2_ddot = -hatOperator(x_c_des_ddot) * b3c - hatOperator(x_c_des_dot) * b3c_dot * 2 - hatOperator(x_c_des) * b3c_ddot;

    VectorN<float, 9> b2cCollection;               // collection of three three-dimensional vectors b2c, b2c_dot, b2c_ddot
    b2cCollection = unit_vec(A2, A2_dot, A2_ddot); // unit_vec function is from geometric controller's git repo: https://github.com/fdcl-gwu/uav_geometric_control/blob/master/matlab/aux_functions/deriv_unit_vector.m

    Vector3f b2c;
    Vector3f b2c_dot;
    Vector3f b2c_ddot;

    b2c[0] = b2cCollection[0];
    b2c[1] = b2cCollection[1];
    b2c[2] = b2cCollection[2];

    b2c_dot[0] = b2cCollection[3];
    b2c_dot[1] = b2cCollection[4];
    b2c_dot[2] = b2cCollection[5];

    b2c_ddot[0] = b2cCollection[6];
    b2c_ddot[1] = b2cCollection[7];
    b2c_ddot[2] = b2cCollection[8];

    Vector3f b1c_dot = hatOperator(b2c_dot) * b3c + hatOperator(b2c) * b3c_dot;
    Vector3f b1c_ddot = hatOperator(b2c_ddot) * b3c + hatOperator(b2c_dot) * b3c_dot * 2 + hatOperator(b2c) * b3c_ddot;

    Matrix3f Rd_dot;  // derivative of Rdes
    Matrix3f Rd_ddot; // derivative of Rd_dot

    Rd_dot.a = b1c_dot;
    Rd_dot.b = b2c_dot;
    Rd_dot.c = b3c_dot;
    Rd_dot.transpose();

    Rd_ddot.a = b1c_ddot;
    Rd_ddot.b = b2c_ddot;
    Rd_ddot.c = b3c_ddot;
    Rd_ddot.transpose();

    Vector3f Omegad = veeOperator(Rdes.transposed() * Rd_dot);
    Vector3f Omegad_dot = veeOperator(Rdes.transposed() * Rd_ddot - hatOperator(Omegad) * hatOperator(Omegad));

    // eomega (angular velocity error)
    ew = Omega - R.transposed() * Rdes * Omegad;

    // Compute the moment
    M.x = -gs.krx * eR.x - gs.kox * ew.x;
    M.y = -gs.kry * eR.y - gs.koy * ew.y;
    M.z = -gs.krz * eR.z - gs.koz * ew.z;
    M = M - J * (hatOperator(Omega) * R.transposed() * Rdes * Omegad - R.transposed() * Rdes * Omegad_dot);
    Vector3f momentAdd = Omega % (J * Omega); // J is the inertia matrix
    M = M + momentAdd;

    VectorN<float, 4> thrustMomentCmd;
    thrustMomentCmd[0] = target_thrust;
    thrustMomentCmd[1] = M.x;
    thrustMomentCmd[2] = M.y;
    thrustMomentCmd[3] = M.z;

    // logging
    // log the desired rotation matrix and the actual rotation matrix
    AP::logger().Write("L1AF", "Rd11,Rd12,Rd13,Rd21,Rd22,Rd23,Rd31,Rd32,Rd33", "fffffffff",
                       Rdes.a.x,
                       Rdes.a.y,
                       Rdes.a.z,
                       Rdes.b.x,
                       Rdes.b.y,
                       Rdes.b.z,
                       Rdes.c.x,
                       Rdes.c.y,
                       Rdes.c.z);
    AP::logger().Write("L1AG", "R11,R12,R13,R21,R22,R23,R31,R32,R33", "fffffffff",
                       R.a.x,
                       R.a.y,
                       R.a.z,
                       R.b.x,
                       R.b.y,
                       R.b.z,
                       R.c.x,
                       R.c.y,
                       R.c.z);

    return thrustMomentCmd;
}

VectorN<float, 4> ModeAdaptive::L1AdaptiveAugmentation(VectorN<float, 4> thrustMomentCmd, bool suppress_yaw_control)
{
    // state predictor
    Vector3f v_hat;          // state predictor value of translational speed
    Vector3f omega_hat;      // state predictor value of rotational speed
    Vector3f e3 = {0, 0, 1}; // unit vector

    const float dt = 0.0025; // sampling time (update rate at 400 Hz)

    int8_t l1enable = g.l1enable;

    // load translational velocity
    Vector3f v_now;
    if (ahrs.have_inertial_nav())
    {
        if(ahrs.get_velocity_NED(v_now)){;} // state predictor value of translational speed
    }
    else
    {
        gcs().send_text(MAV_SEVERITY_CRITICAL, "inertial navigation is inactive in state predictor");
    }
    // coefficient for As
    float As_v = g.Asv;         //
    float As_omega = g.Asomega; //

    // load rotational velocity
    Vector3f omega_now = AP::ahrs().get_gyro();

    float massInverse = 1.0f / kg_vehicleMass;

    // compute prediction error (on previous step)
    Vector3f vpred_error_prev = v_hat_prev - v_prev;
    Vector3f omegapred_error_prev = omega_hat_prev - omega_prev;

    // NOTE: per the definition in vector3.h, vector * scalaer must have vector first then multiply the scalar later
    v_hat = v_hat_prev + (e3 * GRAVITY_MAGNITUDE - R_prev.colz() * (u_b_prev[0] + u_ad_prev[0] + sigma_m_hat_prev[0]) * massInverse + R_prev.colx() * sigma_um_hat_prev[0] * massInverse + R_prev.coly() * sigma_um_hat_prev[1] * massInverse + vpred_error_prev * As_v) * dt;

    // Jin is the inverse of inertia J, which has been done in mode.h

    // temp vector: thrustMomentCmd[1--3] + u_ad_prev[1--3] + sigma_m_hat_prev[1--3]
    Vector3f tempVec = {u_b_prev[1] + u_ad_prev[1] + sigma_m_hat_prev[1], u_b_prev[2] + u_ad_prev[2] + sigma_m_hat_prev[2], u_b_prev[3] + u_ad_prev[3] + sigma_m_hat_prev[3]};
    omega_hat = omega_hat_prev + (-Jinv * (omega_prev % (J * omega_prev)) + Jinv * tempVec + omegapred_error_prev * As_omega) * dt;

    // update the state prediction storage
    v_hat_prev = v_hat;
    omega_hat_prev = omega_hat;

    // compute prediction error (for this step)
    Vector3f vpred_error = v_hat - v_now;
    Vector3f omegapred_error = omega_hat - omega_now;

    // exponential coefficients for As
    float exp_As_v_dt = expf(As_v * dt);
    float exp_As_omega_dt = expf(As_omega * dt);

    // later part of uncertainty estimation (piecewise constant)
    Vector3f PhiInvmu_v = vpred_error / (exp_As_v_dt - 1) * As_v * exp_As_v_dt;
    Vector3f PhiInvmu_omega = omegapred_error / (exp_As_omega_dt - 1) * As_omega * exp_As_omega_dt;

    VectorN<float, 4> sigma_m_hat; // estimated matched uncertainty
    Vector3f sigma_m_hat_2to4;     // second to fourth element of the estimated matched uncertainty
    Vector2f sigma_um_hat;         // estimated unmatched uncertainty

    // use the rotation matrix in the current step
    Quaternion q;
    ahrs.get_quat_body_to_ned(q);
    Matrix3f R;
    q.rotation_matrix(R); // transforming the quaternion q to rotation matrix R

    sigma_m_hat[0] = R.colz() * PhiInvmu_v * kg_vehicleMass;
    sigma_m_hat_2to4 = -J * PhiInvmu_omega;
    sigma_m_hat[1] = sigma_m_hat_2to4[0];
    sigma_m_hat[2] = sigma_m_hat_2to4[1];
    sigma_m_hat[3] = sigma_m_hat_2to4[2];

    sigma_um_hat[0] = -R.colx() * PhiInvmu_v * kg_vehicleMass;
    sigma_um_hat[1] = -R.coly() * PhiInvmu_v * kg_vehicleMass;

    // store uncertainty estimations
    sigma_m_hat_prev = sigma_m_hat;
    sigma_um_hat_prev = sigma_um_hat;

    // compute lpf1 coefficients
    float lpf1_coefficientThrust1 = expf(-g.ctoffq1Thrust * 0.0025);
    float lpf1_coefficientThrust2 = 1.0 - lpf1_coefficientThrust1;

    float lpf1_coefficientMoment1 = expf(-g.ctoffq1Moment * 0.0025);
    float lpf1_coefficientMoment2 = 1.0 - lpf1_coefficientMoment1;

    // update the adaptive control
    VectorN<float, 4> u_ad_int;
    VectorN<float, 4> u_ad;

    // low-pass filter 1 (negation is added to u_ad_prev to filter the correct signal)
    u_ad_int[0] = lpf1_coefficientThrust1 * (lpf1_prev[0]) + lpf1_coefficientThrust2 * sigma_m_hat[0];
    u_ad_int[1] = lpf1_coefficientMoment1 * (lpf1_prev[1]) + lpf1_coefficientMoment2 * sigma_m_hat[1];
    u_ad_int[2] = lpf1_coefficientMoment1 * (lpf1_prev[2]) + lpf1_coefficientMoment2 * sigma_m_hat[2];
    u_ad_int[3] = lpf1_coefficientMoment1 * (lpf1_prev[3]) + lpf1_coefficientMoment2 * sigma_m_hat[3];

    lpf1_prev = u_ad_int; // store the current state

    float lpf2_coefficientMoment1 = expf(-g.ctoffq2Moment * 0.0025);
    float lpf2_coefficientMoment2 = 1.0 - lpf2_coefficientMoment1;

    // low-pass filter 2 (optional)
    u_ad[0] = u_ad_int[0]; // only one filter on the thrust channel
    u_ad[1] = lpf2_coefficientMoment1 * lpf2_prev[1] + lpf2_coefficientMoment2 * u_ad_int[1];
    u_ad[2] = lpf2_coefficientMoment1 * lpf2_prev[2] + lpf2_coefficientMoment2 * u_ad_int[2];
    u_ad[3] = lpf2_coefficientMoment1 * lpf2_prev[3] + lpf2_coefficientMoment2 * u_ad_int[3];

    lpf2_prev = u_ad; // store the current state
    // negate
    u_ad = -u_ad;

    // In degraded yaw-free mode we still estimate the yaw disturbance for
    // diagnostics/prediction, but intentionally do not command adaptive yaw
    // torque.
    if (suppress_yaw_control) {
        u_ad[3] = 0.0f;
    }

    AP::logger().Write("L1AD", "v1,v2,v3,v1hat,v2hat,v3hat,o1,o2,o3,o1hat,o2hat,o3hat", "ffffffffffff",
                       (double)v_now.x,
                       (double)v_now.y,
                       (double)v_now.z,
                       (double)v_hat_prev.x,
                       (double)v_hat_prev.y,
                       (double)v_hat_prev.z,
                       (double)omega_now.x,
                       (double)omega_now.y,
                       (double)omega_now.z,
                       (double)omega_hat_prev.x,
                       (double)omega_hat_prev.y,
                       (double)omega_hat_prev.z);

    AP::logger().Write("L1AE", "sm1,sm2,sm3,sm4,sum1,sum2,uad1,uad2,uad3,uad4,l1enable", "ffffffffffb",
                       (double)sigma_m_hat_prev[0],
                       (double)sigma_m_hat_prev[1],
                       (double)sigma_m_hat_prev[2],
                       (double)sigma_m_hat_prev[3],
                       (double)sigma_um_hat_prev[0],
                       (double)sigma_um_hat_prev[1],
                       (double)u_ad[0],
                       (double)u_ad[1],
                       (double)u_ad[2],
                       (double)u_ad[3],
                       l1enable);
    // store the values for next iteration
    u_ad_prev = u_ad * l1enable;

    v_prev = v_now;
    omega_prev = omega_now;
    R_prev = R;
    u_b_prev = thrustMomentCmd;

    return u_ad_prev; // return the updated l1 control
}

VectorN<float, 9> ModeAdaptive::unit_vec(Vector3f q, Vector3f q_dot, Vector3f q_ddot)
{
    // This function comes from Appendix F in https://arxiv.org/pdf/1003.2005v3.pdf
    VectorN<float, 9> uCollection; // for storage of the output
    float nq = q.length();
    Vector3f u = q / nq;
    Vector3f u_dot = q_dot / nq - q * (q * q_dot) / powF(nq, 3);
    Vector3f u_ddot = q_ddot / nq - q_dot / powF(nq, 3) * 2 * (q * q_dot) - q / powF(nq, 3) * (q_dot * q_dot + q * q_ddot) + q * 3 / powF(nq, 5) * powF(q * q_dot, 2);

    uCollection[0] = u[0];
    uCollection[1] = u[1];
    uCollection[2] = u[2];

    uCollection[3] = u_dot[0];
    uCollection[4] = u_dot[1];
    uCollection[5] = u_dot[2];

    uCollection[6] = u_ddot[0];
    uCollection[7] = u_ddot[1];
    uCollection[8] = u_ddot[2];

    return uCollection;
}

Matrix3f ModeAdaptive::hatOperator(Vector3f input)
{
    // hatOperator: convert R^3 to so(3)
    Matrix3f output;
    output = output * 0; // initialize by zero
    // const T ax, const T ay, const T az,
    // const T bx, const T by, const T bz,
    // const T cx, const T cy, const T cz
    output.a.x = 0;
    output.a.y = -input.z;
    output.a.z = input.y;
    output.b.x = input.z;
    output.b.y = 0;
    output.b.z = -input.x;
    output.c.x = -input.y;
    output.c.y = input.x;
    output.c.z = 0;

    return output;
}

Vector3f ModeAdaptive::veeOperator(Matrix3f input)
{
    // veeOperator: convert so(3) to R^3
    Vector3f output;
    // const T ax, const T ay, const T az,
    // const T bx, const T by, const T bz,
    // const T cx, const T cy, const T cz
    output.x = input.c.y;
    output.y = input.a.z;
    output.z = input.b.x;

    return output;
}

VectorN<float, 4> ModeAdaptive::motorMixingYawFree(VectorN<float, 4> thrustMomentCmd)
{
    // Reduced-attitude allocator: satisfy total thrust, roll moment and pitch
    // moment without constraining reaction-torque/yaw moment.
#if (!REAL_OR_SITL)
    const float L = 0.25f;
    const float D = 0.25f;
    const float a_F = 0.0014597f;
    const float b_F = 0.043693f;
#elif (REAL_OR_SITL)
    const float L = 0.28f;
    const float D = 0.28f;
#endif

    const float F = thrustMomentCmd[0];
    const float Mx = thrustMomentCmd[1];
    const float My = thrustMomentCmd[2];

    // Minimum-norm solution of:
    // F  = f1 + f2 + f3 + f4
    // Mx = L/2 * (-f1 + f2 + f3 - f4)
    // My = D/2 * ( f1 - f2 + f3 - f4)
    // No Mz equation is imposed.
    VectorN<float, 4> motorThrust;
    motorThrust[0] = F * 0.25f - Mx / (2.0f * L) + My / (2.0f * D);
    motorThrust[1] = F * 0.25f + Mx / (2.0f * L) - My / (2.0f * D);
    motorThrust[2] = F * 0.25f + Mx / (2.0f * L) + My / (2.0f * D);
    motorThrust[3] = F * 0.25f - Mx / (2.0f * L) - My / (2.0f * D);

    VectorN<float, 4> w;
    for (uint8_t i = 0; i < 4; i++) {
        const float fi = MAX(0.0f, motorThrust[i]);
#if (!REAL_OR_SITL)
        const float disc = b_F * b_F + 4.0f * a_F * fi;
        w[i] = (-b_F + sqrtF(MAX(0.0f, disc))) / (2.0f * a_F);
#elif (REAL_OR_SITL)
        w[i] = softdrone_w_from_thrust(fi);
#endif
    }

    return w;
}

VectorN<float, 4> ModeAdaptive::motorMixingYawFreeEffectivenessAware(
    VectorN<float, 4> thrustMomentCmd,
    uint8_t degraded_motor_id,
    float estimated_loss_pct)
{
    // Reduced-attitude allocation with one estimated motor effectiveness.
    //
    // The commanded per-motor thrust vector f_cmd is solved from
    //     [F Mx My]^T = B * diag(eta) * f_cmd
    // where eta=1 for healthy motors and eta=(1-loss) for the isolated motor.
    // We use the minimum-norm pseudoinverse A^T(AA^T)^-1 with
    // A = B*diag(eta).  This preserves the remaining authority of a partially
    // degraded motor instead of forcing it to zero.  At 100% estimated loss,
    // eta becomes zero and the same formulation naturally reduces to a
    // three-motor yaw-free allocator.
#if (!REAL_OR_SITL)
    const float L = 0.25f;
    const float D = 0.25f;
    const float a_F = 0.0014597f;
    const float b_F = 0.043693f;
#elif (REAL_OR_SITL)
    const float L = 0.28f;
    const float D = 0.28f;
#endif

    if (degraded_motor_id < 1 || degraded_motor_id > 4 ||
        !isfinite(estimated_loss_pct)) {
        return motorMixingYawFree(thrustMomentCmd);
    }

    float eta[4] = {1.0f, 1.0f, 1.0f, 1.0f};
    eta[degraded_motor_id - 1U] =
        1.0f -
        constrain_float(estimated_loss_pct, 0.0f, 100.0f) * 0.01f;

    const float roll_coeff[4] = {
        -0.5f * L, +0.5f * L, +0.5f * L, -0.5f * L
    };
    const float pitch_coeff[4] = {
        +0.5f * D, -0.5f * D, +0.5f * D, -0.5f * D
    };

    // Build G = A*A^T, a symmetric 3x3 matrix.
    float g00 = 0.0f;
    float g01 = 0.0f;
    float g02 = 0.0f;
    float g11 = 0.0f;
    float g12 = 0.0f;
    float g22 = 0.0f;

    for (uint8_t i = 0; i < 4; i++) {
        const float a0 = eta[i];
        const float a1 = eta[i] * roll_coeff[i];
        const float a2 = eta[i] * pitch_coeff[i];

        g00 += a0 * a0;
        g01 += a0 * a1;
        g02 += a0 * a2;
        g11 += a1 * a1;
        g12 += a1 * a2;
        g22 += a2 * a2;
    }

    const float det =
        g00 * (g11 * g22 - g12 * g12) -
        g01 * (g01 * g22 - g12 * g02) +
        g02 * (g01 * g12 - g11 * g02);

    if (!isfinite(det) || fabsf(det) < 1.0e-9f) {
        return motorMixingYawFree(thrustMomentCmd);
    }

    const float inv00 = (g11 * g22 - g12 * g12) / det;
    const float inv01 = (g02 * g12 - g01 * g22) / det;
    const float inv02 = (g01 * g12 - g02 * g11) / det;
    const float inv11 = (g00 * g22 - g02 * g02) / det;
    const float inv12 = (g01 * g02 - g00 * g12) / det;
    const float inv22 = (g00 * g11 - g01 * g01) / det;

    const float F = thrustMomentCmd[0];
    const float Mx = thrustMomentCmd[1];
    const float My = thrustMomentCmd[2];

    // y = (A*A^T)^-1 * desired_wrench
    const float y0 = inv00 * F + inv01 * Mx + inv02 * My;
    const float y1 = inv01 * F + inv11 * Mx + inv12 * My;
    const float y2 = inv02 * F + inv12 * Mx + inv22 * My;

    VectorN<float, 4> w;
    for (uint8_t i = 0; i < 4; i++) {
        // f_cmd = A^T*y
        const float fi_cmd = MAX(
            0.0f,
            eta[i] *
                (y0 + roll_coeff[i] * y1 + pitch_coeff[i] * y2)
        );

#if (!REAL_OR_SITL)
        const float disc = b_F * b_F + 4.0f * a_F * fi_cmd;
        w[i] =
            (-b_F + sqrtF(MAX(0.0f, disc))) / (2.0f * a_F);
#elif (REAL_OR_SITL)
        w[i] = softdrone_w_from_thrust(fi_cmd);
#endif
        w[i] = constrain_float(w[i], 0.0f, 100.0f);
    }

    return w;
}

VectorN<float, 4> ModeAdaptive::motorMixing(VectorN<float, 4> thrustMomentCmd)
{
    VectorN<float, 4> w;
#if (!REAL_OR_SITL)       // SITL
    const float L = 0.25; // for x layout
    const float D = 0.25;
    const float a_F = 0.0014597;
    const float b_F = 0.043693;
    const float a_M = 0.000011667;
    const float b_M = 0.0059137;
#elif (REAL_OR_SITL) // softdrone real-airframe parameters
    // Body frame: +X forward, +Y right, +Z down (FRD).
    // Square X layout, measured motor-center spans:
    //   L = left-right motor-row spacing (roll lever-arm span)
    //   D = front-rear motor-row spacing (pitch lever-arm span)
    const float L = 0.28f;
    const float D = 0.28f;

    // Static motor/propeller model is the dead-zone cubic defined at the top
    // of this file.  It is shared by normal allocation, yaw-free allocation,
    // and motor-degradation effectiveness injection so all three paths use
    // one internally consistent physical model.

    // Softdrone motor/output order expected by the mixer:
    //   w[0] / output 1: front-right, CCW
    //   w[1] / output 2: rear-left,   CCW
    //   w[2] / output 3: front-left,  CW
    //   w[3] / output 4: rear-right,  CW
#endif

    // Solve for a common-motor linearizing point at one quarter of the total
    // collective thrust, then take two Newton-style allocator refinements.
#if (!REAL_OR_SITL)
    float w0 = (-b_F + sqrtF(b_F * b_F + a_F * thrustMomentCmd[0])) / 2 / a_F;
    float d_F = 2 * a_F * w0 + b_F;
    float d_M = 2 * a_M * w0 + b_M;
    const float offset_F = -a_F * w0 * w0;
#elif (REAL_OR_SITL)
    float w0 = softdrone_w_from_thrust(MAX(0.0f, 0.25f * thrustMomentCmd[0]));
    float d_F = softdrone_thrust_slope_from_w(w0);
    float d_M = softdrone_moment_slope_from_w(w0);
    const float offset_F = softdrone_thrust_from_w(w0) - d_F * w0;
#endif

    const float thrust_biased = thrustMomentCmd[0] - 4.0f * offset_F;
    const float M1 = thrustMomentCmd[1];
    const float M2 = thrustMomentCmd[2];
    const float M3 = thrustMomentCmd[3];

    // Motor mixing for x layout using the local slopes of F(w) and M(w).
    const float d_F4_inv = 1.0f / (4.0f * d_F);
    const float d_FL_inv = 1.0f / (2.0f * L * d_F);
    const float d_FD_inv = 1.0f / (2.0f * D * d_F);
    const float d_M4_inv = 1.0f / (4.0f * d_M);

    w[0] = d_F4_inv * thrust_biased - d_FL_inv * M1 + d_FD_inv * M2 + d_M4_inv * M3;
    w[1] = d_F4_inv * thrust_biased + d_FL_inv * M1 - d_FD_inv * M2 + d_M4_inv * M3;
    w[2] = d_F4_inv * thrust_biased + d_FL_inv * M1 + d_FD_inv * M2 - d_M4_inv * M3;
    w[3] = d_F4_inv * thrust_biased - d_FL_inv * M1 - d_FD_inv * M2 - d_M4_inv * M3;

    VectorN<float, 4> w2;
    w2 = iterativeMotorMixing(w, thrustMomentCmd, L, D);

    VectorN<float, 4> w3;
    w3 = iterativeMotorMixing(w2, thrustMomentCmd, L, D);

    // logging
    // AP::logger().Write("L1A1", "m1,m2,m3,m4", "ffff",
    //                      (double)w[0],
    //                      (double)w[1],
    //                      (double)w[2],
    //                      (double)w[3]);
    // AP::logger().Write("L1A2", "m1,m2,m3,m4", "ffff",
    //                      (double)w2[0],
    //                      (double)w2[1],
    //                      (double)w2[2],
    //                      (double)w2[3]);
    // AP::logger().Write("L1A3", "m1,m2,m3,m4", "ffff",
    //                      (double)w3[0],
    //                      (double)w3[1],
    //                      (double)w3[2],
    //                      (double)w3[3]);
    return w3;
}

VectorN<float, 4> ModeAdaptive::iterativeMotorMixing(VectorN<float, 4> w_input,
                                                      VectorN<float, 4> thrustMomentCmd,
                                                      float L,
                                                      float D)
{
    // One local Newton-style allocation step.  Each motor's nonlinear thrust
    // and reaction-torque curve is linearized independently about w_input.
    VectorN<float, 4> w_new;

    float c_F1, c_F2, c_F3, c_F4;
    float c_M1, c_M2, c_M3, c_M4;
    float d_F1, d_F2, d_F3, d_F4;
    float d_M1, d_M2, d_M3, d_M4;

#if (!REAL_OR_SITL)
    const float a_F = 0.0014597f;
    const float b_F = 0.043693f;
    const float a_M = 0.000011667f;
    const float b_M = 0.0059137f;

    const float w1_square = w_input[0] * w_input[0];
    const float w2_square = w_input[1] * w_input[1];
    const float w3_square = w_input[2] * w_input[2];
    const float w4_square = w_input[3] * w_input[3];

    c_F1 = -a_F * w1_square;
    c_F2 = -a_F * w2_square;
    c_F3 = -a_F * w3_square;
    c_F4 = -a_F * w4_square;

    c_M1 = -a_M * w1_square;
    c_M2 = -a_M * w2_square;
    c_M3 = -a_M * w3_square;
    c_M4 = -a_M * w4_square;

    d_F1 = 2.0f * a_F * w_input[0] + b_F;
    d_F2 = 2.0f * a_F * w_input[1] + b_F;
    d_F3 = 2.0f * a_F * w_input[2] + b_F;
    d_F4 = 2.0f * a_F * w_input[3] + b_F;

    d_M1 = 2.0f * a_M * w_input[0] + b_M;
    d_M2 = 2.0f * a_M * w_input[1] + b_M;
    d_M3 = 2.0f * a_M * w_input[2] + b_M;
    d_M4 = 2.0f * a_M * w_input[3] + b_M;
#elif (REAL_OR_SITL)
    const float F1 = softdrone_thrust_from_w(w_input[0]);
    const float F2 = softdrone_thrust_from_w(w_input[1]);
    const float F3 = softdrone_thrust_from_w(w_input[2]);
    const float F4 = softdrone_thrust_from_w(w_input[3]);

    d_F1 = softdrone_thrust_slope_from_w(w_input[0]);
    d_F2 = softdrone_thrust_slope_from_w(w_input[1]);
    d_F3 = softdrone_thrust_slope_from_w(w_input[2]);
    d_F4 = softdrone_thrust_slope_from_w(w_input[3]);

    c_F1 = F1 - d_F1 * w_input[0];
    c_F2 = F2 - d_F2 * w_input[1];
    c_F3 = F3 - d_F3 * w_input[2];
    c_F4 = F4 - d_F4 * w_input[3];

    const float M1 = softdrone_moment_from_w(w_input[0]);
    const float M2 = softdrone_moment_from_w(w_input[1]);
    const float M3 = softdrone_moment_from_w(w_input[2]);
    const float M4 = softdrone_moment_from_w(w_input[3]);

    d_M1 = softdrone_moment_slope_from_w(w_input[0]);
    d_M2 = softdrone_moment_slope_from_w(w_input[1]);
    d_M3 = softdrone_moment_slope_from_w(w_input[2]);
    d_M4 = softdrone_moment_slope_from_w(w_input[3]);

    c_M1 = M1 - d_M1 * w_input[0];
    c_M2 = M2 - d_M2 * w_input[1];
    c_M3 = M3 - d_M3 * w_input[2];
    c_M4 = M4 - d_M4 * w_input[3];
#endif

    VectorN<float, 4> coefficientRow1;
    VectorN<float, 4> coefficientRow2;
    VectorN<float, 4> coefficientRow3;
    VectorN<float, 4> coefficientRow4;

    coefficientRow1[0] = d_F1;
    coefficientRow1[1] = d_F2;
    coefficientRow1[2] = d_F3;
    coefficientRow1[3] = d_F4;

    coefficientRow2[0] = -d_F1;
    coefficientRow2[1] = d_F2;
    coefficientRow2[2] = d_F3;
    coefficientRow2[3] = -d_F4;

    coefficientRow3[0] = d_F1;
    coefficientRow3[1] = -d_F2;
    coefficientRow3[2] = d_F3;
    coefficientRow3[3] = -d_F4;

    coefficientRow4[0] = d_M1;
    coefficientRow4[1] = d_M2;
    coefficientRow4[2] = -d_M3;
    coefficientRow4[3] = -d_M4;

    VectorN<float, 16> coefficientMatrixInv = mat4Inv(coefficientRow1, coefficientRow2, coefficientRow3, coefficientRow4);

    VectorN<float, 4> coefficientInvRow1;
    VectorN<float, 4> coefficientInvRow2;
    VectorN<float, 4> coefficientInvRow3;
    VectorN<float, 4> coefficientInvRow4;

    coefficientInvRow1[0] = coefficientMatrixInv[0];
    coefficientInvRow1[1] = coefficientMatrixInv[1];
    coefficientInvRow1[2] = coefficientMatrixInv[2];
    coefficientInvRow1[3] = coefficientMatrixInv[3];

    coefficientInvRow2[0] = coefficientMatrixInv[4];
    coefficientInvRow2[1] = coefficientMatrixInv[5];
    coefficientInvRow2[2] = coefficientMatrixInv[6];
    coefficientInvRow2[3] = coefficientMatrixInv[7];

    coefficientInvRow3[0] = coefficientMatrixInv[8];
    coefficientInvRow3[1] = coefficientMatrixInv[9];
    coefficientInvRow3[2] = coefficientMatrixInv[10];
    coefficientInvRow3[3] = coefficientMatrixInv[11];

    coefficientInvRow4[0] = coefficientMatrixInv[12];
    coefficientInvRow4[1] = coefficientMatrixInv[13];
    coefficientInvRow4[2] = coefficientMatrixInv[14];
    coefficientInvRow4[3] = coefficientMatrixInv[15];

    VectorN<float, 4> shiftedCmd;
    shiftedCmd[0] = thrustMomentCmd[0] - c_F1 - c_F2 - c_F3 - c_F4;
    shiftedCmd[1] = 2 * thrustMomentCmd[1] / L + c_F1 - c_F2 - c_F3 + c_F4;
    shiftedCmd[2] = 2 * thrustMomentCmd[2] / D - c_F1 + c_F2 - c_F3 + c_F4;
    shiftedCmd[3] = thrustMomentCmd[3] - c_M1 - c_M2 + c_M3 + c_M4;

    w_new[0] = coefficientInvRow1 * shiftedCmd;
    w_new[1] = coefficientInvRow2 * shiftedCmd;
    w_new[2] = coefficientInvRow3 * shiftedCmd;
    w_new[3] = coefficientInvRow4 * shiftedCmd;

    return w_new;
}

VectorN<float, 16> ModeAdaptive::mat4Inv(VectorN<float, 4> coefficientRow1, VectorN<float, 4> coefficientRow2, VectorN<float, 4> coefficientRow3, VectorN<float, 4> coefficientRow4)
{
    // inverse of a 4x4 matrix
    // modified from https://stackoverflow.com/a/44446912
    float A2323 = coefficientRow3[2] * coefficientRow4[3] - coefficientRow3[3] * coefficientRow4[2];
    float A1323 = coefficientRow3[1] * coefficientRow4[3] - coefficientRow3[3] * coefficientRow4[1];
    float A1223 = coefficientRow3[1] * coefficientRow4[2] - coefficientRow3[2] * coefficientRow4[1];
    float A0323 = coefficientRow3[0] * coefficientRow4[3] - coefficientRow3[3] * coefficientRow4[0];
    float A0223 = coefficientRow3[0] * coefficientRow4[2] - coefficientRow3[2] * coefficientRow4[0];
    float A0123 = coefficientRow3[0] * coefficientRow4[1] - coefficientRow3[1] * coefficientRow4[0];
    float A2313 = coefficientRow2[2] * coefficientRow4[3] - coefficientRow2[3] * coefficientRow4[2];
    float A1313 = coefficientRow2[1] * coefficientRow4[3] - coefficientRow2[3] * coefficientRow4[1];
    float A1213 = coefficientRow2[1] * coefficientRow4[2] - coefficientRow2[2] * coefficientRow4[1];
    float A2312 = coefficientRow2[2] * coefficientRow3[3] - coefficientRow2[3] * coefficientRow3[2];
    float A1312 = coefficientRow2[1] * coefficientRow3[3] - coefficientRow2[3] * coefficientRow3[1];
    float A1212 = coefficientRow2[1] * coefficientRow3[2] - coefficientRow2[2] * coefficientRow3[1];
    float A0313 = coefficientRow2[0] * coefficientRow4[3] - coefficientRow2[3] * coefficientRow4[0];
    float A0213 = coefficientRow2[0] * coefficientRow4[2] - coefficientRow2[2] * coefficientRow4[0];
    float A0312 = coefficientRow2[0] * coefficientRow3[3] - coefficientRow2[3] * coefficientRow3[0];
    float A0212 = coefficientRow2[0] * coefficientRow3[2] - coefficientRow2[2] * coefficientRow3[0];
    float A0113 = coefficientRow2[0] * coefficientRow4[1] - coefficientRow2[1] * coefficientRow4[0];
    float A0112 = coefficientRow2[0] * coefficientRow3[1] - coefficientRow2[1] * coefficientRow3[0];

    float det = coefficientRow1[0] * (coefficientRow2[1] * A2323 - coefficientRow2[2] * A1323 + coefficientRow2[3] * A1223) - coefficientRow1[1] * (coefficientRow2[0] * A2323 - coefficientRow2[2] * A0323 + coefficientRow2[3] * A0223) + coefficientRow1[2] * (coefficientRow2[0] * A1323 - coefficientRow2[1] * A0323 + coefficientRow2[3] * A0123) - coefficientRow1[3] * (coefficientRow2[0] * A1223 - coefficientRow2[1] * A0223 + coefficientRow2[2] * A0123);
    det = 1 / det;

    VectorN<float, 16> inv;
    inv[0] = det * (coefficientRow2[1] * A2323 - coefficientRow2[2] * A1323 + coefficientRow2[3] * A1223);
    inv[1] = det * -(coefficientRow1[1] * A2323 - coefficientRow1[2] * A1323 + coefficientRow1[3] * A1223);
    inv[2] = det * (coefficientRow1[1] * A2313 - coefficientRow1[2] * A1313 + coefficientRow1[3] * A1213);
    inv[3] = det * -(coefficientRow1[1] * A2312 - coefficientRow1[2] * A1312 + coefficientRow1[3] * A1212);
    inv[4] = det * -(coefficientRow2[0] * A2323 - coefficientRow2[2] * A0323 + coefficientRow2[3] * A0223);
    inv[5] = det * (coefficientRow1[0] * A2323 - coefficientRow1[2] * A0323 + coefficientRow1[3] * A0223);
    inv[6] = det * -(coefficientRow1[0] * A2313 - coefficientRow1[2] * A0313 + coefficientRow1[3] * A0213);
    inv[7] = det * (coefficientRow1[0] * A2312 - coefficientRow1[2] * A0312 + coefficientRow1[3] * A0212);
    inv[8] = det * (coefficientRow2[0] * A1323 - coefficientRow2[1] * A0323 + coefficientRow2[3] * A0123);
    inv[9] = det * -(coefficientRow1[0] * A1323 - coefficientRow1[1] * A0323 + coefficientRow1[3] * A0123);
    inv[10] = det * (coefficientRow1[0] * A1313 - coefficientRow1[1] * A0313 + coefficientRow1[3] * A0113);
    inv[11] = det * -(coefficientRow1[0] * A1312 - coefficientRow1[1] * A0312 + coefficientRow1[3] * A0112);
    inv[12] = det * -(coefficientRow2[0] * A1223 - coefficientRow2[1] * A0223 + coefficientRow2[2] * A0123);
    inv[13] = det * (coefficientRow1[0] * A1223 - coefficientRow1[1] * A0223 + coefficientRow1[2] * A0123);
    inv[14] = det * -(coefficientRow1[0] * A1213 - coefficientRow1[1] * A0213 + coefficientRow1[2] * A0113);
    inv[15] = det * (coefficientRow1[0] * A1212 - coefficientRow1[1] * A0212 + coefficientRow1[2] * A0112);

    return inv;
}

#endif
