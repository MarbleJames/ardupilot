#pragma once

#include <AP_Math/AP_Math.h>

// Time-to-capture for the final precision-landing descent.
//
// The horizontal loop brakes with the position P gain, passed through
// sqrt_controller and slewed at the position controller acceleration limit.
// Integrating that response gives the time to enter the capture radius. A
// constant-velocity coast is not used: an overspeed approach that will
// shoot past the pin produces a long capture time (or none within the limit).
namespace AC_PrecLand_Capture {

static constexpr float CAPTURE_RADIUS_M = 0.03f;
static constexpr float MIN_DESCENT_MS = 0.10f;
static constexpr float MAX_CAPTURE_TIME_S = 5.0f;
static constexpr float MOVING_AWAY_MS = 0.05f;
static constexpr float FREEZE_HEIGHT_M = 0.12f;
static constexpr float FINAL_HEIGHT_M = 0.35f;
static constexpr float INTEGRATE_DT_S = 0.05f;
// A step that merely crosses the pin at high speed is not a capture.
// The vehicle has to be inside the radius and slow enough to stay there.
static constexpr float CAPTURE_SPEED_MS = 0.5f;

// Advance vehicle-to-target error by one controller step.
// err_dot is the derivative of err (target velocity relative to the vehicle).
inline void step_horizontal(Vector2f &err, Vector2f &err_dot, float dt, float pos_p, float accel_mss, float vel_max_ms)
{
    const float dist = err.length();
    Vector2f vel_dem;
    if (dist > 1.0e-6f) {
        // dt of 0 disables the sqrt_controller end-step clamp. The slew below is the accel limit.
        float speed = sqrt_controller(dist, pos_p, accel_mss, 0.0f);
        if (is_positive(vel_max_ms)) {
            speed = MIN(speed, vel_max_ms);
        }
        // Demand shrinks the vehicle-to-target error.
        vel_dem = err * (-speed / dist);
    }

    Vector2f delta_vel = vel_dem - err_dot;
    const float max_step = accel_mss * dt;
    const float delta_mag = delta_vel.length();
    if (delta_mag > max_step && is_positive(delta_mag)) {
        delta_vel *= max_step / delta_mag;
    }
    err_dot += delta_vel;
    err += err_dot * dt;
}

// Seconds until the braking model is inside CAPTURE_RADIUS_M and slower than CAPTURE_SPEED_MS.
// A constant-velocity coast through the pin does not count. Returns false if that
// does not happen within MAX_CAPTURE_TIME_S.
inline bool time_to_capture_s(const Vector2f &pos_to_target_m, const Vector2f &vel_target_rel_ms,
                              float pos_p, float accel_mss, float vel_max_ms, float &capture_s)
{
    capture_s = 0.0f;
    if (pos_to_target_m.length() <= CAPTURE_RADIUS_M) {
        return true;
    }
    if (!is_positive(pos_p) || !is_positive(accel_mss)) {
        return false;
    }

    Vector2f err = pos_to_target_m;
    Vector2f err_dot = vel_target_rel_ms;
    float t_s = 0.0f;
    while (t_s < MAX_CAPTURE_TIME_S) {
        step_horizontal(err, err_dot, INTEGRATE_DT_S, pos_p, accel_mss, vel_max_ms);
        t_s += INTEGRATE_DT_S;
        if (err.length() <= CAPTURE_RADIUS_M && err_dot.length() <= CAPTURE_SPEED_MS) {
            capture_s = t_s;
            return true;
        }
    }
    return false;
}

// Descent rate (m/s, positive down) for the final stage.
// Returns false, with rate 0, when the vehicle is moving away, the gains are
// unusable, or capture would take longer than MAX_CAPTURE_TIME_S.
// Inside the capture radius the rate is land_spd_ms.
inline bool descent_rate_down_ms(float height_m, const Vector2f &pos_to_target_m, const Vector2f &vel_target_rel_ms,
                                 float land_spd_ms, float pos_p, float accel_mss, float vel_max_ms, float &rate_down_ms)
{
    rate_down_ms = 0.0f;
    if (!is_positive(land_spd_ms)) {
        return false;
    }
    const float dist = pos_to_target_m.length();
    if (dist <= CAPTURE_RADIUS_M) {
        rate_down_ms = land_spd_ms;
        return true;
    }
    if (!is_positive(pos_p) || !is_positive(accel_mss)) {
        return false;
    }
    // Positive closing speed means the vehicle-to-target error is shrinking.
    const float closing_ms = -(pos_to_target_m * vel_target_rel_ms) / dist;
    if (closing_ms < -MOVING_AWAY_MS) {
        return false;
    }

    float capture_s = 0.0f;
    if (!time_to_capture_s(pos_to_target_m, vel_target_rel_ms, pos_p, accel_mss, vel_max_ms, capture_s)) {
        return false;
    }
    if (capture_s < 1.0e-3f) {
        rate_down_ms = land_spd_ms;
        return true;
    }
    rate_down_ms = constrain_float(height_m / capture_s, MIN_DESCENT_MS, land_spd_ms);
    return true;
}

} // namespace AC_PrecLand_Capture
