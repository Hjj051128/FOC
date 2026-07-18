#include "predictive_vision_tracker.h"

#include <math.h>

namespace
{
constexpr float TWO_PI_F = 6.28318530718f;

float clampFloat(float value, float minimum, float maximum)
{
    if (value < minimum) {
        return minimum;
    }
    if (value > maximum) {
        return maximum;
    }
    return value;
}
}

PredictiveVisionTracker::PredictiveVisionTracker(
    float observer_bandwidth_hz,
    float lead_time_s,
    float max_prediction_time_s,
    float max_pixel_speed,
    uint32_t command_hold_ms,
    uint32_t stale_timeout_ms
)
    : state_x{}
    , state_y{}
    , observer_bandwidth_hz(observer_bandwidth_hz)
    , lead_time_s(lead_time_s)
    , max_prediction_time_s(max_prediction_time_s)
    , max_pixel_speed(max_pixel_speed)
    , command_hold_us(command_hold_ms * 1000U)
    , stale_timeout_us(stale_timeout_ms * 1000U)
    , last_measurement_us(0)
    , last_output_us(0)
    , initialized(false)
{
}

void PredictiveVisionTracker::reset()
{
    state_x = {};
    state_y = {};
    last_measurement_us = 0;
    last_output_us = 0;
    initialized = false;
}

void PredictiveVisionTracker::update(
    float error_x,
    float error_y,
    uint32_t timestamp_us
)
{
    if (!initialized) {
        state_x.error = error_x;
        state_y.error = error_y;
        state_x.velocity = 0.0f;
        state_y.velocity = 0.0f;
        last_measurement_us = timestamp_us;
        initialized = true;
        return;
    }

    uint32_t elapsed_us = timestamp_us - last_measurement_us;
    float dt = elapsed_us * 1e-6f;

    if (dt <= 0.0f || dt > 0.25f) {
        state_x.error = error_x;
        state_y.error = error_y;
        state_x.velocity = 0.0f;
        state_y.velocity = 0.0f;
        last_measurement_us = timestamp_us;
        return;
    }

    dt = clampFloat(dt, 0.005f, 0.1f);
    updateAxis(state_x, error_x, dt);
    updateAxis(state_y, error_y, dt);
    last_measurement_us = timestamp_us;
}

void PredictiveVisionTracker::updateAxis(
    AxisState &state,
    float measurement,
    float dt
) const
{
    float prediction = state.error + state.velocity * dt;
    float innovation = measurement - prediction;

    float pole = expf(-TWO_PI_F * observer_bandwidth_hz * dt);
    float alpha = 1.0f - pole * pole;
    float beta = (1.0f - pole) * (1.0f - pole);

    state.error = prediction + alpha * innovation;
    state.velocity += (beta / dt) * innovation;
    state.velocity = clampFloat(
        state.velocity,
        -max_pixel_speed,
        max_pixel_speed
    );
}

PredictiveVisionOutput PredictiveVisionTracker::output(
    uint32_t now_us,
    bool active,
    const VisionPidGains &gains_x,
    const VisionPidGains &gains_y,
    float direction_x,
    float direction_y,
    float dead_zone,
    float max_speed_x,
    float max_speed_y,
    float integral_zone,
    float integral_speed_limit
)
{
    PredictiveVisionOutput result = {};
    if (!active || !initialized) {
        state_x.integral = 0.0f;
        state_y.integral = 0.0f;
        last_output_us = now_us;
        return result;
    }

    uint32_t age_us = now_us - last_measurement_us;
    if (age_us >= stale_timeout_us) {
        state_x.integral = 0.0f;
        state_y.integral = 0.0f;
        last_output_us = now_us;
        return result;
    }

    float dt = 0.0f;
    if (last_output_us != 0) {
        dt = (now_us - last_output_us) * 1e-6f;
        if (dt <= 0.0f || dt > 0.02f) {
            dt = 0.0f;
        }
    }
    last_output_us = now_us;

    float age_s = age_us * 1e-6f;
    result.predicted_error_x = predictedError(state_x, age_s);
    result.predicted_error_y = predictedError(state_y, age_s);

    float freshness = 1.0f;
    if (age_us > command_hold_us && stale_timeout_us > command_hold_us) {
        freshness =
            static_cast<float>(stale_timeout_us - age_us) /
            static_cast<float>(stale_timeout_us - command_hold_us);
    }

    result.speed_x = pidSpeed(
        state_x,
        result.predicted_error_x,
        freshness,
        dt,
        gains_x,
        direction_x,
        dead_zone,
        max_speed_x,
        integral_zone,
        integral_speed_limit
    );
    result.speed_y = pidSpeed(
        state_y,
        result.predicted_error_y,
        freshness,
        dt,
        gains_y,
        direction_y,
        dead_zone,
        max_speed_y,
        integral_zone,
        integral_speed_limit
    );
    return result;
}

float PredictiveVisionTracker::predictedError(
    const AxisState &state,
    float measurement_age_s
) const
{
    float horizon = clampFloat(
        measurement_age_s + lead_time_s,
        0.0f,
        max_prediction_time_s
    );
    return state.error + state.velocity * horizon;
}

float PredictiveVisionTracker::pidSpeed(
    AxisState &state,
    float predicted_error,
    float freshness,
    float dt,
    const VisionPidGains &gains,
    float direction,
    float dead_zone,
    float max_speed,
    float integral_zone,
    float integral_speed_limit
)
{
    float error = applySoftDeadZone(predicted_error, dead_zone);
    float candidate_integral = state.integral;

    if (gains.ki <= 0.0f) {
        candidate_integral = 0.0f;
    }
    else if (
        dt > 0.0f &&
        freshness >= 0.999f &&
        fabsf(error) <= integral_zone
    ) {
        candidate_integral += error * dt;
        float integral_limit = integral_speed_limit / gains.ki;
        candidate_integral = clampFloat(
            candidate_integral,
            -integral_limit,
            integral_limit
        );
    }

    float proportional = gains.kp * error;
    float derivative = gains.kd * state.velocity;
    float integral = gains.ki * candidate_integral;
    float unsaturated = proportional + integral + derivative;

    bool pushes_into_saturation =
        fabsf(unsaturated) > max_speed &&
        unsaturated * error > 0.0f;

    if (pushes_into_saturation) {
        candidate_integral = state.integral;
        integral = gains.ki * candidate_integral;
        unsaturated = proportional + integral + derivative;
    }

    state.integral = candidate_integral;
    return direction * clampFloat(
        unsaturated * freshness,
        -max_speed,
        max_speed
    );
}

float PredictiveVisionTracker::applySoftDeadZone(
    float value,
    float dead_zone
)
{
    float magnitude = fabsf(value);
    if (magnitude <= dead_zone) {
        return 0.0f;
    }
    return copysignf(magnitude - dead_zone, value);
}
