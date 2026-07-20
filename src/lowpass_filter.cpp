#include "lowpass_filter.h"

#include <math.h>

namespace
{
constexpr float TWO_PI_F = 6.28318530718f;
constexpr float OBSERVER_MAX_DT_S = 0.05f;

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

float wrapAngleError(float error)
{
    while (error > PI) {
        error -= TWO_PI_F;
    }

    while (error < -PI) {
        error += TWO_PI_F;
    }

    return error;
}

void calculateAlphaBetaGains(
    float bandwidth_hz,
    float dt,
    float &alpha,
    float &beta
)
{
    float pole = expf(-TWO_PI_F * bandwidth_hz * dt);
    float one_minus_pole = 1.0f - pole;

    // 两个重复实极点形成临界阻尼响应。
    alpha = 1.0f - pole * pole;
    beta = one_minus_pole * one_minus_pole;
}
}

LowPassFilter::LowPassFilter(float time_constant_s)
    : Tf(time_constant_s)
    , timestamp_prev(micros())
    , y_prev(0.0f)
{
}

void LowPassFilter::reset(float value)
{
    y_prev = value;
    timestamp_prev = micros();
}

float LowPassFilter::operator()(float input)
{
    if (!isfinite(input)) {
        return y_prev;
    }

    uint32_t timestamp = micros();
    float dt = (timestamp - timestamp_prev) * 1e-6f;

    if (dt <= 0.0f) {
        return y_prev;
    }

    if (dt > 0.3f || Tf <= 0.0f) {
        y_prev = input;
        timestamp_prev = timestamp;
        return input;
    }

    float alpha = Tf / (Tf + dt);
    y_prev =
        alpha * y_prev +
        (1.0f - alpha) * input;
    timestamp_prev = timestamp;
    return y_prev;
}

AlphaBetaFilter::AlphaBetaFilter(
    float quiet_bandwidth_hz,
    float motion_bandwidth_hz,
    float motion_threshold,
    float max_derivative
)
    : quiet_bandwidth_hz(quiet_bandwidth_hz)
    , motion_bandwidth_hz(motion_bandwidth_hz)
    , motion_threshold(motion_threshold)
    , max_derivative(max_derivative)
    , value_estimate(0.0f)
    , derivative_estimate(0.0f)
    , active_bandwidth_hz(quiet_bandwidth_hz)
    , timestamp_prev(0)
    , initialized(false)
{
}

void AlphaBetaFilter::reset(
    float value,
    float derivative
)
{
    value_estimate = isfinite(value) ? value : 0.0f;
    derivative_estimate =
        isfinite(derivative) ? derivative : 0.0f;
    active_bandwidth_hz = quiet_bandwidth_hz;
    timestamp_prev = micros();
    initialized = isfinite(value);
}

float AlphaBetaFilter::update(float measurement)
{
    uint32_t timestamp_now = micros();

    if (!isfinite(measurement)) {
        return value_estimate;
    }

    if (!initialized) {
        reset(measurement, 0.0f);
        return value_estimate;
    }

    float dt = (timestamp_now - timestamp_prev) * 1e-6f;

    if (dt <= 0.0f) {
        return value_estimate;
    }

    if (
        dt > OBSERVER_MAX_DT_S ||
        quiet_bandwidth_hz <= 0.0f ||
        motion_bandwidth_hz <= 0.0f
    ) {
        reset(measurement, 0.0f);
        return value_estimate;
    }

    float prediction =
        value_estimate +
        derivative_estimate * dt;
    float innovation = measurement - prediction;

    float motion_ratio = 1.0f;
    if (motion_threshold > 0.0f) {
        motion_ratio = clampFloat(
            fabsf(innovation) / motion_threshold,
            0.0f,
            1.0f
        );

        // 平滑插值避免带宽在阈值附近突变。
        motion_ratio =
            motion_ratio *
            motion_ratio *
            (3.0f - 2.0f * motion_ratio);
    }

    active_bandwidth_hz =
        quiet_bandwidth_hz +
        (motion_bandwidth_hz - quiet_bandwidth_hz) *
        motion_ratio;

    float alpha = 0.0f;
    float beta = 0.0f;
    calculateAlphaBetaGains(
        active_bandwidth_hz,
        dt,
        alpha,
        beta
    );

    value_estimate =
        prediction +
        alpha * innovation;
    derivative_estimate +=
        (beta / dt) * innovation;

    if (max_derivative > 0.0f) {
        derivative_estimate = clampFloat(
            derivative_estimate,
            -max_derivative,
            max_derivative
        );
    }

    timestamp_prev = timestamp_now;
    return value_estimate;
}

AngleVelocityObserver::AngleVelocityObserver(float bandwidth_hz)
    : bandwidth_hz(bandwidth_hz)
    , angle_estimate(0.0f)
    , velocity_estimate(0.0f)
    , timestamp_prev(0)
    , initialized(false)
{
}

void AngleVelocityObserver::reset()
{
    angle_estimate = 0.0f;
    velocity_estimate = 0.0f;
    timestamp_prev = 0;
    initialized = false;
}

float AngleVelocityObserver::update(float measured_angle)
{
    uint32_t timestamp_now = micros();

    if (!isfinite(measured_angle)) {
        return velocity_estimate;
    }

    if (!initialized) {
        angle_estimate = measured_angle;
        velocity_estimate = 0.0f;
        timestamp_prev = timestamp_now;
        initialized = true;
        return 0.0f;
    }

    float dt = (timestamp_now - timestamp_prev) * 1e-6f;

    if (dt <= 0.0f) {
        return velocity_estimate;
    }

    if (dt > OBSERVER_MAX_DT_S || bandwidth_hz <= 0.0f) {
        angle_estimate = measured_angle;
        velocity_estimate = 0.0f;
        timestamp_prev = timestamp_now;
        return 0.0f;
    }

    float predicted_angle =
        angle_estimate +
        velocity_estimate * dt;
    float innovation =
        wrapAngleError(measured_angle - predicted_angle);

    float alpha = 0.0f;
    float beta = 0.0f;
    calculateAlphaBetaGains(
        bandwidth_hz,
        dt,
        alpha,
        beta
    );

    angle_estimate =
        predicted_angle +
        alpha * innovation;
    velocity_estimate +=
        (beta / dt) * innovation;
    timestamp_prev = timestamp_now;

    return velocity_estimate;
}
