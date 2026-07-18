#include "angle_velocity_observer.h"

static float wrapAngleError(float error)
{
    while (error > PI) {
        error -= 2.0f * PI;
    }

    while (error < -PI) {
        error += 2.0f * PI;
    }

    return error;
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

    if (!initialized || !isfinite(measured_angle)) {
        angle_estimate = isfinite(measured_angle)
            ? measured_angle
            : 0.0f;
        velocity_estimate = 0.0f;
        timestamp_prev = timestamp_now;
        initialized = isfinite(measured_angle);
        return 0.0f;
    }

    uint32_t elapsed_us = timestamp_now - timestamp_prev;
    float dt = elapsed_us * 1e-6f;

    if (dt <= 0.0f) {
        return velocity_estimate;
    }

    if (dt > 0.05f || bandwidth_hz <= 0.0f) {
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

    // A repeated real pole keeps the observer critically damped.
    // Gains are recomputed from dt, so loop-time jitter does not retune it.
    float pole =
        expf(-2.0f * PI * bandwidth_hz * dt);
    float one_minus_pole = 1.0f - pole;
    float alpha = 1.0f - pole * pole;
    float beta = one_minus_pole * one_minus_pole;

    angle_estimate =
        predicted_angle +
        alpha * innovation;
    velocity_estimate +=
        (beta / dt) * innovation;
    timestamp_prev = timestamp_now;

    return velocity_estimate;
}
