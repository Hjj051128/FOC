#ifndef PREDICTIVE_VISION_TRACKER_H
#define PREDICTIVE_VISION_TRACKER_H

#include <Arduino.h>

struct PredictiveVisionOutput
{
    float speed_x;
    float speed_y;
    float predicted_error_x;
    float predicted_error_y;
};

struct VisionPidGains
{
    float kp;
    float ki;
    float kd;
};

class PredictiveVisionTracker
{
public:
    PredictiveVisionTracker(
        float observer_bandwidth_hz,
        float lead_time_s,
        float max_prediction_time_s,
        float max_pixel_speed,
        uint32_t command_hold_ms,
        uint32_t stale_timeout_ms
    );

    void reset();
    void update(float error_x, float error_y, uint32_t timestamp_us);

    PredictiveVisionOutput output(
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
    );

private:
    struct AxisState
    {
        float error;
        float velocity;
        float integral;
    };

    void updateAxis(
        AxisState &state,
        float measurement,
        float dt
    ) const;

    float predictedError(
        const AxisState &state,
        float measurement_age_s
    ) const;

    static float pidSpeed(
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
    );

    static float applySoftDeadZone(float value, float dead_zone);

    AxisState state_x;
    AxisState state_y;
    float observer_bandwidth_hz;
    float lead_time_s;
    float max_prediction_time_s;
    float max_pixel_speed;
    uint32_t command_hold_us;
    uint32_t stale_timeout_us;
    uint32_t last_measurement_us;
    uint32_t last_output_us;
    bool initialized;
};

#endif
