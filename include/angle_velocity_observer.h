#ifndef ANGLE_VELOCITY_OBSERVER_H
#define ANGLE_VELOCITY_OBSERVER_H

#include <Arduino.h>

class AngleVelocityObserver
{
public:
    explicit AngleVelocityObserver(float bandwidth_hz);

    float update(float measured_angle);
    void reset();

    float bandwidth_hz;

private:
    float angle_estimate;
    float velocity_estimate;
    uint32_t timestamp_prev;
    bool initialized;
};

#endif
