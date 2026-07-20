#ifndef LOWPASS_FILTER_H
#define LOWPASS_FILTER_H

#include <Arduino.h>

// 一阶低通滤波器。
// 保留原接口，现有电机速度滤波代码无需修改。
class LowPassFilter
{
public:
    explicit LowPassFilter(float time_constant_s);
    ~LowPassFilter() = default;

    float operator()(float input);
    void reset(float value = 0.0f);

    // 一阶低通时间常数，单位s。
    float Tf;

protected:
    uint32_t timestamp_prev;
    float y_prev;
};

// 线性信号自适应α-β状态观测器。
//
// 状态1估计平滑后的输入值，状态2估计输入变化率。以陀螺仪角速度
// deg/s作为输入时：
//   value()      -> 平滑角速度，deg/s
//   derivative() -> 估计角加速度，deg/s^2
//
// 小创新量使用quiet_bandwidth_hz压制静止噪声；检测到明显运动后，
// 带宽连续提高至motion_bandwidth_hz，以减小转弯开始和停止时的延迟。
// α、β增益每次都根据实际dt重算，因此任务调度抖动不会改变滤波特性。
class AlphaBetaFilter
{
public:
    AlphaBetaFilter(
        float quiet_bandwidth_hz,
        float motion_bandwidth_hz,
        float motion_threshold,
        float max_derivative
    );

    float update(float measurement);
    void reset(
        float value = 0.0f,
        float derivative = 0.0f
    );

    float value() const { return value_estimate; }
    float derivative() const { return derivative_estimate; }
    float activeBandwidthHz() const { return active_bandwidth_hz; }
    bool isInitialized() const { return initialized; }

    float quiet_bandwidth_hz;
    float motion_bandwidth_hz;
    float motion_threshold;
    float max_derivative;

private:
    float value_estimate;
    float derivative_estimate;
    float active_bandwidth_hz;
    uint32_t timestamp_prev;
    bool initialized;
};

// 带角度回绕处理的α-β观测器。
// AS5600角度经过±PI边界时仍可连续估计角速度。
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
