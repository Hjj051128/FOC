#ifndef PID_H
#define PID_H

class PIDController
{
public:
    PIDController(
        float P,
        float I,
        float D,
        float ramp,
        float limit,
        float derivative_filter_Tf = 0.0f
    );
    ~PIDController() = default;

    float operator() (float error);
    void reset(float output = 0.0f);

    float P; //!< 比例增益(P环增益)
    float I; //!< 积分增益（I环增益）
    float D; //!< 微分增益（D环增益）
    float output_ramp; 
    float limit;
    float derivative_filter_Tf; //!< D支路低通滤波时间常数，0表示关闭
protected:
    float error_prev; //!< 最后的跟踪误差值
    float output_prev;  //!< 最后一个 pid 输出值
    float integral_prev; //!< 最后一个积分分量值
    float derivative_prev; //!< 上一次滤波后的误差变化率
    unsigned long timestamp_prev; //!< 上次执行时间戳
    bool initialized; //!< 避免PID首次运行时产生微分冲击
};

#endif
