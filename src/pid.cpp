#include "pid.h"
#include <Arduino.h>
#define _constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))


PIDController::PIDController(
    float P,
    float I,
    float D,
    float ramp,
    float limit,
    float derivative_filter_Tf
)
    : P(P)
    , I(I)
    , D(D)
    , output_ramp(ramp)    // PID控制器加速度限幅
    , limit(limit)         // PID控制器输出限幅
    , derivative_filter_Tf(derivative_filter_Tf)
    , error_prev(0.0f)
    , output_prev(0.0f)
    , integral_prev(0.0f)
    , derivative_prev(0.0f)
    , initialized(false)
{
    timestamp_prev = micros();
}

// PID 控制器函数
float PIDController::operator() (float error){
    // 计算两次循环中间的间隔时间
    unsigned long timestamp_now = micros();
    float Ts = (timestamp_now - timestamp_prev) * 1e-6f;
    if(Ts <= 0 || Ts > 0.5f) Ts = 1e-3f;
    
    // P环
    float proportional = P * error;
    // Tustin 散点积分（I环）
    float integral = integral_prev + I*Ts*0.5f*(error + error_prev);
    integral = _constrain(integral, -limit, limit);

    // D环采用带限微分器，只滤波误差变化率，不拖慢P和I。
    float derivative = 0.0f;
    if (D != 0.0f) {
        float derivative_raw = initialized
            ? (error - error_prev) / Ts
            : 0.0f;

        if (derivative_filter_Tf > 0.0f) {
            float alpha =
                derivative_filter_Tf /
                (derivative_filter_Tf + Ts);

            derivative_prev =
                alpha * derivative_prev +
                (1.0f - alpha) * derivative_raw;
        }
        else {
            derivative_prev = derivative_raw;
        }

        derivative = D * derivative_prev;
    }
    else {
        derivative_prev = 0.0f;
    }

    // 将P,I,D三环的计算值加起来
    float output = proportional + integral + derivative;
    output = _constrain(output, -limit, limit);

    if(output_ramp > 0){
        // 对PID的变化速率进行限制
        float output_rate = (output - output_prev)/Ts;
        if (output_rate > output_ramp)
            output = output_prev + output_ramp*Ts;
        else if (output_rate < -output_ramp)
            output = output_prev - output_ramp*Ts;
    }
    // 保存值（为了下一次循环）
    integral_prev = integral;
    output_prev = output;
    error_prev = error;
    timestamp_prev = timestamp_now;
    initialized = true;
    return output;
}
