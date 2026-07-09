#ifndef DENGFOC_H
#define DENGFOC_H

#include <Arduino.h>

// DengFOC 简化版无刷电机 FOC 控制接口。
//
// 当前工程使用：
// - 三相 PWM 输出驱动无刷电机
// - AS5600 磁编码器反馈角度/速度
// - 电压型 FOC：控制 Uq 电压，不是相电流闭环
//
// 常用控制模式：
// - 速度控制：DFOC_M0_setVelocity()
// - 位置串级控制：DFOC_M0_set_Velocity_Angle()，云台优先用这个
// - 位置力矩控制：DFOC_M0_set_Force_Angle()
// - 直接力矩电压控制：DFOC_M0_setTorque()

// ==================== 底层 PWM / FOC 函数 ====================

/**
 * @brief 设置三相 PWM 输出。
 *
 * 一般不在 main.cpp 里直接调用，由 setTorque() 内部调用。
 *
 * @param Ua A 相目标电压，单位 V，范围会被限制到 0 ~ 母线电压。
 * @param Ub B 相目标电压，单位 V，范围会被限制到 0 ~ 母线电压。
 * @param Uc C 相目标电压，单位 V，范围会被限制到 0 ~ 母线电压。
 */
void setPwm(float Ua, float Ub, float Uc);

/**
 * @brief 按给定 q 轴电压和电角度输出 FOC 三相 PWM。
 *
 * 这是库的核心输出函数。速度环、位置环最终都会算出 Uq，
 * 然后调用这个函数输出三相 PWM。
 *
 * @param Uq q 轴电压，单位 V。正负决定力矩方向，绝对值越大出力越大。
 *           内部会限制到 -母线电压/2 ~ +母线电压/2。
 * @param angle_el 电角度，单位 rad，通常由 _electricalAngle() 自动计算。
 */
void setTorque(float Uq, float angle_el);

/**
 * @brief 将任意角度归一化到 0 ~ 2PI。
 *
 * 一般不在 main.cpp 里直接调用。
 *
 * @param angle 输入角度，单位 rad。
 * @return 归一化后的角度，单位 rad，范围 0 ~ 2PI。
 */
float _normalizeAngle(float angle);

/**
 * @brief 计算当前电角度。
 *
 * 电角度 = 机械角度 * 极对数 * 方向 - 零点偏移。
 * FOC 输出必须依赖正确的电角度。
 *
 * @return 当前电角度，单位 rad，范围 0 ~ 2PI。
 */
float _electricalAngle();

// ==================== 初始化 / 校准 ====================

/**
 * @brief 初始化驱动电压、三相 PWM 和 AS5600 编码器。
 *
 * 必须在 setup() 里先调用，再调用 DFOC_alignSensor()。
 *
 * @param power_supply 驱动器母线电压，单位 V。
 *                     例如 12V 供电就填 12.0，3 串锂电满电可填 12.6。
 */
void DFOC_Vbus(float power_supply);

/**
 * @brief 对齐传感器，校准电角度零点。
 *
 * 启动时会给电机一个固定方向的力矩，让转子吸到固定电角度，
 * 然后读取 AS5600 角度作为零点参考。
 *
 * @param _PP 电机极对数 Pole Pairs。
 *            例如 14 极电机是 7 极对，就填 7。
 * @param _DIR 传感器方向，只能用 1 或 -1。
 *             如果电机抖动、不稳定、反向失控，可以尝试把 1 和 -1 对调。
 */
void DFOC_alignSensor(int _PP, int _DIR);

// ==================== 串口目标值 ====================

/**
 * @brief 读取串口输入并更新内部目标值 motor_target。
 *
 * 串口发送一个数字并带换行，例如：
 * 5
 * -3.2
 * 1.57
 *
 * 收到换行后，函数会把字符串转成 float 保存起来。
 *
 * @return 本次收到的完整串口命令字符串；没有新命令时返回空字符串。
 */
String serialReceiveUserCommand();

/**
 * @brief 获取最近一次串口输入的目标值。
 *
 * 常见用法：
 * DFOC_M0_setVelocity(serial_motor_target());
 * DFOC_M0_set_Velocity_Angle(serial_motor_target());
 *
 * @return 最近一次串口接收到的数字。上电未接收前默认为 0。
 */
float serial_motor_target();

// ==================== 传感器读取 ====================

/**
 * @brief 获取当前电机机械角度。
 *
 * 使用 AS5600 的累计角度，并乘上传感器方向 DIR。
 *
 * @return 当前机械角度，单位 rad。可能超过 0 ~ 2PI，是累计角度。
 */
float DFOC_M0_Angle();

/**
 * @brief 获取当前电机速度。
 *
 * 使用 AS5600 测得速度，并经过低通滤波。
 *
 * @return 当前机械角速度，通常单位为 rad/s。
 */
float DFOC_M0_Velocity();

// ==================== PID 参数设置 ====================

/**
 * @brief 设置角度环 PID 参数。
 *
 * 角度环用于位置控制：
 * 目标角度 - 当前角度 -> 角度 PID -> 目标速度或 Uq。
 *
 * @param P 比例增益。越大位置响应越快，太大会抖动。
 * @param I 积分增益。用于消除静态误差，太大会震荡；新手建议先填 0。
 * @param D 微分增益。用于抑制快速变化；新手建议先填 0。
 * @param ramp 输出变化速率限制。越小输出越柔，越大限制越弱。
 */
void DFOC_M0_SET_ANGLE_PID(float P, float I, float D, float ramp);

/**
 * @brief 设置速度环 PID 参数。
 *
 * 速度环用于速度闭环和位置串级控制：
 * 目标速度 - 当前速度 -> 速度 PID -> Uq。
 *
 * @param P 比例增益。越大速度响应越快，太大会抖动/啸叫。
 * @param I 积分增益。用于消除速度稳态误差，太大会震荡；新手建议先填 0。
 * @param D 微分增益。普通云台/电机测试建议先填 0。
 * @param ramp 输出变化速率限制。越小输出越柔，越大限制越弱。
 */
void DFOC_M0_SET_VEL_PID(float P, float I, float D, float ramp);

/**
 * @brief 执行一次速度环 PID 计算。
 *
 * 一般不在 main.cpp 里直接调用，由 DFOC_M0_setVelocity() 内部调用。
 *
 * @param error 速度误差，通常是目标速度 - 当前速度。
 * @return PID 输出的 q 轴电压 Uq，单位 V。
 */
float DFOC_M0_VEL_PID(float error);

/**
 * @brief 执行一次角度环 PID 计算。
 *
 * 一般不在 main.cpp 里直接调用，由位置控制函数内部调用。
 *
 * @param error 角度误差，通常是目标角度 - 当前角度。
 * @return PID 输出。串级位置控制中通常作为目标速度；力位控制中作为 Uq。
 */
float DFOC_M0_ANGLE_PID(float error);

// ==================== 用户常用控制接口 ====================

/**
 * @brief 位置串级控制：角度环 + 速度环。
 *
 * 云台位置控制优先使用这个函数。
 *
 * 控制链路：
 * 目标角度 - 当前角度 -> 角度 PID -> 目标速度
 * 目标速度 - 当前速度 -> 速度 PID -> Uq -> FOC PWM
 *
 * @param Target 目标机械角度，单位 rad。
 *               例如 1.5708 rad 约等于 90 度，3.1416 rad 约等于 180 度。
 */
void DFOC_M0_set_Velocity_Angle(float Target);

/**
 * @brief 速度闭环控制。
 *
 * 控制链路：
 * 目标速度 - 当前速度 -> 速度 PID -> Uq -> FOC PWM
 *
 * @param Target 目标机械角速度，通常单位为 rad/s。
 *               正负决定旋转方向，0 表示停止并尽量保持速度为 0。
 */
void DFOC_M0_setVelocity(float Target);

/**
 * @brief 位置力矩控制：角度误差直接输出 Uq。
 *
 * 控制链路：
 * 目标角度 - 当前角度 -> 角度 PID -> Uq -> FOC PWM
 *
 * 响应直接，但比位置串级控制更容易抖。云台新手建议先用
 * DFOC_M0_set_Velocity_Angle()。
 *
 * @param Target 目标机械角度，单位 rad。
 */
void DFOC_M0_set_Force_Angle(float Target);

/**
 * @brief 直接力矩电压控制。
 *
 * 没有速度闭环，也没有位置闭环。适合测试电机是否能出力。
 *
 * @param Target q 轴目标电压 Uq，单位 V。
 *               正负决定出力方向，绝对值越大出力越大。
 */
void DFOC_M0_setTorque(float Target);

#endif
