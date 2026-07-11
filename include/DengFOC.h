#ifndef DENGFOC_H
#define DENGFOC_H

#include <Arduino.h>

// DengFOC 简化版无刷电机 FOC 控制接口。
//
// 命名规则：
// - X轴 = 你已经调好的老M0轴，所以 DFOC_M0_* 仍然保留，等价于 DFOC_X_*。
// - Y轴 = 新增的第二个云台轴，所以 DFOC_M1_* 等价于 DFOC_Y_*。
//
// 当前接线规划：
// - X轴 PWM A/B/C = GPIO4/GPIO5/GPIO6
// - X轴 AS5600 SDA/SCL = GPIO8/GPIO9，使用 I2C0
// - Y轴 PWM A/B/C = GPIO10/GPIO11/GPIO12
// - Y轴 AS5600 SDA/SCL = GPIO14/GPIO15，使用 I2C1

// ==================== 底层工具函数 ====================

// 把任意角度归一化到 0 ~ 2PI，单位 rad。
float _normalizeAngle(float angle);

// 旧底层接口：为了兼容以前代码保留，默认操作X轴。
float _electricalAngle();
void setPwm(float Ua, float Ub, float Uc);
void setTorque(float Uq, float angle_el);

// X/Y轴底层接口。
// 一般不在 main.cpp 里直接用，主要由闭环控制函数内部调用。
float _electricalAngleX();
float _electricalAngleY();
void setPwmX(float Ua, float Ub, float Uc);
void setPwmY(float Ua, float Ub, float Uc);
void setTorqueX(float Uq, float angle_el);
void setTorqueY(float Uq, float angle_el);

// ==================== 初始化和传感器校准 ====================

// 旧接口：初始化X轴。
// power_supply：驱动器母线电压，单位 V，例如 12V 供电就填 12.0。
void DFOC_Vbus(float power_supply);

// 旧接口：校准X轴。
// _PP：电机极对数，例如14极电机填7。
// _DIR：编码器方向，只能填 1 或 -1。
void DFOC_alignSensor(int _PP, int _DIR);

// X轴初始化和校准。
void DFOC_X_Vbus(float power_supply);
void DFOC_X_alignSensor(int _PP, int _DIR);

// Y轴初始化和校准。
void DFOC_Y_Vbus(float power_supply);
void DFOC_Y_alignSensor(int _PP, int _DIR);

// M1兼容接口：初始化Y轴。
void DFOC_M1_Vbus(float power_supply);

// ==================== 串口目标值 ====================

// 从串口接收一个数字目标值，例如发送 "1.57\n"。
String serialReceiveUserCommand();

// 两个函数名字相同但参数不同，C++可以通过参数判断调用哪个函数，这叫“函数重载”。
String serialReceiveUserCommandXY();

// 从指定串口接收
String serialReceiveUserCommandXY(Stream &port);

// 返回最近一次串口接收到的目标值。
float serial_motor_target();
float serial_motor_target_X();
float serial_motor_target_Y();

// ==================== 编码器读取 ====================

// 获取X/Y轴累计机械角度，单位 rad。
float DFOC_X_Angle();
float DFOC_Y_Angle();

// 获取X/Y轴速度，单位一般是 rad/s。
float DFOC_X_Velocity();
float DFOC_Y_Velocity();

// 旧M0接口：等价于X轴。
float DFOC_M0_Angle();
float DFOC_M0_Velocity();

// M1接口：等价于Y轴。
float DFOC_M1_Angle();
float DFOC_M1_Velocity();

// ==================== PID参数设置 ====================

// X轴角度环PID。
// P：比例，越大响应越快，但太大会抖或过冲。
// I：积分，用于消除静态误差，太大会慢慢晃或噪声变大。
// D：微分，普通云台先保持0。
// ramp：PID输出变化率限制，越小越柔，越大限制越弱。
void DFOC_X_SET_ANGLE_PID(float P, float I, float D, float ramp);

// X轴速度环PID。
void DFOC_X_SET_VEL_PID(float P, float I, float D, float ramp);

// X轴执行一次PID计算，一般不在 main.cpp 里直接调用。
float DFOC_X_ANGLE_PID(float error);
float DFOC_X_VEL_PID(float error);

// Y轴角度环和速度环PID。
void DFOC_Y_SET_ANGLE_PID(float P, float I, float D, float ramp);
void DFOC_Y_SET_VEL_PID(float P, float I, float D, float ramp);
float DFOC_Y_ANGLE_PID(float error);
float DFOC_Y_VEL_PID(float error);

// 旧M0接口：等价于X轴PID。
void DFOC_M0_SET_ANGLE_PID(float P, float I, float D, float ramp);
void DFOC_M0_SET_VEL_PID(float P, float I, float D, float ramp);
float DFOC_M0_ANGLE_PID(float error);
float DFOC_M0_VEL_PID(float error);

// M1接口：等价于Y轴PID。
void DFOC_M1_SET_ANGLE_PID(float P, float I, float D, float ramp);
void DFOC_M1_SET_VEL_PID(float P, float I, float D, float ramp);
float DFOC_M1_ANGLE_PID(float error);
float DFOC_M1_VEL_PID(float error);

// ==================== 常用控制模式 ====================

// 位置 + 速度串级控制。Target单位 rad。
// 控制链路：目标角度 - 当前角度 -> 角度环 -> 速度环 -> Uq -> FOC输出。
void DFOC_X_set_Velocity_Angle(float Target);
void DFOC_Y_set_Velocity_Angle(float Target);
void DFOC_M0_set_Velocity_Angle(float Target);
void DFOC_M1_set_Velocity_Angle(float Target);

// 速度闭环控制。Target单位 rad/s。
void DFOC_X_setVelocity(float Target);
void DFOC_Y_setVelocity(float Target);
void DFOC_M0_setVelocity(float Target);
void DFOC_M1_setVelocity(float Target);

// 位置力矩控制。Target单位 rad。
// 角度误差直接经过角度环输出Uq，比串级控制更直接，也更容易抖。
void DFOC_X_set_Force_Angle(float Target);
void DFOC_Y_set_Force_Angle(float Target);
void DFOC_M0_set_Force_Angle(float Target);
void DFOC_M1_set_Force_Angle(float Target);

// 直接Uq电压控制。Target单位 V。
// 适合测试电机是否能出力，不是位置闭环。
void DFOC_X_setTorque(float Target);
void DFOC_Y_setTorque(float Target);
void DFOC_M0_setTorque(float Target);
void DFOC_M1_setTorque(float Target);

#endif
