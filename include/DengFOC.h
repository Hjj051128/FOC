#ifndef DENGFOC_H
#define DENGFOC_H

#include <Arduino.h>

// DengFOC simplified BLDC FOC interface.
//
// Axis naming:
// - X axis = old M0 axis, keeps all DFOC_M0_* names for compatibility.
// - Y axis = new M1 axis, also provides DFOC_M1_* compatibility names.
//
// Current pin plan:
// - X PWM A/B/C = GPIO4/GPIO5/GPIO6
// - X AS5600 SDA/SCL = GPIO8/GPIO9, I2C0
// - Y PWM A/B/C = GPIO10/GPIO11/GPIO12
// - Y AS5600 SDA/SCL = GPIO14/GPIO15, I2C1

// ---------- Low level helpers ----------

float _normalizeAngle(float angle);

// Old low-level names. They still operate on X axis.
float _electricalAngle();
void setPwm(float Ua, float Ub, float Uc);
void setTorque(float Uq, float angle_el);

// X/Y low-level names.
float _electricalAngleX();
float _electricalAngleY();
void setPwmX(float Ua, float Ub, float Uc);
void setPwmY(float Ua, float Ub, float Uc);
void setTorqueX(float Uq, float angle_el);
void setTorqueY(float Uq, float angle_el);

// ---------- Init and sensor alignment ----------

// Old name, initializes X axis.
void DFOC_Vbus(float power_supply);
void DFOC_alignSensor(int _PP, int _DIR);

// X axis.
void DFOC_X_Vbus(float power_supply);
void DFOC_X_alignSensor(int _PP, int _DIR);

// Y axis.
void DFOC_Y_Vbus(float power_supply);
void DFOC_Y_alignSensor(int _PP, int _DIR);

// M1 compatibility names for Y axis.
void DFOC_M1_Vbus(float power_supply);

// ---------- Serial target ----------

String serialReceiveUserCommand();
float serial_motor_target();

// ---------- Sensor read ----------

float DFOC_X_Angle();
float DFOC_X_Velocity();
float DFOC_Y_Angle();
float DFOC_Y_Velocity();

// Old M0 names are X axis.
float DFOC_M0_Angle();
float DFOC_M0_Velocity();

// M1 names are Y axis.
float DFOC_M1_Angle();
float DFOC_M1_Velocity();

// ---------- PID setup ----------

// X axis PID.
void DFOC_X_SET_ANGLE_PID(float P, float I, float D, float ramp);
void DFOC_X_SET_VEL_PID(float P, float I, float D, float ramp);
float DFOC_X_ANGLE_PID(float error);
float DFOC_X_VEL_PID(float error);

// Y axis PID.
void DFOC_Y_SET_ANGLE_PID(float P, float I, float D, float ramp);
void DFOC_Y_SET_VEL_PID(float P, float I, float D, float ramp);
float DFOC_Y_ANGLE_PID(float error);
float DFOC_Y_VEL_PID(float error);

// Old M0 names are X axis.
void DFOC_M0_SET_ANGLE_PID(float P, float I, float D, float ramp);
void DFOC_M0_SET_VEL_PID(float P, float I, float D, float ramp);
float DFOC_M0_ANGLE_PID(float error);
float DFOC_M0_VEL_PID(float error);

// M1 names are Y axis.
void DFOC_M1_SET_ANGLE_PID(float P, float I, float D, float ramp);
void DFOC_M1_SET_VEL_PID(float P, float I, float D, float ramp);
float DFOC_M1_ANGLE_PID(float error);
float DFOC_M1_VEL_PID(float error);

// ---------- Control modes ----------

// Position + velocity cascade control. Target unit: rad.
void DFOC_X_set_Velocity_Angle(float Target);
void DFOC_Y_set_Velocity_Angle(float Target);
void DFOC_M0_set_Velocity_Angle(float Target);
void DFOC_M1_set_Velocity_Angle(float Target);

// Velocity closed-loop control. Target unit: rad/s.
void DFOC_X_setVelocity(float Target);
void DFOC_Y_setVelocity(float Target);
void DFOC_M0_setVelocity(float Target);
void DFOC_M1_setVelocity(float Target);

// Position-to-torque control. Target unit: rad.
void DFOC_X_set_Force_Angle(float Target);
void DFOC_Y_set_Force_Angle(float Target);
void DFOC_M0_set_Force_Angle(float Target);
void DFOC_M1_set_Force_Angle(float Target);

// Direct q-axis voltage control. Target unit: V.
void DFOC_X_setTorque(float Target);
void DFOC_Y_setTorque(float Target);
void DFOC_M0_setTorque(float Target);
void DFOC_M1_setTorque(float Target);

#endif
