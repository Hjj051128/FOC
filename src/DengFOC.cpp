#include <Arduino.h>
#include "DengFOC.h"
#include "AS5600.h"
#include "lowpass_filter.h"
#include "pid.h"

#define _constrain(amt, low, high) ((amt) < (low) ? (low) : ((amt) > (high) ? (high) : (amt)))
#define _3PI_2 4.71238898038f

float voltage_power_supply = 12.0f;

// X axis hardware config. Old M0 functions are kept as wrappers around X.
float zero_electric_angle_X = 0.0f;
int pole_pairs_X = 1;
int sensor_dir_X = 1;
int pwmAX = 4;
int pwmBX = 5;
int pwmCX = 6;

// Y axis hardware config. AS5600 uses a second I2C bus because the address is fixed.
float zero_electric_angle_Y = 0.0f;
int pole_pairs_Y = 1;
int sensor_dir_Y = 1;
int pwmAY = 10;
int pwmBY = 11;
int pwmCY = 12;

LowPassFilter vel_filter_X = LowPassFilter(0.01f);
LowPassFilter vel_filter_Y = LowPassFilter(0.01f);

PIDController vel_loop_X(2, 0, 0, 100000, 0);
PIDController angle_loop_X(2, 0, 0, 100000, 100);
PIDController vel_loop_Y(2, 0, 0, 100000, 0);
PIDController angle_loop_Y(2, 0, 0, 100000, 100);

Sensor_AS5600 sensorX = Sensor_AS5600(0);
TwoWire i2cX = TwoWire(0);

Sensor_AS5600 sensorY = Sensor_AS5600(1);
TwoWire i2cY = TwoWire(1);

float motor_target = 0.0f;
int commaPosition = 0;

static void writePwmToPins(float Ua, float Ub, float Uc, int pinA, int pinB, int pinC)
{
  Ua = _constrain(Ua, 0.0f, voltage_power_supply);
  Ub = _constrain(Ub, 0.0f, voltage_power_supply);
  Uc = _constrain(Uc, 0.0f, voltage_power_supply);

  float dc_a = _constrain(Ua / voltage_power_supply, 0.0f, 1.0f);
  float dc_b = _constrain(Ub / voltage_power_supply, 0.0f, 1.0f);
  float dc_c = _constrain(Uc / voltage_power_supply, 0.0f, 1.0f);

  ledcWrite(pinA, dc_a * 255);
  ledcWrite(pinB, dc_b * 255);
  ledcWrite(pinC, dc_c * 255);
}

static void torqueToPhaseVoltage(float Uq, float angle_el, float &Ua, float &Ub, float &Uc)
{
  Uq = _constrain(Uq, -voltage_power_supply / 2.0f, voltage_power_supply / 2.0f);
  angle_el = _normalizeAngle(angle_el);

  float Ualpha = -Uq * sin(angle_el);
  float Ubeta = Uq * cos(angle_el);

  Ua = Ualpha + voltage_power_supply / 2.0f;
  Ub = (sqrt(3.0f) * Ubeta - Ualpha) / 2.0f + voltage_power_supply / 2.0f;
  Uc = (-Ualpha - sqrt(3.0f) * Ubeta) / 2.0f + voltage_power_supply / 2.0f;
}

float _normalizeAngle(float angle)
{
  float a = fmod(angle, 2.0f * PI);
  return a >= 0.0f ? a : (a + 2.0f * PI);
}

// Old low-level API: kept for compatibility and mapped to X axis.
void setPwm(float Ua, float Ub, float Uc)
{
  setPwmX(Ua, Ub, Uc);
}

void setPwmX(float Ua, float Ub, float Uc)
{
  writePwmToPins(Ua, Ub, Uc, pwmAX, pwmBX, pwmCX);
}

void setPwmY(float Ua, float Ub, float Uc)
{
  writePwmToPins(Ua, Ub, Uc, pwmAY, pwmBY, pwmCY);
}

// Old low-level API: kept for compatibility and mapped to X axis.
void setTorque(float Uq, float angle_el)
{
  setTorqueX(Uq, angle_el);
}

void setTorqueX(float Uq, float angle_el)
{
  sensorX.Sensor_update();
  float Ua = 0.0f;
  float Ub = 0.0f;
  float Uc = 0.0f;
  torqueToPhaseVoltage(Uq, angle_el, Ua, Ub, Uc);
  setPwmX(Ua, Ub, Uc);
}

void setTorqueY(float Uq, float angle_el)
{
  sensorY.Sensor_update();
  float Ua = 0.0f;
  float Ub = 0.0f;
  float Uc = 0.0f;
  torqueToPhaseVoltage(Uq, angle_el, Ua, Ub, Uc);
  setPwmY(Ua, Ub, Uc);
}

float _electricalAngle()
{
  return _electricalAngleX();
}

float _electricalAngleX()
{
  return _normalizeAngle((float)(sensor_dir_X * pole_pairs_X) * sensorX.getMechanicalAngle() - zero_electric_angle_X);
}

float _electricalAngleY()
{
  return _normalizeAngle((float)(sensor_dir_Y * pole_pairs_Y) * sensorY.getMechanicalAngle() - zero_electric_angle_Y);
}

void DFOC_X_Vbus(float power_supply)
{
  voltage_power_supply = power_supply;

  pinMode(pwmAX, OUTPUT);
  pinMode(pwmBX, OUTPUT);
  pinMode(pwmCX, OUTPUT);
  ledcAttach(pwmAX, 30000, 8);
  ledcAttach(pwmBX, 30000, 8);
  ledcAttach(pwmCX, 30000, 8);

  i2cX.begin(8, 9, 400000UL);
  sensorX.Sensor_init(&i2cX);

  vel_loop_X = PIDController(2, 0, 0, 100000, voltage_power_supply / 2.0f);
  Serial.println("X axis PWM and AS5600 init done");
}

void DFOC_Y_Vbus(float power_supply)
{
  voltage_power_supply = power_supply;

  pinMode(pwmAY, OUTPUT);
  pinMode(pwmBY, OUTPUT);
  pinMode(pwmCY, OUTPUT);
  ledcAttach(pwmAY, 30000, 8);
  ledcAttach(pwmBY, 30000, 8);
  ledcAttach(pwmCY, 30000, 8);

  i2cY.begin(14, 15, 400000UL);
  sensorY.Sensor_init(&i2cY);

  vel_loop_Y = PIDController(2, 0, 0, 100000, voltage_power_supply / 2.0f);
  Serial.println("Y axis PWM and AS5600 init done");
}

void DFOC_Vbus(float power_supply)
{
  DFOC_X_Vbus(power_supply);
}

void DFOC_M1_Vbus(float power_supply)
{
  DFOC_Y_Vbus(power_supply);
}

void DFOC_X_alignSensor(int _PP, int _DIR)
{
  pole_pairs_X = _PP;
  sensor_dir_X = _DIR;

  setTorqueX(3.0f, _3PI_2);
  delay(1000);
  sensorX.Sensor_update();
  zero_electric_angle_X = _electricalAngleX();
  setTorqueX(0.0f, _3PI_2);

  Serial.print("X zero electric angle: ");
  Serial.println(zero_electric_angle_X);
}

void DFOC_Y_alignSensor(int _PP, int _DIR)
{
  pole_pairs_Y = _PP;
  sensor_dir_Y = _DIR;

  setTorqueY(3.0f, _3PI_2);
  delay(1000);
  sensorY.Sensor_update();
  zero_electric_angle_Y = _electricalAngleY();
  setTorqueY(0.0f, _3PI_2);

  Serial.print("Y zero electric angle: ");
  Serial.println(zero_electric_angle_Y);
}

void DFOC_alignSensor(int _PP, int _DIR)
{
  DFOC_X_alignSensor(_PP, _DIR);
}

float DFOC_X_Angle()
{
  return sensor_dir_X * sensorX.getAngle();
}

float DFOC_Y_Angle()
{
  return sensor_dir_Y * sensorY.getAngle();
}

float DFOC_M0_Angle()
{
  return DFOC_X_Angle();
}

float DFOC_M1_Angle()
{
  return DFOC_Y_Angle();
}

float DFOC_X_Velocity()
{
  float velocity_raw_X = sensorX.getVelocity();
  return vel_filter_X(sensor_dir_X * velocity_raw_X);
}

float DFOC_Y_Velocity()
{
  float velocity_raw_Y = sensorY.getVelocity();
  return vel_filter_Y(sensor_dir_Y * velocity_raw_Y);
}

float DFOC_M0_Velocity()
{
  return DFOC_X_Velocity();
}

float DFOC_M1_Velocity()
{
  return DFOC_Y_Velocity();
}

void DFOC_X_SET_VEL_PID(float P, float I, float D, float ramp)
{
  vel_loop_X.P = P;
  vel_loop_X.I = I;
  vel_loop_X.D = D;
  vel_loop_X.output_ramp = ramp;
}

void DFOC_Y_SET_VEL_PID(float P, float I, float D, float ramp)
{
  vel_loop_Y.P = P;
  vel_loop_Y.I = I;
  vel_loop_Y.D = D;
  vel_loop_Y.output_ramp = ramp;
}

void DFOC_M0_SET_VEL_PID(float P, float I, float D, float ramp)
{
  DFOC_X_SET_VEL_PID(P, I, D, ramp);
}

void DFOC_M1_SET_VEL_PID(float P, float I, float D, float ramp)
{
  DFOC_Y_SET_VEL_PID(P, I, D, ramp);
}

void DFOC_X_SET_ANGLE_PID(float P, float I, float D, float ramp)
{
  angle_loop_X.P = P;
  angle_loop_X.I = I;
  angle_loop_X.D = D;
  angle_loop_X.output_ramp = ramp;
}

void DFOC_Y_SET_ANGLE_PID(float P, float I, float D, float ramp)
{
  angle_loop_Y.P = P;
  angle_loop_Y.I = I;
  angle_loop_Y.D = D;
  angle_loop_Y.output_ramp = ramp;
}

void DFOC_M0_SET_ANGLE_PID(float P, float I, float D, float ramp)
{
  DFOC_X_SET_ANGLE_PID(P, I, D, ramp);
}

void DFOC_M1_SET_ANGLE_PID(float P, float I, float D, float ramp)
{
  DFOC_Y_SET_ANGLE_PID(P, I, D, ramp);
}

float DFOC_X_VEL_PID(float error)
{
  return vel_loop_X(error);
}

float DFOC_Y_VEL_PID(float error)
{
  return vel_loop_Y(error);
}

float DFOC_M0_VEL_PID(float error)
{
  return DFOC_X_VEL_PID(error);
}

float DFOC_M1_VEL_PID(float error)
{
  return DFOC_Y_VEL_PID(error);
}

float DFOC_X_ANGLE_PID(float error)
{
  return angle_loop_X(error);
}

float DFOC_Y_ANGLE_PID(float error)
{
  return angle_loop_Y(error);
}

float DFOC_M0_ANGLE_PID(float error)
{
  return DFOC_X_ANGLE_PID(error);
}

float DFOC_M1_ANGLE_PID(float error)
{
  return DFOC_Y_ANGLE_PID(error);
}

String serialReceiveUserCommand()
{
  static String received_chars;
  String command = "";

  while (Serial.available()) {
    char inChar = (char)Serial.read();
    received_chars += inChar;

    if (inChar == '\n') {
      command = received_chars;
      commaPosition = command.indexOf('\n');
      if (commaPosition != -1) {
        motor_target = command.substring(0, commaPosition).toDouble();
        Serial.println(motor_target);
      }
      received_chars = "";
    }
  }
  return command;
}

float serial_motor_target()
{
  return motor_target;
}

void DFOC_X_set_Velocity_Angle(float Target)
{
  setTorqueX(DFOC_X_VEL_PID(DFOC_X_ANGLE_PID((Target - DFOC_X_Angle()) * 180.0f / PI)), _electricalAngleX());
}

void DFOC_Y_set_Velocity_Angle(float Target)
{
  setTorqueY(DFOC_Y_VEL_PID(DFOC_Y_ANGLE_PID((Target - DFOC_Y_Angle()) * 180.0f / PI)), _electricalAngleY());
}

void DFOC_M0_set_Velocity_Angle(float Target)
{
  DFOC_X_set_Velocity_Angle(Target);
}

void DFOC_M1_set_Velocity_Angle(float Target)
{
  DFOC_Y_set_Velocity_Angle(Target);
}

void DFOC_X_setVelocity(float Target)
{
  setTorqueX(DFOC_X_VEL_PID((Target - DFOC_X_Velocity()) * 180.0f / PI), _electricalAngleX());
}

void DFOC_Y_setVelocity(float Target)
{
  setTorqueY(DFOC_Y_VEL_PID((Target - DFOC_Y_Velocity()) * 180.0f / PI), _electricalAngleY());
}

void DFOC_M0_setVelocity(float Target)
{
  DFOC_X_setVelocity(Target);
}

void DFOC_M1_setVelocity(float Target)
{
  DFOC_Y_setVelocity(Target);
}

void DFOC_X_set_Force_Angle(float Target)
{
  setTorqueX(DFOC_X_ANGLE_PID((Target - DFOC_X_Angle()) * 180.0f / PI), _electricalAngleX());
}

void DFOC_Y_set_Force_Angle(float Target)
{
  setTorqueY(DFOC_Y_ANGLE_PID((Target - DFOC_Y_Angle()) * 180.0f / PI), _electricalAngleY());
}

void DFOC_M0_set_Force_Angle(float Target)
{
  DFOC_X_set_Force_Angle(Target);
}

void DFOC_M1_set_Force_Angle(float Target)
{
  DFOC_Y_set_Force_Angle(Target);
}

void DFOC_X_setTorque(float Target)
{
  setTorqueX(Target, _electricalAngleX());
}

void DFOC_Y_setTorque(float Target)
{
  setTorqueY(Target, _electricalAngleY());
}

void DFOC_M0_setTorque(float Target)
{
  DFOC_X_setTorque(Target);
}

void DFOC_M1_setTorque(float Target)
{
  DFOC_Y_setTorque(Target);
}
