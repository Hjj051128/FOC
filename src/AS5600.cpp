#include "AS5600.h"
#include "Wire.h"
#include <Arduino.h> 

#define _2PI 6.28318530718f
static constexpr uint8_t AS5600_ADDRESS = 0x36;
static constexpr uint8_t AS5600_ANGLE_REG = 0x0C;
static constexpr float AS5600_RAD_PER_COUNT = _2PI / 4096.0f;


// AS5600 相关
float Sensor_AS5600::getSensorAngle()
{
  wire->beginTransmission(AS5600_ADDRESS);
  wire->write(AS5600_ANGLE_REG);

  // false表示发送寄存器地址后保持总线，用于重复起始读取。
  if (wire->endTransmission(false) != 0) {
    return angle_prev;
  }

  size_t received = wire->requestFrom(
    AS5600_ADDRESS,
    static_cast<uint8_t>(2)
  );

  if (received != 2 || wire->available() < 2) {
    // 清掉可能残留的不完整数据。
    while (wire->available()) {
      wire->read();
    }

    // 通信失败时保持上一帧，避免错误角度让电机突然跳动。
    return angle_prev;
  }

  uint8_t msb = static_cast<uint8_t>(wire->read());
  uint8_t lsb = static_cast<uint8_t>(wire->read());

  // AS5600角度值为12位：高寄存器低4位 + 低寄存器8位。
  uint16_t rawAngle =
    (static_cast<uint16_t>(msb & 0x0F) << 8) |
    lsb;

  return rawAngle * AS5600_RAD_PER_COUNT;
}
//AS5600 相关

//=========角度处理相关=============
Sensor_AS5600::Sensor_AS5600(int Mot_Num) {
   _Mot_Num=Mot_Num;  //使得 Mot_Num 可以统一在该文件调用
   
}
void Sensor_AS5600::Sensor_init(TwoWire* _wire) {
    wire = _wire;
    delay(500);
    getSensorAngle(); 
    delayMicroseconds(1);
    vel_angle_prev = getSensorAngle(); 
    vel_angle_prev_ts = micros();
    delay(1);
    getSensorAngle(); 
    delayMicroseconds(1);
    angle_prev = getSensorAngle(); 
    angle_prev_ts = micros();
}
void Sensor_AS5600::Sensor_update() {
    float val = getSensorAngle();
    angle_prev_ts = micros();
    float d_angle = val - angle_prev;
    // 圈数检测
    if(abs(d_angle) > (0.8f*_2PI) ) full_rotations += ( d_angle > 0 ) ? -1 : 1; 
    angle_prev = val;
}

void Sensor_AS5600::resetVelocity() {
    vel_angle_prev = angle_prev;
    vel_full_rotations = full_rotations;
    vel_angle_prev_ts = angle_prev_ts;
}

float Sensor_AS5600::getMechanicalAngle() {
    return angle_prev;
}

float Sensor_AS5600::getAngle(){
    return (float)full_rotations * _2PI + angle_prev;
}

float Sensor_AS5600::getVelocity() {
    // 计算采样时间
    uint32_t elapsed_us = angle_prev_ts - vel_angle_prev_ts;
    float Ts = elapsed_us * 1e-6f;
    // 快速修复奇怪的情况（微溢出）
    if(Ts <= 0) Ts = 1e-3f;
    // 速度计算
    float vel = ( (float)(full_rotations - vel_full_rotations)*_2PI + (angle_prev - vel_angle_prev) ) / Ts;    
    // 保存变量以待将来使用
    vel_angle_prev = angle_prev;
    vel_full_rotations = full_rotations;
    vel_angle_prev_ts = angle_prev_ts;
    return vel;
}
