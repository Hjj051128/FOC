#include <Arduino.h>
#include "DengFOC.h"

// X axis is the axis you have already tuned.
#define ANGKP_X       2.3f
#define ANGKI_X       0.01f
#define ANGKD_X       0.0f

#define SPEEDKP_X     0.006f
#define SPEEDKI_X     0.0f
#define SPEEDKD_X     0.0f

// Y axis parameters are reserved here. Do not enable Y until the hardware is ready.
#define ANGKP_Y       2.3f
#define ANGKI_Y       0.01f
#define ANGKD_Y       0.0f

#define SPEEDKP_Y     0.006f
#define SPEEDKI_Y     0.0f
#define SPEEDKD_Y     0.0f

int Sensor_DIR_X = -1;
int Motor_PP_X = 7;
int EN_X = 7;

int Sensor_DIR_Y = -1;
int Motor_PP_Y = 7;
int EN_Y = 13;

float targetX = 0.0f;
float angleX = 0.0f;
float velocityX = 0.0f;
float errorX = 0.0f;

float targetY = 0.0f;
float angleY = 0.0f;
float velocityY = 0.0f;
float errorY = 0.0f;

hw_timer_t *debug_timer = NULL;
volatile bool print_flag = false;

void onDebugTimer();

void setup()
{
  // 使能X电机
  pinMode(EN_X, OUTPUT);
  digitalWrite(EN_X, LOW);  // 暂时让X不动

  // 使能Y电机
  pinMode(EN_Y, OUTPUT);
  digitalWrite(EN_Y, LOW);

  Serial.begin(115200);

  // 初始化X电机电压
  DFOC_X_Vbus(12.0f);
  DFOC_X_alignSensor(Motor_PP_X, Sensor_DIR_X);

  // X角度环PID
  DFOC_X_SET_ANGLE_PID(ANGKP_X, ANGKI_X, ANGKD_X, 100000);
  // X速度环PID
  DFOC_X_SET_VEL_PID(SPEEDKP_X, SPEEDKI_X, SPEEDKD_X, 0);

  // Later, when testing Y alone, enable these lines:
  // 使能Y电机
  digitalWrite(EN_Y, HIGH);
  DFOC_Y_Vbus(12.0f);
  DFOC_Y_alignSensor(Motor_PP_Y, Sensor_DIR_Y);
  DFOC_Y_SET_ANGLE_PID(ANGKP_Y, ANGKI_Y, ANGKD_Y, 100000);
  DFOC_Y_SET_VEL_PID(SPEEDKP_Y, SPEEDKI_Y, SPEEDKD_Y, 0);

  // 定时器中断用于调试串口
  debug_timer = timerBegin(1000000);
  timerAttachInterrupt(debug_timer, &onDebugTimer);
  timerAlarm(debug_timer, 20000, true, 0);
}

void loop()
{
  serialReceiveUserCommand();

  // targetX = serial_motor_target();
  // DFOC_X_set_Velocity_Angle(targetX);

  // Y is not controlled yet. Keep this disabled until Y hardware and direction are tested.
  targetY = serial_motor_target();
  DFOC_Y_set_Velocity_Angle(targetY);

  if (print_flag) {
    print_flag = false;

    // angleX = DFOC_X_Angle();
    // velocityX = DFOC_X_Velocity();
    // errorX = targetX - angleX;

    // Serial.print(velocityX);
    // Serial.print(",");
    // Serial.print(angleX);
    // Serial.print(",");
    // Serial.println(errorX);

  
    angleY = DFOC_Y_Angle();        // 获取当前Y角度
    velocityY = DFOC_Y_Velocity();  // 获取当前Y速度
    errorY = targetY - angleY;      // 角度误差
    Serial.print(velocityY); 
    Serial.print(",");
    Serial.print(angleY); 
    Serial.print(",");
    Serial.println(errorY);
  }
}

void onDebugTimer()
{
  print_flag = true;
}
