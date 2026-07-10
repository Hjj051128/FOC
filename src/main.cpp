#include <Arduino.h>
#include "DengFOC.h"

// X PID参数
#define ANGKP_X       2.3f
#define ANGKI_X       0.01f
#define ANGKD_X       0.0f

#define SPEEDKP_X     0.006f
#define SPEEDKI_X     0.0f
#define SPEEDKD_X     0.0f

// Y PID参数
#define ANGKP_Y       2.3f
#define ANGKI_Y       0.01f
#define ANGKD_Y       0.0f

#define SPEEDKP_Y     0.006f
#define SPEEDKI_Y     0.0f
#define SPEEDKD_Y     0.0f

// 限位
#define X_ANGLE_MIN  -1.5f
#define X_ANGLE_MAX  1.5f

#define Y_ANGLE_MIN  -1.5f
#define Y_ANGLE_MAX  1.5f

// 通信超时
#define COMMAND_TIMEOUT_MS  500


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

unsigned long last_command_ms;
bool command_received = false;

hw_timer_t *debug_timer = NULL;
volatile bool print_flag = false;

void onDebugTimer();

void setup()
{
  // 使能X电机
  pinMode(EN_X, OUTPUT);
  digitalWrite(EN_X, HIGH);  // 暂时让X不动

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

  DFOC_X_setTorque(0.0f);
  DFOC_Y_setTorque(0.0f);

  // 定时器中断用于调试串口
  debug_timer = timerBegin(1000000);
  timerAttachInterrupt(debug_timer, &onDebugTimer);
  timerAlarm(debug_timer, 20000, true, 0);
}

void loop()
{
  String command = serialReceiveUserCommandXY();

  if(command.length() > 0) {
    targetX = constrain(
      serial_motor_target_X(),
      X_ANGLE_MIN,
      X_ANGLE_MAX
    );

    targetY = constrain(
      serial_motor_target_Y(),
      Y_ANGLE_MIN,
      Y_ANGLE_MAX
    );

    last_command_ms = millis();
    command_received = true;
  }

  DFOC_X_set_Velocity_Angle(targetX);
  DFOC_Y_set_Velocity_Angle(targetY);

  bool communication_online =
    command_received &&
    millis() - last_command_ms <= COMMAND_TIMEOUT_MS;

  if (print_flag) {
    print_flag = false;
  
    angleY = DFOC_Y_Angle();        // 获取当前Y角度
    velocityY = DFOC_Y_Velocity();  // 获取当前Y速度
    errorY = targetY - angleY;      // 角度误差

    angleX = DFOC_X_Angle();
    velocityX = DFOC_X_Velocity();
    errorX = targetX - angleX;
    Serial.print(velocityY); 
    Serial.print(",");
    Serial.print(angleY); 
    Serial.print(",");
    Serial.print(velocityX);
    Serial.print(",");
    Serial.print(angleX);
    Serial.print(",");
    Serial.println(errorY);
  }
}

void onDebugTimer()
{
  print_flag = true;
}
