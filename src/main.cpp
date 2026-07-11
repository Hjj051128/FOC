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

// 电机方向极性
#define SENSOR_DIR_X  -1
#define SENSOR_DIR_Y  -1

// 电机极对数
#define MOTOR_PP_X    7
#define MOTOR_PP_Y    7

// 限位
#define X_ANGLE_MIN  -1.5f
#define X_ANGLE_MAX  1.5f

#define Y_ANGLE_MIN  -1.5f
#define Y_ANGLE_MAX  1.5f

// 通信超时
#define COMMAND_TIMEOUT_MS  500


// 视觉跟踪参数
#define VISION_KX            0.002f  // x轴像素误差转换到速度的比例
#define VISION_KY            0.002f  // y轴像素误差转换到速度的比例
#define MAX_TRACK_SPEED_X    0.6f    // x轴跟踪最大速度
#define MAX_TRACK_SPEED_Y    0.6f    // y轴跟踪最大速度
#define VISION_DEAD_ZONE     5.0f    // 中心死区像素
#define VISION_DIR_X         1.0     // X轴视觉误差方向
#define VISION_DIR_Y         1.0     // Y轴视觉误差方向


// 电机引脚
int EN_X = 7;
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

// 创建串口对象
HardwareSerial VisionSerial(1);   // (1) 表示绑定ESP32的UART1控制器

// 创建定时器对象
hw_timer_t *debug_timer = NULL;

// 串口打印标志位
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

  // 启动默认串口
  Serial.begin(115200);

  // UART1用于视觉模块
  VisionSerial.begin(
    115200,       // 波特率
     SERIAL_8E1,  // 8位数据位，无校验位，1位停止位
     16,          // RX脚
     17           // TX脚
    );         

  // 初始化X电机电压
  DFOC_X_Vbus(12.0f);
  DFOC_X_alignSensor(MOTOR_PP_X, SENSOR_DIR_X);

  // X角度环PID
  DFOC_X_SET_ANGLE_PID(ANGKP_X, ANGKI_X, ANGKD_X, 100000);
  // X速度环PID
  DFOC_X_SET_VEL_PID(SPEEDKP_X, SPEEDKI_X, SPEEDKD_X, 0);

  // Later, when testing Y alone, enable these lines:
  // 使能Y电机
  digitalWrite(EN_Y, HIGH);
  DFOC_Y_Vbus(12.0f);
  DFOC_Y_alignSensor(MOTOR_PP_Y, SENSOR_DIR_Y);
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
  String command = serialReceiveUserCommandXY(VisionSerial);

  // 只有接收到一条完整数据才计算一次目标角度
  if(command.length() > 0) {
    unsigned long now = millis();  // 获取当前时间

    // 计算两帧视觉数据是时间间隔，第一帧没有上一次时间所以加上条件语句判断是否是第一帧并且给默认数值
    float dt = command_received ? (now - last_command_ms) * 0.001f : 0.033f;
   
    // 防止dt过小通信停顿后突然变大
    dt = constrain(dt, 0.005f, 0.1f);

    float visionErroeX = serial_motor_target_X(); // 获取X像素误差
    float visionErroeY = serial_motor_target_Y(); // 获取Y像素误差

    // 判断是不是在死区里。不能用 else if 必须都要执行
    if(fabs(visionErroeX) < VISION_DEAD_ZONE) {
      visionErroeX = 0.0f;
    } 
    if(fabs(visionErroeY) < VISION_DEAD_ZONE) {
      visionErroeY = 0.0f;
    }

    // 计算云台角速度
    float trackSpeedX = VISION_KX * VISION_DIR_X * visionErroeX;
    float trackSpeedY = VISION_KY * VISION_DIR_Y * visionErroeY;

    // 速度限幅
    trackSpeedX = constrain(trackSpeedX, -MAX_TRACK_SPEED_X, MAX_TRACK_SPEED_X);
    trackSpeedY = constrain(trackSpeedY, -MAX_TRACK_SPEED_Y, MAX_TRACK_SPEED_Y);
    
    // 一帧只让云台运动一小段距离
    // 角度增量 = 角速度 * 时间 是累计数值，为了后面的角度限位
    targetX += trackSpeedX * dt;
    targetY += trackSpeedY * dt;

    // 角度限位
    targetX = constrain(
      targetX,
      X_ANGLE_MIN,
      X_ANGLE_MAX
    );

    targetY = constrain(
      targetY,
      Y_ANGLE_MIN,
      Y_ANGLE_MAX
    );

    // 保留本次视觉数据到达时间
    last_command_ms = millis();
    command_received = true;
  }

  // 不管有没有数据电机闭环必须持续运行
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
