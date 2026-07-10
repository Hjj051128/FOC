#include <Arduino.h>
#include "DengFOC.h"

int Sensor_DIR = -1;    // 传感器方向
int Motor_PP = 7;       // 电机极对数
int EN = 7;             // 定义使能引脚

float target;
float angle;
float velocity;
float error;

// 定义一个定时器指针
hw_timer_t *debug_timer = NULL;
volatile bool print_flag = false;

void onDebugTimer();

void setup() {
  // 初始化使能引脚高电平
  pinMode(EN, OUTPUT);
  digitalWrite(EN, HIGH);

  // 初始化串口波特率
  Serial.begin(115200);

  //设定驱动器供电电压
  DFOC_Vbus(12.0);  
  
  // 对齐电角度零点
  DFOC_alignSensor(Motor_PP,Sensor_DIR);

  // 位置环PID
  // 有误差改I P也可以，P可以增加力

  DFOC_M0_SET_ANGLE_PID(2.3, 0.01, 0.0, 100000);

  // 设置速度环PID 最后一个参数是变化速率限制，越小，输出变化越慢更柔；越大，几乎不限制
  // 增大P可以提速，但是可能过冲
  DFOC_M0_SET_VEL_PID(0.006,0.00,0,0);

  // 创建定时器设置频率
  debug_timer = timerBegin(1000000);
  timerAttachInterrupt(debug_timer, &onDebugTimer);  // 绑定中断函数
  timerAlarm(debug_timer, 20000, true, 0);   // 设置闹钟无限重复
}

void loop() 
{
  // 接收串口，接收的数据送入全局变量_motor_target
  serialReceiveUserCommand();

  // 设置目标角度
  // 位置 + 速度串级控制
  DFOC_M0_set_Velocity_Angle(serial_motor_target());

  // 串口打印数据部分
  if(print_flag) {
    print_flag = false;
    // 这些变量不应该放在中断函数里面。因为会改变FOC状态
    target = serial_motor_target(); // 目标角度
    angle = DFOC_M0_Angle();        // 获取当前角度
    velocity = DFOC_M0_Velocity();  // 获取当前速度
    error = target - angle;         // 误差：目标减去实际
    Serial.print(velocity);
    Serial.print(",");
    Serial.print(angle);
    Serial.print(",");
    Serial.println(error);
  }
}

void onDebugTimer() {
  print_flag = true;
}
