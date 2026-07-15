#include <Arduino.h>
#include "DengFOC.h"
#include "ICM42688.h"
#include "WiFi.h"
#include "WiFiUdp.h"
#include <Preferences.h>

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

// WiFi调参范围，X/Y角度环共用
#define ANGKP_MIN     0.0f
#define ANGKP_MAX     10.0f
#define ANGKI_MIN     0.0f
#define ANGKI_MAX     1.0f
#define ANGKD_MIN     0.0f
#define ANGKD_MAX     1.0f

// WiFi视觉跟踪比例调参范围
#define VISION_K_MIN  0.0f
#define VISION_K_MAX  0.1f

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

// 视觉跟踪参数
#define VISION_KX            0.002f  // x轴像素误差转换到速度的比例
#define VISION_KY            0.002f  // y轴像素误差转换到速度的比例
#define MAX_TRACK_SPEED_X    0.6f    // x轴跟踪最大速度
#define MAX_TRACK_SPEED_Y    0.6f    // y轴跟踪最大速度
#define VISION_DEAD_ZONE     5.0f    // 中心死区像素
#define VISION_DIR_X         1.0     // X轴视觉误差方向
#define VISION_DIR_Y         1.0     // Y轴视觉误差方向

// ICM-42688使用独立SPI总线
#define IMU_SPI_SCLK         42      // SCLK引脚
#define IMU_SPI_MISO         40      // MISO引脚
#define IMU_SPI_MOSI         41      // MOSI引脚
#define IMU_SPI_CS           39      // CS片选引脚
#define IMU_SAMPLE_PERIOD_US 2000UL  // 500Hz采样周期。

float xAngKp = ANGKP_X;
float xAngKi = ANGKI_X;
float xAngKd = ANGKD_X;

float yAngKp = ANGKP_Y;
float yAngKi = ANGKI_Y;
float yAngKd = ANGKD_Y;

float visionKx = VISION_KX;
float visionKy = VISION_KY;

Preferences preferences;

// 电机引脚
int EN_X = 7;
int EN_Y = 13;

float targetX = 0.0f;

float targetY = 0.0f;

unsigned long last_command_ms;
bool command_received = false;

// 设置创建热点的信息
const char *ssid = "ESP32_S3";      // ID
const char *password = "66666666"; // 连接密码

// 创建UDP对象，不需要建立握手。使用 UDP 协议收发数据包，实现上位机和云台无线通信。
WiFiUDP udp;

// vofa地址和端口
const IPAddress vofaIp(192, 168, 4, 255);  // 广播转发
const uint16_t vofaPort = 1347;            // ESP发送端口号
const uint16_t udpPort = 1346;             // ESP接收端口号

// VOFA发送函数
void sendVofa(
  float ch1,
  float ch2,
  float ch3,
  float ch4,
  float ch5,
  float ch6
);

// 本地接收
void receiveUdpCommand();
void loadParameters();
void saveParameters();
void resetParameters();

// 创建串口对象
HardwareSerial VisionSerial(1);   // (1) 表示绑定ESP32的UART1控制器

// 创建ICM42688Sensor类
ICM42688Sensor imu(
  SPI,
  IMU_SPI_SCLK,
  IMU_SPI_MISO,
  IMU_SPI_MOSI,
  IMU_SPI_CS
);

// IMU初始化成功标志位。失败不进行数据读取
bool imu_ready = false;

// IMU上一次采样时间戳，用来控制采样频率
unsigned long last_imu_sample_us = 0;

// 创建定时器对象
hw_timer_t *debug_timer = NULL;

// 串口打印标志位
volatile bool print_flag = false;

void onDebugTimer();

void setup()
{
  preferences.begin("gimbal", false);
  loadParameters();

  // 使能X电机
  pinMode(EN_X, OUTPUT);
  digitalWrite(EN_X, HIGH);  // 暂时让X不动

  // 使能Y电机
  pinMode(EN_Y, OUTPUT);
  digitalWrite(EN_Y, LOW);

  // UART1用于视觉模块
  VisionSerial.begin(
    115200,       // 波特率
     SERIAL_8E1,  // 8位数据位，无校验位，1位停止位
     16,          // RX脚
     17           // TX脚
    );         

  // 初始化后保持云台静止约1秒，用于估算陀螺仪零偏。
  imu_ready = imu.begin();
  if (imu_ready) {
    imu_ready = imu.calibrateGyro();
  }

  // 初始化X电机电压
  DFOC_X_Vbus(12.0f);
  DFOC_X_alignSensor(MOTOR_PP_X, SENSOR_DIR_X);

  // X角度环PID
  DFOC_X_SET_ANGLE_PID(xAngKp, xAngKi, xAngKd, 100000);
  // X速度环PID
  DFOC_X_SET_VEL_PID(SPEEDKP_X, SPEEDKI_X, SPEEDKD_X, 0);

  // Later, when testing Y alone, enable these lines:
  // 使能Y电机
  digitalWrite(EN_Y, HIGH);
  DFOC_Y_Vbus(12.0f);
  DFOC_Y_alignSensor(MOTOR_PP_Y, SENSOR_DIR_Y);
  DFOC_Y_SET_ANGLE_PID(yAngKp, yAngKi, yAngKd, 100000);
  DFOC_Y_SET_VEL_PID(SPEEDKP_Y, SPEEDKI_Y, SPEEDKD_Y, 0);

  DFOC_X_setTorque(0.0f);
  DFOC_Y_setTorque(0.0f);

  // 定时器中断用于调试串口
  debug_timer = timerBegin(1000000);
  timerAttachInterrupt(debug_timer, &onDebugTimer);
  timerAlarm(debug_timer, 20000, true, 0);

  // 设置WIFI模式 AP：Access Point：路由器模式
  WiFi.mode(WIFI_AP);
  // 创建热点 ID 密码 
  WiFi.softAP(ssid, password);
  // 监听本地端口传来的数据
  udp.begin(udpPort);
}

void loop()
{
  receiveUdpCommand();
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
    float trackSpeedX = visionKx * VISION_DIR_X * visionErroeX;
    float trackSpeedY = visionKy * VISION_DIR_Y * visionErroeY;

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

  // 以500Hz读取IMU。读取失败时保留上一帧，不影响现有电机闭环。
  const unsigned long now_us = micros();
  if (
    imu_ready &&
    static_cast<unsigned long>(now_us - last_imu_sample_us) >=
      IMU_SAMPLE_PERIOD_US
  ) {
    last_imu_sample_us = now_us;
    imu.read();
  }

  // VOFA打印
  if (print_flag) {
    print_flag = false;

    // VOFA新增六个通道：陀螺仪XYZ和加速度计XYZ。
    // 库内定义结构体，存放读取的数据 sample:IMU类成员函数，返回上一次read缓存的数据
    const ICM42688Sample &imu_sample = imu.sample();  // 这里取地址，不用拷贝内存，直接使用
    sendVofa(
      imu_ready ? imu_sample.gyroX : 0.0f,
      imu_ready ? imu_sample.gyroY : 0.0f,
      imu_ready ? imu_sample.gyroZ : 0.0f,
      imu_ready ? imu_sample.accelX : 0.0f,
      imu_ready ? imu_sample.accelY : 0.0f,
      imu_ready ? imu_sample.accelZ : 0.0f
    );
  }
}

void onDebugTimer()
{
  print_flag = true;
}

// VOFA发送函数
void sendVofa(float ch1, float ch2, float ch3, float ch4, float ch5, float ch6) {
  // 创建一个缓冲区
  char buffer[96];
  snprintf(
    buffer,
    sizeof(buffer),
    "%.3f,%.3f,%.3f,%.3f,%.3f,%.3f\n",
    ch1,
    ch2,
    ch3,
    ch4,
    ch5,
    ch6
  );

  // 创建一个UDP包，参数是IP地址跟端口
  udp.beginPacket(vofaIp, vofaPort);

  // 把buffer里面的数据存放包里
  udp.write((const uint8_t *)buffer, strlen(buffer));

  // 发送数据
  udp.endPacket();
}

float loadFloatInRange(
  const char *key,
  float defaultValue,
  float minValue,
  float maxValue
) {
  float value = preferences.getFloat(key, defaultValue);

  if (value >= minValue && value <= maxValue) {
    return value;
  }

  return defaultValue;
}

void loadParameters() {
  xAngKp = loadFloatInRange("xap", ANGKP_X, ANGKP_MIN, ANGKP_MAX);
  xAngKi = loadFloatInRange("xai", ANGKI_X, ANGKI_MIN, ANGKI_MAX);
  xAngKd = loadFloatInRange("xad", ANGKD_X, ANGKD_MIN, ANGKD_MAX);

  yAngKp = loadFloatInRange("yap", ANGKP_Y, ANGKP_MIN, ANGKP_MAX);
  yAngKi = loadFloatInRange("yai", ANGKI_Y, ANGKI_MIN, ANGKI_MAX);
  yAngKd = loadFloatInRange("yad", ANGKD_Y, ANGKD_MIN, ANGKD_MAX);

  visionKx = loadFloatInRange(
    "vkx",
    VISION_KX,
    VISION_K_MIN,
    VISION_K_MAX
  );
  visionKy = loadFloatInRange(
    "vky",
    VISION_KY,
    VISION_K_MIN,
    VISION_K_MAX
  );
}

void saveParameters() {
  preferences.putFloat("xap", xAngKp);
  preferences.putFloat("xai", xAngKi);
  preferences.putFloat("xad", xAngKd);
  preferences.putFloat("yap", yAngKp);
  preferences.putFloat("yai", yAngKi);
  preferences.putFloat("yad", yAngKd);
  preferences.putFloat("vkx", visionKx);
  preferences.putFloat("vky", visionKy);
}

void resetParameters() {
  xAngKp = ANGKP_X;
  xAngKi = ANGKI_X;
  xAngKd = ANGKD_X;
  yAngKp = ANGKP_Y;
  yAngKi = ANGKI_Y;
  yAngKd = ANGKD_Y;
  visionKx = VISION_KX;
  visionKy = VISION_KY;

  saveParameters();
}

void receiveUdpCommand() {
  // 解析接收数据包，返回值是数据包大小单位字节
  int packetSize = udp.parsePacket();

  if (packetSize <= 0) {
    return;
  }

  // 定义缓冲区
  char buffer[64];

  // 读取数据到缓冲区
  int len = udp.read(buffer, sizeof(buffer) - 1);

  if (len <= 0) {
    return;
  }

  buffer[len] = '\0';

  // 定义名称缓冲区
  char name[8];
  // 数值缓冲区
  float value;

  // 格式转换
  int count = sscanf(
    buffer,
    "%7[^,],%f",
    name,
    &value
  );

  if (count != 2) {
    return;
  }

  bool updated = false;
  bool xAnglePidUpdated = false;
  bool yAnglePidUpdated = false;

  if (strcmp(name, "XAP") == 0) {
    if (value >= ANGKP_MIN && value <= ANGKP_MAX) {
      xAngKp = value;
      updated = true;
      xAnglePidUpdated = true;
    }
  }
  else if (strcmp(name, "XAI") == 0) {
    if (value >= ANGKI_MIN && value <= ANGKI_MAX) {
      xAngKi = value;
      updated = true;
      xAnglePidUpdated = true;
    }
  }
  else if (strcmp(name, "XAD") == 0) {
    if (value >= ANGKD_MIN && value <= ANGKD_MAX) {
      xAngKd = value;
      updated = true;
      xAnglePidUpdated = true;
    }
  }
  else if (strcmp(name, "YAP") == 0) {
    if (value >= ANGKP_MIN && value <= ANGKP_MAX) {
      yAngKp = value;
      updated = true;
      yAnglePidUpdated = true;
    }
  }
  else if (strcmp(name, "YAI") == 0) {
    if (value >= ANGKI_MIN && value <= ANGKI_MAX) {
      yAngKi = value;
      updated = true;
      yAnglePidUpdated = true;
    }
  }
  else if (strcmp(name, "YAD") == 0) {
    if (value >= ANGKD_MIN && value <= ANGKD_MAX) {
      yAngKd = value;
      updated = true;
      yAnglePidUpdated = true;
    }
  }
  else if (strcmp(name, "VKX") == 0) {
    if (value >= VISION_K_MIN && value <= VISION_K_MAX) {
      visionKx = value;
      updated = true;
    }
  }
  else if (strcmp(name, "VKY") == 0) {
    if (value >= VISION_K_MIN && value <= VISION_K_MAX) {
      visionKy = value;
      updated = true;
    }
  }
  else if (strcmp(name, "SAVE") == 0) {
    if (value == 1.0f) {
      saveParameters();
      updated = true;
    }
  }
  else if (strcmp(name, "RESET") == 0) {
    if (value == 1.0f) {
      resetParameters();
      updated = true;
      xAnglePidUpdated = true;
      yAnglePidUpdated = true;
    }
  }

  if (!updated) {
    return;
  }

  if (xAnglePidUpdated) {
    DFOC_X_SET_ANGLE_PID(
      xAngKp,
      xAngKi,
      xAngKd,
      100000
    );
  }

  if (yAnglePidUpdated) {
    DFOC_Y_SET_ANGLE_PID(
      yAngKp,
      yAngKi,
      yAngKd,
      100000
    );
  }

  // 把当前X、Y角度环参数返回VOFA
  // char reply[96];

  // snprintf(
  //   reply,
  //   sizeof(reply),
  //   "%.3f,%.3f,%.3f,%.3f,%.3f,%.3f\n",
  //   xAngKp,
  //   xAngKi,
  //   xAngKd,
  //   yAngKp,
  //   yAngKi,
  //   yAngKd
  // );

  // udp.beginPacket(vofaIp, vofaPort);
  // udp.write((const uint8_t *)reply, strlen(reply));
  // udp.endPacket();
}
