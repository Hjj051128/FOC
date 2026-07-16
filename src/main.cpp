#include <Arduino.h>
#include "DengFOC.h"
#include "WiFi.h"
#include "WiFiUdp.h"
#include <Preferences.h>  // 把参数保存到 ESP32 的非易失性 Flash 中。


// ============================== X轴 PID 默认参数 ==============================
// ANGKP/ANGKI/ANGKD：X轴角度外环 PID 参数。
// 角度环的输入通常是“目标角度与实际角度之间的误差”，
// 输出通常作为速度环的目标速度。
//
// SPEEDKP/SPEEDKI/SPEEDKD：X轴速度内环 PID 参数。
// 速度环负责让电机实际速度跟随角度环给出的目标速度。
//
// 这些宏是出厂默认值。程序启动时会优先读取 Flash 中已经保存的参数；
// Flash 中没有有效值时，才使用这里的默认值。
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


// ============================== WiFi调参安全范围 ==============================
// VOFA 通过 UDP 修改参数时，程序会检查新数值是否位于这些范围内。
// WiFi调参范围，X/Y角度环共用
#define ANGKP_MIN     0.0f
#define ANGKP_MAX     10.0f
#define ANGKI_MIN     0.0f
#define ANGKI_MAX     1.0f
#define ANGKD_MIN     0.0f
#define ANGKD_MAX     1.0f


// visionKx、visionKy 用来把“视觉像素误差”换算成“目标角速度”。
// 这里限定 WiFi 在线调节时允许输入的最小值和最大值。
// WiFi视觉跟踪比例调参范围
#define VISION_K_MIN  0.0f
#define VISION_K_MAX  0.1f


// ============================== 编码器方向 ==============================
#define SENSOR_DIR_X  -1
#define SENSOR_DIR_Y  -1


// ============================== 电机极对数 ==============================
#define MOTOR_PP_X    7
#define MOTOR_PP_Y    7


// ============================== 云台机械角度限位 ==============================
// -1.5 rad 约等于 -85.9°，1.5 rad 约等于 85.9°。
#define X_ANGLE_MIN  -1.5f
#define X_ANGLE_MAX  1.5f

#define Y_ANGLE_MIN  -1.5f
#define Y_ANGLE_MAX  1.5f


// ============================== 视觉追踪参数 ==============================
#define VISION_KX            0.002f  // x轴像素误差转换到速度的比例
#define VISION_KY            0.002f  // y轴像素误差转换到速度的比例
#define MAX_TRACK_SPEED_X    0.6f    // x轴跟踪最大速度
#define MAX_TRACK_SPEED_Y    0.6f    // y轴跟踪最大速度
#define VISION_DEAD_ZONE     5.0f    // 中心死区像素
#define VISION_DIR_X         1.0     // X轴视觉误差方向
#define VISION_DIR_Y         1.0     // Y轴视觉误差方向

#define DEBUG_PIN   18  // 调试引脚

bool DebugMode = false;  // 调试模式


// ============================== 可运行时修改的参数变量 ==============================
// 宏定义本身不能在程序运行过程中改变，所以另外建立 float 变量。
// VOFA 修改的是这些变量，电机 PID 也从这些变量读取当前参数。
float xAngKp = ANGKP_X;
float xAngKi = ANGKI_X;
float xAngKd = ANGKD_X;

float yAngKp = ANGKP_Y;
float yAngKi = ANGKI_Y;
float yAngKd = ANGKD_Y;

float visionKx = VISION_KX;
float visionKy = VISION_KY;


// Preferences 对象用于访问 ESP32 的 NVS 非易失性存储区。
// 保存后即使断电重启，参数仍然存在。
Preferences preferences;


// ============================== 电机使能引脚 ==============================
int EN_X = 7;
int EN_Y = 13;


// ============================== 当前目标角度 ==============================
// targetX、targetY 是电机角度环最终要追踪的目标值，单位通常为弧度。
// 视觉每来一帧数据，就给它们增加一小段“角度增量”。
float targetX = 0.0f;
float targetY = 0.0f;


// last_command_ms：上一帧有效视觉数据到达的毫秒时间。
// command_received：是否至少收到过一帧视觉数据。
// 两者共同用于计算相邻两帧之间的时间 dt。
unsigned long last_command_ms;
bool command_received = false;


// ============================== ESP32热点信息 ==============================
const char *ssid = "ESP32_S3";      // ID
const char *password = "66666666"; // 连接密码


// ============================== UDP通信对象 ==============================
WiFiUDP udp;


// ============================== UDP调参端口 ==============================
const uint16_t udpPort = 1346;


// ============================== 函数提前声明 ==============================
// UDP调参和参数存储相关函数。

// 本地接收
void receiveUdpCommand();  // 检查并解析一条VOFA命令
void loadParameters();     // 开机时从Flash读取参数
void saveParameters();     // 把当前参数写入Flash
void resetParameters();    // 恢复代码中的默认值并写入Flash


// ============================== 视觉模块串口 ==============================
// 创建串口对象
HardwareSerial VisionSerial(1);   // (1) 表示绑定ESP32的UART1控制器

void setup() {

  // 读取模式
  pinMode(DEBUG_PIN, INPUT_PULLUP);
  delay(30);  // 按键消抖

  DebugMode = digitalRead(DEBUG_PIN) == LOW;

  // 打开名为“gimbal”的NVS命名空间。
  // false 表示以“可读可写”方式打开；若为 true 则仅允许读取。
  preferences.begin("gimbal", false);
  loadParameters();

  // 使能电机引脚 
  pinMode(EN_X, OUTPUT);
  digitalWrite(EN_X, HIGH);  
  pinMode(EN_Y, OUTPUT);
  digitalWrite(EN_Y, HIGH);


  // 初始化视觉通信串口：
  VisionSerial.begin(
    115200,       // 波特率
     SERIAL_8E1,  // 8位数据位、偶校验、1位停止位
     16,          // RX脚
     17           // TX脚
    );         

  DFOC_X_Vbus(12.0f);
  DFOC_X_alignSensor(MOTOR_PP_X, SENSOR_DIR_X); // 随后进行编码器与电机电角度的对齐。

  // X角度环PID
  DFOC_X_SET_ANGLE_PID(xAngKp, xAngKi, xAngKd, 100000);
  // X速度环PID
  DFOC_X_SET_VEL_PID(SPEEDKP_X, SPEEDKI_X, SPEEDKD_X, 0);

  DFOC_Y_Vbus(12.0f);
  DFOC_Y_alignSensor(MOTOR_PP_Y, SENSOR_DIR_Y);
  DFOC_Y_SET_ANGLE_PID(yAngKp, yAngKi, yAngKd, 100000);
  DFOC_Y_SET_VEL_PID(SPEEDKP_Y, SPEEDKI_Y, SPEEDKD_Y, 0);


  // 初始化结束后先把两个轴的转矩指令设为0，
  DFOC_X_setTorque(0.0f);
  DFOC_Y_setTorque(0.0f);


  if(DebugMode) {
    WiFi.mode(WIFI_AP);
    // 创建热点 ID 密码 
    WiFi.softAP(ssid, password);
    // 监听本地端口传来的数据
    udp.begin(udpPort);
  }
  
}


void loop()
{

  // 非阻塞检查UDP接收区。没有数据时函数会立即return，不会卡住主循环。
  if(DebugMode) {
    receiveUdpCommand();
  }

  // 从UART1读取并解析视觉模块的一条完整命令。
  String command = serialReceiveUserCommandXY(VisionSerial);


  // 只有视觉数据更新时，才重新计算一次视觉目标角度。
  // 如果本轮没有新视觉帧，targetX和targetY保持原值，
  if(command.length() > 0) {

    // millis()返回系统启动后的毫秒数，类型为unsigned long。
    unsigned long now = millis();  // 获取当前时间

    // 计算两帧视觉数据是时间间隔，第一帧没有上一次时间所以加上条件语句判断是否是第一帧并且给默认数值
    float dt = command_received ? (now - last_command_ms) * 0.001f : 0.033f;
   
    // 太小会让角度增量接近0；太大会导致网络停顿后云台突然跳一大步。
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


    // 视觉比例控制：
    float trackSpeedX = visionKx * VISION_DIR_X * visionErroeX;
    float trackSpeedY = visionKy * VISION_DIR_Y * visionErroeY;

    // 速度限幅
    trackSpeedX = constrain(trackSpeedX, -MAX_TRACK_SPEED_X, MAX_TRACK_SPEED_X);
    trackSpeedY = constrain(trackSpeedY, -MAX_TRACK_SPEED_Y, MAX_TRACK_SPEED_Y);
    

    // 离散积分：
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


  // 电机闭环必须尽可能高频、连续调用。
  // 即使视觉暂时没有新数据，电机也要继续保持上一次目标角度。
  // 函数名中 Velocity_Angle 表示库内部采用速度环+角度环串级控制。
  // 不管有没有数据电机闭环必须持续运行
  DFOC_X_set_Velocity_Angle(targetX);
  DFOC_Y_set_Velocity_Angle(targetY);
}


// ============================================================================
// loadFloatInRange()：从Flash读取一个float，并验证范围
//
// key：Flash中的短名称，例如“xap”。
// defaultValue：没有存储值或存储值非法时使用的默认值。
// minValue/maxValue：允许的安全范围。
// 返回值：有效的Flash值，或者默认值。
// ============================================================================
float loadFloatInRange(
  const char *key,
  float defaultValue,
  float minValue,
  float maxValue
) {

  // getFloat读取指定key。
  // 如果key不存在，Preferences库直接返回defaultValue。
  float value = preferences.getFloat(key, defaultValue);


  // 读取到的值只有位于安全区间内才采用。
  // 这样即使Flash数据损坏，也不会把异常参数交给电机控制器。
  if (value >= minValue && value <= maxValue) {
    return value;
  }


  // 数值越界时恢复为代码中定义的默认值。
  return defaultValue;
}


// ============================================================================
// loadParameters()：开机时加载所有可保存参数
//
// Flash键名采用短字符串，是为了节省NVS空间：
// xap/xai/xad：X轴角度环Kp/Ki/Kd
// yap/yai/yad：Y轴角度环Kp/Ki/Kd
// vkx/vky：视觉X/Y比例系数
// ============================================================================
void loadParameters() {

  // 每个参数都通过loadFloatInRange读取并做范围检查。
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


// ============================================================================
// saveParameters()：把当前运行参数写入Flash
//
// putFloat会写入NVS。Flash有擦写寿命，不要在loop中高频调用。
// 当前程序只有收到“SAVE,1”或执行RESET时才保存，做法是合理的。
// ============================================================================
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

// ============================================================================
// resetParameters()：恢复代码默认值
//
// 先把运行变量恢复为宏定义的默认参数，再调用saveParameters写入Flash。
// 因此复位后重新上电仍会保持这些默认值。
// ============================================================================
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


// ============================================================================
// receiveUdpCommand()：接收并处理一条VOFA调参命令
//
// ESP32本地监听端口：1346
// 命令格式：参数名,数值
//
// 示例：
// XAP,2.5      修改X轴角度环Kp
// XAI,0.01     修改X轴角度环Ki
// XAD,0.0      修改X轴角度环Kd
// YAP,2.5      修改Y轴角度环Kp
// VKX,0.0025   修改X轴视觉比例
// SAVE,1       把当前参数写入Flash
// RESET,1      恢复默认参数并写入Flash
//
// 处理流程：
// 检查数据包 → 读取字符串 → sscanf拆分名称和值 →
// 判断命令 → 检查范围 → 更新变量 → 必要时立即刷新PID。
// ============================================================================
void receiveUdpCommand() {

  // 解析接收数据包，返回值是数据包大小单位字节
  int packetSize = udp.parsePacket();

  // 没收到有效数据包时立即结束函数，主循环继续执行其他任务。
  if (packetSize <= 0) {
    return;
  }

  // 定义缓冲区
  char buffer[64];

  // 读取数据到缓冲区
  int len = udp.read(buffer, sizeof(buffer) - 1);

  // read失败或没有读到内容时，直接退出。
  if (len <= 0) {
    return;
  }

  // UDP收到的是原始字节，不一定自带C字符串结束符。
  buffer[len] = '\0';

  // 定义名称缓冲区
  char name[8];
  // 数值缓冲区

  // value保存逗号后解析出的浮点数。
  float value;

  // 格式转换
  int count = sscanf(
    buffer,
    "%7[^,],%f",
    name,
    &value
  );


  // 正常命令必须成功解析出name和value两个项目，因此count应等于2。
  if (count != 2) {
    return;
  }


  // updated：本条命令是否被识别且数值有效。
  // xAnglePidUpdated：X轴角度PID是否发生变化。
  // yAnglePidUpdated：Y轴角度PID是否发生变化。
  //
  // 分开设置标志，是为了只刷新真正发生变化的电机PID。
  bool updated = false;
  bool xAnglePidUpdated = false;
  bool yAnglePidUpdated = false;


  // strcmp比较两个C字符串是否完全相同。
  // 返回0表示相同，因此“strcmp(...) == 0”就是命令名称匹配。
  //
  // XAP：X Angle Proportional，X轴角度环Kp。
  if (strcmp(name, "XAP") == 0) {
    if (value >= ANGKP_MIN && value <= ANGKP_MAX) {
      xAngKp = value;
      updated = true;
      xAnglePidUpdated = true;
    }
  }

  // XAI：X轴角度环Ki。更新前检查ANGKI允许范围。
  else if (strcmp(name, "XAI") == 0) {
    if (value >= ANGKI_MIN && value <= ANGKI_MAX) {
      xAngKi = value;
      updated = true;
      xAnglePidUpdated = true;
    }
  }

  // XAD：X轴角度环Kd。
  else if (strcmp(name, "XAD") == 0) {
    if (value >= ANGKD_MIN && value <= ANGKD_MAX) {
      xAngKd = value;
      updated = true;
      xAnglePidUpdated = true;
    }
  }

  // YAP：Y轴角度环Kp。
  else if (strcmp(name, "YAP") == 0) {
    if (value >= ANGKP_MIN && value <= ANGKP_MAX) {
      yAngKp = value;
      updated = true;
      yAnglePidUpdated = true;
    }
  }

  // YAI：Y轴角度环Ki。
  else if (strcmp(name, "YAI") == 0) {
    if (value >= ANGKI_MIN && value <= ANGKI_MAX) {
      yAngKi = value;
      updated = true;
      yAnglePidUpdated = true;
    }
  }

  // YAD：Y轴角度环Kd。
  else if (strcmp(name, "YAD") == 0) {
    if (value >= ANGKD_MIN && value <= ANGKD_MAX) {
      yAngKd = value;
      updated = true;
      yAnglePidUpdated = true;
    }
  }

  // VKX：视觉X轴像素误差到目标角速度的比例系数。
  // 它不属于电机内部角度PID，所以修改后无需调用DFOC_X_SET_ANGLE_PID。
  else if (strcmp(name, "VKX") == 0) {
    if (value >= VISION_K_MIN && value <= VISION_K_MAX) {
      visionKx = value;
      updated = true;
    }
  }

  // VKY：视觉Y轴比例系数。
  else if (strcmp(name, "VKY") == 0) {
    if (value >= VISION_K_MIN && value <= VISION_K_MAX) {
      visionKy = value;
      updated = true;
    }
  }

  // SAVE,1：将当前全部参数写入Flash。
  // 这里要求value严格等于1.0f，因为VOFA发送的是简单控制命令。
  else if (strcmp(name, "SAVE") == 0) {
    if (value == 1.0f) {
      saveParameters();
      updated = true;
    }
  }

  // RESET,1：恢复全部默认参数并保存。
  // 由于X、Y角度PID都恢复了默认值，所以两个PID刷新标志都要置true。
  else if (strcmp(name, "RESET") == 0) {
    if (value == 1.0f) {
      resetParameters();
      updated = true;
      xAnglePidUpdated = true;
      yAnglePidUpdated = true;
    }
  }


  // 命令名不认识、数值越界，或者SAVE/RESET的值不是1时，
  // updated仍为false，本条命令不执行任何后续操作。
  if (!updated) {
    return;
  }

  // X轴角度参数变化后，把新的Kp、Ki、Kd立即写入DengFOC控制器。
  // 否则只修改普通变量，正在运行的控制器内部参数可能不会同步。
  if (xAnglePidUpdated) {
    DFOC_X_SET_ANGLE_PID(
      xAngKp,
      xAngKi,
      xAngKd,
      100000
    );
  }

  // Y轴角度参数变化后同样立即刷新控制器。
  if (yAnglePidUpdated) {
    DFOC_Y_SET_ANGLE_PID(
      yAngKp,
      yAngKi,
      yAngKd,
      100000
    );
  }
}
