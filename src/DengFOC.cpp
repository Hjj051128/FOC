#include <Arduino.h>
#include "DengFOC.h"
#include "AS5600.h"
#include "angle_velocity_observer.h"
#include "lowpass_filter.h"
#include "pid.h"

#define _constrain(amt, low, high) ((amt) < (low) ? (low) : ((amt) > (high) ? (high) : (amt)))
#define _3PI_2 4.71238898038f

// 1 uses the second-order observer in the optimized cascade.
// 0 returns that cascade to the original velocity-difference low-pass filter.
#define USE_ADVANCED_VELOCITY_OBSERVER  1

static constexpr float RAD_TO_DEG_F = 180.0f / PI;
static constexpr float DEG_TO_RAD_F = PI / 180.0f;

// 驱动器母线电压，单位 V。setTorque() 最后会根据它限制最大输出电压。
float voltage_power_supply = 12.0f;

// ==================== X轴硬件和校准参数 ====================
float zero_electric_angle_X = 0.0f;
int pole_pairs_X = 1;        // X轴电机极对数，例如 14极电机就是 7极对。
int sensor_dir_X = 1;        // X轴编码器方向，只能填 1 或 -1，方向反了闭环会乱跑。
int pwmAX = 4;               // X轴A相PWM引脚
int pwmBX = 5;               // X轴B相PWM引脚
int pwmCX = 6;               // X轴C相PWM引脚

// ==================== Y轴硬件和校准参数 ====================
float zero_electric_angle_Y = 0.0f;
int pole_pairs_Y = 1;        // Y轴电机极对数
int sensor_dir_Y = 1;        // Y轴编码器方向
int pwmAY = 10;              // Y轴A相PWM引脚
int pwmBY = 11;              // Y轴B相PWM引脚
int pwmCY = 12;              // Y轴C相PWM引脚

// 用户定义的机械零点。默认关闭，避免改变现有工程的角度行为。
static float mechanical_zero_X = 0.0f;
static float mechanical_zero_Y = 0.0f;
static bool mechanical_zero_enabled = false;

// 云台只在 +/-PI 范围内运动，把跨越AS5600零点后的角度保持连续。
static float normalizeMechanicalAngle(float angle)
{
  while (angle > PI) {
    angle -= 2.0f * PI;
  }

  while (angle < -PI) {
    angle += 2.0f * PI;
  }

  return angle;
}

// 速度缓存低通滤波器：X/Y轴都是10ms。
LowPassFilter vel_filter_X = LowPassFilter(0.01f);
LowPassFilter vel_filter_Y = LowPassFilter(0.01f);

static constexpr float VELOCITY_OBSERVER_BANDWIDTH_HZ_X = 25.0f;
static constexpr float VELOCITY_OBSERVER_BANDWIDTH_HZ_Y = 25.0f;
AngleVelocityObserver velocity_observer_X(
  VELOCITY_OBSERVER_BANDWIDTH_HZ_X
);
AngleVelocityObserver velocity_observer_Y(
  VELOCITY_OBSERVER_BANDWIDTH_HZ_Y
);

// 角度PID的D支路滤波时间常数。5ms约对应31.8Hz截止频率。
static constexpr float ANGLE_D_FILTER_TF_X = 0.005f;
static constexpr float ANGLE_D_FILTER_TF_Y = 0.005f;
// 速度D支路更容易放大编码器差分噪声，因此采用10ms滤波。
// SPEEDKD为0时这部分不参与计算，也没有额外运行开销。
static constexpr float SPEED_D_FILTER_TF_X = 0.010f;
static constexpr float SPEED_D_FILTER_TF_Y = 0.010f;

// X/Y各自独立的PID控制器。
// angle_loop 输出给速度环，vel_loop 输出给FOC的Uq电压。
PIDController vel_loop_X(
  2,
  0,
  0,
  100000,
  0,
  SPEED_D_FILTER_TF_X
);
PIDController angle_loop_X(
  2,
  0,
  0,
  100000,
  100,
  ANGLE_D_FILTER_TF_X
);
PIDController vel_loop_Y(
  2,
  0,
  0,
  100000,
  0,
  SPEED_D_FILTER_TF_Y
);
PIDController angle_loop_Y(
  2,
  0,
  0,
  100000,
  100,
  ANGLE_D_FILTER_TF_Y
);

static DFOCControlState control_state_X = {};
static DFOCControlState control_state_Y = {};

// X轴AS5600：I2C0，当前接线 SDA=GPIO8，SCL=GPIO9。
Sensor_AS5600 sensorX = Sensor_AS5600(0);
TwoWire i2cX = TwoWire(0);

// Y轴AS5600：I2C1，当前接线 SDA=GPIO14，SCL=GPIO15。
Sensor_AS5600 sensorY = Sensor_AS5600(1);
TwoWire i2cY = TwoWire(1);

// 一个目标值
float motor_target = 0.0f;

// 两个目标值。
float motor_target_X = 0.0f;
float motor_target_Y = 0.0f;

int commaPosition = 0;

// 把三相电压 Ua/Ub/Uc 写到指定的三个PWM引脚。
// 这里做成通用函数，X/Y轴都可以复用，区别只在传入的引脚不同。
static void writePwmToPins(float Ua, float Ub, float Uc, int pinA, int pinB, int pinC)
{
  // 先把相电压限制在 0 ~ 母线电压，避免占空比越界。
  Ua = _constrain(Ua, 0.0f, voltage_power_supply);
  Ub = _constrain(Ub, 0.0f, voltage_power_supply);
  Uc = _constrain(Uc, 0.0f, voltage_power_supply);

  // 把电压换算成 0~1 的占空比，再写入8位PWM，也就是 0~255。
  float dc_a = _constrain(Ua / voltage_power_supply, 0.0f, 1.0f);
  float dc_b = _constrain(Ub / voltage_power_supply, 0.0f, 1.0f);
  float dc_c = _constrain(Uc / voltage_power_supply, 0.0f, 1.0f);

  ledcWrite(pinA, dc_a * 255);
  ledcWrite(pinB, dc_b * 255);
  ledcWrite(pinC, dc_c * 255);
}

// FOC核心变换：把q轴电压Uq和电角度转换成三相电压Ua/Ub/Uc。
// Uq可以简单理解成“力矩电压”，正负决定出力方向，绝对值决定出力大小。
static void torqueToPhaseVoltage(float Uq, float angle_el, float &Ua, float &Ub, float &Uc)
{
  // 限制最大Uq。12V供电时默认最大输出约 +/-6V，避免电机和驱动过热。
  Uq = _constrain(Uq, -voltage_power_supply / 2.0f, voltage_power_supply / 2.0f);
  angle_el = _normalizeAngle(angle_el);

  // Park逆变换：dq坐标转 alpha/beta 坐标。这里Ud=0，只控制Uq。
  float Ualpha = -Uq * sin(angle_el);
  float Ubeta = Uq * cos(angle_el);

  // Clarke逆变换：alpha/beta 坐标转三相电压。
  // 加上 voltage_power_supply/2 是为了把正负交流量平移到PWM能输出的正电压范围。
  Ua = Ualpha + voltage_power_supply / 2.0f;
  Ub = (sqrt(3.0f) * Ubeta - Ualpha) / 2.0f + voltage_power_supply / 2.0f;
  Uc = (-Ualpha - sqrt(3.0f) * Ubeta) / 2.0f + voltage_power_supply / 2.0f;
}

// Apply an already calculated torque command without sampling the encoder again.
// Closed-loop functions sample first, calculate from that sample, then call this.
static void applyTorqueX(float Uq, float angle_el)
{
  float Ua = 0.0f;
  float Ub = 0.0f;
  float Uc = 0.0f;
  torqueToPhaseVoltage(Uq, angle_el, Ua, Ub, Uc);
  setPwmX(Ua, Ub, Uc);
}

static void applyTorqueY(float Uq, float angle_el)
{
  float Ua = 0.0f;
  float Ub = 0.0f;
  float Uc = 0.0f;
  torqueToPhaseVoltage(Uq, angle_el, Ua, Ub, Uc);
  setPwmY(Ua, Ub, Uc);
}

// 把任意角度归一化到 0 ~ 2PI。
float _normalizeAngle(float angle)
{
  float a = fmod(angle, 2.0f * PI);
  return a >= 0.0f ? a : (a + 2.0f * PI);
}

// 旧底层接口：保留给以前代码用，内部实际操作X轴。
void setPwm(float Ua, float Ub, float Uc)
{
  setPwmX(Ua, Ub, Uc);
}

// X轴三相PWM输出。
void setPwmX(float Ua, float Ub, float Uc)
{
  writePwmToPins(Ua, Ub, Uc, pwmAX, pwmBX, pwmCX);
}

// Y轴三相PWM输出。
void setPwmY(float Ua, float Ub, float Uc)
{
  writePwmToPins(Ua, Ub, Uc, pwmAY, pwmBY, pwmCY);
}

// 旧底层接口：保留给以前代码用，内部实际操作X轴。
void setTorque(float Uq, float angle_el)
{
  setTorqueX(Uq, angle_el);
}

// X轴FOC输出。先更新X编码器，再根据电角度输出三相PWM。
void setTorqueX(float Uq, float angle_el)
{
  sensorX.Sensor_update();
  applyTorqueX(Uq, angle_el);
}

// Y轴FOC输出。和X轴逻辑相同，但是使用Y轴编码器和Y轴PWM引脚。
void setTorqueY(float Uq, float angle_el)
{
  sensorY.Sensor_update();
  applyTorqueY(Uq, angle_el);
}

// 旧电角度接口：保留给以前代码用，默认返回X轴电角度。
float _electricalAngle()
{
  return _electricalAngleX();
}

// X轴电角度 = 机械角度 * 极对数 * 方向 - 校准零点。
float _electricalAngleX()
{
  return _normalizeAngle((float)(sensor_dir_X * pole_pairs_X) * sensorX.getMechanicalAngle() - zero_electric_angle_X);
}

// Y轴电角度 = 机械角度 * 极对数 * 方向 - 校准零点。
float _electricalAngleY()
{
  return _normalizeAngle((float)(sensor_dir_Y * pole_pairs_Y) * sensorY.getMechanicalAngle() - zero_electric_angle_Y);
}

// 初始化X轴：配置三相PWM，启动X轴AS5600，重置速度环输出限幅。
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

  vel_loop_X = PIDController(
    2,
    0,
    0,
    100000,
    voltage_power_supply / 2.0f,
    SPEED_D_FILTER_TF_X
  );
  sensorX.resetVelocity();
  vel_filter_X.reset();
  velocity_observer_X.reset();
}

// 初始化Y轴：配置三相PWM，启动第二路I2C上的Y轴AS5600。
void DFOC_Y_Vbus(float power_supply)
{
  voltage_power_supply = power_supply;

  pinMode(pwmAY, OUTPUT);
  pinMode(pwmBY, OUTPUT);
  pinMode(pwmCY, OUTPUT);
  ledcAttach(pwmAY, 30000, 8);
  ledcAttach(pwmBY, 30000, 8);
  ledcAttach(pwmCY, 30000, 8);

  // Y轴I2C经过导电滑环，降到100kHz以提高抗干扰能力。
  i2cY.begin(14, 15, 100000UL);
  sensorY.Sensor_init(&i2cY);

  vel_loop_Y = PIDController(
    2,
    0,
    0,
    100000,
    voltage_power_supply / 2.0f,
    SPEED_D_FILTER_TF_Y
  );
  sensorY.resetVelocity();
  vel_filter_Y.reset();
  velocity_observer_Y.reset();
}

// 旧初始化接口：保留给以前代码用，默认初始化X轴。
void DFOC_Vbus(float power_supply)
{
  DFOC_X_Vbus(power_supply);
}

// M1兼容接口：M1等价于Y轴。
void DFOC_M1_Vbus(float power_supply)
{
  DFOC_Y_Vbus(power_supply);
}

// X轴传感器校准。
// 校准时给电机一个固定方向的力矩，让转子吸到固定电角度，然后记录零点。
void DFOC_X_alignSensor(int _PP, int _DIR)
{
  pole_pairs_X = _PP;
  sensor_dir_X = _DIR;

  setTorqueX(3.0f, _3PI_2);
  delay(1000);
  sensorX.Sensor_update();
  zero_electric_angle_X = _electricalAngleX();
  setTorqueX(0.0f, _3PI_2);
  sensorX.resetVelocity();
  vel_filter_X.reset();
  velocity_observer_X.reset();
  angle_loop_X.reset();
  vel_loop_X.reset();
  control_state_X = {};
}

// Y轴传感器校准。逻辑和X轴完全一样，但使用Y轴硬件。
void DFOC_Y_alignSensor(int _PP, int _DIR)
{
  pole_pairs_Y = _PP;
  sensor_dir_Y = _DIR;

  setTorqueY(3.0f, _3PI_2);
  delay(1000);
  sensorY.Sensor_update();
  zero_electric_angle_Y = _electricalAngleY();
  setTorqueY(0.0f, _3PI_2);
  sensorY.resetVelocity();
  vel_filter_Y.reset();
  velocity_observer_Y.reset();
  angle_loop_Y.reset();
  vel_loop_Y.reset();
  control_state_Y = {};
}

// 旧校准接口：保留给以前代码用，默认校准X轴。
void DFOC_alignSensor(int _PP, int _DIR)
{
  DFOC_X_alignSensor(_PP, _DIR);
}

float DFOC_X_RawAngle()
{
  // X轴使用AS5600累计圈数，支持连续多圈旋转。
  return sensor_dir_X * sensorX.getAngle();
}

float DFOC_Y_RawAngle()
{
  return sensor_dir_Y * sensorY.getMechanicalAngle();
}

void DFOC_SET_MECHANICAL_ZERO(float zeroX, float zeroY)
{
  mechanical_zero_X = zeroX;
  mechanical_zero_Y = zeroY;
  mechanical_zero_enabled = true;
}

void DFOC_CLEAR_MECHANICAL_ZERO()
{
  mechanical_zero_X = 0.0f;
  mechanical_zero_Y = 0.0f;
  mechanical_zero_enabled = false;
}

// 获取X轴机械角度。启用机械零点后返回相对角度，否则保持旧行为。
float DFOC_X_Angle()
{
  if (mechanical_zero_enabled) {
    // X轴不能归一化到+/-PI，否则跨圈时位置会突然跳变2PI。
    return DFOC_X_RawAngle() - mechanical_zero_X;
  }

  return sensor_dir_X * sensorX.getAngle();
}

// 获取Y轴机械角度。启用机械零点后返回相对角度，否则保持旧行为。
float DFOC_Y_Angle()
{
  if (mechanical_zero_enabled) {
    return normalizeMechanicalAngle(
      DFOC_Y_RawAngle() - mechanical_zero_Y
    );
  }

  return sensor_dir_Y * sensorY.getAngle();
}

// M0兼容接口：M0等价于X轴。
float DFOC_M0_Angle()
{
  return DFOC_X_Angle();
}

// M1兼容接口：M1等价于Y轴。
float DFOC_M1_Angle()
{
  return DFOC_Y_Angle();
}

// 获取X轴速度，单位一般是 rad/s，并通过低通滤波减小噪声。
float DFOC_X_Velocity()
{
  float velocity_raw_X = sensorX.getVelocity();
  return vel_filter_X(sensor_dir_X * velocity_raw_X);
}

// 获取Y轴速度，单位一般是 rad/s，并通过低通滤波减小噪声。
float DFOC_Y_Velocity()
{
  float velocity_raw_Y = sensorY.getVelocity();
  return vel_filter_Y(sensor_dir_Y * velocity_raw_Y);
}

DFOCControlState DFOC_X_ControlState()
{
  return control_state_X;
}

DFOCControlState DFOC_Y_ControlState()
{
  return control_state_Y;
}

void DFOC_RESET_CONTROLLERS()
{
  sensorX.resetVelocity();
  sensorY.resetVelocity();
  vel_filter_X.reset();
  vel_filter_Y.reset();
  velocity_observer_X.reset();
  velocity_observer_Y.reset();
  angle_loop_X.reset();
  vel_loop_X.reset();
  angle_loop_Y.reset();
  vel_loop_Y.reset();
  control_state_X = {};
  control_state_Y = {};
}

float DFOC_M0_Velocity()
{
  return DFOC_X_Velocity();
}

float DFOC_M1_Velocity()
{
  return DFOC_Y_Velocity();
}

// 设置X轴速度环PID参数。
void DFOC_X_SET_VEL_PID(float P, float I, float D, float ramp)
{
  vel_loop_X.P = P;
  vel_loop_X.I = I;
  vel_loop_X.D = D;
  vel_loop_X.output_ramp = ramp;
  vel_loop_X.reset();
}

// 设置Y轴速度环PID参数。
void DFOC_Y_SET_VEL_PID(float P, float I, float D, float ramp)
{
  vel_loop_Y.P = P;
  vel_loop_Y.I = I;
  vel_loop_Y.D = D;
  vel_loop_Y.output_ramp = ramp;
  vel_loop_Y.reset();
}

// M0兼容接口：设置X轴速度环PID。
void DFOC_M0_SET_VEL_PID(float P, float I, float D, float ramp)
{
  DFOC_X_SET_VEL_PID(P, I, D, ramp);
}

// M1兼容接口：设置Y轴速度环PID。
void DFOC_M1_SET_VEL_PID(float P, float I, float D, float ramp)
{
  DFOC_Y_SET_VEL_PID(P, I, D, ramp);
}

// 设置X轴角度环PID参数。
void DFOC_X_SET_ANGLE_PID(float P, float I, float D, float ramp)
{
  angle_loop_X.P = P;
  angle_loop_X.I = I;
  angle_loop_X.D = D;
  angle_loop_X.output_ramp = ramp;
  angle_loop_X.reset();
}

// 设置Y轴角度环PID参数。
void DFOC_Y_SET_ANGLE_PID(float P, float I, float D, float ramp)
{
  angle_loop_Y.P = P;
  angle_loop_Y.I = I;
  angle_loop_Y.D = D;
  angle_loop_Y.output_ramp = ramp;
  angle_loop_Y.reset();
}

// M0兼容接口：设置X轴角度环PID。
void DFOC_M0_SET_ANGLE_PID(float P, float I, float D, float ramp)
{
  DFOC_X_SET_ANGLE_PID(P, I, D, ramp);
}

// M1兼容接口：设置Y轴角度环PID。
void DFOC_M1_SET_ANGLE_PID(float P, float I, float D, float ramp)
{
  DFOC_Y_SET_ANGLE_PID(P, I, D, ramp);
}

// 执行一次X轴速度环PID计算：速度误差 -> Uq电压。
float DFOC_X_VEL_PID(float error)
{
  return vel_loop_X(error);
}

// 执行一次Y轴速度环PID计算：速度误差 -> Uq电压。
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

// 执行一次X轴角度环PID计算：角度误差 -> 速度环输入。
float DFOC_X_ANGLE_PID(float error)
{
  return angle_loop_X(error);
}

// 执行一次Y轴角度环PID计算：角度误差 -> 速度环输入。
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

// 串口接收一个浮点数目标值。
// 当前格式：发送 "1.57\n" 这种单个数字，保存到 motor_target。
String serialReceiveUserCommand()
{
  return "";
}

bool serialReceiveUserCommandXY(Stream &port)
{
  static char buffer[64];
  static size_t length = 0;
  static bool overflow = false;
  bool valid_frame = false;

  while (port.available()) {
    char inChar = static_cast<char>(port.read());

    if (inChar == '\r') {
      continue;
    }

    if (inChar == '\n') {
      if (!overflow && length > 0) {
        buffer[length] = '\0';

        float target_x = 0.0f;
        float target_y = 0.0f;
        char extra = '\0';

        int count = sscanf(
          buffer,
          " %f , %f %c",
          &target_x,
          &target_y,
          &extra
        );

        if (
          count == 2 &&
          isfinite(target_x) &&
          isfinite(target_y)
        ) {
          motor_target_X = target_x;
          motor_target_Y = target_y;
          valid_frame = true;
        }
      }

      length = 0;
      overflow = false;
      continue;
    }

    if (overflow) {
      continue;
    }

    if (length < sizeof(buffer) - 1) {
      buffer[length++] = inChar;
    }
    else {
      length = 0;
      overflow = true;
    }
  }

  return valid_frame;
}

bool serialReceiveUserCommandXY()
{
  return serialReceiveUserCommandXY(Serial);
}

float serial_motor_target()
{
  return motor_target;
}

float serial_motor_target_X()
{
  return motor_target_X;
}

float serial_motor_target_Y()
{
  return motor_target_Y;
}

// 保留原来的X轴控制算法，便于随时切换回来对比。
void DFOC_X_set_Velocity_Angle(float Target)
{
  setTorqueX(DFOC_X_VEL_PID(DFOC_X_ANGLE_PID((Target - DFOC_X_Angle()) * 180.0f / PI)), _electricalAngleX());
}

// 保留原来的Y轴控制算法。
void DFOC_Y_set_Velocity_Angle(float Target)
{
  setTorqueY(DFOC_Y_VEL_PID(DFOC_Y_ANGLE_PID((Target - DFOC_Y_Angle()) * 180.0f / PI)), _electricalAngleY());
}

// 优化后的X轴位置+速度串级控制。
// 目标角度 -> 角度误差 -> 角度环 -> 目标速度
// 目标速度 - 实际速度 -> 速度环 -> Uq -> FOC输出。
void DFOC_X_set_Position_Velocity(
  float targetAngle,
  float targetVelocityFeedforward
)
{
  // One encoder sample drives angle, velocity, and electrical angle in this cycle.
  sensorX.Sensor_update();

  float angle = DFOC_X_Angle();
  float angle_error = targetAngle - angle;
  float position_correction_deg =
    DFOC_X_ANGLE_PID(angle_error * RAD_TO_DEG_F);
  float target_velocity =
    targetVelocityFeedforward +
    position_correction_deg * DEG_TO_RAD_F;
#if USE_ADVANCED_VELOCITY_OBSERVER
  float velocity = velocity_observer_X.update(angle);
#else
  float velocity = DFOC_X_Velocity();
#endif
  float velocity_error = target_velocity - velocity;
  float voltage =
    DFOC_X_VEL_PID(velocity_error * RAD_TO_DEG_F);

  control_state_X = {
    targetAngle,
    angle,
    angle_error,
    target_velocity,
    velocity,
    velocity_error,
    voltage
  };

  applyTorqueX(voltage, _electricalAngleX());
}

void DFOC_X_set_Optimized_Velocity_Angle(float Target)
{
  DFOC_X_set_Position_Velocity(Target, 0.0f);
}

// 优化后的Y轴位置+速度串级控制。
void DFOC_Y_set_Position_Velocity(
  float targetAngle,
  float targetVelocityFeedforward
)
{
  sensorY.Sensor_update();

  float angle = DFOC_Y_Angle();
  float angle_error = targetAngle - angle;
  float position_correction_deg =
    DFOC_Y_ANGLE_PID(angle_error * RAD_TO_DEG_F);
  float target_velocity =
    targetVelocityFeedforward +
    position_correction_deg * DEG_TO_RAD_F;
#if USE_ADVANCED_VELOCITY_OBSERVER
  float velocity = velocity_observer_Y.update(angle);
#else
  float velocity = DFOC_Y_Velocity();
#endif
  float velocity_error = target_velocity - velocity;
  float voltage =
    DFOC_Y_VEL_PID(velocity_error * RAD_TO_DEG_F);

  control_state_Y = {
    targetAngle,
    angle,
    angle_error,
    target_velocity,
    velocity,
    velocity_error,
    voltage
  };

  applyTorqueY(voltage, _electricalAngleY());
}

void DFOC_Y_set_Optimized_Velocity_Angle(float Target)
{
  DFOC_Y_set_Position_Velocity(Target, 0.0f);
}

// M0兼容接口：控制X轴位置。
void DFOC_M0_set_Velocity_Angle(float Target)
{
  DFOC_X_set_Velocity_Angle(Target);
}

// M1兼容接口：控制Y轴位置。
void DFOC_M1_set_Velocity_Angle(float Target)
{
  DFOC_Y_set_Velocity_Angle(Target);
}

void DFOC_M0_set_Optimized_Velocity_Angle(float Target)
{
  DFOC_X_set_Optimized_Velocity_Angle(Target);
}

void DFOC_M1_set_Optimized_Velocity_Angle(float Target)
{
  DFOC_Y_set_Optimized_Velocity_Angle(Target);
}

// X轴速度闭环控制，Target单位 rad/s。
void DFOC_X_setVelocity(float Target)
{
  sensorX.Sensor_update();

  float angle = DFOC_X_Angle();
  float velocity = DFOC_X_Velocity();
  float velocity_error = Target - velocity;
  float voltage =
    DFOC_X_VEL_PID(velocity_error * RAD_TO_DEG_F);

  control_state_X = {
    angle,
    angle,
    0.0f,
    Target,
    velocity,
    velocity_error,
    voltage
  };

  applyTorqueX(voltage, _electricalAngleX());
}

// Y轴速度闭环控制，Target单位 rad/s。
void DFOC_Y_setVelocity(float Target)
{
  sensorY.Sensor_update();

  float angle = DFOC_Y_Angle();
  float velocity = DFOC_Y_Velocity();
  float velocity_error = Target - velocity;
  float voltage =
    DFOC_Y_VEL_PID(velocity_error * RAD_TO_DEG_F);

  control_state_Y = {
    angle,
    angle,
    0.0f,
    Target,
    velocity,
    velocity_error,
    voltage
  };

  applyTorqueY(voltage, _electricalAngleY());
}

void DFOC_M0_setVelocity(float Target)
{
  DFOC_X_setVelocity(Target);
}

void DFOC_M1_setVelocity(float Target)
{
  DFOC_Y_setVelocity(Target);
}

// X轴位置力矩控制：角度误差直接经过角度环输出Uq。
// 它比串级控制更直接，但更容易抖，云台一般优先用串级控制。
void DFOC_X_set_Force_Angle(float Target)
{
  sensorX.Sensor_update();
  applyTorqueX(
    DFOC_X_ANGLE_PID(
      (Target - DFOC_X_Angle()) * RAD_TO_DEG_F
    ),
    _electricalAngleX()
  );
}

// Y轴位置力矩控制。
void DFOC_Y_set_Force_Angle(float Target)
{
  sensorY.Sensor_update();
  applyTorqueY(
    DFOC_Y_ANGLE_PID(
      (Target - DFOC_Y_Angle()) * RAD_TO_DEG_F
    ),
    _electricalAngleY()
  );
}

void DFOC_M0_set_Force_Angle(float Target)
{
  DFOC_X_set_Force_Angle(Target);
}

void DFOC_M1_set_Force_Angle(float Target)
{
  DFOC_Y_set_Force_Angle(Target);
}

// X轴直接Uq电压控制。适合测试电机是否能出力，不是位置闭环。
void DFOC_X_setTorque(float Target)
{
  setTorqueX(Target, _electricalAngleX());
}

// Y轴直接Uq电压控制。
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
