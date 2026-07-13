#pragma once

#include <Arduino.h>
#include <SPI.h>

// ICM-42688一次采样得到的物理量。
// 加速度单位为g，角速度单位为deg/s，温度单位为摄氏度。
struct ICM42688Sample {
  float accelX = 0.0f;
  float accelY = 0.0f;
  float accelZ = 0.0f;
  float gyroX = 0.0f;
  float gyroY = 0.0f;
  float gyroZ = 0.0f;
  float temperature = 0.0f;
};

// 从STM32 HAL版本移植的ICM-42688 SPI驱动。
// 当前固定配置为：加速度±4g、陀螺仪±250deg/s、输出速率500Hz。
class ICM42688Sensor {
 public:
  static constexpr uint8_t EXPECTED_WHO_AM_I = 0x47;

  ICM42688Sensor(
    SPIClass &spi,
    int8_t sclkPin,
    int8_t misoPin,
    int8_t mosiPin,
    int8_t csPin,
    uint32_t spiClockHz = 1000000UL
  );

  // 初始化SPI和ICM-42688。返回true表示WHO_AM_I及寄存器配置正常。
  bool begin();

  // 读取一次温度、加速度和角速度，并更新sample()返回的数据。
  bool read();

  // 云台保持静止时采样陀螺仪零偏，后续read()会自动减去该零偏。
  bool calibrateGyro(uint16_t sampleCount = 500, uint32_t sampleIntervalUs = 2000);

  const ICM42688Sample &sample() const { return sample_; }
  bool isReady() const { return ready_; }
  uint8_t whoAmI() const { return whoAmI_; }

 private:
  static constexpr uint8_t REG_DEVICE_CONFIG = 0x11;
  static constexpr uint8_t REG_TEMP_DATA1 = 0x1D;
  static constexpr uint8_t REG_PWR_MGMT0 = 0x4E;
  static constexpr uint8_t REG_GYRO_CONFIG0 = 0x4F;
  static constexpr uint8_t REG_ACCEL_CONFIG0 = 0x50;
  static constexpr uint8_t REG_WHO_AM_I = 0x75;
  static constexpr uint8_t REG_BANK_SEL = 0x76;

  static constexpr uint8_t ACCEL_FS_4G = 0x02;
  static constexpr uint8_t GYRO_FS_250DPS = 0x03;
  static constexpr uint8_t ODR_500HZ = 0x0F;

  static constexpr float ACCEL_SCALE_G = 4.0f / 32768.0f;
  static constexpr float GYRO_SCALE_DPS = 250.0f / 32768.0f;

  uint8_t readRegister(uint8_t reg);
  void readRegisters(uint8_t reg, uint8_t *data, size_t length);
  void writeRegister(uint8_t reg, uint8_t value);
  bool readRawFrame(int16_t accel[3], int16_t gyro[3], int16_t &temperature);

  SPIClass &spi_;
  SPISettings spiSettings_;
  int8_t sclkPin_;
  int8_t misoPin_;
  int8_t mosiPin_;
  int8_t csPin_;

  bool ready_ = false;
  uint8_t whoAmI_ = 0;
  float gyroBiasX_ = 0.0f;
  float gyroBiasY_ = 0.0f;
  float gyroBiasZ_ = 0.0f;
  ICM42688Sample sample_;
};
