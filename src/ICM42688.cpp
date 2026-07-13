#include "ICM42688.h"

namespace {

int16_t bytesToInt16(uint8_t highByte, uint8_t lowByte)
{
  return static_cast<int16_t>(
    (static_cast<uint16_t>(highByte) << 8) |
    static_cast<uint16_t>(lowByte)
  );
}

}  // namespace

ICM42688Sensor::ICM42688Sensor(
  SPIClass &spi,
  int8_t sclkPin,
  int8_t misoPin,
  int8_t mosiPin,
  int8_t csPin,
  uint32_t spiClockHz
)
  : spi_(spi),
    spiSettings_(spiClockHz, MSBFIRST, SPI_MODE3),
    sclkPin_(sclkPin),
    misoPin_(misoPin),
    mosiPin_(mosiPin),
    csPin_(csPin)
{
}

bool ICM42688Sensor::begin()
{
  // 先让CS保持高电平，防止上电和SPI初始化过程中误选中IMU。
  pinMode(csPin_, OUTPUT);
  digitalWrite(csPin_, HIGH);
  spi_.begin(sclkPin_, misoPin_, mosiPin_, csPin_);
  delay(2);

  writeRegister(REG_BANK_SEL, 0x00);
  writeRegister(REG_DEVICE_CONFIG, 0x01);  // 软件复位。
  delay(100);

  writeRegister(REG_BANK_SEL, 0x00);
  whoAmI_ = readRegister(REG_WHO_AM_I);
  if (whoAmI_ != EXPECTED_WHO_AM_I) {
    ready_ = false;
    return false;
  }

  // 量程位位于[7:5]，输出速率位位于[3:0]。
  writeRegister(
    REG_ACCEL_CONFIG0,
    static_cast<uint8_t>((ACCEL_FS_4G << 5) | ODR_500HZ)
  );
  writeRegister(
    REG_GYRO_CONFIG0,
    static_cast<uint8_t>((GYRO_FS_250DPS << 5) | ODR_500HZ)
  );

  // 陀螺仪和加速度计都进入低噪声模式。
  writeRegister(REG_PWR_MGMT0, 0x0F);
  delay(50);

  ready_ = true;
  return read();
}

bool ICM42688Sensor::read()
{
  int16_t accelRaw[3] = {0, 0, 0};
  int16_t gyroRaw[3] = {0, 0, 0};
  int16_t temperatureRaw = 0;

  if (!ready_ || !readRawFrame(accelRaw, gyroRaw, temperatureRaw)) {
    return false;
  }

  sample_.accelX = accelRaw[0] * ACCEL_SCALE_G;
  sample_.accelY = accelRaw[1] * ACCEL_SCALE_G;
  sample_.accelZ = accelRaw[2] * ACCEL_SCALE_G;

  sample_.gyroX = gyroRaw[0] * GYRO_SCALE_DPS - gyroBiasX_;
  sample_.gyroY = gyroRaw[1] * GYRO_SCALE_DPS - gyroBiasY_;
  sample_.gyroZ = gyroRaw[2] * GYRO_SCALE_DPS - gyroBiasZ_;
  sample_.temperature = temperatureRaw / 132.48f + 25.0f;
  return true;
}

bool ICM42688Sensor::calibrateGyro(
  uint16_t sampleCount,
  uint32_t sampleIntervalUs
)
{
  if (!ready_ || sampleCount == 0) {
    return false;
  }

  double sumX = 0.0;
  double sumY = 0.0;
  double sumZ = 0.0;
  uint16_t validSamples = 0;

  for (uint16_t i = 0; i < sampleCount; ++i) {
    int16_t accelRaw[3] = {0, 0, 0};
    int16_t gyroRaw[3] = {0, 0, 0};
    int16_t temperatureRaw = 0;

    if (readRawFrame(accelRaw, gyroRaw, temperatureRaw)) {
      sumX += gyroRaw[0] * GYRO_SCALE_DPS;
      sumY += gyroRaw[1] * GYRO_SCALE_DPS;
      sumZ += gyroRaw[2] * GYRO_SCALE_DPS;
      ++validSamples;
    }

    delayMicroseconds(sampleIntervalUs);
  }

  const uint16_t minimumValidSamples = static_cast<uint16_t>(
    (static_cast<uint32_t>(sampleCount) + 1U) / 2U
  );
  if (validSamples < minimumValidSamples) {
    return false;
  }

  gyroBiasX_ = static_cast<float>(sumX / validSamples);
  gyroBiasY_ = static_cast<float>(sumY / validSamples);
  gyroBiasZ_ = static_cast<float>(sumZ / validSamples);
  return read();
}

uint8_t ICM42688Sensor::readRegister(uint8_t reg)
{
  spi_.beginTransaction(spiSettings_);
  digitalWrite(csPin_, LOW);
  spi_.transfer(static_cast<uint8_t>(reg | 0x80));
  const uint8_t value = spi_.transfer(0x00);
  digitalWrite(csPin_, HIGH);
  spi_.endTransaction();
  return value;
}

void ICM42688Sensor::readRegisters(
  uint8_t reg,
  uint8_t *data,
  size_t length
)
{
  spi_.beginTransaction(spiSettings_);
  digitalWrite(csPin_, LOW);
  spi_.transfer(static_cast<uint8_t>(reg | 0x80));

  for (size_t i = 0; i < length; ++i) {
    data[i] = spi_.transfer(0x00);
  }

  digitalWrite(csPin_, HIGH);
  spi_.endTransaction();
}

void ICM42688Sensor::writeRegister(uint8_t reg, uint8_t value)
{
  spi_.beginTransaction(spiSettings_);
  digitalWrite(csPin_, LOW);
  spi_.transfer(static_cast<uint8_t>(reg & 0x7F));
  spi_.transfer(value);
  digitalWrite(csPin_, HIGH);
  spi_.endTransaction();
}

bool ICM42688Sensor::readRawFrame(
  int16_t accel[3],
  int16_t gyro[3],
  int16_t &temperature
)
{
  // 从TEMP_DATA1开始连续读取，顺序为温度、三轴加速度、三轴陀螺仪。
  uint8_t buffer[14] = {0};
  readRegisters(REG_TEMP_DATA1, buffer, sizeof(buffer));

  bool allZero = true;
  bool allOnes = true;
  for (uint8_t value : buffer) {
    allZero = allZero && value == 0x00;
    allOnes = allOnes && value == 0xFF;
  }
  if (allZero || allOnes) {
    return false;
  }

  temperature = bytesToInt16(buffer[0], buffer[1]);
  accel[0] = bytesToInt16(buffer[2], buffer[3]);
  accel[1] = bytesToInt16(buffer[4], buffer[5]);
  accel[2] = bytesToInt16(buffer[6], buffer[7]);
  gyro[0] = bytesToInt16(buffer[8], buffer[9]);
  gyro[1] = bytesToInt16(buffer[10], buffer[11]);
  gyro[2] = bytesToInt16(buffer[12], buffer[13]);
  return true;
}
