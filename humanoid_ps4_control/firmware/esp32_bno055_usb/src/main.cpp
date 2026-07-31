#include <Arduino.h>
#include <SparkFun_VL53L5CX_Library.h>
#include <Wire.h>

namespace {
constexpr uint8_t SDA_PIN = 21;
constexpr uint8_t SCL_PIN = 22;
constexpr uint8_t LEFT_FSR_PIN = 34;
constexpr uint8_t RIGHT_FSR_PIN = 35;
constexpr uint8_t BNO055_ADDRESS = 0x28;
constexpr uint8_t BNO055_IMUPLUS_MODE = 0x08;
constexpr uint8_t VL53L5CX_ADDRESS = 0x29;
constexpr uint32_t BNO055_I2C_CLOCK = 100000;
constexpr uint32_t VL53L5CX_I2C_CLOCK = 400000;
constexpr uint32_t SERIAL_BAUD = 115200;
constexpr uint32_t SAMPLE_PERIOD_MS = 20;  // 50 Hz
constexpr uint8_t FSR_ADC_SAMPLES = 8;

SparkFun_VL53L5CX tof;
VL53L5CX_ResultsData tof_data;
uint32_t last_sample_ms = 0;
bool bno_ready = false;
bool tof_ready = false;

int readAveragedAdc(uint8_t pin) {
  uint32_t total = 0;
  for (uint8_t i = 0; i < FSR_ADC_SAMPLES; ++i) {
    total += analogRead(pin);
  }
  return static_cast<int>(total / FSR_ADC_SAMPLES);
}

void printI2cDevices() {
  Serial.println("# I2C scan:");
  for (uint8_t address = 1; address < 127; ++address) {
    Wire.beginTransmission(address);
    if (Wire.endTransmission() == 0) {
      Serial.print("# found 0x");
      if (address < 16) {
        Serial.print('0');
      }
      Serial.println(address, HEX);
    }
  }
}

int readBnoRegister(uint8_t reg) {
  Wire.beginTransmission(BNO055_ADDRESS);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0 ||
      Wire.requestFrom(static_cast<uint8_t>(BNO055_ADDRESS), static_cast<uint8_t>(1)) != 1) {
    return -1;
  }
  return Wire.read();
}

bool writeBnoRegister(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(BNO055_ADDRESS);
  Wire.write(reg);
  Wire.write(value);
  return Wire.endTransmission() == 0;
}

bool readBnoBlock(uint8_t reg, uint8_t *data, uint8_t size) {
  Wire.beginTransmission(BNO055_ADDRESS);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0 ||
      Wire.requestFrom(static_cast<uint8_t>(BNO055_ADDRESS), size) != size) {
    return false;
  }
  for (uint8_t i = 0; i < size; ++i) {
    data[i] = Wire.read();
  }
  return true;
}

int16_t readInt16(const uint8_t *data) {
  return static_cast<int16_t>(static_cast<uint16_t>(data[0]) |
                              (static_cast<uint16_t>(data[1]) << 8));
}

bool configureBnoWithoutReset() {
  if (!writeBnoRegister(0x3D, 0x00)) {
    return false;
  }
  delay(25);
  if (!writeBnoRegister(0x3E, 0x00) || !writeBnoRegister(0x07, 0x00) ||
      !writeBnoRegister(0x3D, BNO055_IMUPLUS_MODE)) {
    return false;
  }
  delay(25);
  return true;
}

bool i2cDevicePresent(uint8_t address) {
  Wire.beginTransmission(address);
  return Wire.endTransmission() == 0;
}

void printDepthFrame(uint32_t now) {
  Serial.print("D,");
  Serial.print(now);
  for (uint8_t y = 0; y < 8; ++y) {
    for (int8_t x = 7; x >= 0; --x) {
      Serial.print(',');
      Serial.print(tof_data.distance_mm[x + y * 8]);
    }
  }
  Serial.println();
}
}  // namespace

void setup() {
  Serial.begin(SERIAL_BAUD);
  delay(800);

  analogReadResolution(12);
  analogSetPinAttenuation(LEFT_FSR_PIN, ADC_11db);
  analogSetPinAttenuation(RIGHT_FSR_PIN, ADC_11db);

  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(BNO055_I2C_CLOCK);
  printI2cDevices();
  const int bno_chip_id = readBnoRegister(0x00);
  Serial.printf("# BNO chip-id=0x%02X op-mode=0x%02X\n", bno_chip_id,
                readBnoRegister(0x3D));
  bno_ready = bno_chip_id == 0xA0 && configureBnoWithoutReset();
  if (!bno_ready) {
    Serial.println("# ERROR: BNO055 setup failed.");
  }

  if (i2cDevicePresent(VL53L5CX_ADDRESS)) {
    Wire.setClock(VL53L5CX_I2C_CLOCK);
    tof_ready = tof.begin(VL53L5CX_ADDRESS, Wire) && tof.setResolution(8 * 8) &&
                tof.setRangingFrequency(5) && tof.startRanging();
    Wire.setClock(BNO055_I2C_CLOCK);
  }

  delay(50);
  if (bno_ready) {
    Serial.println("# READY format=Q,ms,w,x,y,z,heading,roll,pitch,sys,gyro,accel,mag,gx,gy,gz");
  }
  Serial.println("# READY format=F,ms,left_norm,left_voltage,left_raw,right_norm,right_voltage,right_raw");
  Serial.println(tof_ready ? "# READY format=D,ms,64_distance_mm"
                           : "# VL53L5CX not detected at 0x29");
}

void loop() {
  const uint32_t now = millis();
  if (now - last_sample_ms < SAMPLE_PERIOD_MS) {
    delay(1);
    return;
  }
  last_sample_ms = now;

  uint8_t quat_data[8];
  uint8_t euler_data[6];
  uint8_t gravity_data[6];
  int calibration = 0;
  if (bno_ready && readBnoBlock(0x20, quat_data, sizeof(quat_data)) &&
      readBnoBlock(0x1A, euler_data, sizeof(euler_data)) &&
      readBnoBlock(0x2E, gravity_data, sizeof(gravity_data)) &&
      (calibration = readBnoRegister(0x35)) >= 0) {
    const float quat_w = readInt16(&quat_data[0]) / 16384.0f;
    const float quat_x = readInt16(&quat_data[2]) / 16384.0f;
    const float quat_y = readInt16(&quat_data[4]) / 16384.0f;
    const float quat_z = readInt16(&quat_data[6]) / 16384.0f;
    const float heading = readInt16(&euler_data[0]) / 16.0f;
    const float roll = readInt16(&euler_data[2]) / 16.0f;
    const float pitch = readInt16(&euler_data[4]) / 16.0f;
    const float gravity_x = readInt16(&gravity_data[0]) / 100.0f;
    const float gravity_y = readInt16(&gravity_data[2]) / 100.0f;
    const float gravity_z = readInt16(&gravity_data[4]) / 100.0f;

    Serial.printf("Q,%lu,%.6f,%.6f,%.6f,%.6f,%.2f,%.2f,%.2f,%d,%d,%d,%d,%.4f,%.4f,%.4f\n",
                  now, quat_w, quat_x, quat_y, quat_z, heading, roll, pitch,
                  (calibration >> 6) & 0x03, (calibration >> 4) & 0x03,
                  (calibration >> 2) & 0x03, calibration & 0x03, gravity_x,
                  gravity_y, gravity_z);
  }

  const int left_raw = readAveragedAdc(LEFT_FSR_PIN);
  const int right_raw = readAveragedAdc(RIGHT_FSR_PIN);
  const float left_norm = static_cast<float>(left_raw) / 4095.0f;
  const float right_norm = static_cast<float>(right_raw) / 4095.0f;

  Serial.print("F,");
  Serial.print(now);
  Serial.print(',');
  Serial.print(left_norm, 4);
  Serial.print(',');
  Serial.print(left_norm * 3.3f, 3);
  Serial.print(',');
  Serial.print(left_raw);
  Serial.print(',');
  Serial.print(right_norm, 4);
  Serial.print(',');
  Serial.print(right_norm * 3.3f, 3);
  Serial.print(',');
  Serial.println(right_raw);

  if (tof_ready) {
    Wire.setClock(VL53L5CX_I2C_CLOCK);
    const bool depth_ready = tof.isDataReady() && tof.getRangingData(&tof_data);
    Wire.setClock(BNO055_I2C_CLOCK);
    if (depth_ready) {
      printDepthFrame(now);
    }
  }
}
