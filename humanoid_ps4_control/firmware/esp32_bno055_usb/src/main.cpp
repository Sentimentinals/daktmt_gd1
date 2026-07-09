#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

namespace {
constexpr uint8_t SDA_PIN = 21;
constexpr uint8_t SCL_PIN = 22;
constexpr uint8_t BNO055_ADDRESS = 0x28;
constexpr uint32_t SERIAL_BAUD = 115200;
constexpr uint32_t SAMPLE_PERIOD_MS = 20;  // 50 Hz

Adafruit_BNO055 bno(55, BNO055_ADDRESS, &Wire);
uint32_t last_sample_ms = 0;

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
}  // namespace

void setup() {
  Serial.begin(SERIAL_BAUD);
  delay(800);

  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(100000);

  if (!bno.begin()) {
    Serial.println("# ERROR: BNO055 not found. Expected I2C address 0x28.");
    printI2cDevices();
    while (true) {
      delay(1000);
    }
  }

  delay(1000);
  bno.setExtCrystalUse(true);
  Serial.println("# READY format=Q,ms,w,x,y,z,heading,roll,pitch,sys,gyro,accel,mag");
}

void loop() {
  const uint32_t now = millis();
  if (now - last_sample_ms < SAMPLE_PERIOD_MS) {
    delay(1);
    return;
  }
  last_sample_ms = now;

  const imu::Quaternion quat = bno.getQuat();
  const imu::Vector<3> euler =
      bno.getVector(Adafruit_BNO055::VECTOR_EULER);

  uint8_t system_cal = 0;
  uint8_t gyro_cal = 0;
  uint8_t accel_cal = 0;
  uint8_t mag_cal = 0;
  bno.getCalibration(&system_cal, &gyro_cal, &accel_cal, &mag_cal);

  Serial.print("Q,");
  Serial.print(now);
  Serial.print(',');
  Serial.print(quat.w(), 6);
  Serial.print(',');
  Serial.print(quat.x(), 6);
  Serial.print(',');
  Serial.print(quat.y(), 6);
  Serial.print(',');
  Serial.print(quat.z(), 6);
  Serial.print(',');
  Serial.print(euler.x(), 2);  // heading
  Serial.print(',');
  Serial.print(euler.z(), 2);  // roll
  Serial.print(',');
  Serial.print(euler.y(), 2);  // pitch
  Serial.print(',');
  Serial.print(system_cal);
  Serial.print(',');
  Serial.print(gyro_cal);
  Serial.print(',');
  Serial.print(accel_cal);
  Serial.print(',');
  Serial.println(mag_cal);
}
