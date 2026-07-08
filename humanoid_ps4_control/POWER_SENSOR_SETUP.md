# Power And Sensor Setup

Muc tieu: bao ve Raspberry Pi, tach dong servo khoi Pi, va khong cam sensor
truc tiep vao GPIO cua Pi khi test robot that.

## Kien truc khuyen nghi

```text
Raspberry Pi nguon rieng
  USB 1 -> 32-channel servo controller logic/control
  USB 2 -> ESP32/Pico sensor hub -> BNO055 + FSR

LiPo 3S
  -> relay / emergency cut
  -> buck 6.0V dong lon
      -> servo controller V+ / servo power
      -> servos
```

Khong cap Raspberry Pi tu rail servo 6V. Khong cap servo tu USB cua Pi.

## ESP32 DevKit sensor hub

BNO055:

```text
BNO055 VIN/VCC -> ESP32 3V3
BNO055 GND     -> ESP32 GND
BNO055 SDA     -> ESP32 GPIO21
BNO055 SCL     -> ESP32 GPIO22
```

FSR trai:

```text
ESP32 3V3 -> FSR -> GPIO34/ADC -> 10k resistor -> GND
```

FSR phai:

```text
ESP32 3V3 -> FSR -> GPIO35/ADC -> 10k resistor -> GND
```

Dien tro `10k 1/4W` la du. Tin hieu vao ADC/GPIO cua ESP32 chi duoc toi da
3.3V.

## Servo controller

```text
Pi USB -> servo controller USB/control
Buck 6V -> servo controller V+ / servo power
Buck GND -> servo controller GND
```

Neu board servo controller co jumper USB/EXT power, dam bao servo power dung
nguon ngoai 6V. USB cua Pi chi dung cho logic/control.

## Giam rui ro hu Pi

1. Sensor chi noi vao ESP32/Pico, khong noi vao GPIO Pi.
2. Servo dung nguon rieng, day nguon ngan va to.
3. Them tu `4700uF` den `10000uF` tren rail servo 6V gan servo controller.
4. Dung star-ground tai mot diem chung, khong cho dong servo chay qua GND Pi.
5. Neu can an toan hon, gan USB isolator giua Pi va ESP32/Pico.

## Thu tu test

1. Chua bat nguon servo.
2. Cam ESP32/Pico vao laptop truoc, test BNO055/FSR.
3. Do dien ap: sensor chi co 3.3V, khong co 5V/6V tren signal.
4. Cam ESP32/Pico vao Pi qua USB.
5. Kiem tra serial port tren Pi:

```bash
ls /dev/ttyUSB* /dev/ttyACM*
```

6. Doc sensor on dinh roi moi bat nguon servo rieng.
