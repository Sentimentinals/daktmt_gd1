# Humanoid Robot Control

Runtime Python cho robot humanoid 17-DOF dung Raspberry Pi va keyboard,
32-channel PWM servo controller, walking engine, IMU balance va handshake bang FSR tay.

## Files chinh

```text
src/config.py          Cau hinh robot, servo, gait, balance, sensor
src/walking_engine.py  Loi gait: ZMP preview, IK, phase walking
src/main.py            Vong lap chay robot
src/balance.py         IMU balance correction
src/sensors.py         BNO055 + FSR tay reader qua ESP32 USB serial
POWER_SENSOR_SETUP.md  Huong dan wiring nguon va sensor an toan
```

Kenh servo tren board: ben trai `9..16` (khuyu tay den ban chan), ben phai
`17..24` (ban chan den khuyu tay), dau `25`. Bang remap nam trong
`SERVO_CHANNELS` cua `src/config.py`.

## Cai dat

Tren PC hoac Raspberry Pi:

```bash
python -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
```

Windows PowerShell:

```powershell
python -m venv .venv
.\.venv\Scripts\Activate.ps1
pip install -r requirements.txt
```

Khong commit `.venv` len Git. Moi may tu tao lai moi truong rieng.

## Kiem tra va chay

Compile check an toan, khong cham hardware:

```bash
python -m compileall -q src
```

Chay robot that:

```bash
python -m src.main
```

Phim dieu khien walking:

```text
W/S       Tien/lui
A/D       Xoay trai/phai
J/K       Di ngang trai/phai
V         Handshake
X         Test dung mot chan
L/M       Bat/tat arm dance
G/B       Dung day tu nam sap/nam ngua
C         Dung va ve STANDING
E/T       Reset walking engine
O/Escape  Ve menu
Q         Thoat che do hien tai
```

Mac dinh hien tai nam trong `src/config.py`:

```python
backend = "serial"
port = "/dev/ttyACM0"        # RTrobot servo controller
update_ms = 30
sensor_feedback = True
sensor_port = "/dev/ttyUSB0" # ESP32 sensor hub
imu_balance = True            # auto-reference cho BNO055 lap doc
```

## Workflow Git

Tren may code:

```bash
git add .
git commit -m "Update robot runtime"
git push origin main
```

Tren Raspberry Pi:

```bash
git pull origin main
pip install -r requirements.txt
python -m src.main
```

Nen sua code tren PC, Raspberry Pi chi `git pull` va chay robot.

## Safety checklist

1. Pi dung nguon rieng, servo dung nguon 6V rieng.
2. Khong cap Pi tu rail servo.
3. Sensor nen di qua ESP32/Pico sensor hub bang USB serial, khong cam truc tiep GPIO Pi.
4. Test compile va sensor truoc khi bat servo.
5. Treo/giu robot khi test gait moi.
6. Nut stop/emergency cut phai san sang.
