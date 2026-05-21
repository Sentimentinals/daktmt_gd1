# 🤖 Humanoid PS4 Control — RTrobot XML Engine

Hệ thống điều khiển robot hình người 17 bậc tự do qua tay cầm PS4 (DualShock 4),
không cần ROS, chạy được trên Windows/Linux laptop và Raspberry Pi 5.

---

## Mục lục

1. [Giới thiệu dự án](#1-giới-thiệu-dự-án)
2. [Phân tích file XML](#2-phân-tích-file-xml-standingxml)
3. [Cài đặt trên Windows / Linux Laptop](#3-cài-đặt-trên-windows--linux-laptop)
4. [Cài đặt trên Raspberry Pi 5](#4-cài-đặt-trên-raspberry-pi-5)
5. [Cấu trúc dự án](#5-cấu-trúc-dự-án)
6. [Cách sử dụng CLI](#6-cách-sử-dụng-cli)
7. [Chế độ PS4](#7-chế-độ-ps4)
8. [Lưu ý an toàn](#8-lưu-ý-an-toàn)
9. [Vai trò của RTrobot Slider và code này](#9-vai-trò-của-rtrobot-slider-và-code-này)

---

## 1. Giới thiệu dự án

Dự án này cung cấp **runtime engine** để phát lại các action group đã ghi bằng phần mềm RTrobot Slider, đồng thời điều khiển robot theo thời gian thực bằng tay cầm PS4 (DualShock 4) hoặc bàn phím.

**Không yêu cầu:**
- ROS
- Driver Linux đặc biệt (không cần evdev)
- Cài đặt pyPS4Controller (tự viết code riêng)

**Yêu cầu:**
- Python 3.9+
- `pygame` (đọc joystick, đa nền tảng)
- `pyserial` (gửi lệnh serial đến bo mạch RTrobot)

---

## 2. Phân tích file XML `standing.xml`

File `actions/standing.xml` (định dạng RTrobot) chứa các nhóm action như sau:

### Group 0 — Tư thế đứng (Standing)

- **Số frame:** 1
- **Lệnh frame duy nhất:**
  ```
  #1P1551#2P2050#3P1660#4P900#5P1800#6P1572#7P500#8P1119
  #16P1700#25P2500#26P500#27P1550#28P1700#29P1428#30P1760#31P800#32P1100
  T1000D500
  ```
- **Giải thích:** `#<id>P<pulse>` = servo id và độ rộng xung (µs).  
  `T1000` = thời gian chuyển động 1000ms, `D500` = delay sau 500ms.

### Group 1 — Smooth_Walking (Đi bộ mượt mà)

- **Số frame:** 8
- **Tất cả servo trong group:** `[1, 2, 3, 4, 5, 6, 7, 8, 16, 25, 26, 27, 28, 29, 30, 31, 32]`
- **Servo thay đổi giữa các frame:** `[1, 2, 3, 4, 5, 28, 29, 30, 31, 32]`
- **Servo cố định (không đổi):** `[6, 7, 8, 16, 25, 26, 27]`

| Frame | Servo 1 | Servo 2 | Servo 3 | Servo 4 | Servo 5 | Servo 28 | Servo 29 | Servo 30 | Servo 31 | Servo 32 |
|-------|---------|---------|---------|---------|---------|----------|----------|----------|----------|----------|
| 0     | 1551    | 2050    | 1660    | 900     | 1700    | 1600     | 1428     | 1760     | 800      | 1100     |
| 1     | 1551    | 1950    | 1760    | 900     | 1650    | 1550     | 1428     | 1760     | 800      | 1100     |
| 2     | 1651    | 1850    | 1860    | 1000    | 1650    | 1550     | 1428     | 1760     | 800      | 1100     |
| 3     | 1751    | 2050    | 1660    | 900     | 1800    | 1700     | 1428     | 1760     | 800      | 1100     |
| 4     | 1551    | 2050    | 1660    | 900     | 1900    | 1800     | 1428     | 1760     | 800      | 1100     |
| 5     | 1551    | 2050    | 1660    | 900     | 1950    | 1850     | 1428     | 1660     | 900      | 1100     |
| 6     | 1551    | 2050    | 1660    | 900     | 1950    | 1850     | 1328     | 1560     | 1000     | 900      |
| 7     | 1551    | 2050    | 1660    | 900     | 1800    | 1700     | 1428     | 1760     | 800      | 1100     |

> **Nhận xét:** Frame 0 và Frame 7 giống hệt nhau — đây là chu kỳ đi bộ khép kín (closed-loop gait cycle).  
> Servo 1–5 (chân phải) và servo 28–32 (chân trái) là các servo dịch chuyển chính.

---

## 3. Cài đặt trên Windows / Linux Laptop

### Bước 1: Cài đặt Python 3.9+

- **Windows:** Tải từ [python.org](https://www.python.org/downloads/) → tick "Add to PATH"
- **Linux (Ubuntu/Debian):**
  ```bash
  sudo apt update && sudo apt install python3 python3-pip python3-venv
  ```

### Bước 2: Clone hoặc copy dự án

```bash
# Nếu dùng git
git clone <repo_url> humanoid_ps4_control
cd humanoid_ps4_control

# Hoặc giải nén file zip
unzip humanoid_ps4_control.zip
cd humanoid_ps4_control
```

### Bước 3: Tạo virtual environment

```bash
python -m venv venv

# Windows
venv\Scripts\activate

# Linux / macOS
source venv/bin/activate
```

### Bước 4: Cài đặt dependencies

```bash
pip install -r requirements.txt
```

### Bước 5: Test chạy mock (không cần robot)

```bash
# Test tư thế đứng (Group 0)
python -m src.main --backend mock --xml actions/standing.xml --group 0

# Test đi bộ (Group 1)
python -m src.main --backend mock --xml actions/standing.xml --group 1

# Ghi log CSV
python -m src.main --backend csv --xml actions/standing.xml --group 1 --csv out/test_walk.csv
```

### Bước 6: Kết nối robot qua serial (tùy chọn)

```bash
# Windows (kiểm tra Device Manager để biết cổng COM)
python -m src.main --backend serial --port COM3 --xml actions/standing.xml --group 0

# Linux
python -m src.main --backend serial --port /dev/ttyUSB0 --xml actions/standing.xml --group 0
```

### Bước 7: Chế độ PS4

Kết nối tay cầm PS4 qua USB hoặc Bluetooth, sau đó:

```bash
python -m src.main --ps4 --backend serial --port COM3 --xml actions/standing.xml
```

---

## 4. Cài đặt trên Raspberry Pi 5

### Bước 1: Cập nhật hệ thống

```bash
sudo apt update && sudo apt upgrade -y
```

### Bước 2: Cài đặt Python và thư viện hệ thống

```bash
sudo apt install -y python3 python3-pip python3-venv \
    libsdl2-dev libsdl2-mixer-2.0-0 libsdl2-image-2.0-0 \
    libsdl2-ttf-2.0-0 libportmidi-dev
```

### Bước 3: Copy dự án lên Raspberry Pi

```bash
# Từ laptop, dùng scp
scp -r humanoid_ps4_control pi@raspberrypi.local:~/

# Hoặc dùng USB drive
```

### Bước 4: Tạo virtual environment và cài dependencies

```bash
cd ~/humanoid_ps4_control
python3 -m venv venv
source venv/bin/activate
pip install -r requirements.txt
```

### Bước 5: Cấp quyền serial port

```bash
sudo usermod -a -G dialout $USER
# Đăng xuất và đăng nhập lại để áp dụng
```

### Bước 6: Kết nối tay cầm PS4 qua Bluetooth

```bash
# Cài bluetoothctl nếu chưa có
sudo apt install -y bluetooth bluez

# Pair tay cầm (giữ Share + PS button đến khi đèn nhấp nháy nhanh)
bluetoothctl
> scan on
> pair <MAC_ADDRESS>
> connect <MAC_ADDRESS>
> trust <MAC_ADDRESS>
> exit
```

### Bước 7: Chạy hệ thống

```bash
# Test mock
python3 -m src.main --backend mock --xml actions/standing.xml --group 0

# Chạy với robot thật và PS4
python3 -m src.main --ps4 --backend serial --port /dev/ttyUSB0 \
    --xml actions/standing.xml
```

### Bước 8: Tự động chạy khi khởi động (tùy chọn)

```bash
# Tạo systemd service
sudo nano /etc/systemd/system/robot.service
```

Nội dung file:
```ini
[Unit]
Description=Humanoid Robot PS4 Control
After=network.target

[Service]
User=pi
WorkingDirectory=/home/pi/humanoid_ps4_control
ExecStart=/home/pi/humanoid_ps4_control/venv/bin/python -m src.main \
    --ps4 --backend serial --port /dev/ttyUSB0 --xml actions/standing.xml
Restart=on-failure

[Install]
WantedBy=multi-user.target
```

```bash
sudo systemctl enable robot.service
sudo systemctl start robot.service
```

---

## 5. Cấu trúc dự án

```
humanoid_ps4_control/
├── README.md                   # Tài liệu này
├── requirements.txt            # pygame, pyserial
├── actions/
│   └── standing.xml            # File XML RTrobot (Group 0 Standing, Group 1 Walking)
├── out/                        # CSV log output (tạo tự động)
└── src/
    ├── __init__.py
    ├── rtrobot_xml.py           # Parser XML → ActionGroup / PoseFrame
    ├── motion.py                # Easing functions + playback generator
    ├── backends.py              # MockBackend, SerialRTBackend, CsvLogBackend
    ├── ps4_pygame.py            # PS4 / keyboard reader (cross-platform)
    └── main.py                  # CLI entry point
```

---

## 6. Cách sử dụng CLI

```
usage: python -m src.main [-h] [--xml XML] [--backend {mock,serial,csv}]
                           [--port PORT] [--baudrate BAUDRATE]
                           [--csv CSV] [--group GROUP] [--loop]
                           [--ps4] [--update-ms UPDATE_MS]

Tùy chọn:
  --xml          Đường dẫn file XML (mặc định: actions/standing.xml)
  --backend      mock | serial | csv
  --port         Cổng serial (mặc định: /dev/ttyUSB0, Windows: COM3)
  --baudrate     Baud rate (mặc định: 115200)
  --csv          Đường dẫn file CSV log (mặc định: out/log.csv)
  --group        Group ID để phát (mặc định: 0)
  --loop         Lặp vô hạn (không dùng --ps4)
  --ps4          Bật chế độ PS4 controller
  --update-ms    Khoảng thời gian cập nhật pose (ms, mặc định: 20)
```

### Ví dụ:

```bash
# Chạy Group 0 một lần, backend mock
python -m src.main --backend mock --xml actions/standing.xml --group 0

# Chạy Group 1 lặp liên tục
python -m src.main --backend mock --xml actions/standing.xml --group 1 --loop

# Ghi log CSV
python -m src.main --backend csv --xml actions/standing.xml --group 1 \
    --csv out/test_walk.csv

# Chạy với robot thật (Windows)
python -m src.main --backend serial --port COM3 --group 0 \
    --xml actions/standing.xml

# Chế độ PS4 với serial backend
python -m src.main --ps4 --backend serial --port /dev/ttyUSB0 \
    --xml actions/standing.xml
```

---

## 7. Chế độ PS4

### Mapping nút bấm DualShock 4

| Nút              | Hành động                        |
|------------------|----------------------------------|
| **Triangle (△)**  | Phát Group 0 — Standing (1 lần) |
| **D-pad Up (↑)** | Phát Group 1 — Smooth_Walking (lặp) |
| **Circle (○)**   | Dừng chuyển động                 |
| **Options**      | Emergency Stop                   |
| **Q** (keyboard) | Thoát chương trình               |

### Keyboard fallback (không có tay cầm)

| Phím           | Tương đương         |
|----------------|---------------------|
| `T`            | Triangle            |
| `U` / `↑`     | D-pad Up            |
| `C`            | Circle (Stop)       |
| `O` / `Escape` | Options (Emergency) |
| `Q`            | Quit                |

---

## 8. Lưu ý an toàn

> **⚠️ ĐỌC KỸ TRƯỚC KHI SỬ DỤNG VỚI ROBOT THẬT**

1. **Treo/giữ robot trước khi test:** Luôn đỡ robot bằng tay hoặc treo lên giá đỡ khi thử nghiệm động tác mới. Tránh để robot đứng tự do khi chưa kiểm tra xong.

2. **Bắt đầu bằng Group 0 (Standing):** Luôn phát Group 0 trước tiên để đặt robot về tư thế đứng và kiểm tra tất cả servo hoạt động đúng hướng.

3. **Emergency Stop luôn sẵn sàng:** Trong chế độ PS4, nút Circle (○) hoặc Options sẽ dừng ngay lập tức. Đặt tay gần nút này khi chạy thực.

4. **KHÔNG loop đi bộ ngay trên sàn:** Chỉ loop Group 1 khi robot đã được kiểm tra cẩn thận và biết chắc sẽ không ngã.

5. **Kiểm tra giới hạn servo:** Đảm bảo các giá trị pulse trong XML nằm trong dải cho phép của servo (thường 500–2500 µs). Sai số có thể gây hỏng cơ khí.

6. **Kiểm tra kết nối serial:** Sử dụng backend mock trước, sau đó mới kết nối serial với robot.

---

## 9. Vai trò của RTrobot Slider và code này

| RTrobot Slider (phần mềm gốc) | humanoid_ps4_control (dự án này) |
|-------------------------------|----------------------------------|
| Calibrate (hiệu chỉnh) servo | Runtime playback engine |
| Record keyframes (ghi tư thế) | Đọc và phát keyframes từ XML |
| Xuất file XML | Đọc file XML |
| Test thủ công từng servo | Điều khiển theo thời gian thực qua PS4 |
| Giao diện đồ họa | CLI + keyboard/PS4 input |

**Workflow khuyến nghị:**
1. Dùng RTrobot Slider để hiệu chỉnh và ghi các tư thế → xuất XML
2. Copy XML vào `actions/` trong dự án này
3. Test bằng `--backend mock` trên laptop
4. Kết nối Raspberry Pi 5 và tay cầm PS4 để điều khiển thực

---

*Phát triển cho robot hình người 17-DOF. Không yêu cầu ROS.*
