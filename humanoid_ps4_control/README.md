# Humanoid Robot Control

Runtime cho robot humanoid 17-DOF dùng Raspberry Pi, mạch RTrobot 32 kênh,
ESP32 sensor hub và dashboard web. Raspberry Pi không dùng GPIO trực tiếp cho
cảm biến.

## Chức năng

### Manual Control

- `↑` / `↓`: đi tiến/lùi.
- `←` / `→`: quay trái/phải; đầu quay dẫn hướng.
- `J` / `K`: đi ngang trái/phải.
- `Space`: dừng và giữ tư thế đứng.
- `L`: bật/tắt arm dance.
- `G`: đứng dậy từ tư thế ngã sấp.
- `C`: reset về tư thế đứng.
- `Esc`: dừng khẩn cấp và ngắt quyền điều khiển.

Manual vẫn điều khiển được khi ESP32 hoặc cảm biến mất kết nối. ToF chỉ hiển
thị cảnh báo vật cản trong mode này, không ghi đè lệnh của người điều khiển.

Walking mặt phẳng dùng lại gait trước bản bước lết, với độ nâng mục tiêu giảm
từ 60 xuống 20 mm (`walk_step_height_mm`). Giữ sải bước và chuyển trọng tâm cũ;
chân trụ ở mặt sàn, chân bước vẫn có nâng/hạ, không khóa cả hai chân tại Z = 0.
Thả phím sẽ hoàn tất bước rồi về standing. Manual không có IMU/push recovery
ghi đè; fall detection vẫn được ưu tiên. Các giá trị là quỹ đạo tính toán, cần
thử có người giữ robot để xác nhận tiếp xúc sàn và tải servo thực tế.

### Terrain Auto

- `V`: bật/tắt cân bằng bằng BNO055.
- `U`: bật/tắt nhận diện và bước cầu thang.
- Camera dùng model ONNX và đường biên ngang để tìm cầu thang.
- VL53L5CX xác nhận hướng lên/xuống và khoảng cách đến mép bậc.
- Độ nhấc chân bằng chiều cao bậc cộng khoảng hở. Bậc 20 mm hiện dùng mục tiêu
  nhấc 32 mm.

Auto-step đang khóa ở chế độ preview cho đến khi nhập kích thước bàn chân và
vị trí gắn ToF. Khi nhận diện đúng, dashboard hiển thị
`PREVIEW UP <distance> MM | LIFT 32 MM`.

### Person Follow

- `Y`: theo một người đã được phát hiện ổn định.
- `N`: dừng theo hoặc bỏ qua người hiện tại.
- Camera điều khiển hướng; ToF giữ khoảng cách và chặn tiến khi có vật cản.
- Đầu giữ cố định trong khi follow.

### Balance và an toàn

- Fall detection chạy toàn cục khi IMU hoạt động và ưu tiên hơn mọi mode.
- Khi phát hiện ngã, hai tay đưa nhanh ra trước; khi robot thẳng lại, tay trở về
  tư thế đứng.
- Balance/push recovery chỉ dùng bộ IMU balance sẵn có trong Terrain Auto khi
  standing, Auto stair đã tắt và không còn động tác đang chạy hoặc đang dừng.
  Không áp dụng trong Manual/Person Follow; không tạo bộ bù hay bước dậm riêng.
- FSR hiện chỉ trả lực hai chân lên dashboard, không khóa walking hoặc balance.

## Phần cứng

```text
Raspberry Pi USB -> RTrobot servo controller
Raspberry Pi USB -> ESP32 -> BNO055 + 2 FSR + VL53L5CX
Nguồn servo riêng -> RTrobot V+ / servo rail
```

Xem [POWER_SENSOR_SETUP.md](POWER_SENSOR_SETUP.md) trước khi cấp nguồn.

## Cài đặt

Trên Raspberry Pi:

```bash
cd ~/daktmt_gd1/humanoid_ps4_control
sudo apt install python3-venv python3-picamera2 python3-opencv
python3 -m venv --system-site-packages .venv
source .venv/bin/activate
python -m pip install -r requirements.txt
```

OpenCV/Picamera2 nên cài từ repository của Raspberry Pi OS để dùng đúng camera
stack của hệ điều hành.

## Chạy

```bash
cd ~/daktmt_gd1/humanoid_ps4_control
source .venv/bin/activate
python -m src.main
```

Mở dashboard trên laptop cùng mạng:

```text
http://<IP-của-Pi>:8765
```

Kiểm tra cảm biến mà không chạy servo:

```bash
python -m tools.sensor_monitor --port auto --seconds 20
```

Không chạy `src.main` và `sensor_monitor` cùng lúc vì chỉ một tiến trình được
giữ cổng serial ESP32.

## Trạng thái hiện tại

- Manual, dashboard và keyboard đã được tích hợp vào `src.main`.
- Person Follow đã có state machine nhưng cần kiểm tra thực tế với
  camera/ToF đúng vị trí.
- Terrain balance cần IMU hợp lệ và mốc đứng yên.
- Camera-ToF đã nhận diện hình học cầu thang; chuyển động auto-step chỉ được mở
  sau calibration hình học bàn chân và ToF.
- One-foot balance, camera mimic và get-up-back không thuộc runtime hiện tại.

## An toàn

1. Raspberry Pi và servo dùng hai nguồn riêng.
2. Không cấp servo từ USB hoặc rail nguồn của Pi.
3. Treo hoặc giữ chắc robot khi kiểm tra gait mới.
4. Luôn sẵn sàng `Space`, `Esc` và công tắc cắt nguồn servo.
5. Compile/dry-run không thay thế kiểm tra robot thật.
