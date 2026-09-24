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
- `L`: bật/tắt chuỗi arm dance 28,6 giây, tự lặp: dance cũ (6 tư thế × 2 vòng,
  11,2 giây), Tarzan (6 nhịp khuỷu tay luân phiên, 9,2 giây), Victory (giơ chữ V
  và nhún hai tay đồng bộ, 8,2 giây). Mỗi bài hạ tay về standing trước bài tiếp theo.
- `R`: squat; nhấn lại để đứng lên.
- `G`: đứng dậy từ tư thế ngã sấp.
- `C`: reset về tư thế đứng.
- `Esc`: dừng khẩn cấp và ngắt quyền điều khiển.

Manual vẫn điều khiển được khi ESP32 hoặc cảm biến mất kết nối. ToF chỉ hiển
thị cảnh báo vật cản trong mode này, không ghi đè lệnh của người điều khiển.

Walking mặt phẳng dùng độ nâng mục tiêu 26 mm (`walk_step_height_mm`),
sải bước cấu hình 38 mm và hệ số lệnh tiến/lùi 0.50 (`walk_speed`);
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
  nhấc 38 mm (20 mm + khoảng hở 18 mm).

Auto-step đang khóa ở chế độ preview cho đến khi nhập kích thước bàn chân và
vị trí gắn ToF. Khi nhận diện đúng, dashboard hiển thị
`PREVIEW UP <distance> MM | LIFT 38 MM`.

### Person Follow

- `Y`: theo một người đã được phát hiện ổn định.
- `N`: dừng theo hoặc bỏ qua người hiện tại.
- Camera điều khiển hướng; ToF giữ khoảng cách và chặn tiến khi có vật cản.
- Đầu giữ cố định trong khi follow.
- Chỉ khóa target khi có đúng một người ổn định. Sau khi khóa, dùng track ID để
  tiếp tục theo người đó; mất target thì dừng, không tự chọn người khác.
- Không có ToF hợp lệ thì không tiến theo người. Đây là tránh vật cản cục bộ,
  không phải bản đồ đường đi hay bảo đảm giữ ID khi người che khuất nhau.

### Balance và an toàn

- Fall detection chạy toàn cục khi IMU hoạt động và ưu tiên hơn mọi mode.
- Khi phát hiện ngã, hai tay đưa nhanh ra trước; khi robot thẳng lại, tay trở về
  tư thế đứng.
- Balance/push recovery chỉ dùng bộ IMU balance sẵn có trong Terrain Auto khi
  standing, Auto stair đã tắt và không còn động tác đang chạy hoặc đang dừng.
  Không áp dụng trong Manual/Person Follow; không tạo bộ bù hay bước dậm riêng.
- FSR hiện chỉ trả lực hai chân qua telemetry, không khóa walking hoặc balance.
- Trong chuỗi đứng dậy chủ động, fall detection tạm nhường quyền. Chỉ trả tay
  về standing khi IMU xác nhận thẳng và ổn định; mất IMU sẽ giữ tay chống đỡ.

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
stack của hệ điều hành. Model Person Follow hiện dùng Caffe, cần OpenCV 4.x;
OpenCV 5 đã bỏ Caffe importer. Kiểm tra bằng `python -c "import cv2; print(cv2.__version__)"`.

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
- Camera-ToF có thuật toán nhận diện hình học cầu thang; mặc định chỉ DETECT/PREVIEW.
  Auto-step vẫn khóa đến khi hiệu chuẩn bàn chân/ToF và kiểm chứng trên robot thật.
- TinyML Gait Anomaly Monitor có luồng thu dữ liệu/train, nhưng chưa có baseline
  `deploy/models/gait_anomaly.json` trong Git. Chưa được xem là mô hình đã huấn luyện
  hoặc dự đoán tuổi thọ servo; monitor không điều khiển servo.
- One-foot balance, camera mimic, pickup và get-up-back không thuộc runtime hiện tại.

## Kiểm tra trước demo

- Rà soát phần mềm ngày 24/09/2026: dry-run các hướng đi và về standing, squat,
  ba đoạn arm dance lặp lại, get-up có/không có IMU, balance chỉ bù ankle/hip,
  fall override, parser Q/F/D, khóa target, ToF dừng tiến, chuyển card và timeout 0.6 s.
- Đã nạp model người/cầu thang và thử ảnh trống bằng OpenCV 4. Đây không phải
  phép đo độ chính xác nhận diện hoặc kiểm chứng robot leo cầu thang.
- Dashboard đã kiểm tra bằng Chrome ở chiều rộng 1366/390 px với backend giả:
  giữ/thả phím, squat, dance, reset, chuyển card và Esc; không gửi lệnh phần cứng.
- Giữ robot thẳng và đứng yên lúc khởi động; phải thấy `FALL READY` trước khi
  thử chống ngã. `FALL IMU WAIT/STALE` nghĩa là chưa có bảo vệ IMU sẵn sàng.
- Thử trên robot có người giữ: đi ngắn rồi thả phím, quay/ngang, squat, dance,
  sau đó balance. Đứng dậy cần xác nhận cơ khí và khả năng chịu tải riêng.
- Kiểm tra live camera và ToF trước Person Follow; quay một người trước, nhiều
  người/vật cản sau. Manual chỉ cảnh báo ToF, không tự dừng trước vật cản.
- Không dùng kết quả dry-run làm bằng chứng đứng dậy/leo thang thành công.

## An toàn

1. Raspberry Pi và servo dùng hai nguồn riêng.
2. Không cấp servo từ USB hoặc rail nguồn của Pi.
3. Treo hoặc giữ chắc robot khi kiểm tra gait mới.
4. Luôn sẵn sàng `Space`, `Esc` và công tắc cắt nguồn servo.
5. Compile/dry-run không thay thế kiểm tra robot thật.
