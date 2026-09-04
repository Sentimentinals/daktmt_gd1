# Chức năng hiện tại của robot humanoid 17-DOF

Robot chạy trên Raspberry Pi, điều khiển servo qua mạch RTrobot 32 kênh và nhận
dữ liệu cảm biến từ ESP32 qua USB. Giao diện chính là dashboard web tại
`http://<IP-của-Pi>:8765`.

## 1. Manual Control

- Đi tiến/lùi bằng `↑` / `↓`.
- Quay trái/phải bằng `←` / `→`; đầu quay dẫn hướng trước thân.
- Đi ngang trái/phải bằng `J` / `K`.
- Dừng và giữ tư thế đứng bằng `Space`.
- Arm dance bằng `L`.
- Đứng dậy từ tư thế ngã sấp bằng `G`.
- Trả robot về tư thế đứng bằng `C`.
- Dừng khẩn cấp và ngắt quyền điều khiển bằng `Esc`.
- Camera, trạng thái gait, góc servo và dữ liệu cảm biến được hiển thị trực tiếp
  trên dashboard.

Manual vẫn hoạt động khi ESP32 hoặc cảm biến mất kết nối. Trong mode này ToF chỉ
cảnh báo vật cản gần, không tự ghi đè lệnh di chuyển của người điều khiển.

## 2. Terrain Auto

- Bật/tắt cân bằng IMU bằng `V`.
- Bật/tắt nhận diện và bước cầu thang bằng `U`.
- BNO055 cung cấp roll/pitch để giữ thăng bằng trên mặt phẳng nghiêng.
- Camera nhận diện cầu thang bằng model ONNX; nếu model bỏ sót, hệ thống dùng
  đường biên ngang làm phương án dự phòng.
- VL53L5CX xác nhận hướng lên/xuống và khoảng cách đến mép bậc.
- Độ nhấc chân được tính bằng `chiều cao bậc + khoảng hở bàn chân`. Với bậc
  20 mm hiện tại, độ nhấc mục tiêu là 32 mm.
- Robot có thể tự căn giữa, tiến gần mép và tạo quỹ đạo bước từng chân khi toàn
  bộ điều kiện an toàn hợp lệ.

Hiện auto-step đang khóa ở chế độ preview vì chưa nhập số đo bàn chân và vị trí
gắn ToF. Dashboard vẫn phải hiển thị được `PREVIEW UP ... | LIFT 32 MM`; chỉ bật
di chuyển thật sau khi hoàn tất calibration hình học.

## 3. Person Follow

- Camera chỉ nhận diện người trong mode này.
- `Y` bắt đầu theo một người đã được xác nhận ổn định.
- `N` dừng theo hoặc bỏ qua người đang được phát hiện.
- Camera điều khiển hướng quay; ToF giữ khoảng cách và chặn tiến khi có vật cản.
- Đầu được giữ cố định trong lúc theo người.

## 4. Pick Up

- Model hiện nhận diện ba nhóm: lon nước, bóng và Rubik.
- `R` bắt đầu chu trình sau khi một vật thể được nhận diện ổn định.
- Robot lần lượt căn hướng, tiến đến khoảng cách phù hợp, squat theo vị trí vật,
  đưa vai/cánh tay đến mục tiêu, duỗi khuỷu tay, nâng thân và giữ vật.
- Camera xác định loại, màu và vị trí vật; ToF bổ sung khoảng cách.
- Nhận diện vật thể chỉ chạy trong mode Pick Up.

## 5. Balance và an toàn

- Fall detection chạy toàn cục khi IMU hoạt động và có quyền ưu tiên cao hơn
  mode hiện tại.
- Khi phát hiện ngã, hai tay đưa nhanh ra trước để bảo vệ; khi robot thẳng lại,
  tay trở về tư thế đứng.
- Push recovery dùng IMU để bù ankle/hip khi đứng và có thể tạo bước dậm ngắn
  khi độ nghiêng vượt ngưỡng.
- Lệnh đứng dậy phía trước dùng chân, bàn chân và tay để tạo lực, sau đó trả toàn
  bộ servo về tư thế đứng.

## 6. Cảm biến đang hỗ trợ

- BNO055: roll, pitch, yaw và trạng thái calibration cho balance/fall detection.
- Hai FSR dưới chân: trả lực chân trái/phải lên dashboard; hiện không phải điều
  kiện bắt buộc để walking hoặc balance hoạt động.
- VL53L5CX 8x8: khoảng cách đa vùng cho person follow, pickup, cảnh báo vật cản
  và xác nhận hình học cầu thang.
- ESP32 truyền các gói IMU, FSR và ToF về Pi qua USB serial.

## 7. Trạng thái xác nhận

- Manual walking, dashboard và điều khiển keyboard đã được tích hợp trong main.
- Person Follow và Pick Up đã có đầy đủ state machine nhưng vẫn cần kiểm tra trên
  robot thật với camera/ToF đúng vị trí.
- Terrain balance cần IMU hợp lệ và đã lấy mốc đứng yên.
- Nhận diện camera-ToF cho cầu thang đã có; auto-step chưa được phép chạy thật
  cho đến khi calibration kích thước chân và vị trí ToF hoàn tất.
- One-foot balance, camera mimic và get-up-back không còn thuộc runtime hiện tại.

Khi thử gait, cầu thang hoặc đứng dậy, phải có người giữ robot và nút dừng khẩn
cấp sẵn sàng. Không cấp nguồn servo từ Raspberry Pi.
