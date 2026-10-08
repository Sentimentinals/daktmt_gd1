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

Walking mặt phẳng dùng độ nâng mục tiêu 58,24 mm (`walk_step_height_mm`),
sải bước 24 mm (`walk_step_length_mm`), không nhân thêm hệ số `walk_speed`;
chân trụ ở mặt sàn, chân bước vẫn có nâng/hạ, không khóa cả hai chân tại Z = 0.
Thả phím sẽ hoàn tất bước rồi về standing. Manual không có IMU/push recovery
ghi đè; fall detection vẫn được ưu tiên. Các giá trị là quỹ đạo tính toán, cần
thử có người giữ robot để xác nhận tiếp xúc sàn và tải servo thực tế.

Xem [thông số walking, quay và đi ngang](#2-walking-quay-và-đi-ngang), gồm
cấu hình, hiệu chuẩn servo và các hệ số bên trong thuật toán.

Follow và tiếp cận cầu thang dùng `auto_step_time_s=1.26`,
`auto_settle_time_s=1.47`, cùng biên độ riêng `person_follow_*_length_mm` và
`stair_*_step_mm`; tăng Manual không làm tăng tốc các mode tự động.
Đã bỏ `GAIT`, `walk_speed/turn_speed/side_speed`, `manual_walk_tempo`,
`t_step/t_dbl` và tham số nhả ankle bị trùng mốc kết thúc lift. Với chu kỳ hiện tại
30 ms, cấu hình mới giữ nguyên PWM ở các ca dry-run so với trước khi tinh giản.
Các thời gian được làm tròn theo chu kỳ cập nhật; không có tham số tăng mô-men
servo bằng phần mềm.

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
- ToF đọc nền liên tục (firmware 5 Hz), dùng ô gần nhất trong vùng trước mặt
  để dừng tiến tại 100 mm; không suy ra mét từ chiều cao khung người trong ảnh.
- Chỉ một ngưỡng dừng `tof_obstacle_stop_mm` cho cả người và vật cản.
  `person_follow_crawl_band_mm` là vùng chạy chậm phía ngoài ngưỡng này,
  không phải một khoảng dừng bổ sung.
- Camera ở đầu, ToF ở ngực thấp hơn 130 mm, cùng nhìn thẳng. Phần đối chiếu
  chiếu vùng ToF lên ảnh, cần 3 cặp ảnh/ToF mới trước khi xác nhận. Vùng trùng
  bất kỳ khung người nào được xử lý là người; vùng hoàn toàn nằm trong ảnh và
  ngoài các khung người là vật có thể né. Đây là suy đoán hình học, không phải
  model nhận diện mọi vật thể. Vật che người hoặc detector bỏ sót vẫn có thể gây nhầm.
- Tới 100 mm: `WAIT PERSON` giữ target và chờ, `AVOID LEFT/RIGHT` chỉ né khi
  có bằng chứng vật và bên thoáng ít nhất 200 mm; `WAIT UNKNOWN` đứng chờ.
  Lịch sử đối chiếu chỉ giữ tối đa 2 giây khi vùng đo/khoảng cách liên tục;
  mất ảnh/ToF, đổi ID hoặc số đo nhảy sẽ hủy bằng chứng. Người đi xa thì tiếp tục
  cùng ID; mất người thì `WAIT CAMERA/TARGET`, không tự khóa người khác.
- Khi đang né, đường trước phải thoáng ít nhất 400 mm qua 3 mẫu mới để thoát né.
  Hai bên bị chắn thì chờ. 100 mm tính từ ToF, không phải mép robot; walking
  còn hoàn tất bước đang thực hiện nên chưa bảo đảm khoảng dừng thực tế 10 cm.
- `person_camera_*`, `person_tof_fov_deg`, `person_tof_flip_vertical` trong
  `src/config.py` chỉ áp dụng cho follow, không dùng góc nghiêng ToF của stair.
  Giả định hai trục song song, thẳng hàng ngang, không lệch trước/sau; chưa có
  hiệu chuẩn lens/crop. FOV danh định Pi Camera V1 là 53.5 x 41.41 độ theo
  [Raspberry Pi](https://www.raspberrypi.com/documentation/accessories/camera.html),
  ToF dùng 45 x 45 độ theo [ST](https://www.st.com/resource/en/datasheet/vl53l5cx.pdf).
  Cần kiểm tra hướng hàng ToF và vùng ảnh thực tế trước khi thử né gần người.

### Balance và an toàn

- Fall detection chạy toàn cục khi IMU hoạt động và ưu tiên hơn mọi mode.
- Khi phát hiện ngã, hai tay đưa nhanh ra trước; khi robot thẳng lại, tay trở về
  tư thế đứng.
- Balance/push recovery chỉ dùng bộ IMU balance sẵn có trong Terrain Auto khi
  standing, Auto stair đã tắt và không còn động tác đang chạy hoặc đang dừng.
  Không áp dụng trong Manual/Person Follow; không tạo bộ bù hay bước dậm riêng.
- FSR hiện chỉ trả lực hai chân qua telemetry, không khóa walking hoặc balance.
- Bật/tắt từng cảm biến bằng `sensor_use_imu`, `sensor_use_foot_fsr`,
  `sensor_use_depth`; không còn cờ tổng `sensor_feedback`. Fall safety bật thì
  vẫn yêu cầu IMU. Balance chỉ bù lúc đứng hai chân, không còn gain chân swing
  hay phần tích phân không sử dụng.
- Trong chuỗi đứng dậy chủ động, fall detection tạm nhường quyền. Chỉ trả tay
  về standing khi IMU xác nhận thẳng và ổn định; ở cuối chuỗi, mất IMU sẽ giữ
  tay dang ngang. Chuỗi gồm chống tay, đẩy ngực, thu chân, chuyển trọng tâm,
  duỗi chân và dang tay, rồi hạ tay.

## Phần cứng

```text
Raspberry Pi USB -> RTrobot servo controller
Raspberry Pi USB -> ESP32 -> BNO055 + 2 FSR + VL53L5CX
Nguồn servo riêng -> RTrobot V+ / servo rail
```

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
- TinyML Gait Anomaly có model thử nghiệm dữ liệu người UCI HAR, hiển thị
  `PUBLIC TEST`. Baseline riêng của robot `deploy/models/gait_anomaly.json`
  được ưu tiên khi có; model công khai không đánh giá bảo trì và không điều khiển servo.
- One-foot balance, camera mimic, pickup và get-up-back không thuộc runtime hiện tại.

## Train Gait Anomaly

Model thử nghiệm: `deploy/models/gait_anomaly_public.json`. Nguồn:
[UCI HAR](https://doi.org/10.24432/C54S4K), Reyes-Ortiz, Anguita, Ghio, Oneto,
Parra (2013), CC BY 4.0. File model chứa nguồn, SHA256 archive, cách tiền xử lý,
các subject train/calibration/test và kết quả đánh giá. Góc được suy ra từ
`total_acc - body_acc`, không phải orientation BNO055 đo trên robot. Chỉ dùng
10 đặc trưng góc; bỏ gravity norm, bất đối xứng chân và nhịp bước vì dataset
không cung cấp thông tin tương đương. Các hoạt động ngoài walking không phải
nhãn lỗi, nên tỷ lệ bị đánh dấu không được gọi là độ chính xác phát hiện hỏng.

Train lại từ ZIP chính thức tải tại UCI:

```bash
python -m tools.gait_anomaly train-public --dataset /path/to/UCI_HAR_Dataset.zip
python -m tools.gait_anomaly inspect --model deploy/models/gait_anomaly_public.json
```

Để có baseline đúng robot, chạy main rồi mở terminal thứ hai; chỉ thu lúc robot
đi bình thường trên sàn/nguồn đã ghi nhận:

```bash
python -m tools.gait_anomaly collect --url http://127.0.0.1:8765 --seconds 180
python -m tools.gait_anomaly train
python -m tools.gait_anomaly inspect
```

Khởi động lại main để nạp baseline robot. Dữ liệu người chỉ dùng thử pipeline;
chưa chứng minh dự đoán hỏng servo hoặc tuổi thọ còn lại.

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

## Thông số chỉnh tay

Thông số walking đối chiếu mã ngày 06/10/2026. Các giá trị là cấu hình nguồn,
không phải phép đo độ nâng, quãng đường, lực hoặc khả năng leo thật.

### 1. Servo và standing

Chỉnh `STANDING` trong [config.py](src/config.py) để thay tư thế nghỉ. Các động tác
đều tính từ mốc này nên thay standing ảnh hưởng nhiều chức năng, không chỉ lúc idle.
Mapping đang dùng trong code:

| Servo | Khớp | `STANDING` (us) |
| --- | --- | --- |
| 9 | Khuỷu trái | 1500 |
| 10 | Nâng/dang tay trái | 2450 |
| 11 | Vai trái trước/sau | 1500 |
| 12 | Hông trái dạng ngang | 1500 |
| 13 | Đùi trái trước/sau | 1522 |
| 14 | Gối trái | 1500 |
| 15 | Cổ chân trái trước/sau, ankle pitch | 1500 |
| 16 | Cổ chân trái nghiêng ngang, ankle roll | 1500 |
| 17 | Cổ chân phải nghiêng ngang, ankle roll | 1500 |
| 18 | Cổ chân phải trước/sau, ankle pitch | 1500 |
| 19 | Gối phải | 1500 |
| 20 | Đùi phải trước/sau | 1478 |
| 21 | Hông phải dạng ngang | 1500 |
| 22 | Vai phải trước/sau | 1470 |
| 23 | Nâng/dang tay phải | 500 |
| 24 | Khuỷu phải | 1500 |
| 25 | Đầu quay trái/phải | 1500 |

Trong chiều lắp đang được code giả định, tăng mốc 13 và giảm mốc 20 làm tăng
góc đùi trước/sau theo hướng chúi của thuật toán; phải kiểm tra chiều thực tế trước
khi đổi. Đừng nhầm 12/21 là servo đùi pitch. Với walking, ưu tiên chỉnh
`walk_forward_lean_deg` thay vì sửa standing để chữa một vấn đề chỉ xảy ra lúc đi.

| Mốc hình học trong `config.py` | Giá trị | Ý nghĩa |
| --- | --- | --- |
| `PWM_PER_DEG` | `2000.0 / 180.0` | Khoảng 11.111 us/độ danh định; không phải phép đo servo thật |
| `ROBOT["com_height"]` | 147.4 mm | Cao độ thân/CoM dùng trong mô hình chân |
| `ROBOT["half_hip"]` | 28 mm | Nửa khoảng cách hai hông; khoảng cách danh định hai chân 56 mm |
| `ROBOT["upper_leg"]` | 80 mm | Chiều dài đùi trong IK |
| `ROBOT["lower_leg"]` | 75 mm | Chiều dài cẳng chân trong IK |
| `STAND_ANG["L_hip_pitch"]`, `STAND_ANG["R_hip_pitch"]` | 18 độ | Góc đùi gắn với mốc PWM standing |
| `STAND_ANG["L_knee"]`, `STAND_ANG["R_knee"]` | 36 độ | Góc gối gắn với mốc PWM standing |
| `STAND_ANG["L_ankle"]`, `STAND_ANG["R_ankle"]` | 18 độ | Góc ankle pitch gắn với mốc PWM standing |
| `STAND_ANG["L_hip_abduct"]`, `STAND_ANG["R_hip_abduct"]` | 4 độ | Mốc hình học cho mô phỏng stand-up; không tạo thêm độ dạng hông trong gait |
| `DIR` | 12..16: `+1`; 17..21: `-1` | Chiều chuyển góc chân sang PWM; không đổi để tăng tốc/lực |

Quan hệ chuyển đổi: `PWM = STANDING[id] + DIR[id] * (góc_mới - góc_mốc) * PWM_PER_DEG`.
Thông số chiều dài chỉ nên sửa sau khi đo khung, không dùng để thay thế độ nâng chân.
Backend kiểm tra xung nguyên trong 500..2500 us và báo lỗi nếu vượt; không tự cắt
biên độ để che một tư thế sai. Đây là giới hạn lệnh của dự án, không phải giới hạn
cơ khí đã xác nhận của mọi khớp.

### 2. Walking, quay và đi ngang

Đối chiếu mã hiện tại ngày 06/10/2026. Tất cả dòng dưới nằm trong
[config.py](src/config.py); đây là giá trị mục tiêu, không phải phép đo robot thật.

| Thông số | Giá trị hiện tại | Tác dụng khi chỉnh |
| --- | --- | --- |
| `walk_step_length_mm` | 24.0 | Khoảng tiến giữa đích chân swing và chân trụ; tăng để bước dài hơn |
| `walk_turn_length_mm` | 8.1 | Biên độ lệch bước quay Manual; là mm, không phải góc yaw |
| `walk_side_length_mm` | 18.15 | Sải ngang Manual; tăng để chân swing mở rộng hơn |
| `walk_step_time_s` | 1.725 | Thời gian một bước Manual; tốc độ khoảng 40% so với chu kỳ 0.69 s, giữ nguyên sải và độ nâng |
| `walk_settle_time_s` | 0.81 | Thời gian về standing sau bước cuối; không phải tốc độ từng bước |
| `side_swing_tempo` | 2.25 | Tăng để vung chân ngang nhanh hơn trong cùng chu kỳ; code không nhận dưới 1 |
| `walk_step_height_mm` | 58.24 | Độ nâng mục tiêu của bước tiến/lùi; dùng chung với Follow |
| `walk_hip_out_deg` | 3.0 | Dạng hông và bù ankle khi walking không đi ngang; tăng để giãn chân |
| `walk_crouch_depth_mm` | 8.0 | Hạ thân khi tiến/lùi; dùng chung với Follow; không áp vào sideways thuần |
| `walk_forward_lean_deg` | 1.0 | Thêm chúi thân khi tiến; không áp lúc đứng, đi lùi hoặc ngang thuần |
| `walk_lift_start_phase` | 0.30 | Bắt đầu lift sau 30% chu kỳ; phần trước dùng chuyển tải/chuẩn bị |
| `walk_swing_advance_end_phase` | 0.60 | Mốc kết thúc đưa chân tới và đạt đỉnh nâng, khoảng 60% chu kỳ |
| `walk_lift_end_phase` | 0.86 | Kết thúc hạ chân ở 86%; phần còn lại chuyển tải sau tiếp đất |
| `zmp_support_ratio` | 0.80 | Biên độ dịch tải ngang theo nửa khoảng cách hông; tăng thì chuyển tải mạnh hơn |
| `ankle_roll_gain` | -1.00 | Hệ số và chiều bù cổ chân ngang; tăng trị tuyệt đối để bù nhiều hơn, không tự đảo dấu |
| `arm_swing_pwm` | 50 | Biên độ đánh tay khi Manual walking; `0` tắt đánh tay |
| `auto_step_time_s` | 1.26 | Thời gian bước Follow và bước tiếp cận cầu thang; giảm để nhanh hơn |
| `auto_settle_time_s` | 1.47 | Thời gian các engine tự động về standing sau bước cuối |
| `person_follow_step_length_mm` | 8.64 | Sải tiến tối đa khi Follow; lệnh thực tế có thể nhỏ hơn theo ToF |
| `person_follow_turn_length_mm` | 1.44 | Sải quay tối đa khi Follow; không dùng sải quay Manual |
| `stair_approach_step_mm` | 2.16 | Sải tiến khi tiếp cận cầu thang; không phải sải bước leo bậc |
| `stair_turn_step_mm` | 0.40 | Sải quay khi căn hướng cầu thang |
| `update_ms` | 30 | Chu kỳ engine; 23 khung/bước Manual và 27 khung/settle hiện tại |
| `stop_ms` | 250 | Thời gian lệnh reset/standing cưỡng bức; không thay thời gian bước |
| `head_pan_pwm` | 220 | Biên độ đầu khi quay Manual; Follow giữ đầu cố định |
| `head_pan_direction` | 1.0 | Chiều đầu khi quay Manual; dùng -1 nếu chiều cơ khí bị đảo |
| `gait_dashboard_command_timeout_s` | 0.6 | Timeout lệnh web; mất heartbeat thì ngắt quyền điều khiển |

Phạm vi cần phân biệt:

- Sải và thời gian Manual không đổi sải/tốc độ Follow. Độ nâng, dạng hông, hạ thân,
  độ chúi và mốc lift được dùng chung giữa Manual và Follow.
- `zmp_support_ratio` và `ankle_roll_gain` còn dùng cho bước cầu thang. Không phải gain
  IMU balance: Manual vẫn không tự bù theo IMU.
- Đi ngang thuần dùng `0.90 * walk_side_length_mm`, hiện là **16.335 mm** mỗi chân.
  Đây là hệ số quỹ đạo trong [walking_engine.py](src/walking_engine.py), không phải
  một `side_speed` khác. Đích bàn chân không nâng Z; chân đi trước mở ra, chân sau kéo theo.
- Quay thuần dùng `0.45 * walk_step_height_mm`, hiện là **26.208 mm** độ nâng mục tiêu.
  Không suy ra một bước quay được bao nhiêu độ trên sàn chỉ từ `walk_turn_length_mm`.
- Tay khi đi ngang dùng `round(0.55 * arm_swing_pwm)`; Follow và tiếp cận cầu thang
  truyền `arm_swing_pwm=0`, nên chỉnh đánh tay Manual không bật tay ở hai mode đó.
- Mốc lift bị ràng buộc trong engine: start 0..0.40, end không quá 0.95 và phải sau
  start ít nhất 0.20; mốc đưa chân tới nằm giữa start + 0.10 và end - 0.05.
  Đừng đặt các mốc chồng nhau rồi kỳ vọng code dùng nguyên giá trị nhập.
- Chu kỳ lệnh 30 ms làm thời gian được lượng tử hóa theo khung. Dừng phím không
  có nghĩa bàn chân đứng yên ngay: engine hoàn tất bước rồi mới settle.
- Đi ngang nhanh chỉ đổi đoạn đưa chân ngang: từ phase 0.30 đến
  `0.30 + (0.60 - 0.30) / 2.25 = 0.4333`. Thời gian toàn bước vẫn 0.69 s.
- Khi tiến/lùi, hạ thân diễn ra trong đoạn chuẩn bị trước lift và giữ khi đi liên tục.
  Đi lùi vẫn hạ thân nhưng không thêm chúi thân; quay/ngang thuần không hạ thân.
- Không có tham số lực/mô-men servo trực tiếp. Giảm thời gian tăng tốc chuyển động,
  không tăng dòng điện hoặc bảo đảm lực dưới tải. Không đổi `baudrate`, `DIR`, kích
  thước khung hay trọng số LQR để tìm thêm lực.
- Engine dùng lệnh chuẩn hóa -1..1. Tiến: `forward * walk_step_length_mm`;
  quay: `turn * walk_turn_length_mm`; ngang: `-side * walk_side_length_mm`.
  Quãng vung chân không luôn bằng giá trị cấu hình: tiến thuần 24 mm cho đích đầu
  cách chân trụ 24 mm, các bước đều sau có thể vung 48 mm từ vị trí trước đó.
  Quay thuần dùng `abs(turn_len) + (-turn_len nếu chân trái, +turn_len nếu chân phải)`;
  với lệnh quay hết biên độ, đích so với chân trụ là 0 hoặc 16.2 mm. Đây không phải
  góc yaw đã đạt trên mặt sàn; cần đo thực tế để đánh giá quay.

Thông số thuật toán cấp thấp, không phải nút chỉnh lực thông thường:

| File / vị trí | Giá trị | Vai trò |
| --- | --- | --- |
| `walking_engine.py`: `command_deadzone` | 0.02 | Bỏ lệnh chuẩn hóa quá nhỏ; bàn phím bình thường dùng -1, 0, +1 |
| `walking_engine.py`: `preview_steps` | 24 | Số khung nhìn trước của bộ ZMP; khoảng 0.72 s khi dt = 30 ms |
| `walking_engine.py`: bước hiệu dụng tối thiểu | 0.1 mm | Lệnh dưới ngưỡng này không tạo bước; không phải chiều cao lift |
| `walking_engine.py`: `is_idle_ready(tolerance)` | 0.05 | Dung sai trạng thái/quỹ đạo khi xác nhận dừng; không chỉnh lực |
| `walking_engine.py`: sai lệch PWM khi idle | 3 us | Dung sai so với standing dùng để xác nhận engine đã nghỉ |
| `walking_engine.py`: hệ số sải ngang | 0.90 | Chỉ dùng khi ngang chiếm ưu thế; đi ngang thuần không nâng Z |
| `walking_engine.py`: hệ số lift khi quay | 0.45 | Chỉ dùng quay thuần; quay kết hợp tiến/lùi vẫn dùng lift đầy đủ |
| `walking_engine.py`: độ trễ đưa chân tiến | `min(lift start + 0.10, advance end - 0.10)` | Bắt đầu đưa chân tới sau khi lift đã bắt đầu |
| `walking_engine.py`: ngưỡng sẵn sàng ngang phối hợp | lift factor / 0.45 | Điều tiết ngang khi đồng thời tiến/quay; không dùng cho ngang thuần |
| `walking_engine.py`: tay ngang | 0.55 | `round(0.55 * arm_swing_pwm)`, hiện 28 us |
| `walking_engine.py`: chọn chân trụ từ ZMP | ±0.5 * half_hip | Hiện ±14 mm so với tâm hai chân; khi đang swing vẫn giữ chân đối diện làm trụ |
| `walking_engine.py`: khoảng cách chân ngang tối thiểu | 2 * half_hip | Hiện 56 mm; chặn chân sau kéo vượt qua chân trước |
| `zmp_controller.py`: `g` | 9800 mm/s² | Gia tốc trọng trường trong mô hình, không phải lực servo |
| `zmp_controller.py`: `Qe` | 1.0 | Trọng số sai số ZMP trong LQR |
| `zmp_controller.py`: `R` | 0.000001 | Trọng số jerk; không đổi tùy tiện để tăng lực |
| `zmp_controller.py`: `riccati_iters` | 3000 | Giới hạn vòng lặp giải hệ số, không phải số bước chân |
| `zmp_controller.py`: điều kiện settled | 0.2 mm; 1 mm/s; 20 mm/s²; integral 1.0 | Các sai số được coi là ổn định trong bộ điều khiển |
| `zmp_controller.py`: dung sai Riccati | 1e-10 | Ngưỡng dừng giải hệ số, chỉ chạy khi khởi tạo controller |
| `leg_ik.py`: khoảng tránh duỗi/gập tuyệt đối | 0.5 mm | Chặn chiều dài chân trong `abs(L1-L2)+0.5` đến `L1+L2-0.5`; không phải clearance bàn chân |

Lift: start -> đỉnh tại advance end -> hạ về 0 tại lift end. Đưa chân tiến bắt đầu
ở phase 0.40 và kết thúc ở 0.60 với cấu hình hiện tại. Chuyển tải sau landing bắt
đầu ở 0.86 và kết thúc ở 1.0; không có biến landing speed riêng.
Đường cong nối là `t² * (3 - 2t)`, không có lớp rate limit PWM trong walking.
Backend vẫn kiểm tra miền xung 500..2500 us, không cắt biên độ để che lỗi.

Ankle roll của bước thường chỉ tính tại nhánh walking, không tính rồi ghi đè trong
IK chung. Trước khi cộng dạng hông, góc bù trụ cực đại danh định là
`atan2(28 * 0.80, 147.4) * (-1) ≈ -8.641 độ`; không phải góc nghiêng IMU đã đo.
Sidewalk giữ hông/ankle chân trụ tại tư thế đầu bước; chỉ chân swing mở hoặc kéo theo.
IMU balance không ghi đè Manual; FSR không phải điều kiện để Manual bước.

### 8. Stair detect, tiếp cận và bước cầu thang

| Thông số trong `config.py` | Giá trị Git | Tác dụng |
| --- | --- | --- |
| `stair_model` | `deploy/models/stair_detector.onnx` | Model nhận diện cầu thang |
| `stair_model_confidence` | 0.55 | Ngưỡng điểm model/xác nhận hình học; tăng chặt hơn |
| `stair_model_iou_threshold` | 0.45 | IoU loại khung trùng bằng NMS; không phải độ chính xác nhận diện |
| `stair_model_input_size` | 416 | Kích thước đầu vào; phải phù hợp model ONNX đã export |
| `stair_detect_every_frames` | 3 | Chu kỳ inference detector; tăng thì bớt tải, trễ hơn |
| `stair_detect_stable_frames` | 4 | Số cặp phát hiện/ToF mới ổn định trước khi cho bước |
| `stair_camera_align_deadband` | 0.12 | Sai lệch tâm cầu thang cho phép trước khi quay căn hướng |
| `stair_approach_step_mm` | 2.16 | Sải tiến của bước tiếp cận, không phải sải leo bậc |
| `stair_turn_step_mm` | 0.40 | Biên độ bước quay khi căn cầu thang |
| `stair_default_riser_mm` | 20.0 | Chiều cao bậc danh định trong suy đoán hình học |
| `stair_min_riser_mm` | 15.0 | Cận dưới chiều cao bậc trong mô hình hiện tại |
| `stair_max_riser_mm` | 25.0 | Cận trên chiều cao bậc trong mô hình hiện tại |
| `stair_tread_depth_mm` | 160.0 | Chiều sâu mặt bậc, không phải sải chân |
| `stair_width_mm` | 320.0 | Bề rộng mặt bậc để kiểm tra chỗ đặt hai chân |
| `stair_step_depth_mm` | 120.0 | Sải leo tối đa; sải thực tế tính từ mép bậc và kích thước bàn chân |
| `stair_foot_clearance_mm` | 18.0 | Khoảng hở qua mép bậc; lên bậc 20 mm có đỉnh nâng 38 mm |
| `stair_crouch_depth_mm` | 35.0 | Độ hạ thân khi thực hiện bước cầu thang |
| `stair_phase_shift_s` | 1.20 | Thời gian chuyển tải trước khi nâng chân đầu |
| `stair_phase_swing_s` | 2.80 | Thời gian swing của mỗi chân; dùng hai lần trong một chuỗi |
| `stair_phase_transfer_s` | 1.20 | Chuyển trọng lượng lên chân đã đặt trên bậc |
| `stair_phase_settle_s` | 1.50 | Thu về tư thế đứng ở cuối chuỗi |
| `stair_step_pause_s` | 0.70 | Tạm nghỉ/xác nhận trước bậc kế tiếp |
| `stair_geometry_calibrated` | False | Khóa bước tự động; chỉ True sau khi đo và kiểm chứng hình học |
| `stair_foot_toe_mm` | 0.0 | Chiều dài từ mốc bàn chân tới mũi; phải nhập số đo thật |
| `stair_foot_heel_mm` | 0.0 | Chiều dài từ mốc bàn chân tới gót; phải nhập số đo thật |
| `stair_foot_width_mm` | 0.0 | Bề rộng bàn chân; không phải khoảng cách hai hông |
| `stair_landing_margin_mm` | 8.0 | Khoảng dự phòng để cả bàn chân nằm trên mặt bậc |
| `stair_tof_forward_offset_mm` | 0.0 | Vị trí ToF theo trục trước/sau so với mốc hình học đặt chân |
| `stair_tof_mount_height_mm` | 220.0 | Cao độ ToF so với sàn dùng để dựng hình học cầu thang |
| `stair_tof_pitch_down_deg` | 0.0 | ToF ngực nhìn thẳng theo setup; nếu lắp nghiêng phải nhập góc đo thật |
| `stair_tof_vertical_fov_deg` | 45.0 | FOV dọc để đổi hàng ToF sang góc nhìn |
| `stair_tof_flip_vertical` | True | Đảo hàng lưới ToF riêng cho thuật toán stair |

**Điểm chưa hiệu chuẩn:** góc mặc định đã khớp setup nhìn thẳng, nhưng chiều cao
220 mm và vị trí trước/sau của ToF vẫn cần đo. Không bật khóa leo khi chưa đo bàn
chân/ToF và kiểm chứng hình học. Cảm biến nhìn thẳng không bảo đảm thấy được sàn và
mặt bậc gần bàn chân; khi không tách được hai mặt phẳng, chỉ báo UNKNOWN/PREVIEW.

Trong [stair_main.py](src/stair_main.py), bước tiếp cận có lift cố định 24 mm, không
dùng `walk_step_height_mm`; nó dùng thời gian `auto_*`. Bước leo thực ở
[stair_motion.py](src/stair_motion.py) có các pha shift -> chân đầu -> transfer ->
chân sau -> settle. Tổng danh định 9.5 s, chưa tính pause 0.7 s và chờ xác nhận.
Trong mỗi swing: 30% đầu nâng, 40% giữa đưa chân tới ở cao độ đỉnh, 30% cuối hạ.
Đỉnh lên bằng `riser + clearance`; xuống thì nhấc clearance trước rồi mới hạ xuống bậc.
Thân ở trên chân sau trong pha đưa chân đầu; pha transfer đưa thân đến vị trí chân
đầu đã đặt trên bậc trước khi nhấc chân sau. Độ hạ/nâng thân còn phụ thuộc giới hạn
vươn chân ở sải dài, không dùng các hệ số tiến thân 0.05/0.45 cũ.

Các pha có thời gian tối thiểu được chặn trong constructor: shift 0.25 s, swing
0.50 s, transfer 0.35 s, settle 0.30 s; clearance tối thiểu 5 mm. Điều kiện runtime
còn yêu cầu IMU và balance sẵn sàng, roll/pitch lệch reference không quá 3 độ,
và đủ chỗ đặt toàn bàn chân. Các giá trị 0 chưa đo ở toe/heel/width là khóa an toàn,
không phải biến thừa để xóa.

ToF phải tách được hai mặt ngang, mỗi mặt có nhiều zone hợp lệ; chênh khoảng cách
dọc tia đơn thuần không còn được dùng để đoán lên/xuống. Xác nhận gồm bốn cặp
camera/ToF mới; lặp lại cùng timestamp không được tính thêm. Không cần FSR để bật
walking, nhưng bước tự động cần IMU upright/calibrated, camera và ToF còn dữ liệu.
Mất sensor hoặc vượt giới hạn nghiêng trong lúc bước thì giữ tư thế, không tự tiếp
tục khi sensor trở lại; cần dữ liệu hợp lệ và nhấn U. Fall detection vẫn ưu tiên cao nhất.
Thoát card giữa bước giữ tư thế và disarm; phải đỡ robot trước khi reset.

## An toàn

1. Raspberry Pi và servo dùng hai nguồn riêng.
2. Không cấp servo từ USB hoặc rail nguồn của Pi.
3. Treo hoặc giữ chắc robot khi kiểm tra gait mới.
4. Luôn sẵn sàng `Space`, `Esc` và công tắc cắt nguồn servo.
5. Compile/dry-run không thay thế kiểm tra robot thật.
