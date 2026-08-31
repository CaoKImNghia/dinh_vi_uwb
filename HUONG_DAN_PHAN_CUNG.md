# KẾ HOẠCH & HƯỚNG DẪN TRIỂN KHAI PHẦN CỨNG
**Hệ thống Định vị Trong nhà Dung hợp UWB + IMU (6-State Kalman Fusion v2.0)**

---

## 1. Danh sách Thiết bị (BOM - Bill of Materials)

| Thiết bị | Số lượng | Mô tả / Vai trò | Ghi chú |
| :--- | :---: | :--- | :--- |
| **ESP32 NodeMCU / DevKitC** | 1 | Vi điều khiển trung tâm (Thẻ Tag) | Xử lý đọc IMU (100Hz), UWB UART (10Hz) và chạy bộ lọc Kalman Fusion |
| **Module Ai-Thinker BU03** | 4 | 3 Trạm gốc (Anchor) + 1 Thẻ di động (Tag) | Tích hợp chip UWB DW1000 + STM32 onboard xuất dữ liệu định vị qua UART |
| **Module IMU Bosch BMI270** | 1 | Cảm biến quán tính 6 trục cho Tag | Đo gia tốc ($a_x, a_y, a_z$) và vận tốc góc ($g_x, g_y, g_z$) qua I2C. Yêu cầu thư viện SparkFun BMI270. |
| **Nguồn cấp / Pin sạc** | 4 | Nguồn cho 3 Anchor và 1 Tag (ESP32) | Pin LiPo 3.7V (kèm mạch tăng áp 5V) hoặc sạc dự phòng / adapter 5V-1A |
| **Dây nối & Breadboard** | 1 bộ | Kết nối các module trên Tag | Dây Dupont (Male-Female, Male-Male) |

---

## 2. Nguyên lý Hoạt động & Thuật toán Dung hợp (System Architecture & Principles)

Hệ thống định vị này giải quyết bài toán cốt lõi trong định vị trong nhà: **Làm sao có được tọa độ vừa chính xác tuyệt đối, vừa mượt mà theo thời gian thực mà không bị trôi (drift)?** Để làm được điều này, dự án kết hợp 2 công nghệ bổ trợ hoàn hảo cho nhau thông qua **Bộ lọc Kalman 6 Trạng thái (6-State Kalman Fusion)**:

### 1. Tại sao phải dung hợp UWB + IMU?
* **UWB (Ultra-Wideband - BU03):** Đo khoảng cách bằng thời gian bay của sóng vô tuyến (ToF/TWR), cho vị trí tuyệt đối cực kỳ chính xác (sai số chỉ vài cm) và **không bao giờ bị trôi theo thời gian**. Tuy nhiên, nhược điểm là tần suất cập nhật chậm (~10Hz) và dễ bị nhảy điểm (outlier/spike) khi có người đi ngang qua che khuất sóng (hiện tượng NLOS - Non-Line of Sight).
* **IMU (Inertial Measurement Unit - BMI270):** Đo gia tốc và vận tốc góc với tần suất rất cao (**100Hz**), phản hồi chuyển động tức thời (low latency). Tuy nhiên, nếu dùng IMU độc lập để tính vị trí bằng cách tích phân hai lần gia tốc ($x = \iint a \, dt^2$), sai số cảm biến sẽ tích lũy theo cấp số nhân làm tọa độ **trôi bay xa chỉ sau vài giây**.
* **Giải pháp Dung hợp:** UWB làm "mỏ neo" giữ cho vị trí không bị trôi về lâu dài, còn IMU làm "cầu nối" lấp đầy các khoảng trống thời gian giữa các gói tin UWB, mang lại quỹ đạo chuyển động mượt mà ở tần suất 100Hz!

### 2. Bức tranh Xử lý Kép theo Thời gian (Dual-Rate Processing Loop)
Thuật toán hoạt động song song ở 2 tần số khác nhau trên ESP32:
* **Bước Dự đoán (Prediction @ 100Hz - mỗi 10ms):**
  - ESP32 đọc cảm biến BMI270. Bộ lọc **Madgwick** tính toán góc xoay 3D (quaternion) để loại bỏ chính xác vectơ trọng trường trái đất ($1g$), thu được gia tốc chuyển động thực tế trong khung tọa độ thế giới ($a_{x\_world}, a_{y\_world}$).
  - Bộ lọc Kalman đẩy phương trình chuyển động lên bước tiếp theo: $x_{mới} = x + v \cdot dt + 0.5 \cdot a \cdot dt^2$.
* **Bước Cập nhật (Update @ ~10Hz - khi có gói tin UWB):**
  - Khi module BU03 gửi về khung UART 35 byte chứa khoảng cách đến 3 Anchor, thuật toán **Tam giác đạc 2D (`trilaterate2d`)** giải hệ phương trình giao điểm 3 đường tròn để lấy tọa độ thô $(x_{raw}, y_{raw})$.
  - Bộ lọc Kalman dùng tọa độ thô này để **hiệu chỉnh (correct)** lại vị trí đang dự đoán và kéo vận tốc, gia tốc của IMU về chuẩn, triệt tiêu hoàn toàn sai số tích lũy.

### 3. Các Cơ chế Bảo vệ Thông minh (Smart Protection)
* **Tự động chống nhiễu che khuất (Adaptive NLOS Detection):** Khi nhận tọa độ UWB mới, bộ lọc kiểm tra độ lệch (residual) so với vị trí dự đoán từ IMU. Nếu độ lệch vượt ngưỡng `0.5m` (`nlos_threshold`), hệ thống nhận diện sóng UWB đang bị phản xạ/che khuất (NLOS). Lập tức, ma trận nhiễu đo đạc UWB ($R_{uwb}$) được tự động tăng lên gấp 20 lần ($1.0$ thay vì $0.05$). Hệ thống tạm thời "không tin" UWB và lướt đi mượt mà bằng quán tính IMU.
* **Cập nhật Vận tốc Không (ZUPT - Zero-Velocity Update):** Khi đứng yên, nhiễu trắng của gia tốc kế có thể làm hệ thống tưởng thẻ Tag đang trượt đi slowly. Nhờ hàm `imu.isStationary()`, khi phát hiện thẻ Tag đứng yên, bộ lọc lập tức ép phương trình vận tốc về $v_x = 0, v_y = 0$, khóa chặt vị trí đứng yên như bàn thạch!

---

## 3. Sơ đồ Đấu nối Mạch (Wiring Diagram cho Tag)

Trên thiết bị di động (Tag), **ESP32** đóng vai trò xử lý trung tâm, nhận dữ liệu khoảng cách từ **BU03 Tag** qua UART và gia tốc/quán tính từ **BMI270** qua I2C.

### Bảng Kết nối Chân (Pinout)

| ESP32 GPIO | Module BU03 (Tag) | Module BMI270 | Chức năng |
| :---: | :---: | :---: | :--- |
| **GPIO 16 (RX2)** | TX / TXD | — | Nhận gói tin UART từ BU03 (Baudrate 115200) |
| **GPIO 17 (TX2)** | RX / RXD | — | Gửi lệnh cấu hình đến BU03 (nếu cần) |
| **GPIO 21** | — | SDA | Dữ liệu I2C cho cảm biến BMI270 |
| **GPIO 22** | — | SCL | Xung nhịp I2C cho cảm biến BMI270 |
| **3V3** | VCC (tùy board) | VCC (1.8V–3.3V) | Cấp nguồn cho các module |
| **GND** | GND | GND | Nối chung mass (Bắt buộc) |

> [!IMPORTANT]
> **Lưu ý mức điện áp (Logic Level):**
> - Các chân GPIO của ESP32 hoạt động ở mức logic **3.3V**.
> - Module BMI270 hoạt động ở **1.8V – 3.6V**, hoàn toàn tương thích trực tiếp với ESP32 3.3V. **Không dùng 5V** cho BMI270.
> - Chân **SDO** của BMI270: nối **GND** → địa chỉ I2C `0x68`, nối **VDDIO** → địa chỉ `0x69`.
> - Module BU03 thường có chân UART giao tiếp mức 3.3V, tương thích trực tiếp với GPIO 16/17 của ESP32.
> - Bắt buộc nối chung chân **GND** của cả 3 board (ESP32, BU03, BMI270).

---

## 4. Bố trí & Lắp đặt Trạm gốc (Anchor Geometry)

Hệ thống sử dụng **3 Anchor** làm mốc tọa độ cố định trong không gian 2D. Thuật toán `trilaterate2d` trong code giải phương trình giao điểm của 3 đường tròn để tìm ra tọa độ $(x, y)$.

### Tọa độ Thiết lập trong Code
Trong file `dinh-vi-uwb.ino`, các trạm gốc được khai báo như sau:
* **Base 0 (Anchor 0):** `(x: 0.0m, y: 0.7m)`
* **Base 1 (Anchor 1):** `(x: 6.0m, y: 6.942m)`
* **Base 2 (Anchor 2):** `(x: 6.0m, y: 0.0m)`

```
  Y (mét)
   ▲
   │        [Base 1] (6.0, 6.942)
  7┼               ■
   │               │
  6┼               │
   │               │
  5┼               │  Khu vực đo hiệu quả nhất
   │               │  (Trong lòng tam giác)
  4┼               │
   │               │
  3┼               │
   │               │
  2┼               │
   │               │
  1┼  ■ [Base 0]   │
   │   (0, 0.7)    │
  0┼───────────────■────────► X (mét)
  0                6.0   [Base 2] (6.0, 0.0)
```

### Nguyên tắc Lắp đặt Thực tế:
1. **Độ cao đồng nhất (2D Assumption):**
   - Đặt 3 Anchor **cùng trên một mặt phẳng ngang** (ví dụ: treo trên chân máy ảnh hoặc tường ở độ cao **1.5m - 2.0m** so với mặt đất).
   - Thẻ di động (Tag) khi di chuyển cũng nên giữ ở độ cao tương đương với mặt phẳng Anchor để tránh sai số hình chiếu 3D xuống 2D.
2. **Đường truyền thẳng (LOS - Line of Sight):**
   - Tránh đặt Anchor sau bức tường bê tông, tủ sắt, hoặc các vật cản lớn hấp thụ sóng vô tuyến.
   - Hướng anten của module BU03 ra phía vùng không gian di chuyển.
3. **Độ mở hình học (GDOP):**
   - 3 Anchor tạo thành một tam giác đủ rộng (như cấu hình hiện tại là tam giác gần vuông với cạnh ~6m × ~6.24m). Tránh đặt 3 Anchor trên cùng một đường thẳng.

---

## 5. Hướng dẫn Cấu hình Module BU03

Module Ai-Thinker BU03 (hoặc các board tương tự dùng chip DW1000 + STM32) cần được thiết lập vai trò trước khi lắp vào hệ thống:
1. **Cấu hình Anchor (3 module):**
   - Sử dụng công cụ AT Command hoặc phần mềm cấu hình từ Ai-Thinker qua cáp USB-to-UART.
   - Thiết lập chế độ: **Anchor** (Trạm gốc).
   - Gán ID lần lượt là: `0` (cho Base0), `1` (cho Base1), và `2` (cho Base2).
2. **Cấu hình Tag (1 module):**
   - Thiết lập chế độ: **Tag** (Thẻ di động).
   - Cấu hình tần suất phát định vị: **10Hz** (100ms/lần).
   - Kiểm tra chuỗi dữ liệu xuất ra UART ở baudrate `115200`: Gói tin hợp lệ bắt đầu bằng 3 byte header `0xAA 0x25 0x01` theo chuẩn giao thức trong hàm `parseUwbData()`.

---

## 6. Quy trình Kiểm tra & Hiệu chỉnh (Calibration & Testing)

### Bước 1: Khởi động & Kiểm tra I2C (IMU)
- Cấp nguồn cho Tag nối với máy tính qua cổng USB.
- Mở **Serial Monitor** trên Arduino IDE ở Baudrate **115200**.
- Kiểm tra thông báo khởi động:
  - Nếu thành công: BMI270 sẽ tự động bắt đầu quá trình calibrate bias.
  - Nếu thấy `[WARN] IMU init failed! Running UWB-only mode.`: Kiểm tra lại dây nối SDA (Pin 21), SCL (Pin 22), chân SDO (nối GND cho addr 0x68) và chân nguồn của BMI270.

### Bước 2: Hiệu chỉnh IMU (Tĩnh)
> [!WARNING]
> **RẤT QUAN TRỌNG:**
> Khi bật nguồn ESP32, trong **3 giây đầu tiên** (khi hàm `imu.calibrate(500)` đang chạy), bạn **BẮT BUỘC phải đặt thẻ Tag đứng yên hoàn toàn** trên mặt bàn phẳng. Nếu rung lắc trong giai đoạn này, gia tốc tĩnh sẽ bị tính sai bias dẫn đến hiện tượng trôi (drift) vị trí liên tục!

### Bước 3: Hiệu chỉnh Sai số Khoảng cách UWB (Distance Offsets)
Sóng UWB có thể bị sai số cố định do trễ phần cứng hoặc anten (antenna delay). Để hiệu chỉnh:
1. Đặt Tag ở một vị trí biết trước tọa độ chính xác, hoặc đo khoảng cách thực tế bằng thước laser từ Tag đến từng Anchor 0, 1, 2.
2. So sánh khoảng cách đo được bằng thước với khoảng cách thô xuất ra từ UWB.
3. Nếu có sai số cố định (ví dụ UWB luôn đo dài hơn thực tế `0.15m`), điều chỉnh mảng bù sai số ở **dòng 27** trong file `dinh-vi-uwb.ino`:
   ```cpp
   // Sai số hiệu chỉnh: [Bù Anchor 0, Bù Anchor 1, Bù Anchor 2]
   float distance_offsets[3] = {-0.15, -0.10, -0.12}; 
   ```

### Bước 4: Kiểm thử Dung hợp (Kalman Fusion Verification)
- Mở **Serial Plotter** (hoặc dùng Python script / MATLAB / Excel) để vẽ đồ thị theo chuỗi đầu ra CSV:
  ```text
  x_fused,y_fused
  1.234,2.456
  1.236,2.458
  ...
  ```
- **Kiểm tra tính năng ZUPT (Zero-Velocity Update):** Khi đặt Tag đứng yên, nhờ cảm biến BMI270 phát hiện trạng thái tĩnh (`imu.isStationary()`), vận tốc được ép về 0 và tọa độ `(x, y)` sẽ đứng yên không bị dao động rớt điểm.
- **Kiểm tra chống NLOS / Mất sóng UWB:** Thử lấy tay hoặc tấm kim loại che anten của 1 Anchor, bộ lọc sẽ nhận diện residual cao (`nlos_threshold > 0.5m`) và tự động giảm tin cậy của UWB, chuyển sang dùng quán tính từ IMU để giữ mượt quỹ đạo!
