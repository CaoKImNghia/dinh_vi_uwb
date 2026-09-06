# UWB + IMU Fusion Positioning

## Runtime output and alignment

Serial output is valid CSV with three numeric columns:
`x_fused,y_fused,uwb_age_s`. `uwb_age_s` is the time since the last valid UWB
position; values above 3 seconds mean the estimate is currently IMU-only.

Mount the BMI270 with its X/Y axes aligned to the UWB anchor coordinate system.
If that is not practical, set `IMU_TO_UWB_YAW_OFFSET_DEG` at the top of the
sketch before uploading. The BMI270 has no magnetometer, so this fixed mounting
alignment is required for IMU acceleration to match the UWB axes.

Firmware định vị trong nhà 2D cho ESP32. Hệ thống kết hợp số đo UWB từ ba anchor BU03 với BMI270 để tạo vị trí mượt hơn bằng bộ lọc Kalman 6 trạng thái và ZUPT.

## Cấu trúc project

```text
.
├── dinh-vi-uwb/
│   ├── dinh-vi-uwb.ino   # Sketch chính
│   └── imu_driver.h      # BMI270, Madgwick và ZUPT
├── HUONG_DAN_PHAN_CUNG.md
└── .github/workflows/arduino-ci.yml
```

Thư mục sketch phải giữ tên `dinh-vi-uwb`, trùng với file `dinh-vi-uwb.ino`. Đây là cấu trúc Arduino IDE nhận diện trực tiếp; không đặt các file firmware trở lại thư mục gốc.

## Nạp bằng Arduino IDE

1. Cài **Arduino IDE 2.x**.
2. Trong **Boards Manager**, cài `esp32` by Espressif Systems.
3. Trong **Library Manager**, cài `SparkFun BMI270 Arduino Library` (bản 1.0.3 hoặc tương thích).
4. Mở [dinh-vi-uwb.ino](dinh-vi-uwb/dinh-vi-uwb.ino), chọn board **ESP32 Dev Module** (hoặc đúng biến thể ESP32 đang dùng) và đúng cổng serial.
5. Nạp sketch. Khi khởi động, để tag đứng yên trong lúc BMI270 tự hiệu chuẩn (~3 giây).
6. Mở Serial Monitor/Serial Plotter tại **115200 baud**. Dòng dữ liệu bình thường có dạng `x_fused,y_fused,uwb_age_s` theo đơn vị mét và giây.

## Kết nối mặc định

| ESP32 | Thiết bị | Vai trò |
| --- | --- | --- |
| GPIO 16 | BU03 TX | UART2 RX |
| GPIO 17 | BU03 RX | UART2 TX |
| GPIO 21 | BMI270 SDA | I²C SDA |
| GPIO 22 | BMI270 SCL | I²C SCL |
| 3V3/GND | BU03, BMI270 | Nguồn và mass chung |

BU03 UART dùng 115200 baud. BMI270 mặc định dùng địa chỉ I²C `0x68` khi chân SDO nối GND. Xem [hướng dẫn phần cứng](HUONG_DAN_PHAN_CUNG.md) để biết bố trí anchor, hiệu chuẩn và cảnh báo mức điện áp.

## Thiết lập theo khu vực lắp đặt

- Cập nhật `base_stations` trong [dinh-vi-uwb.ino](dinh-vi-uwb/dinh-vi-uwb.ino) theo tọa độ thực tế của ba anchor, đơn vị mét.
- Cập nhật `distance_offsets` sau khi đo sai lệch UWB tại vị trí chuẩn.
- Giữ ba anchor không thẳng hàng và đặt tag gần cùng mặt phẳng cao độ với anchor để giả định định vị 2D còn đúng.

## Quy trình phát triển

1. Chỉ sửa firmware trong `dinh-vi-uwb/`.
2. Kiểm tra bằng Arduino IDE trên phần cứng trước khi commit.
3. Xem `git status` và chỉ commit source, tài liệu, hay cấu hình cần chia sẻ; file build, log và cấu hình editor cục bộ đã được bỏ qua.
4. Mỗi push/PR lên `main` sẽ được GitHub Actions compile bằng Arduino CLI với board `ESP32 Dev Module`. CI chỉ kiểm tra khả năng biên dịch, không thể thay thế thử nghiệm với BU03/BMI270 thực tế.

## Ghi chú an toàn phần cứng

GPIO của ESP32 hoạt động ở 3.3 V. Không cấp 5 V trực tiếp cho BMI270 và luôn nối chung GND giữa ESP32, BU03 và BMI270.
