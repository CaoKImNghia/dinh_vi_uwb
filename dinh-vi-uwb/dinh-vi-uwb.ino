#include <Arduino.h>
#include "imu_driver.h"

// BU03 UART frame and validation limits. Keep the range comfortably above
// the documented 6 m test area while rejecting corrupt/overflowed readings.
constexpr size_t UWB_FRAME_LENGTH = 35;
constexpr float MIN_UWB_DISTANCE_M = 0.10f;
constexpr float MAX_UWB_DISTANCE_M = 30.0f;
constexpr float MAX_TRILATERATION_RMS_RESIDUAL_M = 1.5f;

// Align the IMU's forward axis to the UWB anchor coordinate system. Set this
// only when the mounted IMU is rotated relative to the UWB X/Y axes.
constexpr float IMU_TO_UWB_YAW_OFFSET_DEG = 0.0f;

#define RXD2 16 // Nối với chân TX của STM32 trên BU03
#define TXD2 17 // Nối với chân RX của STM32 trên BU03

// Cấu hình hệ thống
#define UART_TIMEOUT_MS 50   // Thời gian chờ tối đa cho 1 frame (chống kẹt buffer)
#define IMU_UPDATE_MS 10     // Chu kỳ đọc IMU = 10ms → 100Hz
#define IMU_SDA_PIN 21       // I2C SDA cho BMI270
#define IMU_SCL_PIN 22       // I2C SCL cho BMI270

// --- Tọa độ các Trạm gốc (Base Stations) ---
struct Position {
  float x, y;
};

Position base_stations[3] = {{0, 0.7}, {6, 6.942}, {6, 0}};

// Sai số hiệu chỉnh (Calibration offsets)
float distance_offsets[3] = {0.0, 0.0, 0.0};

// ============================================================
// Bộ lọc Kalman Fusion 6-State: [x, y, vx, vy, ax, ay]
// Kết hợp UWB (chính xác tuyệt đối) + IMU (mượt mà, nhanh)
// ============================================================
class FusionKalmanFilter {
private:
  // State vector: [x, y, vx, vy, ax, ay]
  float state[6];

  // Covariance matrix P[6x6]
  float P[6][6];

  // Process noise base values
  float Q_pos;    // Process noise cho position
  float Q_vel;    // Process noise cho velocity
  float Q_acc;    // Process noise cho acceleration

  // Measurement noise
  // R values are measurement variances (not standard deviations).
  float R_uwb_base;   // Base UWB measurement noise
  float R_uwb_nlos;   // NLOS UWB measurement noise (cao hơn)

  // NLOS detection threshold
  float R_imu_acc;    // IMU acceleration measurement variance
  float nlos_threshold;

  // Timing
  unsigned long last_predict_time;
  bool initialized;

  // Thống kê UWB
  unsigned long last_uwb_time;          // Thời điểm update UWB cuối cùng
  static const unsigned long UWB_TIMEOUT_MS = 3000;  // 3 giây không có UWB → cảnh báo

  // Scalar Kalman measurement update. P is copied first so every element of
  // the covariance update is calculated from the same prior matrix.
  void updateScalarMeasurement(int stateIndex, float measurement,
                               float measurementVariance) {
    if (!isfinite(measurement) || !isfinite(measurementVariance) ||
        measurementVariance <= 0.0f) {
      return;
    }

    float innovationVariance = P[stateIndex][stateIndex] + measurementVariance;
    if (!isfinite(innovationVariance) || innovationVariance <= 1e-8f) {
      return;
    }

    float P_old[6][6];
    float gain[6];
    for (int i = 0; i < 6; i++) {
      gain[i] = P[i][stateIndex] / innovationVariance;
      for (int j = 0; j < 6; j++) {
        P_old[i][j] = P[i][j];
      }
    }

    float residual = measurement - state[stateIndex];
    for (int i = 0; i < 6; i++) {
      state[i] += gain[i] * residual;
    }

    float P_new[6][6];
    for (int i = 0; i < 6; i++) {
      for (int j = 0; j < 6; j++) {
        P_new[i][j] = P_old[i][j] - gain[i] * P_old[stateIndex][j];
      }
    }

    for (int i = 0; i < 6; i++) {
      P[i][i] = P_new[i][i];
      for (int j = i + 1; j < 6; j++) {
        P[i][j] = P[j][i] = 0.5f * (P_new[i][j] + P_new[j][i]);
      }
    }
  }

public:
  FusionKalmanFilter() {
    reset();
  }

  void reset() {
    for (int i = 0; i < 6; i++) {
      state[i] = 0;
      for (int j = 0; j < 6; j++) {
        P[i][j] = 0;
      }
    }
    // Initial uncertainty
    P[0][0] = 1.0;   P[1][1] = 1.0;   // Position: moderate uncertainty
    P[2][2] = 10.0;  P[3][3] = 10.0;  // Velocity: high uncertainty
    P[4][4] = 5.0;   P[5][5] = 5.0;   // Acceleration: moderate uncertainty

    // Tuning parameters
    Q_pos = 0.01;
    Q_vel = 0.1;
    Q_acc = 1.0;

    nlos_threshold = 0.5;  // > 0.5m residual → nghi NLOS

    // Measurement variances (sigma squared).
    R_uwb_base = 0.05f * 0.05f;  // LOS UWB sigma = 5 cm
    R_uwb_nlos = 1.0f * 1.0f;    // NLOS UWB sigma = 1 m
    R_imu_acc = 0.15f * 0.15f;   // World-frame IMU sigma (m/s^2)

    initialized = false;
    last_predict_time = 0;
    last_uwb_time = 0;
  }

  // --------------------------------------------------------
  // PREDICT với gia tốc IMU (chạy ở 100Hz)
  // Mô hình: x = x + vx*dt + 0.5*ax*dt²
  //           vx = vx + ax*dt
  //           ax = ax (constant acceleration between updates)
  // --------------------------------------------------------
  void predictWithIMU(float ax_imu, float ay_imu) {
    if (!initialized) return;
    if (!isfinite(ax_imu) || !isfinite(ay_imu)) return;

    unsigned long now = millis();
    float dt = (now - last_predict_time) / 1000.0f;
    last_predict_time = now;

    // A stalled loop invalidates the integration interval. last_predict_time is
    // already reset above, so skip this stale sample and resume next cycle.
    if (dt > 0.5f) return;
    if (dt < 0.001f) return;      // Quá nhanh, bỏ qua

    float dt2 = dt * dt;
    float dt3 = dt2 * dt;
    float half_dt2 = 0.5f * dt2;

    // --- State prediction ---
    // Dùng gia tốc IMU trực tiếp thay vì giả định constant velocity
    // IMU acceleration is a measurement of the acceleration states, not an
    // assignment. This keeps the state and covariance models consistent.
    updateScalarMeasurement(4, ax_imu, R_imu_acc);
    updateScalarMeasurement(5, ay_imu, R_imu_acc);

    state[0] += state[2] * dt + half_dt2 * state[4];  // x
    state[1] += state[3] * dt + half_dt2 * state[5];  // y
    state[2] += state[4] * dt;                        // vx
    state[3] += state[5] * dt;                        // vy
    // state[4] and state[5] are updated via updateScalarMeasurement above.

    // --- Covariance prediction: P = F*P*F' + Q ---
    // State transition F (implicit):
    // [1  0  dt 0  dt²/2  0   ]
    // [0  1  0  dt 0      dt²/2]
    // [0  0  1  0  dt     0   ]
    // [0  0  0  1  0      dt  ]
    // [0  0  0  0  1      0   ]
    // [0  0  0  0  0      1   ]

    // Simplified covariance propagation (keep computation light for ESP32)
    // Chỉ cập nhật diagonal + cross-terms quan trọng nhất
    float P_new[6][6];

    // Full F*P*F' cho block X (indices 0,2,4) — tương tự cho Y (1,3,5)
    // Nhưng để tiết kiệm CPU, ta dùng simplified version

    // Copy P hiện tại
    for (int i = 0; i < 6; i++)
      for (int j = 0; j < 6; j++)
        P_new[i][j] = P[i][j];

    // Propagate position uncertainty (x-block: indices 0,2,4)
    P_new[0][0] = P[0][0] + 2*dt*P[0][2] + dt2*P[2][2] + dt2*P[0][4] + dt3*P[2][4] + 0.25f*dt2*dt2*P[4][4];
    P_new[0][2] = P[0][2] + dt*(P[2][2] + P[0][4]) +
                  1.5f*dt2*P[2][4] + 0.5f*dt3*P[4][4];
    P_new[2][0] = P_new[0][2];
    P_new[2][2] = P[2][2] + 2*dt*P[2][4] + dt2*P[4][4];
    P_new[0][4] = P[0][4] + dt*P[2][4] + half_dt2*P[4][4];
    P_new[4][0] = P_new[0][4];
    P_new[2][4] = P[2][4] + dt*P[4][4];
    P_new[4][2] = P_new[2][4];

    // Propagate position uncertainty (y-block: indices 1,3,5)
    P_new[1][1] = P[1][1] + 2*dt*P[1][3] + dt2*P[3][3] + dt2*P[1][5] + dt3*P[3][5] + 0.25f*dt2*dt2*P[5][5];
    P_new[1][3] = P[1][3] + dt*(P[3][3] + P[1][5]) +
                  1.5f*dt2*P[3][5] + 0.5f*dt3*P[5][5];
    P_new[3][1] = P_new[1][3];
    P_new[3][3] = P[3][3] + 2*dt*P[3][5] + dt2*P[5][5];
    P_new[1][5] = P[1][5] + dt*P[3][5] + half_dt2*P[5][5];
    P_new[5][1] = P_new[1][5];
    P_new[3][5] = P[3][5] + dt*P[5][5];
    P_new[5][3] = P_new[3][5];

    // Add process noise Q
    P_new[0][0] += Q_pos * dt2;
    P_new[1][1] += Q_pos * dt2;
    P_new[2][2] += Q_vel * dt;
    P_new[3][3] += Q_vel * dt;
    P_new[4][4] += Q_acc * dt;
    P_new[5][5] += Q_acc * dt;

    // Copy back + enforce symmetry (chống numerical drift)
    for (int i = 0; i < 6; i++) {
      P[i][i] = P_new[i][i];
      for (int j = i + 1; j < 6; j++) {
        P[i][j] = P[j][i] = 0.5f * (P_new[i][j] + P_new[j][i]);
      }
    }
  }

  // --------------------------------------------------------
  // UPDATE với phép đo UWB (chạy ở ~10Hz)
  // Observation: z = [x_uwb, y_uwb]
  // H = [1 0 0 0 0 0]
  //     [0 1 0 0 0 0]
  // --------------------------------------------------------
  void updateWithUWB(float meas_x, float meas_y) {
    if (!isfinite(meas_x) || !isfinite(meas_y)) return;

    unsigned long now = millis();

    if (!initialized) {
      state[0] = meas_x; state[1] = meas_y;
      state[2] = 0; state[3] = 0;
      state[4] = 0; state[5] = 0;
      last_predict_time = now;
      last_uwb_time = now;
      initialized = true;
      return;
    }

    last_uwb_time = now;

    // --- NLOS Detection ---
    // So sánh vị trí predicted (từ IMU) với measurement (từ UWB)
    float residual_x = meas_x - state[0];
    float residual_y = meas_y - state[1];
    float residual_dist = sqrtf(residual_x * residual_x + residual_y * residual_y);

    // Chọn R adaptive: tăng mạnh nếu nghi NLOS
    float R_val;
    if (residual_dist > nlos_threshold) {
      R_val = R_uwb_nlos;  // NLOS: giảm lòng tin vào UWB
    } else {
      R_val = R_uwb_base;  // LOS: tin tưởng UWB
    }

    // --- Kalman Update ---
    // H = [1 0 0 0 0 0; 0 1 0 0 0 0]
    // S = H*P*H' + R = P[0:2][0:2] + R*I
    float S00 = P[0][0] + R_val;
    float S01 = P[0][1];
    float S10 = P[1][0];
    float S11 = P[1][1] + R_val;

    float det = S00 * S11 - S01 * S10;
    if (!isfinite(det) || fabsf(det) < 1e-8f) return;  // Singular → skip update

    float inv_det = 1.0f / det;
    float S_inv00 =  S11 * inv_det;
    float S_inv01 = -S01 * inv_det;
    float S_inv10 = -S10 * inv_det;
    float S_inv11 =  S00 * inv_det;

    // K = P * H' * S_inv  (6x2 matrix)
    float K[6][2];
    for (int i = 0; i < 6; i++) {
      K[i][0] = P[i][0] * S_inv00 + P[i][1] * S_inv10;
      K[i][1] = P[i][0] * S_inv01 + P[i][1] * S_inv11;
    }

    // State update: x = x + K * residual
    for (int i = 0; i < 6; i++) {
      state[i] += K[i][0] * residual_x + K[i][1] * residual_y;
    }

    // Covariance update: P = (I - K*H) * P
    float P_new[6][6];
    for (int i = 0; i < 6; i++) {
      for (int j = 0; j < 6; j++) {
        P_new[i][j] = P[i][j] - (K[i][0] * P[0][j] + K[i][1] * P[1][j]);
      }
    }
    // Copy back + enforce symmetry (chống numerical drift)
    for (int i = 0; i < 6; i++) {
      P[i][i] = P_new[i][i];
      for (int j = i + 1; j < 6; j++) {
        P[i][j] = P[j][i] = 0.5f * (P_new[i][j] + P_new[j][i]);
      }
    }
  }

  // --------------------------------------------------------
  // ZUPT: Zero-Velocity Update
  // Gọi khi IMU phát hiện tag đứng yên
  // Ép vx=vy=0, ax=ay=0 và giảm uncertainty
  // --------------------------------------------------------
  void applyZUPT() {
    if (!initialized) return;

    // "Measure" velocity = 0 with very low noise
    constexpr float R_ZUPT_VELOCITY = 0.01f * 0.01f;

    updateScalarMeasurement(2, 0.0f, R_ZUPT_VELOCITY);
    updateScalarMeasurement(3, 0.0f, R_ZUPT_VELOCITY);

    // Stationary acceleration measurements suppress residual drift.
    updateScalarMeasurement(4, 0.0f, 0.05f * 0.05f);
    updateScalarMeasurement(5, 0.0f, 0.05f * 0.05f);
  }

  // Getters
  void getPosition(float* x, float* y) {
    *x = state[0];
    *y = state[1];
  }

  void getVelocity(float* vx, float* vy) {
    *vx = state[2];
    *vy = state[3];
  }

  bool isInitialized() { return initialized; }

  // Kiểm tra UWB có bị timeout không
  bool isUwbTimedOut() {
    if (!initialized) return false;
    return (millis() - last_uwb_time) > UWB_TIMEOUT_MS;
  }

  unsigned long getTimeSinceLastUwb() {
    return millis() - last_uwb_time;
  }
};

// ============================================================
// Global objects
// ============================================================
FusionKalmanFilter fusionKalman;
IMUDriver imu;

// IMU timing
unsigned long lastIMUReadTime = 0;

// ============================================================
// Hàm giải mã dữ liệu UART từ BU03 (giữ nguyên)
// ============================================================
bool parseUwbData(const uint8_t* data, size_t dataLen, float* distances) {
  if (dataLen != UWB_FRAME_LENGTH) return false;
  if (data[0] != 0xaa || data[1] != 0x25 || data[2] != 0x01) return false;

  for (int i = 0; i < 3; i++) {
    int offset = 3 + (i * 4);
    uint16_t dist_raw = data[offset] | (static_cast<uint16_t>(data[offset + 1]) << 8);
    float distance = (dist_raw / 1000.0f) + distance_offsets[i];

    if (!isfinite(distance) || distance < MIN_UWB_DISTANCE_M ||
        distance > MAX_UWB_DISTANCE_M) {
      return false;
    }
    distances[i] = distance;
  }
  return true;
}

// ============================================================
// Thuật toán Tam giác đạc - Trilateration (giữ nguyên)
// ============================================================
bool trilaterate2d(float* distances, float* x, float* y) {
  struct ValidData { float x, y, dist; };
  ValidData valid_data[3];
  int valid_count = 0;

  for (int i = 0; i < 3; i++) {
    if (isfinite(distances[i]) && distances[i] >= MIN_UWB_DISTANCE_M &&
        distances[i] <= MAX_UWB_DISTANCE_M) {
      valid_data[valid_count] = {base_stations[i].x, base_stations[i].y, distances[i]};
      valid_count++;
    }
  }

  if (valid_count < 3) return false;

  float x1 = valid_data[0].x, y1 = valid_data[0].y, r1 = valid_data[0].dist;
  float A[2][2], b[2];

  for (int i = 0; i < 2; i++) {
    float xi = valid_data[i+1].x;
    float yi = valid_data[i+1].y;
    float ri = valid_data[i+1].dist;

    A[i][0] = 2 * (xi - x1);
    A[i][1] = 2 * (yi - y1);
    b[i] = (ri * ri) - (r1 * r1) - (xi * xi) + (x1 * x1) - (yi * yi) + (y1 * y1);
  }

  float det = A[0][0] * A[1][1] - A[0][1] * A[1][0];
  if (fabsf(det) < 1e-6f) return false;

  *x = -(b[0] * A[1][1] - b[1] * A[0][1]) / det;
  *y = -(A[0][0] * b[1] - A[1][0] * b[0]) / det;

  // A corrupted frame can still produce a mathematical intersection. Reject
  // solutions that do not fit the three measured circles well enough.
  float residualSqSum = 0.0f;
  for (int i = 0; i < valid_count; i++) {
    float dx = *x - valid_data[i].x;
    float dy = *y - valid_data[i].y;
    float rangeResidual = sqrtf(dx * dx + dy * dy) - valid_data[i].dist;
    residualSqSum += rangeResidual * rangeResidual;
  }
  float rmsResidual = sqrtf(residualSqSum / valid_count);
  if (!isfinite(rmsResidual) ||
      rmsResidual > MAX_TRILATERATION_RMS_RESIDUAL_M) {
    return false;
  }

  return true;
}

// ============================================================
// SETUP
// ============================================================
void setup() {
  Serial.begin(115200);
  Serial2.begin(115200, SERIAL_8N1, RXD2, TXD2);

  Serial.println();
  Serial.println("====================================");
  Serial.println("  UWB + IMU Fusion Positioning v2.0");
  Serial.println("====================================");

  // Khởi tạo IMU (BMI270)
  if (!imu.init(IMU_SDA_PIN, IMU_SCL_PIN)) {
    Serial.println("[WARN] IMU init failed! Running UWB-only mode.");
  } else {
    // Calibrate: giữ tag đứng yên ~3 giây
    if (!imu.calibrate(300)) {
      Serial.println("[WARN] IMU calibration failed! Running UWB-only mode.");
    }
  }

  // Header cho Serial Plotter/CSV
  Serial.println("x_fused,y_fused,uwb_age_s");

  lastIMUReadTime = millis();
}

// ============================================================
// MAIN LOOP — Dual-Rate Processing
//   IMU:  100Hz (mỗi 10ms) → predict
//   UWB:  ~10Hz (khi có frame) → update + correct
// ============================================================
void loop() {
  // --- UWB UART variables ---
  static uint8_t buffer[UWB_FRAME_LENGTH];
  static size_t bufferIndex = 0;
  static bool messageStarted = false;
  static unsigned long lastByteTime = 0;

  unsigned long currentTime = millis();

  // ========================================
  // [1] ĐỌC IMU @ 100Hz (mỗi 10ms)
  // ========================================
  if (currentTime - lastIMUReadTime >= IMU_UPDATE_MS) {
    float dt_imu = (currentTime - lastIMUReadTime) / 1000.0f;
    lastIMUReadTime = currentTime;

    float ax_world, ay_world;
    if (imu.getWorldAccel(&ax_world, &ay_world, dt_imu)) {
      constexpr float DEGREES_TO_RADIANS = 0.01745329251994329577f;
      const float yawOffsetRad = IMU_TO_UWB_YAW_OFFSET_DEG * DEGREES_TO_RADIANS;
      const float cosYaw = cosf(yawOffsetRad);
      const float sinYaw = sinf(yawOffsetRad);
      float ax_uwb = cosYaw * ax_world - sinYaw * ay_world;
      float ay_uwb = sinYaw * ax_world + cosYaw * ay_world;

      // ZUPT: nếu đứng yên → ép vận tốc = 0, chống drift
      if (imu.isStationary()) {
        ax_uwb = 0;
        ay_uwb = 0;
        fusionKalman.applyZUPT();
      }

      // Predict: đẩy vị trí dựa trên gia tốc IMU
      fusionKalman.predictWithIMU(ax_uwb, ay_uwb);
    }
  }

  // ========================================
  // [2] ĐỌC UWB UART (event-driven, ~10Hz)
  // ========================================
  while (Serial2.available()) {
    uint8_t incomingByte = Serial2.read();
    currentTime = millis();

    // Reset buffer nếu timeout giữa chừng
    if (messageStarted && (currentTime - lastByteTime > UART_TIMEOUT_MS)) {
      messageStarted = false;
      bufferIndex = 0;
    }
    lastByteTime = currentTime;

    // Reject a malformed header as soon as it is detected. If this byte is a
    // new start marker, the normal code below immediately starts a new frame.
    if (messageStarted &&
        ((bufferIndex == 1 && incomingByte != 0x25) ||
         (bufferIndex == 2 && incomingByte != 0x01))) {
      messageStarted = false;
      bufferIndex = 0;
    }

    if (!messageStarted && incomingByte == 0xAA) {
      messageStarted = true;
      bufferIndex = 0;
      buffer[bufferIndex++] = incomingByte;
    }
    else if (messageStarted) {
      buffer[bufferIndex++] = incomingByte;

      if (bufferIndex >= UWB_FRAME_LENGTH) {
        float distances[3];
        if (parseUwbData(buffer, bufferIndex, distances)) {
          float x_raw, y_raw;
          bool valid = trilaterate2d(distances, &x_raw, &y_raw);

          if (valid) {
            // Update Kalman với phép đo UWB (có NLOS detection tự động)
            fusionKalman.updateWithUWB(x_raw, y_raw);
          }
          // Nếu không valid (< 3 trạm): IMU predict tiếp tục hoạt động
        }
        messageStarted = false;
        bufferIndex = 0;
      }

      // Buffer overflow protection
      if (bufferIndex >= UWB_FRAME_LENGTH) {
        messageStarted = false;
        bufferIndex = 0;
      }
    }
  }

  // ========================================
  // [3] OUTPUT vị trí fusion (mỗi lần có IMU update)
  // ========================================
  if (fusionKalman.isInitialized()) {
    static unsigned long lastPrintTime = 0;
    // In ra 20Hz (mỗi 50ms) để không spam Serial quá nhanh
    unsigned long outputTime = millis();
    if (outputTime - lastPrintTime >= 50) {
      lastPrintTime = outputTime;

      float x_fused, y_fused;
      fusionKalman.getPosition(&x_fused, &y_fused);

      Serial.print(x_fused, 3);
      Serial.print(",");
      Serial.print(y_fused, 3);
      Serial.print(",");
      Serial.print(fusionKalman.getTimeSinceLastUwb() / 1000.0f, 3);

      // uwb_age_s is retained as numeric data for Serial Plotter and CSV tools.

      Serial.println();
    }
  }

  yield();  // Nhường CPU cho FreeRTOS tasks khác
}
