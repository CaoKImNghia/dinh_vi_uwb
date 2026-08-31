#include <Arduino.h>
#include "imu_driver.h"

#define RXD2 16 // Nối với chân TX của STM32 trên BU03
#define TXD2 17 // Nối với chân RX của STM32 trên BU03

// Cấu hình hệ thống
#define Y_OFFSET 0.35        // Độ lệch trục Y cần bù trừ ở đầu ra
#define UART_TIMEOUT_MS 50   // Thời gian chờ tối đa cho 1 frame (chống kẹt buffer)
#define IMU_UPDATE_MS 10     // Chu kỳ đọc IMU = 10ms → 100Hz
#define IMU_SDA_PIN 21       // I2C SDA cho BMI270
#define IMU_SCL_PIN 22       // I2C SCL cho BMI270

// --- Tọa độ các Trạm gốc (Base Stations) ---
struct Position {
  float x, y;
};

Position* base0 = new Position{0, 0.7};
Position* base1 = new Position{6, 6.942};
Position* base2 = new Position{6, 0};

Position* base_stations[3] = {base0, base1, base2};

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
  float R_uwb_base;   // Base UWB measurement noise
  float R_uwb_nlos;   // NLOS UWB measurement noise (cao hơn)

  // NLOS detection threshold
  float nlos_threshold;

  // Timing
  unsigned long last_predict_time;
  bool initialized;

  // Thống kê UWB
  unsigned long last_uwb_time;          // Thời điểm update UWB cuối cùng
  static const unsigned long UWB_TIMEOUT_MS = 3000;  // 3 giây không có UWB → cảnh báo

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

    R_uwb_base = 0.05;    // UWB rất chính xác khi LOS (0.05-0.06m theo paper)
    R_uwb_nlos = 1.0;     // Khi nghi NLOS, giảm tin tưởng UWB ×20
    nlos_threshold = 0.5;  // > 0.5m residual → nghi NLOS

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

    unsigned long now = millis();
    float dt = (now - last_predict_time) / 1000.0f;
    last_predict_time = now;

    if (dt > 0.5f) dt = 0.01f;   // Clamp: quá lâu thì dùng default
    if (dt < 0.001f) return;      // Quá nhanh, bỏ qua

    float dt2 = dt * dt;
    float dt3 = dt2 * dt;
    float half_dt2 = 0.5f * dt2;

    // --- State prediction ---
    // Dùng gia tốc IMU trực tiếp thay vì giả định constant velocity
    state[0] += state[2] * dt + half_dt2 * ax_imu;  // x
    state[1] += state[3] * dt + half_dt2 * ay_imu;  // y
    state[2] += ax_imu * dt;                          // vx
    state[3] += ay_imu * dt;                          // vy
    state[4] = ax_imu;                                // ax (cập nhật từ IMU)
    state[5] = ay_imu;                                // ay

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
    P_new[0][2] = P[0][2] + dt*P[2][2] + half_dt2*P[2][4] + dt*P[0][4] + dt2*P[4][4]*half_dt2;
    P_new[2][0] = P_new[0][2];
    P_new[2][2] = P[2][2] + 2*dt*P[2][4] + dt2*P[4][4];
    P_new[0][4] = P[0][4] + dt*P[2][4] + half_dt2*P[4][4];
    P_new[4][0] = P_new[0][4];
    P_new[2][4] = P[2][4] + dt*P[4][4];
    P_new[4][2] = P_new[2][4];

    // Propagate position uncertainty (y-block: indices 1,3,5)
    P_new[1][1] = P[1][1] + 2*dt*P[1][3] + dt2*P[3][3] + dt2*P[1][5] + dt3*P[3][5] + 0.25f*dt2*dt2*P[5][5];
    P_new[1][3] = P[1][3] + dt*P[3][3] + half_dt2*P[3][5] + dt*P[1][5] + dt2*P[5][5]*half_dt2;
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

    // Copy back
    for (int i = 0; i < 6; i++)
      for (int j = 0; j < 6; j++)
        P[i][j] = P_new[i][j];
  }

  // --------------------------------------------------------
  // UPDATE với phép đo UWB (chạy ở ~10Hz)
  // Observation: z = [x_uwb, y_uwb]
  // H = [1 0 0 0 0 0]
  //     [0 1 0 0 0 0]
  // --------------------------------------------------------
  void updateWithUWB(float meas_x, float meas_y) {
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
    if (fabsf(det) < 1e-8f) return;  // Singular → skip update

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
    for (int i = 0; i < 6; i++)
      for (int j = 0; j < 6; j++)
        P[i][j] = P_new[i][j];
  }

  // --------------------------------------------------------
  // ZUPT: Zero-Velocity Update
  // Gọi khi IMU phát hiện tag đứng yên
  // Ép vx=vy=0, ax=ay=0 và giảm uncertainty
  // --------------------------------------------------------
  void applyZUPT() {
    if (!initialized) return;

    // "Measure" velocity = 0 with very low noise
    float R_zupt = 0.01f;  // Rất tin tưởng ZUPT

    // Update velocity x
    float S_vx = P[2][2] + R_zupt;
    if (S_vx > 1e-8f) {
      float K_vx = P[2][2] / S_vx;
      float residual_vx = 0.0f - state[2];
      // Update all states correlated with vx
      for (int i = 0; i < 6; i++) {
        float Ki = P[i][2] / S_vx;
        state[i] += Ki * residual_vx;
      }
      // Update P
      for (int i = 0; i < 6; i++)
        for (int j = 0; j < 6; j++)
          P[i][j] -= (P[i][2] * P[2][j]) / S_vx;
    }

    // Update velocity y
    float S_vy = P[3][3] + R_zupt;
    if (S_vy > 1e-8f) {
      float K_vy = P[3][3] / S_vy;
      float residual_vy = 0.0f - state[3];
      for (int i = 0; i < 6; i++) {
        float Ki = P[i][3] / S_vy;
        state[i] += Ki * residual_vy;
      }
      for (int i = 0; i < 6; i++)
        for (int j = 0; j < 6; j++)
          P[i][j] -= (P[i][3] * P[3][j]) / S_vy;
    }

    // Reset acceleration state to 0
    state[4] = 0;
    state[5] = 0;
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
bool parseUwbData(uint8_t* data, int dataLen, float* distances) {
  if (dataLen < 35) return false;
  if (data[0] != 0xaa || data[1] != 0x25 || data[2] != 0x01) return false;

  for (int i = 0; i < 3; i++) {
    int offset = 3 + (i * 4);
    if (offset + 1 < dataLen) {
      uint16_t dist_raw = data[offset] | (data[offset + 1] << 8);
      distances[i] = (dist_raw / 1000.0) + distance_offsets[i];
    } else {
      distances[i] = 0;
    }
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
    if (base_stations[i] != NULL && distances[i] > 0.1) {
      valid_data[valid_count] = {base_stations[i]->x, base_stations[i]->y, distances[i]};
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
    b[i] = pow(ri, 2) - pow(r1, 2) - pow(xi, 2) + pow(x1, 2) - pow(yi, 2) + pow(y1, 2);
  }

  float det = A[0][0] * A[1][1] - A[0][1] * A[1][0];
  if (abs(det) < 1e-6) return false;

  *x = -(b[0] * A[1][1] - b[1] * A[0][1]) / det;
  *y = -(A[0][0] * b[1] - A[1][0] * b[0]) / det;

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
    imu.calibrate(500);
  }

  // Header cho Serial Plotter/CSV
  Serial.println("x_fused,y_fused");

  lastIMUReadTime = millis();
}

// ============================================================
// MAIN LOOP — Dual-Rate Processing
//   IMU:  100Hz (mỗi 10ms) → predict
//   UWB:  ~10Hz (khi có frame) → update + correct
// ============================================================
void loop() {
  // --- UWB UART variables ---
  static uint8_t buffer[256];
  static int bufferIndex = 0;
  static bool messageStarted = false;
  static unsigned long lastByteTime = 0;

  unsigned long currentTime = millis();

  // ========================================
  // [1] ĐỌC IMU @ 100Hz (mỗi 10ms)
  // ========================================
  if (currentTime - lastIMUReadTime >= IMU_UPDATE_MS) {
    float dt_imu = (currentTime - lastIMUReadTime) / 1000.0f;
    lastIMUReadTime = currentTime;

    if (imu.isReady()) {
      float ax_world, ay_world;
      imu.getWorldAccel(&ax_world, &ay_world, dt_imu);

      // ZUPT: nếu đứng yên → ép vận tốc = 0, chống drift
      if (imu.isStationary()) {
        ax_world = 0;
        ay_world = 0;
        fusionKalman.applyZUPT();
      }

      // Predict: đẩy vị trí dựa trên gia tốc IMU
      fusionKalman.predictWithIMU(ax_world, ay_world);
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

    if (!messageStarted && incomingByte == 0xAA) {
      messageStarted = true;
      bufferIndex = 0;
      buffer[bufferIndex++] = incomingByte;
    }
    else if (messageStarted) {
      buffer[bufferIndex++] = incomingByte;

      if (bufferIndex >= 35) {
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
      if (bufferIndex >= 256) {
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
    if (currentTime - lastPrintTime >= 50) {
      lastPrintTime = currentTime;

      float x_fused, y_fused;
      fusionKalman.getPosition(&x_fused, &y_fused);

      Serial.print(x_fused, 3);
      Serial.print(",");
      Serial.print(y_fused, 3);

      // Cảnh báo nếu mất UWB quá lâu
      if (fusionKalman.isUwbTimedOut()) {
        Serial.print(",IMU_ONLY(");
        Serial.print(fusionKalman.getTimeSinceLastUwb() / 1000.0f, 1);
        Serial.print("s)");
      }

      Serial.println();
    }
  }

  delay(1);  // Yield nhỏ để không chiếm hết CPU
}