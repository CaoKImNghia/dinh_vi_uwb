#ifndef IMU_DRIVER_H
#define IMU_DRIVER_H

#include "SparkFun_BMI270_Arduino_Library.h"
#include <Wire.h>
#include <math.h>

// ============================================================
// Madgwick Filter — Lightweight orientation filter
// Converts raw accel+gyro → quaternion → rotation matrix
// ============================================================
class MadgwickFilter {
private:
  float q0, q1, q2, q3; // Quaternion
  float beta;           // Filter gain (convergence rate)

public:
  MadgwickFilter() : q0(1.0f), q1(0.0f), q2(0.0f), q3(0.0f), beta(0.1f) {}

  void setBeta(float b) { beta = b; }

  void update(float gx, float gy, float gz, float ax, float ay, float az,
              float dt) {
    // A zero/invalid time step cannot produce a meaningful orientation update.
    // Guarding it also prevents a corrupted scheduler timestamp from poisoning
    // the quaternion.
    if (!isfinite(dt) || dt <= 0.0f)
      return;

    float recipNorm;

    // Normalize accelerometer
    float normA = sqrtf(ax * ax + ay * ay + az * az);
    if (normA < 0.01f)
      return; // Freefall guard
    recipNorm = 1.0f / normA;
    ax *= recipNorm;
    ay *= recipNorm;
    az *= recipNorm;

    // Gradient descent step (simplified Madgwick)
    float _2q0 = 2.0f * q0;
    float _2q1 = 2.0f * q1;
    float _2q2 = 2.0f * q2;
    float _2q3 = 2.0f * q3;
    float _4q0 = 4.0f * q0;
    float _4q1 = 4.0f * q1;
    float _4q2 = 4.0f * q2;
    float _8q1 = 8.0f * q1;
    float _8q2 = 8.0f * q2;
    float q0q0 = q0 * q0;
    float q1q1 = q1 * q1;
    float q2q2 = q2 * q2;
    float q3q3 = q3 * q3;

    // Gradient
    float s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
    float s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * q1 - _2q0 * ay - _4q1 +
               _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
    float s2 = 4.0f * q0q0 * q2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 +
               _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
    float s3 = 4.0f * q1q1 * q3 - _2q1 * ax + 4.0f * q2q2 * q3 - _2q2 * ay;

    // At a perfectly level, stationary pose the gradient is exactly zero.
    // Normalising it would divide by zero and turn the quaternion into NaN.
    // In that case use the gyro-only integration for this sample instead.
    float stepNormSq = s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3;
    if (isfinite(stepNormSq) && stepNormSq > 1e-12f) {
      recipNorm = 1.0f / sqrtf(stepNormSq);
      s0 *= recipNorm;
      s1 *= recipNorm;
      s2 *= recipNorm;
      s3 *= recipNorm;
    } else {
      s0 = 0.0f;
      s1 = 0.0f;
      s2 = 0.0f;
      s3 = 0.0f;
    }

    // Apply feedback
    float qDot0 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz) - beta * s0;
    float qDot1 = 0.5f * (q0 * gx + q2 * gz - q3 * gy) - beta * s1;
    float qDot2 = 0.5f * (q0 * gy - q1 * gz + q3 * gx) - beta * s2;
    float qDot3 = 0.5f * (q0 * gz + q1 * gy - q2 * gx) - beta * s3;

    // Integrate
    q0 += qDot0 * dt;
    q1 += qDot1 * dt;
    q2 += qDot2 * dt;
    q3 += qDot3 * dt;

    // Normalize quaternion
    float quaternionNormSq = q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3;
    if (isfinite(quaternionNormSq) && quaternionNormSq > 1e-12f) {
      recipNorm = 1.0f / sqrtf(quaternionNormSq);
      q0 *= recipNorm;
      q1 *= recipNorm;
      q2 *= recipNorm;
      q3 *= recipNorm;
    } else {
      // Recover deterministically instead of letting a bad sample propagate.
      q0 = 1.0f;
      q1 = 0.0f;
      q2 = 0.0f;
      q3 = 0.0f;
    }
  }

  // Rotate body-frame acceleration to world frame and remove gravity
  // Returns linear acceleration in m/s² (world frame, gravity-free)
  void getWorldLinearAccel(float ax_body, float ay_body, float az_body,
                           float *ax_world, float *ay_world) {
    // Rotation matrix from quaternion (only rows 0,1 needed for 2D)
    float r00 = 1.0f - 2.0f * (q2 * q2 + q3 * q3);
    float r01 = 2.0f * (q1 * q2 - q0 * q3);
    float r02 = 2.0f * (q1 * q3 + q0 * q2);

    float r10 = 2.0f * (q1 * q2 + q0 * q3);
    float r11 = 1.0f - 2.0f * (q1 * q1 + q3 * q3);
    float r12 = 2.0f * (q2 * q3 - q0 * q1);

    // Rotate accel to world frame
    float aw_x = r00 * ax_body + r01 * ay_body + r02 * az_body;
    float aw_y = r10 * ax_body + r11 * ay_body + r12 * az_body;
    // Z component: r20*ax + r21*ay + r22*az - 9.81 (gravity)
    // We only need X, Y for 2D positioning

    *ax_world = aw_x; // m/s², gravity component already minimal in X
    *ay_world = aw_y; // m/s², gravity component already minimal in Y
  }
};

// ============================================================
// IMU Driver — Bosch BMI270 via SparkFun Library (I2C)
// Drop-in replacement for MPU6050 driver
// Requires: SparkFun BMI270 Arduino Library
//   Install: Arduino IDE → Library Manager → "SparkFun BMI270"
// ============================================================
class IMUDriver {
private:
  BMI270 sensor; // SparkFun BMI270 object

  // Calibration offsets (in m/s² and rad/s)
  float accel_offset[3] = {0, 0, 0};
  float gyro_offset[3] = {0, 0, 0};

  // SparkFun BMI270 library returns:
  //   accel in g  → multiply by 9.81 to get m/s²
  //   gyro  in °/s → multiply by π/180 to get rad/s
  static constexpr float G_TO_MS2 = 9.81f;
  static constexpr float GYRO_DEG_TO_RAD = M_PI / 180.0f;

  // Processed readings (m/s² and rad/s after conversion)
  float ax, ay, az;
  float gx, gy, gz;

  // Madgwick filter for orientation
  MadgwickFilter madgwick;

  // ZUPT (Zero-Velocity Update) detection
  static const int ZUPT_WINDOW = 20; // 20 samples @ 100Hz = 200ms
  int stationary_count = 0;
  static constexpr float ZUPT_ACCEL_THRESH =
      0.3f; // m/s² deviation from gravity
  static constexpr float ZUPT_GYRO_THRESH = 0.087f; // ~5°/s in rad/s

  bool _ready = false;

  // I2C address (0x68 when SDO→GND, 0x69 when SDO→VDDIO)
  uint8_t _i2cAddr = BMI2_I2C_PRIM_ADDR; // 0x68

public:
  IMUDriver() {}

  // Initialize BMI270: ±4g accel, ±500°/s gyro
  // BMI270 firmware blob (~8KB) is loaded automatically by SparkFun library
  bool init(int sda_pin = 21, int scl_pin = 22) {
    _ready = false;
    Wire.begin(sda_pin, scl_pin);
    Wire.setClock(400000); // 400kHz I2C fast mode

    // beginI2C() loads the BMI270 config file and initializes the sensor
    int8_t err = sensor.beginI2C(_i2cAddr);
    if (err != BMI2_OK) {
      Serial.print("[IMU] ERROR: BMI270 init failed! Code: ");
      Serial.println(err);
      return false;
    }

    // Configure accelerometer: ±4g range, ODR 100Hz, normal mode
    // BMI270 default is ±8g; we set ±4g for better resolution
    bmi2_sens_config configs[2];
    configs[0].type = BMI2_ACCEL;
    configs[1].type = BMI2_GYRO;
    err = sensor.getConfigs(configs, 2);
    if (err == BMI2_OK) {
      configs[0].cfg.acc.range = BMI2_ACC_RANGE_4G;
      configs[0].cfg.acc.odr = BMI2_ACC_ODR_100HZ;
      configs[1].cfg.gyr.range = BMI2_GYR_RANGE_500;
      configs[1].cfg.gyr.odr = BMI2_GYR_ODR_100HZ;
      err = sensor.setConfigs(configs, 2);
    }

    // Configure gyroscope: ±500°/s range, ODR 100Hz, normal mode

    if (err != BMI2_OK) {
      Serial.print("[IMU] ERROR: BMI270 configuration failed! Code: ");
      Serial.println(err);
      return false;
    }

    delay(50);

    Serial.println("[IMU] BMI270 initialized OK");
    return true;
  }

  // Calibrate: tag must be STATIONARY for ~3 seconds
  // Measures gyro drift and accel bias (assumes Z = +1g)
  bool calibrate(int num_samples = 300) {
    Serial.println("[IMU] Calibrating... keep tag still!");

    if (num_samples <= 0) {
      _ready = false;
      return false;
    }

    // Recalibration must use uncorrected readings, not a previous bias.
    for (int axis = 0; axis < 3; axis++) {
      accel_offset[axis] = 0.0f;
      gyro_offset[axis] = 0.0f;
    }
    stationary_count = 0;

    float sum_ax = 0, sum_ay = 0, sum_az = 0;
    float sum_gx = 0, sum_gy = 0, sum_gz = 0;
    int valid_samples = 0;

    for (int i = 0; i < num_samples; i++) {
      if (readRaw()) {
        sum_ax += ax;
        sum_ay += ay;
        sum_az += az;
        sum_gx += gx;
        sum_gy += gy;
        sum_gz += gz;
        valid_samples++;
      }
      delay(10); // Match the configured 100 Hz sensor output rate.
    }

    if (valid_samples == 0 || valid_samples * 2 < num_samples) {
      Serial.println("[IMU] ERROR: calibration failed; too many I2C read errors.");
      _ready = false;
      stationary_count = 0;
      return false;
    }

    // Gyro offset: should be 0 when stationary
    gyro_offset[0] = sum_gx / valid_samples;
    gyro_offset[1] = sum_gy / valid_samples;
    gyro_offset[2] = sum_gz / valid_samples;

    // Accel offset: X,Y should be 0, Z should be 9.81
    accel_offset[0] = sum_ax / valid_samples;
    accel_offset[1] = sum_ay / valid_samples;
    accel_offset[2] = (sum_az / valid_samples) - 9.81f; // Remove gravity from Z

    _ready = true;

    Serial.print("[IMU] Calibration done. Gyro offsets: ");
    Serial.print(gyro_offset[0] * 180.0f / M_PI, 3);
    Serial.print(", ");
    Serial.print(gyro_offset[1] * 180.0f / M_PI, 3);
    Serial.print(", ");
    Serial.print(gyro_offset[2] * 180.0f / M_PI, 3);
    Serial.println(" deg/s");
    Serial.print("[IMU] Accel offsets: ");
    Serial.print(accel_offset[0], 4);
    Serial.print(", ");
    Serial.print(accel_offset[1], 4);
    Serial.print(", ");
    Serial.print(accel_offset[2], 4);
    Serial.println(" m/s^2");
    return true;
  }

  // Read sensor data from BMI270, convert to m/s² and rad/s, apply calibration
  bool readRaw() {
    int8_t err = sensor.getSensorData();
    if (err != BMI2_OK) {
      return false;
    }

    // Convert from g → m/s² and apply calibration offset
    ax = sensor.data.accelX * G_TO_MS2 - accel_offset[0];
    ay = sensor.data.accelY * G_TO_MS2 - accel_offset[1];
    az = sensor.data.accelZ * G_TO_MS2 - accel_offset[2];

    // Convert from °/s → rad/s and apply calibration offset
    gx = sensor.data.gyroX * GYRO_DEG_TO_RAD - gyro_offset[0];
    gy = sensor.data.gyroY * GYRO_DEG_TO_RAD - gyro_offset[1];
    gz = sensor.data.gyroZ * GYRO_DEG_TO_RAD - gyro_offset[2];
    return isfinite(ax) && isfinite(ay) && isfinite(az) &&
           isfinite(gx) && isfinite(gy) && isfinite(gz);
  }

  // Get world-frame linear acceleration (gravity removed)
  // This is the main function called by the fusion filter
  bool getWorldAccel(float *ax_world, float *ay_world, float dt) {
    *ax_world = 0;
    *ay_world = 0;
    if (!_ready) {
      return false;
    }

    if (!readRaw()) {
      stationary_count = 0;
      return false;
    }

    // Update Madgwick orientation filter
    madgwick.update(gx, gy, gz, ax, ay, az, dt);

    // Rotate body accel to world frame & remove gravity
    madgwick.getWorldLinearAccel(ax, ay, az, ax_world, ay_world);
    if (!isfinite(*ax_world) || !isfinite(*ay_world)) {
      *ax_world = 0;
      *ay_world = 0;
      stationary_count = 0;
      return false;
    }

    // Update ZUPT detector
    float accel_mag = sqrtf(ax * ax + ay * ay + az * az);
    float gyro_mag = fabsf(gx) + fabsf(gy) + fabsf(gz);

    if (fabsf(accel_mag - 9.81f) < ZUPT_ACCEL_THRESH &&
        gyro_mag < ZUPT_GYRO_THRESH) {
      if (stationary_count < ZUPT_WINDOW + 10)
        stationary_count++;
    } else {
      stationary_count = 0;
    }
    return true;
  }

  // Check if tag is stationary (for ZUPT)
  bool isStationary() { return stationary_count >= ZUPT_WINDOW; }

  bool isReady() { return _ready; }

  // Debug: get raw values for testing
  void getRawValues(float *out_ax, float *out_ay, float *out_az, float *out_gx,
                    float *out_gy, float *out_gz) {
    *out_ax = ax;
    *out_ay = ay;
    *out_az = az;
    *out_gx = gx;
    *out_gy = gy;
    *out_gz = gz;
  }
};

#endif // IMU_DRIVER_H
