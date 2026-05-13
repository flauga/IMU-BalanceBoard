#include "imu_driver.h"
#include "config.h"
#include <Arduino.h>
#include <Wire.h>

bool IMUDriver::begin() {
    Wire.begin();
    Wire.setClock(400000);

    for (uint8_t attempt = 0; attempt < IMU_INIT_MAX_RETRIES; attempt++) {
        if (imu_.begin(LSM6DSO_I2C_ADDR, Wire)) {
            // initialize() calls setIncrement() then applies settings to hardware
            imu_.initialize(BASIC_SETTINGS);   // calls setIncrement(), baseline settings
            // Override to 208 Hz and ±4g — higher ODR feeds Mahony more often,
            // reducing angle noise and improving step response.
            imu_.setAccelRange(4);
            imu_.setAccelDataRate(208);
            imu_.setGyroDataRate(208);

            last_data_ms_ = millis();
            Serial.println("[IMU] LSM6DSO connected at 208 Hz");
            return true;
        }
        Serial.printf("[IMU] Init attempt %d/%d failed\n", attempt + 1, IMU_INIT_MAX_RETRIES);
        Wire.end();
        delay(50);
        Wire.begin();
        Wire.setClock(400000);
        delay(IMU_INIT_RETRY_DELAY_MS);
    }

    Serial.println("[IMU] ERROR: LSM6DSO init failed after all retries");
    return false;
}

bool IMUDriver::update() {
    // Gate on data-ready flag (STATUS_REG bit0=XLDA, bit1=GDA)
    uint8_t status = imu_.listenDataReady();
    if (!(status & 0x01)) return false;  // accel not ready yet

    constexpr float G          = 9.80665f;
    constexpr float DPS_TO_RAD = 3.14159265f / 180.0f;

    ax_ = imu_.readFloatAccelX() * G;
    ay_ = imu_.readFloatAccelY() * G;
    az_ = imu_.readFloatAccelZ() * G;
    gx_ = imu_.readFloatGyroX() * DPS_TO_RAD - gx_bias_;
    gy_ = imu_.readFloatGyroY() * DPS_TO_RAD - gy_bias_;
    gz_ = imu_.readFloatGyroZ() * DPS_TO_RAD - gz_bias_;

    new_data_     = true;
    last_data_ms_ = millis();
    return true;
}

bool IMUDriver::calibrateGyro(uint32_t duration_ms) {
    constexpr float DPS_TO_RAD = 3.14159265f / 180.0f;

    // Discard any stale gyro bias from a prior call so we average raw samples.
    gx_bias_ = gy_bias_ = gz_bias_ = 0.0f;

    double sx = 0.0, sy = 0.0, sz = 0.0;
    uint32_t n = 0;
    uint32_t start = millis();
    while (millis() - start < duration_ms) {
        uint8_t status = imu_.listenDataReady();
        if (status & 0x02) {  // gyro data ready
            sx += imu_.readFloatGyroX() * DPS_TO_RAD;
            sy += imu_.readFloatGyroY() * DPS_TO_RAD;
            sz += imu_.readFloatGyroZ() * DPS_TO_RAD;
            n++;
        }
    }

    if (n < 20) {
        Serial.println("[IMU] Gyro calibration: too few samples, skipping");
        return false;
    }

    gx_bias_ = (float)(sx / n);
    gy_bias_ = (float)(sy / n);
    gz_bias_ = (float)(sz / n);

    // Sanity check: a still board should be well under 10°/s ≈ 0.175 rad/s.
    constexpr float MAX_PLAUSIBLE = 0.175f;
    if (fabsf(gx_bias_) > MAX_PLAUSIBLE ||
        fabsf(gy_bias_) > MAX_PLAUSIBLE ||
        fabsf(gz_bias_) > MAX_PLAUSIBLE) {
        Serial.printf("[IMU] Gyro bias implausible (%.3f, %.3f, %.3f rad/s) — board moved during calibration?\n",
                      gx_bias_, gy_bias_, gz_bias_);
        gx_bias_ = gy_bias_ = gz_bias_ = 0.0f;
        return false;
    }

    Serial.printf("[IMU] Gyro bias: %.4f, %.4f, %.4f rad/s (n=%lu)\n",
                  gx_bias_, gy_bias_, gz_bias_, (unsigned long)n);
    last_data_ms_ = millis();
    return true;
}
