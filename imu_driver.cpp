#include "imu_driver.h"
#include "config.h"
#include <Arduino.h>
#include <Wire.h>

// Seeed_Arduino_LSM6DS3 contains an internal `#define Wire Wire1` when
// ARDUINO_XIAO_MG24 is defined, and powers the IMU rail via PIN_LSM6DS3TR_C_POWER
// inside beginCore(). All we have to do is populate settings and call begin().

bool IMUDriver::begin() {
    // Configure desired settings *before* begin() — the library only applies
    // them once during begin(). Match the original ESP32 firmware: 208 Hz ODR,
    // ±4g accel, ±500 dps gyro. The driver itself handles Wire1 + power-pin.
    imu_.settings.gyroEnabled       = 1;
    imu_.settings.gyroRange         = 500;     // dps
    imu_.settings.gyroSampleRate    = 208;     // Hz
    imu_.settings.gyroBandWidth     = 100;     // Hz (LPF cutoff)
    imu_.settings.gyroFifoEnabled   = 0;
    imu_.settings.gyroFifoDecimation = 1;

    imu_.settings.accelEnabled      = 1;
    imu_.settings.accelODROff       = 1;
    imu_.settings.accelRange        = 4;       // g
    imu_.settings.accelSampleRate   = 208;     // Hz
    imu_.settings.accelBandWidth    = 100;     // Hz (LPF cutoff)
    imu_.settings.accelFifoEnabled  = 0;
    imu_.settings.accelFifoDecimation = 1;

    imu_.settings.tempEnabled       = 1;
    imu_.settings.commMode          = 1;

    for (uint8_t attempt = 0; attempt < IMU_INIT_MAX_RETRIES; attempt++) {
        if (imu_.begin() == 0) {  // status_t IMU_SUCCESS = 0
            last_data_ms_ = millis();
            Serial.println("[IMU] LSM6DS3TR-C connected at 208 Hz");
            return true;
        }
        Serial.printf("[IMU] Init attempt %d/%d failed\n",
                      attempt + 1, IMU_INIT_MAX_RETRIES);
        delay(IMU_INIT_RETRY_DELAY_MS);
    }

    Serial.println("[IMU] ERROR: LSM6DS3 init failed after all retries");
    return false;
}

// STATUS_REG (0x1E) — bit0 = XLDA (accel ready), bit1 = GDA (gyro ready)
bool IMUDriver::dataReady(uint8_t mask) const {
    uint8_t status = 0;
    // const_cast: readRegister isn't marked const but doesn't mutate state.
    const_cast<LSM6DS3&>(imu_).readRegister(&status, 0x1E);
    return (status & mask) != 0;
}

bool IMUDriver::update() {
    if (!dataReady(0x01)) return false;  // accel not ready yet

    constexpr float G          = 9.80665f;
    constexpr float DPS_TO_RAD = 3.14159265f / 180.0f;

    // The IMU is both mounted rotated 90° about the board's vertical axis AND
    // flipped front-to-back (mounted upside down). Two corrections compose:
    //
    //   1. 90° clockwise viewed from above (x' = y, y' = -x) — so front-tip
    //      drives pitch and right-tip drives roll, instead of front-tip
    //      driving roll.
    //   2. 180° about the left-right axis (negate Y and Z) — undo the
    //      upside-down mount. Without this the resting roll sits near ±180°,
    //      so a tiny left-tip crosses the atan2 wrap boundary (the −7°→350°
    //      clipping seen on the dashboard).
    //
    // Composed, the sensor→body mapping is x→y, y→x, z→-z. Applied to accel
    // and gyro identically so the Mahony filter stays self-consistent. (Gyro
    // is a rotation rate; under a proper rotation it transforms as a vector.)
    float raw_ax = imu_.readFloatAccelX() * G;
    float raw_ay = imu_.readFloatAccelY() * G;
    float raw_gx = imu_.readFloatGyroX() * DPS_TO_RAD - gx_bias_;
    float raw_gy = imu_.readFloatGyroY() * DPS_TO_RAD - gy_bias_;

    ax_ =  raw_ay;
    ay_ =  raw_ax;
    az_ = -imu_.readFloatAccelZ() * G;
    gx_ =  raw_gy;
    gy_ =  raw_gx;
    gz_ = -(imu_.readFloatGyroZ() * DPS_TO_RAD - gz_bias_);

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
        if (dataReady(0x02)) {  // gyro data ready
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
