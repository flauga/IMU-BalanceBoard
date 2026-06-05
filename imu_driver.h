#pragma once

#include <Arduino.h>
#include <Wire.h>
#include <LSM6DS3.h>
#include "config.h"
#include "types.h"

class IMUDriver {
public:
    bool begin();

    // Poll sensor. Returns true if new data was read.
    // Call as fast as possible in the main loop.
    bool update();

    // Average gyro readings while board is still and store as bias.
    // Caller must keep the board stationary for the duration.
    // Returns true if calibration produced a plausible bias estimate.
    bool calibrateGyro(uint32_t duration_ms = 800);

    // Raw accel in m/s², bias-corrected gyro in rad/s
    float getAccelX() const { return ax_; }
    float getAccelY() const { return ay_; }
    float getAccelZ() const { return az_; }
    float getGyroX()  const { return gx_; }
    float getGyroY()  const { return gy_; }
    float getGyroZ()  const { return gz_; }

    bool hasNewData() const { return new_data_; }
    void clearNewData()     { new_data_ = false; }

    // Reset the no-data watchdog timer (call after blocking operations)
    void resetWatchdog() { last_data_ms_ = millis(); }

private:
    // Seeed_Arduino_LSM6DS3's LSM6DS3 wraps LSM6DS3Core; constructed with
    // (interface, address). The library's I2C path uses the global Wire by
    // default — we patch the bus in begin() before configuration.
    LSM6DS3 imu_{I2C_MODE, LSM6DS3_I2C_ADDR};

    float ax_ = 0.0f, ay_ = 0.0f, az_ = 0.0f;  // m/s²
    float gx_ = 0.0f, gy_ = 0.0f, gz_ = 0.0f;  // rad/s, bias-corrected

    // Per-unit gyro bias (rad/s). Subtracted from raw reads in update().
    float gx_bias_ = 0.0f, gy_bias_ = 0.0f, gz_bias_ = 0.0f;

    bool     new_data_     = false;
    uint32_t last_data_ms_ = 0;

    bool dataReady(uint8_t mask) const;  // mask: 0x01=XLDA, 0x02=GDA
};
