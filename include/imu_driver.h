#pragma once

#include <Adafruit_BNO08x.h>
#include "config.h"
#include "types.h"

class IMUDriver {
public:
    bool begin();
    bool update();         // poll for new data; returns true if any new report received
    void checkReset();     // detect spontaneous BNO085 reset, re-enable reports

    float getQuatW() const { return last_quat_[0]; }
    float getQuatX() const { return last_quat_[1]; }
    float getQuatY() const { return last_quat_[2]; }
    float getQuatZ() const { return last_quat_[3]; }

    float getGyroX() const { return last_gyro_[0]; }
    float getGyroY() const { return last_gyro_[1]; }
    float getGyroZ() const { return last_gyro_[2]; }

    bool hasNewQuaternion() const { return new_quat_; }
    bool hasNewGyroscope()  const { return new_gyro_; }

    void clearNewQuaternion() { new_quat_ = false; }
    void clearNewGyroscope()  { new_gyro_ = false; }

    // Reset the no-data watchdog timer. Call after blocking on user input.
    void resetWatchdog() { last_data_ms_ = millis(); }

private:
    Adafruit_BNO08x bno_{PIN_BNO_RST};
    sh2_SensorValue_t sensor_value_{};

    float last_quat_[4]  = {1.0f, 0.0f, 0.0f, 0.0f};  // w, x, y, z
    float last_gyro_[3]  = {0.0f, 0.0f, 0.0f};

    bool new_quat_ = false;
    bool new_gyro_ = false;

    uint32_t last_data_ms_ = 0;
    void enableReports();
    void hardReset();
};
