#include "imu_driver.h"
#include "config.h"
#include <Arduino.h>
#include <Wire.h>

void IMUDriver::hardReset() {
    digitalWrite(PIN_BNO_RST, LOW);
    delay(IMU_RESET_PULSE_MS);
    digitalWrite(PIN_BNO_RST, HIGH);
    delay(IMU_RESET_WAIT_MS);
}

bool IMUDriver::begin() {
    pinMode(PIN_BNO_INT, INPUT_PULLUP);
    pinMode(PIN_BNO_RST, OUTPUT);

    // Reset the ESP32 I2C peripheral first — on ESP32 reboot without power-cycle
    // the I2C hardware may be in a stale state that confuses the BNO085.
    Wire.end();
    delay(50);
    Wire.begin();
    Wire.setClock(100000);

    // Now assert BNO085 hardware reset
    digitalWrite(PIN_BNO_RST, LOW);
    delay(100);
    digitalWrite(PIN_BNO_RST, HIGH);
    delay(IMU_RESET_WAIT_MS);

    for (uint8_t attempt = 0; attempt < IMU_INIT_MAX_RETRIES; attempt++) {
        if (bno_.begin_I2C()) {
            Wire.setClock(100000);
            Serial.println("[IMU] BNO085 connected — triggering clean reset for report config");
            // Trigger a hardware reset NOW via the library. This puts the sensor
            // into the same state as the working "spontaneous reset" path.
            // checkReset() will see wasReset()=true on the next loop() call
            // and invoke enableReports() from the proven-working code path.
            bno_.hardwareReset();
            last_data_ms_ = millis();
            return true;
        }
        Serial.printf("[IMU] Init attempt %d/%d failed\n", attempt + 1, IMU_INIT_MAX_RETRIES);
        Wire.end();
        delay(50);
        Wire.begin();
        Wire.setClock(100000);
        digitalWrite(PIN_BNO_RST, LOW);
        delay(100);
        digitalWrite(PIN_BNO_RST, HIGH);
        delay(IMU_INIT_RETRY_DELAY_MS);
    }

    Serial.println("[IMU] ERROR: BNO085 init failed after all retries");
    return false;
}

void IMUDriver::enableReports() {
    if (!bno_.enableReport(SH2_GAME_ROTATION_VECTOR, IMU_REPORT_INTERVAL_US)) {
        Serial.println("[IMU] WARNING: Failed to enable Game Rotation Vector");
    }
    if (!bno_.enableReport(SH2_GYROSCOPE_CALIBRATED, IMU_REPORT_INTERVAL_US)) {
        Serial.println("[IMU] WARNING: Failed to enable Calibrated Gyroscope");
    }
}

bool IMUDriver::update() {
    if (!bno_.getSensorEvent(&sensor_value_)) {
        return false;
    }

    last_data_ms_ = millis();

    switch (sensor_value_.sensorId) {
        case SH2_GAME_ROTATION_VECTOR:
            last_quat_[0] = sensor_value_.un.gameRotationVector.real;
            last_quat_[1] = sensor_value_.un.gameRotationVector.i;
            last_quat_[2] = sensor_value_.un.gameRotationVector.j;
            last_quat_[3] = sensor_value_.un.gameRotationVector.k;
            new_quat_ = true;
            break;

        case SH2_GYROSCOPE_CALIBRATED:
            last_gyro_[0] = sensor_value_.un.gyroscope.x;
            last_gyro_[1] = sensor_value_.un.gyroscope.y;
            last_gyro_[2] = sensor_value_.un.gyroscope.z;
            new_gyro_ = true;
            break;

        default:
            break;
    }

    return true;
}

void IMUDriver::checkReset() {
    if (bno_.wasReset()) {
        Serial.println("[IMU] BNO085 booted — configuring reports");
        enableReports();
        last_data_ms_ = millis();
        return;
    }

    // Hard-reset watchdog: no data for IMU_NO_DATA_TIMEOUT_MS
    if (last_data_ms_ > 0 && (millis() - last_data_ms_ > IMU_NO_DATA_TIMEOUT_MS)) {
        Serial.println("[IMU] WARNING: No data — hard-resetting BNO085");

        // Must close SH2 session before re-opening; without this shtp_open()
        // finds the singleton slot occupied, returns NULL, begin_I2C() returns
        // false silently, and the next sh2_service() dereferences NULL → panic.
        sh2_close();

        pinMode(PIN_BNO_RST, OUTPUT);

        bool ok = false;
        for (uint8_t attempt = 0; attempt < IMU_INIT_MAX_RETRIES; attempt++) {
            Wire.end();
            delay(50);
            Wire.begin();
            Wire.setClock(100000);
            digitalWrite(PIN_BNO_RST, LOW);
            delay(100);
            digitalWrite(PIN_BNO_RST, HIGH);
            delay(IMU_RESET_WAIT_MS);
            if (bno_.begin_I2C()) {
                Wire.setClock(100000);
                bno_.hardwareReset();  // let wasReset() path handle enableReports()
                ok = true;
                break;
            }
            Serial.printf("[IMU] Recovery attempt %d/%d failed\n",
                          attempt + 1, IMU_INIT_MAX_RETRIES);
            sh2_close();
            delay(IMU_INIT_RETRY_DELAY_MS);
        }

        Serial.println(ok ? "[IMU] BNO085 recovered" : "[IMU] ERROR: recovery failed");
        last_data_ms_ = millis();
    }
}
