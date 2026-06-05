#pragma once

#include <cstdint>

// --- Firmware version ---
#define FIRMWARE_VERSION "4.0-mg24"

// --- I2C / IMU pin Assignments ---
// On the Seeed XIAO MG24 Sense the built-in LSM6DS3TR-C is wired to the
// internal I2C bus exposed as Wire1 in the Arduino core. The address jumper
// pulls SA0 low, giving 0x6A.
static constexpr uint8_t LSM6DS3_I2C_ADDR = 0x6A;

// --- IMU Configuration ---
// LSM6DS3 ODR: 208 Hz. Accel range: ±4g  Gyro range: ±500 dps
static constexpr uint32_t IMU_NO_DATA_TIMEOUT_MS  = 5000;  // watchdog: 5s
static constexpr uint8_t  IMU_INIT_MAX_RETRIES    = 5;
static constexpr uint32_t IMU_INIT_RETRY_DELAY_MS = 500;

// --- Serial ---
// MG24's USB-CDC tops out well below ESP32's 921600 baud; 115200 is the
// default and reliable everywhere. The per-frame text print is rate-divided
// so this is still well within budget.
static constexpr uint32_t SERIAL_BAUD_RATE         = 115200;
static constexpr uint32_t SERIAL_PRINT_INTERVAL_MS = 20;   // 50 Hz output rate

// --- BLE ---
// 16-byte custom UUIDs for the IMU service and its two characteristics.
// These are random base UUIDs — match them in your Web Bluetooth client.
#define BLE_DEVICE_NAME           "IMUBoard-MG24"
#define BLE_IMU_SERVICE_UUID      "6e400001-b5a3-f393-e0a9-e50e24dcca9e"
#define BLE_IMU_ANGLES_CHAR_UUID  "6e400002-b5a3-f393-e0a9-e50e24dcca9e"  // notify
#define BLE_IMU_COMMAND_CHAR_UUID "6e400003-b5a3-f393-e0a9-e50e24dcca9e"  // write (text cmds)

// --- Mahony Filter ---
// Kp = proportional gain (higher = accel corrects gyro faster, more responsive but noisier)
// Ki = integral gain (corrects slow gyro bias drift)
// Tune Kp down if output is jittery; tune Kp up if it's too sluggish.
static constexpr float MAHONY_KP               = 0.5f;
static constexpr float MAHONY_KI               = 0.00005f;
// Clamp on integral bias estimate (rad/s) so a runaway can never exceed
// real gyro bias. Per-unit offsets are <10°/s; we calibrate them at boot.
static constexpr float MAHONY_BIAS_CLAMP       = 0.05f;  // ~2.9°/s
static constexpr float MAHONY_ACCEL_GATE       = 4.0f;   // magnitude gate sharpness
// Variance gate: reject accel during dynamic motion (high variance = moving)
static constexpr uint8_t  MAHONY_VAR_WINDOW    = 32;     // samples (~154 ms at 208 Hz)
static constexpr float    MAHONY_VAR_THRESHOLD = 0.002f; // g² — above this = dynamic motion
