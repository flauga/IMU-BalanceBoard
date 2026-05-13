#pragma once

#include <cstdint>

// --- Firmware version ---
#define FIRMWARE_VERSION "4.0"

// --- I2C Pin Assignments (ESP32 DevKit V1 default I2C bus) ---
// SDA = GPIO 21, SCL = GPIO 22 — used automatically by Wire.begin()
// LSM6DSO default I2C address (SA0/SDO pin pulled high on SmartElex breakout)
static constexpr uint8_t LSM6DSO_I2C_ADDR = 0x6B;

// --- IMU Configuration ---
// LSM6DSO ODR: 208 Hz. Accel range: ±4g  Gyro range: ±500 dps
static constexpr uint32_t IMU_NO_DATA_TIMEOUT_MS = 5000;  // watchdog: 5s
static constexpr uint8_t  IMU_INIT_MAX_RETRIES   = 5;
static constexpr uint32_t IMU_INIT_RETRY_DELAY_MS = 500;

// --- Serial ---
// Bumped from 115200 → 921600 to reduce time spent in Serial.printf() on the
// network loop. 50 Hz × ~30 chars = ~1.5 KB/s, well within budget at 921600.
static constexpr uint32_t SERIAL_BAUD_RATE        = 921600;
static constexpr uint32_t SERIAL_PRINT_INTERVAL_MS = 20;   // 50 Hz output rate

// --- WiFi ---
static constexpr uint16_t WIFI_WS_PORT   = 81;
static constexpr uint16_t WIFI_HTTP_PORT = 80;

// mDNS responder (board reachable at http://<WIFI_HOSTNAME>.local). Set to 0
// to disable for diagnostic purposes — the board is then only reachable via
// its DHCP-assigned IP address, which is printed prominently at boot.
// Disabling mDNS rules it out as a source of periodic stalls caused by
// incoming query traffic on the LAN.
#define MDNS_ENABLED 1

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
