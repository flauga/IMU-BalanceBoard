#pragma once

#include <cstdint>

// --- Firmware version ---
#define FIRMWARE_VERSION "3.1"

// --- I2C Pin Assignments (ESP32 DevKit V1 default I2C bus) ---
// SDA=21, SCL=22 are the ESP32 default Wire pins — used automatically
static constexpr uint8_t PIN_BNO_RST = 4;
static constexpr uint8_t PIN_BNO_INT = 2;   // BNO085 INT (active-low, data-ready signal)
// BNO085 I2C address: 0x4A (default, DI pin low) or 0x4B (DI pin high)

// --- IMU Configuration ---
// 50 Hz is well within I2C budget. At 400 kHz I2C with ~32-byte SH2 packets
// the bus can handle ~1500 tx/s; two reports at 50 Hz use only ~7% of capacity.
static constexpr uint32_t IMU_REPORT_INTERVAL_US   = 20000; // 50 Hz
static constexpr uint32_t IMU_NO_DATA_TIMEOUT_MS   = 10000; // watchdog: 10s startup headroom
static constexpr uint8_t  IMU_INIT_MAX_RETRIES     = 5;
static constexpr uint32_t IMU_INIT_RETRY_DELAY_MS  = 500;
static constexpr uint32_t IMU_RESET_PULSE_MS       = 100;  // hold RST low long enough to fully reset
static constexpr uint32_t IMU_RESET_WAIT_MS        = 1000; // wait for BNO085 firmware to fully boot

// --- Calibration ---
static constexpr uint16_t CALIBRATION_SAMPLE_COUNT  = 200;   // filter warm-up samples
static constexpr uint32_t CALIBRATION_TIMEOUT_MS    = 10000;
// Six-position accel calibration: samples per face, timeout per face
static constexpr uint16_t ACCELCAL_SAMPLES_PER_FACE = 100;   // ~1 s at 100 Hz decimated
static constexpr uint32_t ACCELCAL_TIMEOUT_MS       = 30000; // 30 s — user needs time to reposition

// --- Serial ---
static constexpr uint32_t SERIAL_BAUD_RATE        = 115200;
static constexpr uint32_t SERIAL_PRINT_INTERVAL_MS = 40;     // 25 Hz display rate

// --- WiFi ---
static constexpr uint16_t WIFI_WS_PORT  = 81;
static constexpr uint16_t WIFI_HTTP_PORT = 80;

// --- Mahony Filter ---
static constexpr float MAHONY_KP               = 2.0f;    // proportional gain
static constexpr float MAHONY_KI               = 0.005f;  // integral gain (bias estimation)
static constexpr float MAHONY_ACCEL_GATE       = 4.0f;    // magnitude gate sharpness
// Variance gate: sliding window length and threshold (g²)
// Trust accel only when both magnitude ≈ 1g AND variance is below threshold.
static constexpr uint8_t  MAHONY_VAR_WINDOW    = 16;      // samples (~128 ms at 125 Hz)
static constexpr float    MAHONY_VAR_THRESHOLD = 0.002f;  // g² — above this = dynamic motion
