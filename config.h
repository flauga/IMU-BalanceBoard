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
// Output cadence for BLE notifies (and the rate-divided serial print). Raised
// from 20 ms (50 Hz) to 11 ms (~90 Hz): smaller, more frequent notifies mean the
// OS BLE stack batches fewer frames per burst, so the browser sees fresher data
// and the dot stutters less. The browser does render-side smoothing on top.
static constexpr uint32_t SERIAL_PRINT_INTERVAL_MS = 11;   // ~90 Hz output rate

// Hard CAP on how slow the board is ever allowed to emit frames. The board used
// to retune its emit cadence to WHATEVER connection interval the central granted
// (see sl_bt_evt_connection_parameters). Windows/Chrome and phones aggressively
// downshift the interval for power saving — sometimes to hundreds of ms or more —
// and the board would obediently drop to a few Hz (or ~1 Hz), producing the
// recurring "board moves, dot freezes for ~1 s, then snaps" bug. The sensor loop
// never stalled; the board just stopped OFFERING fresh frames. We now clamp the
// emit interval to this ceiling so the board ALWAYS has a fresh sample ready each
// connection event, no matter how slowly the central polls — the radio carries
// the latest frame per event instead of the board self-throttling to 1 Hz.
static constexpr uint32_t OUTPUT_INTERVAL_MAX_MS = 20;     // never slower than 50 Hz

// --- BLE ---
// 16-byte custom UUIDs for the IMU service and its two characteristics.
// These are random base UUIDs — match them in your Web Bluetooth client.
#define BLE_DEVICE_NAME           "IMUBoard-MG24"
#define BLE_IMU_SERVICE_UUID      "6e400001-b5a3-f393-e0a9-e50e24dcca9e"
#define BLE_IMU_ANGLES_CHAR_UUID  "6e400002-b5a3-f393-e0a9-e50e24dcca9e"  // notify
#define BLE_IMU_COMMAND_CHAR_UUID "6e400003-b5a3-f393-e0a9-e50e24dcca9e"  // write (text cmds)

// --- BLE connection parameters (requested on connect) ---
// THE batching fix. The board fires a notify every SERIAL_PRINT_INTERVAL_MS, but
// the radio only transmits during a *connection event* — one per *connection
// interval*. If we never ask, the central (Windows/Chrome) defaults to ~30-50 ms,
// so several notifies queue between events and arrive at the browser in one burst
// → the "freeze then snap" dot. On connect we *request* a fast 15 ms interval
// (sl_bt_connection_set_parameters), then retune the output cadence to whatever
// the central actually grants so exactly one fresh frame lands per event.
//
// All values are integers (uint16_t) — deliberately no floats: this is requested
// from the BLE-host task, whose small stack the float-printf path overflows.
//   interval: units 1.25 ms (12 => 15 ms; floor 6 => 7.5 ms, but Windows clamps higher)
//   latency : intervals the peripheral may skip — MUST be 0, we always have data
//   timeout : units 10 ms; rule: timeout_ms > (1 + latency) * max_interval_ms * 2
//   ce_len  : connection-event length, units 0.625 ms; 0 = let the stack decide
static constexpr uint16_t BLE_CONN_INTERVAL_MIN = 12;   // 15 ms
static constexpr uint16_t BLE_CONN_INTERVAL_MAX = 12;   // 15 ms (pinned, no band)
static constexpr uint16_t BLE_CONN_LATENCY      = 0;    // never skip an event
// Supervision timeout was 100 (1000 ms). That's exactly the "solid 1-second gap"
// window: a brief RF/OS scheduling hiccup that stalls events for ~1 s would trip
// the timeout, drop the link, and force a reconnect (~1 s of no data → freeze,
// then snap). Raised to 400 (4000 ms) so a transient stall RIDES THROUGH instead
// of tearing the connection down. Rule: timeout_ms > (1+latency)*max_interval_ms*2;
// easily satisfied. Latency stays 0 so we still never intentionally skip events.
static constexpr uint16_t BLE_CONN_TIMEOUT      = 400;  // x10 ms = 4000 ms supervision
static constexpr uint16_t BLE_CONN_CE_MIN       = 0;    // don't care
static constexpr uint16_t BLE_CONN_CE_MAX       = 0;    // don't care

// --- Mahony Filter ---
// Kp = proportional gain (higher = accel corrects gyro faster, more responsive but noisier)
// Ki = integral gain (corrects slow gyro bias drift)
// Tune Kp down if output is jittery; tune Kp up if it's too sluggish.
//
// MAHONY_KP and MAHONY_VAR_THRESHOLD are the *boot defaults*; the live values are
// runtime-tunable (see MahonyFilter::setKp / setVarThreshold and the KP/VAR BLE
// commands) and may be persisted to NVM. The others are fixed.
static constexpr float MAHONY_KP               = 1.2f;
static constexpr float MAHONY_KI               = 0.00005f;
// Clamp on integral bias estimate (rad/s) so a runaway can never exceed
// real gyro bias. Per-unit offsets are <10°/s; we calibrate them at boot.
static constexpr float MAHONY_BIAS_CLAMP       = 0.05f;  // ~2.9°/s
static constexpr float MAHONY_ACCEL_GATE       = 4.0f;   // magnitude gate sharpness
// Variance gate: reject accel during dynamic motion (high variance = moving)
static constexpr uint8_t  MAHONY_VAR_WINDOW    = 32;     // samples (~154 ms at 208 Hz)
static constexpr float    MAHONY_VAR_THRESHOLD = 0.002f; // g² — above this = dynamic motion

// --- Default smoothing / filter source (runtime-tunable) ---
// EMA output smoothing alpha (higher = less smoothing, more responsive).
// Raised 0.5 → 0.6: with the browser now doing render-side smoothing, we let the
// firmware pass a fresher (lower-lag) signal and keep the visual smoothing client-side.
static constexpr float    EMA_ALPHA_DEFAULT    = 0.6f;
// Filter source modes — what feeds the streamed Euler angles.
enum class FilterMode : uint8_t { Fusion = 0, Gyro = 1, Accel = 2 };
