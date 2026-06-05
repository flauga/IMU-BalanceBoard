#include <Arduino.h>
#include <EEPROM.h>
#include "config.h"
#include "types.h"
#include "imu_driver.h"
#include "mahony_filter.h"
#include "serial_command.h"
#include "ble_manager.h"

// --- globals ---

static IMUDriver     g_imu;
static MahonyFilter  g_mahony;
static SerialCommand g_serial;
// BleManager is a singleton — the BGAPI stack delivers events to a free
// function (sl_bt_on_event), so we route through BleManager::instance().
static BleManager&   g_ble = BleManager::instance();

static uint32_t    g_last_print_ms = 0;
static uint32_t    g_last_diag_ms  = 0;
static EulerAngles g_zero_offset   = {0.0f, 0.0f, 0.0f};
static EulerAngles g_latest_ema    = {0.0f, 0.0f, 0.0f};
static bool        g_ema_ready     = false;
static bool        g_streaming             = true;
static bool        g_drift_log             = false;
static bool        g_serial_print_enabled  = true;
static uint32_t    g_serial_print_div      = 5;
static uint32_t    g_serial_print_counter  = 0;

// EMA alpha: 0.3 = moderate smoothing. Higher = less smoothing, more responsive.
static constexpr float EMA_ALPHA = 0.3f;

// Drift diagnostic log interval (ms)
static constexpr uint32_t DRIFT_LOG_INTERVAL_MS = 1000;

// Sample counter used by BleManager for the once-per-second [STATS] line.
// MG24 is single-core, so no atomic / volatile concerns across cores — but
// volatile makes the intent explicit and matches the ESP32 firmware layout.
volatile uint32_t g_sample_count = 0;

// ────────────────────────────────────────────────────────────────────────────
// Persisted zero offset. The SiliconLabs core's EEPROM library is backed by
// on-chip non-volatile storage, so the captured zero survives power cycles —
// the operator zeroes once and every subsequent boot references that pose.
// A magic + version word guards against reading garbage from a fresh/erased
// chip or a layout change between firmware revisions.
// ────────────────────────────────────────────────────────────────────────────
struct PersistedZero {
    uint32_t    magic;    // sentinel — must equal ZERO_MAGIC to be trusted
    EulerAngles offset;   // roll/pitch/yaw captured at the last ZERO
};
static constexpr uint32_t ZERO_MAGIC      = 0x5A45524F;  // 'ZERO'
static constexpr int      ZERO_EEPROM_ADDR = 0;

static void saveZeroOffset(const EulerAngles& off) {
    PersistedZero rec{ZERO_MAGIC, off};
    EEPROM.put(ZERO_EEPROM_ADDR, rec);  // SiliconLabs EEPROM persists directly
}

// Returns true and fills `out` if a valid persisted offset was found.
static bool loadZeroOffset(EulerAngles& out) {
    PersistedZero rec{};
    EEPROM.get(ZERO_EEPROM_ADDR, rec);
    if (rec.magic != ZERO_MAGIC) return false;
    out = rec.offset;
    return true;
}

void zeroOrientation() {
    if (g_ema_ready) {
        g_zero_offset = g_latest_ema;
        saveZeroOffset(g_zero_offset);
        Serial.printf("[CMD] Zero captured & saved: roll=%.2f pitch=%.2f yaw=%.2f\n",
                      g_zero_offset.roll, g_zero_offset.pitch, g_zero_offset.yaw);
    } else {
        Serial.println("[CMD] Zero ignored: no sample yet");
    }
}

// Clear the persisted zero so the next boot falls back to capturing the boot
// pose. Also re-references the live offset to the current pose immediately so
// the operator isn't left on a stale reference until reboot.
void clearZeroOrientation() {
    PersistedZero blank{};
    EEPROM.put(ZERO_EEPROM_ADDR, blank);  // wipe magic — load will now miss
    if (g_ema_ready) g_zero_offset = g_latest_ema;
    Serial.println("[CMD] Saved zero cleared — next boot will use boot pose");
}

void setDriftLog(bool on) {
    g_drift_log = on;
    Serial.printf("[CMD] Drift log %s\n", on ? "ON" : "OFF");
}

void setSerialPrintEnabled(bool on) {
    g_serial_print_enabled = on;
    Serial.printf("[CMD] Serial frame print %s\n", on ? "ON" : "OFF");
}

void setSerialPrintDivider(uint32_t div) {
    if (div < 1) div = 1;
    g_serial_print_div = div;
    g_serial_print_counter = 0;
    Serial.printf("[CMD] Serial frame print divider = %lu (1 in every %lu frames)\n",
                  (unsigned long)div, (unsigned long)div);
}

// ────────────────────────────────────────────────────────────────────────────
// Sample-the-IMU helper. Reads a fresh sample if one is available, feeds it
// to Mahony, and updates the EMA-smoothed Euler. Returns true if it produced
// a fresh smoothed sample this call.
// ────────────────────────────────────────────────────────────────────────────
static bool pollSensor() {
    static uint32_t last_us = 0;
    if (last_us == 0) last_us = micros();

    if (!g_imu.update()) return false;

    uint32_t now_us = micros();
    float dt = (now_us - last_us) * 1e-6f;
    last_us = now_us;
    if (dt <= 0.0f || dt >= 0.5f) {
        g_imu.clearNewData();
        return false;
    }

    g_mahony.update(g_imu.getAccelX(), g_imu.getAccelY(), g_imu.getAccelZ(),
                    g_imu.getGyroX(),  g_imu.getGyroY(),  g_imu.getGyroZ(),
                    dt);
    EulerAngles e = g_mahony.getEuler();
    if (!g_ema_ready) {
        g_latest_ema = e;
        g_ema_ready  = true;
    } else {
        g_latest_ema.roll  = EMA_ALPHA * e.roll  + (1.0f - EMA_ALPHA) * g_latest_ema.roll;
        g_latest_ema.pitch = EMA_ALPHA * e.pitch + (1.0f - EMA_ALPHA) * g_latest_ema.pitch;
        g_latest_ema.yaw   = EMA_ALPHA * e.yaw   + (1.0f - EMA_ALPHA) * g_latest_ema.yaw;
    }
    g_sample_count++;
    g_imu.clearNewData();
    return true;
}

// --- Arduino entry points ---

void setup() {
    Serial.begin(SERIAL_BAUD_RATE);
    // The XIAO MG24's USB-CDC enumeration takes a moment after reset; give
    // the host PC up to ~2 s to attach before printing, otherwise the boot
    // banner is silently dropped.
    uint32_t serial_wait_start = millis();
    while (!Serial && millis() - serial_wait_start < 2000) { delay(10); }

    Serial.println();
    Serial.println("========================================");
    Serial.printf( "  IMU Balance Board Firmware v%s\n", FIRMWARE_VERSION);
    Serial.println("  XIAO MG24 Sense + LSM6DS3 (I2C, Mahony)");
    Serial.println("========================================");

    Serial.println("[INIT] Initializing LSM6DS3 over I2C (Wire1)...");

    if (!g_imu.begin()) {
        Serial.println("[INIT] FATAL: LSM6DS3 initialization failed. Halting.");
        while (true) { delay(1000); }
    }

    // Calibrate gyro bias while the board is still. Per-unit offsets of 1–3°/s
    // are normal and would otherwise show up as constant drift in roll/pitch/yaw.
    Serial.println("[INIT] Calibrating gyro (keep board still)...");
    g_imu.calibrateGyro();

    g_mahony.reset();

    // Warm up the Mahony filter for 500 ms while stationary so the quaternion
    // converges to the true gravity direction before streaming begins.
    Serial.println("[INIT] Warming up Mahony filter (keep board still)...");
    uint32_t warmup_start = millis();
    uint32_t warmup_us    = micros();
    while (millis() - warmup_start < 500) {
        if (g_imu.update()) {
            uint32_t now_us = micros();
            float dt = (now_us - warmup_us) * 1e-6f;
            if (dt > 0.0f && dt < 0.5f) {
                g_mahony.update(g_imu.getAccelX(), g_imu.getAccelY(), g_imu.getAccelZ(),
                                g_imu.getGyroX(),  g_imu.getGyroY(),  g_imu.getGyroZ(),
                                dt);
            }
            warmup_us = now_us;
            g_imu.clearNewData();
        }
    }
    Serial.println("[INIT] Filter ready.");

    // Establish the zero reference. The warmup loop above converged the
    // quaternion but fed Mahony directly, so g_latest_ema is still empty; run
    // pollSensor for a short window to populate the EMA either way.
    Serial.println("[INIT] Settling orientation (keep board still)...");
    uint32_t zero_start = millis();
    while (millis() - zero_start < 300) {
        pollSensor();
    }

    // Prefer a persisted zero from a previous session — the operator zeroes
    // once and every boot thereafter references that saved pose. Only when no
    // valid offset is stored (fresh/erased chip) do we fall back to capturing
    // the current boot pose and persisting it.
    EulerAngles saved;
    if (loadZeroOffset(saved)) {
        g_zero_offset = saved;
        Serial.printf("[INIT] Loaded saved zero: roll=%.2f pitch=%.2f yaw=%.2f\n",
                      g_zero_offset.roll, g_zero_offset.pitch, g_zero_offset.yaw);
    } else if (g_ema_ready) {
        g_zero_offset = g_latest_ema;
        saveZeroOffset(g_zero_offset);
        Serial.printf("[INIT] No saved zero — captured boot pose: roll=%.2f pitch=%.2f yaw=%.2f\n",
                      g_zero_offset.roll, g_zero_offset.pitch, g_zero_offset.yaw);
    } else {
        Serial.println("[INIT] WARNING: no sample during settle window; offset stays 0.");
    }

    g_serial.setIMU(&g_imu);
    g_serial.setStreamingFlag(&g_streaming);
    g_serial.setBle(&g_ble);
    g_serial.begin();

    g_ble.setStreamingFlag(&g_streaming);
    g_ble.setSerial(&g_serial);
    if (!g_ble.begin()) {
        Serial.println("[INIT] WARNING: BLE init failed — UART-only mode.");
    }

    Serial.println("[INIT] READY. Type HELP for commands.");
}

void loop() {
    // 1. Drain IMU as fast as possible — the 208 Hz ODR means a new sample
    //    every ~4.8 ms, so this just runs whenever data is available.
    pollSensor();

    // 2. Serial command poll
    g_serial.poll();

    // 3. Periodic output at configured rate (BLE notify + serial print).
    uint32_t now_ms = millis();
    if (g_streaming && g_ema_ready &&
        now_ms - g_last_print_ms >= g_serial.getPrintIntervalMs()) {
        g_last_print_ms = now_ms;
        float r = g_latest_ema.roll  - g_zero_offset.roll;
        float p = g_latest_ema.pitch - g_zero_offset.pitch;
        float y = g_latest_ema.yaw   - g_zero_offset.yaw;
        g_ble.sendFrame(now_ms, r, p, y);
        // Serial print is rate-divided so it does not dominate the loop:
        // at 50 Hz BLE rate with div=5 we print at 10 Hz on the UART.
        if (g_serial_print_enabled) {
            if (g_serial_print_div <= 1 ||
                (++g_serial_print_counter % g_serial_print_div) == 0) {
                Serial.printf("%lu,%.2f,%.2f,%.2f\n",
                              (unsigned long)now_ms, r, p, y);
            }
        }
    }

    // 3b. Drift diagnostic log (1 Hz when enabled)
    if (g_drift_log && now_ms - g_last_diag_ms >= DRIFT_LOG_INTERVAL_MS) {
        g_last_diag_ms = now_ms;
        Serial.printf(
            "[DRIFT] euler=(%.2f, %.2f, %.2f) int_bias=(%.4f, %.4f, %.4f) trust=%.2f\n",
            g_latest_ema.roll, g_latest_ema.pitch, g_latest_ema.yaw,
            g_mahony.getBiasX(), g_mahony.getBiasY(), g_mahony.getBiasZ(),
            g_mahony.getLastTrust());
    }

    // 4. Service BLE stack (drains GATT events / dispatches notifies)
    g_ble.poll();
}
