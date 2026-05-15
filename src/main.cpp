#include <Arduino.h>
#include <atomic>
#include "config.h"
#include "types.h"
#include "imu_driver.h"
#include "mahony_filter.h"
#include "serial_command.h"
#include "wifi_manager.h"

// --- globals ---

static IMUDriver     g_imu;
static MahonyFilter  g_mahony;
static SerialCommand g_serial;
static WifiManager   g_wifi;

static uint32_t    g_last_print_ms = 0;
static uint32_t    g_last_diag_ms  = 0;
static EulerAngles g_zero_offset   = {0.0f, 0.0f, 0.0f};  // subtracted on output
static bool        g_streaming             = true;
static bool        g_drift_log             = false;  // toggled by DEBUG ON/OFF
static bool        g_serial_print_enabled  = true;   // toggled by SERIAL ON/OFF
static uint32_t    g_serial_print_div      = 5;      // print 1 of every N samples (5 Hz @ 25 Hz)
static uint32_t    g_serial_print_counter  = 0;

// EMA alpha: 0.3 = moderate smoothing. Higher = less smoothing, more responsive.
static constexpr float EMA_ALPHA = 0.3f;

// Drift diagnostic log interval (ms)
static constexpr uint32_t DRIFT_LOG_INTERVAL_MS = 1000;

// ────────────────────────────────────────────────────────────────────────────
// Latest-sample snapshot. Written by sensor task on core 0; read by network /
// loop code on core 1. A sequence-counter (seqlock) protects readers from
// torn reads of the float fields without needing a mutex on the fast path.
// ────────────────────────────────────────────────────────────────────────────
struct SensorSnapshot {
    EulerAngles ema;       // smoothed euler in degrees
    float       bx, by, bz;  // mahony integral bias (rad/s) for diagnostics
    float       trust;     // mahony accel trust (0..1)
    uint32_t    sample_ms; // millis() when this sample was produced
};

static SensorSnapshot          g_snap         = {};
static std::atomic<uint32_t>   g_snap_seq{0};   // even = stable, odd = writing
static volatile bool           g_snap_init    = false;

// Monotonic sample counter incremented every time the sensor task publishes
// a fresh snapshot. Read by WifiManager::poll() to compute samples/s for
// the diagnostic [STATS] line. Volatile because it crosses cores (writer
// on core 0, reader on core 1).
volatile uint32_t              g_sample_count = 0;

// Publishes the latest snapshot. Called only from the sensor task.
static void publishSnapshot(const EulerAngles& ema,
                            float bx, float by, float bz, float trust,
                            uint32_t sample_ms) {
    uint32_t seq = g_snap_seq.load(std::memory_order_relaxed);
    g_snap_seq.store(seq + 1, std::memory_order_release);  // odd: writing
    g_snap.ema       = ema;
    g_snap.bx        = bx; g_snap.by = by; g_snap.bz = bz;
    g_snap.trust     = trust;
    g_snap.sample_ms = sample_ms;
    g_snap_seq.store(seq + 2, std::memory_order_release);  // even: stable
    g_snap_init = true;
}

// Reads the latest snapshot consistently. Returns true if a sample is available.
// Spins at most a few times under concurrent write; in practice always 1 try.
static bool readSnapshot(SensorSnapshot& out) {
    if (!g_snap_init) return false;
    for (int tries = 0; tries < 4; tries++) {
        uint32_t s1 = g_snap_seq.load(std::memory_order_acquire);
        if (s1 & 1) continue;                  // writer in progress
        out = g_snap;
        uint32_t s2 = g_snap_seq.load(std::memory_order_acquire);
        if (s1 == s2) return true;             // consistent read
    }
    return false;
}

void zeroOrientation() {
    SensorSnapshot s;
    if (readSnapshot(s)) {
        g_zero_offset = s.ema;
        Serial.printf("[CMD] Zero captured: roll=%.2f pitch=%.2f yaw=%.2f\n",
                      g_zero_offset.roll, g_zero_offset.pitch, g_zero_offset.yaw);
    } else {
        Serial.println("[CMD] Zero ignored: no sample yet");
    }
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
// Sensor task — runs on core 0, isolated from network/WebSocket housekeeping
// on core 1. Polls the IMU, runs Mahony, applies EMA, publishes snapshots.
// ────────────────────────────────────────────────────────────────────────────
static void sensorTask(void* /*pv*/) {
    EulerAngles ema       = {0.0f, 0.0f, 0.0f};
    bool        ema_init  = false;
    uint32_t    last_us   = micros();

    for (;;) {
        if (g_imu.update()) {
            uint32_t now_us = micros();
            float dt = (now_us - last_us) * 1e-6f;
            last_us = now_us;

            if (dt > 0.0f && dt < 0.5f) {
                g_mahony.update(g_imu.getAccelX(), g_imu.getAccelY(), g_imu.getAccelZ(),
                                g_imu.getGyroX(),  g_imu.getGyroY(),  g_imu.getGyroZ(),
                                dt);
                EulerAngles e = g_mahony.getEuler();
                if (!ema_init) { ema = e; ema_init = true; }
                else {
                    ema.roll  = EMA_ALPHA * e.roll  + (1.0f - EMA_ALPHA) * ema.roll;
                    ema.pitch = EMA_ALPHA * e.pitch + (1.0f - EMA_ALPHA) * ema.pitch;
                    ema.yaw   = EMA_ALPHA * e.yaw   + (1.0f - EMA_ALPHA) * ema.yaw;
                }
                publishSnapshot(ema,
                                g_mahony.getBiasX(), g_mahony.getBiasY(), g_mahony.getBiasZ(),
                                g_mahony.getLastTrust(),
                                millis());
                g_sample_count++;
            }
            g_imu.clearNewData();
        }
        // Yield briefly. LSM6DSO ODR is 208 Hz (~4.8 ms period), so a 1 ms
        // tick wake-up is plenty fast to never miss a sample.
        vTaskDelay(1);
    }
}

// --- Arduino entry points ---

void setup() {
    Serial.begin(SERIAL_BAUD_RATE);

    Serial.println();
    Serial.println("========================================");
    Serial.printf( "  IMU Balance Board Firmware v%s\n", FIRMWARE_VERSION);
    Serial.println("  ESP32 + LSM6DSO (I2C, Mahony filter)");
    Serial.println("========================================");

    Serial.println("[INIT] Initializing LSM6DSO over I2C...");

    if (!g_imu.begin()) {
        Serial.println("[INIT] FATAL: LSM6DSO initialization failed. Halting.");
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

    g_serial.setIMU(&g_imu);
    g_serial.setStreamingFlag(&g_streaming);
    g_serial.setWifi(&g_wifi);
    g_serial.begin();

    g_wifi.setStreamingFlag(&g_streaming);
    g_wifi.setSerial(&g_serial);
    g_wifi.begin();

    // Pin the sensor task to core 0 so it cannot be starved by _ws->loop()
    // or any other blocking work running on Arduino's core 1.
    BaseType_t ok = xTaskCreatePinnedToCore(
        sensorTask, "imu_sensor", 4096, nullptr,
        configMAX_PRIORITIES - 2,  // high priority but below WiFi driver
        nullptr, 0 /* core 0 */);
    if (ok != pdPASS) {
        Serial.println("[INIT] FATAL: sensor task creation failed. Halting.");
        while (true) { delay(1000); }
    }
    Serial.println("[INIT] Sensor task pinned to core 0.");

    Serial.println("[INIT] READY. Type HELP for commands.");
    Serial.printf("[INIT] Free heap: %lu bytes\n", (unsigned long)ESP.getFreeHeap());
}

void loop() {
    // 1. Serial command poll
    g_serial.poll();

    // 2. Periodic output at configured rate (WiFi + serial). Reads the latest
    //    consistent snapshot published by the sensor task on core 0.
    uint32_t now_ms = millis();
    if (g_streaming && now_ms - g_last_print_ms >= g_serial.getPrintIntervalMs()) {
        SensorSnapshot s;
        if (readSnapshot(s)) {
            g_last_print_ms = now_ms;
            float r = s.ema.roll  - g_zero_offset.roll;
            float p = s.ema.pitch - g_zero_offset.pitch;
            float y = s.ema.yaw   - g_zero_offset.yaw;
            g_wifi.sendFrame(s.sample_ms, r, p, y);
            // Serial print is rate-divided so it does not dominate the loop:
            // at 25 Hz WS rate with div=5 we print at 5 Hz on the UART.
            if (g_serial_print_enabled) {
                if (g_serial_print_div <= 1 ||
                    (++g_serial_print_counter % g_serial_print_div) == 0) {
                    Serial.printf("%lu,%.2f,%.2f,%.2f\n",
                                  (unsigned long)s.sample_ms, r, p, y);
                }
            }
        }
    }

    // 2b. Drift diagnostic log (1 Hz when enabled)
    if (g_drift_log && now_ms - g_last_diag_ms >= DRIFT_LOG_INTERVAL_MS) {
        SensorSnapshot s;
        if (readSnapshot(s)) {
            g_last_diag_ms = now_ms;
            Serial.printf(
                "[DRIFT] euler=(%.2f, %.2f, %.2f) int_bias=(%.4f, %.4f, %.4f) trust=%.2f\n",
                s.ema.roll, s.ema.pitch, s.ema.yaw,
                s.bx, s.by, s.bz, s.trust);
        }
    }

    // 3. Service WiFi WebSocket (must run every loop)
    g_wifi.poll();

    // Yield 1 tick so the FreeRTOS scheduler can run async_tcp / WiFi-driver
    // helper tasks on this core. Without this the Arduino loop hogs core 1
    // and packets queue up, manifesting as periodic dashboard stutters.
    // 1 tick (~1 ms) is small enough that we still service _ws->loop() and
    // broadcast at 50 Hz without trouble (50 Hz period is 20 ms).
    vTaskDelay(1);
}
