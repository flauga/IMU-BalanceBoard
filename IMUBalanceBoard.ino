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
static EulerAngles g_latest_raw    = {0.0f, 0.0f, 0.0f};  // pre-EMA Euler (for accel/gyro modes)
static bool        g_ema_ready     = false;
static bool        g_streaming             = true;
static bool        g_drift_log             = false;
static bool        g_serial_print_enabled  = true;
static uint32_t    g_serial_print_div      = 5;
static uint32_t    g_serial_print_counter  = 0;

// ── Runtime tuning state (live, dashboard-adjustable, optionally persisted) ──
// EMA alpha: higher = less smoothing, more responsive. KP/VAR live in the
// Mahony filter; FilterMode selects what feeds the streamed Euler.
static float       g_ema_alpha  = EMA_ALPHA_DEFAULT;
static FilterMode  g_filter_mode = FilterMode::Fusion;

// ── Tilt-range limits (degrees, in the zeroed reference frame) ───────────────
// Captured in the testing dashboard, stored in NVM, shared with every game.
// g_limit_mask bit0=front bit1=back bit2=left bit3=right; a bit is set once that
// edge has been captured. front/back are pitch extremes; left/right are roll.
static constexpr uint8_t LIMIT_FRONT = 0x01;
static constexpr uint8_t LIMIT_BACK  = 0x02;
static constexpr uint8_t LIMIT_LEFT  = 0x04;
static constexpr uint8_t LIMIT_RIGHT = 0x08;
static float   g_limit_front = 0.0f, g_limit_back = 0.0f;
static float   g_limit_left  = 0.0f, g_limit_right = 0.0f;
static uint8_t g_limit_mask  = 0;

// ── Rolling window of recent ZEROED roll/pitch, maintained by pollSensor() in
// the main loop. captureLimit() (dispatched from the BLE-host task, which must
// not block or spin a sample loop) reads the window's mean as the edge — so a
// captured limit is the average of the last ~0.5 s of a settled hold, not one
// instantaneous wobble sample. A spread check rejects captures taken while the
// user is still moving. ~0.5 s at 208 Hz ≈ 104 samples; 128 gives margin.
static constexpr uint16_t LIMIT_WIN          = 128;
static constexpr float    LIMIT_MAX_SPREAD   = 6.0f;   // ° peak-to-peak: above = not a steady hold
static float    g_lw_roll[LIMIT_WIN]  = {};
static float    g_lw_pitch[LIMIT_WIN] = {};
static uint16_t g_lw_idx    = 0;
static bool     g_lw_full   = false;

// ── Calibration-health code (runtime-derived, NOT persisted). Exposed as the
// trailing field of the TUNE snapshot so any dashboard can warn the user if the
// board's sensor has drifted out of calibration. 0 = OK, 1 = gyro-bias drift
// suspected. (There is intentionally no "zero looks off" code — see
// updateCalibHealth: comparing the resting angle to zero false-tripped while the
// user held a deliberate lean during setup.) Updated a few times a second in
// loop(). Hysteresis: the fault must persist HEALTH_HOLD_S of continuous
// stillness before it latches, and clears as soon as the board reads healthy —
// so it never flickers and (per goal #2) practically never trips in normal use.
static constexpr float    HEALTH_BIAS_DPS   = 2.0f;   // |gyro bias| over this (°/s) = drift suspected
static constexpr float    HEALTH_MIN_TRUST  = 0.9f;   // only judge while genuinely still
static constexpr float    HEALTH_HOLD_S     = 4.0f;   // fault must persist this long to latch
static uint8_t  g_calib_health   = 0;
static float    g_health_bias_s  = 0.0f;   // seconds the bias fault has held
static uint32_t g_health_last_ms = 0;

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

// ── Boot-pose plausibility gate ──────────────────────────────────────────────
// The no-saved-zero fallback in setup() captures whatever pose the board is in
// at power-on and persists it forever. If the board boots tilted or moving, that
// bakes a bad zero into NVM. This gate refuses an implausible boot pose so the
// fallback only ever fires when the board is genuinely flat and still; otherwise
// the operator runs ZERO once (which persists) when they set it up.
static constexpr float BOOT_GATE_LEVEL_DEG  = 8.0f;   // |roll|,|pitch| must be under this
static constexpr float BOOT_GATE_G_BAND     = 0.08f;  // |accel|/1g must be within this of 1.0
static constexpr float BOOT_GATE_MIN_TRUST  = 0.85f;  // Mahony motion gate ≈1.0 when still

static bool bootPosePlausible() {
    if (!g_ema_ready) return false;
    // Flat: live accel magnitude near 1 g and the settled angle near level.
    float ax = g_imu.getAccelX(), ay = g_imu.getAccelY(), az = g_imu.getAccelZ();
    float norm_g = sqrtf(ax*ax + ay*ay + az*az) / 9.80665f;
    if (fabsf(norm_g - 1.0f) > BOOT_GATE_G_BAND) return false;
    if (fabsf(g_latest_ema.roll)  > BOOT_GATE_LEVEL_DEG) return false;
    if (fabsf(g_latest_ema.pitch) > BOOT_GATE_LEVEL_DEG) return false;
    // Still: reuse the Mahony motion/variance gate (≈1.0 when stationary).
    if (g_mahony.getLastTrust() < BOOT_GATE_MIN_TRUST) return false;
    return true;
}

// Update the calibration-health code from resting telemetry. Cheap, integer/
// float arithmetic only, runs in the main loop (never the BLE task). Only judges
// when the board is still; both faults need HEALTH_HOLD_S of continuous evidence
// before latching, and any healthy reading clears them — so the flag is stable
// and only set when recalibration is genuinely warranted.
static void updateCalibHealth(uint32_t now_ms) {
    float dt = (g_health_last_ms == 0) ? 0.0f : (now_ms - g_health_last_ms) * 1e-3f;
    g_health_last_ms = now_ms;
    if (dt <= 0.0f || dt > 5.0f) return;   // first tick or a long stall — skip

    // Only assess while the board is genuinely still; motion tells us nothing
    // about sensor health and would produce false positives.
    if (!g_ema_ready || g_mahony.getLastTrust() < HEALTH_MIN_TRUST) {
        g_health_bias_s = 0.0f;
        return;
    }

    // The ONLY health signal is gyro-bias drift: the magnitude of the Mahony
    // integral correction term. On a healthy board this stays small (well under
    // 1°/s) after the boot gyro calibration; it only grows large if the filter
    // is persistently fighting real drift (thermal/aging) — a true sensor signal
    // that, crucially, does NOT grow when the user merely leans the board. We do
    // NOT compare the resting angle to zero: while calibrating, the user holds
    // deliberate leans that are indistinguishable from a "crooked rest", so that
    // check produced false "re-zero" prompts during normal setup.
    constexpr float RAD2DPS = 57.2957795f;
    float bias_dps = sqrtf(g_mahony.getBiasX()*g_mahony.getBiasX() +
                           g_mahony.getBiasY()*g_mahony.getBiasY() +
                           g_mahony.getBiasZ()*g_mahony.getBiasZ()) * RAD2DPS;

    // Accumulate hold time (clamped); reset the moment it reads healthy.
    g_health_bias_s = (bias_dps > HEALTH_BIAS_DPS) ? fminf(g_health_bias_s + dt, HEALTH_HOLD_S + 1.0f) : 0.0f;

    if      (g_health_bias_s >= HEALTH_HOLD_S) g_calib_health = 1;   // drift suspected
    else if (g_health_bias_s == 0.0f)          g_calib_health = 0;   // healthy
    // (a partially-accumulated fault leaves the previous code unchanged → no flicker)
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

// ────────────────────────────────────────────────────────────────────────────
// Persisted tuning + tilt limits. A second NVM record (placed after the zero
// record) freezes the live tuning the dashboard chose plus the captured tilt
// limits, so they survive power cycles and every game sees the same calibration.
// All fields are simple scalars — EEPROM.put/get copy byte-by-byte so there is
// no alignment requirement on the stored layout.
// ────────────────────────────────────────────────────────────────────────────
struct PersistedTuning {
    uint32_t magic;        // sentinel — must equal TUNE_MAGIC to be trusted
    uint8_t  mode;         // FilterMode
    uint8_t  limit_mask;   // which limits are valid
    uint8_t  _pad0, _pad1; // keep the following floats 4-aligned in the record
    float    kp;
    float    var_thresh;
    float    ema_alpha;
    float    limit_front, limit_back, limit_left, limit_right;
};
static constexpr uint32_t TUNE_MAGIC      = 0x54554E45;  // 'TUNE'
static constexpr int      TUNE_EEPROM_ADDR = (int)(ZERO_EEPROM_ADDR + sizeof(PersistedZero) + 8);

static void saveTuning() {
    PersistedTuning rec{};
    rec.magic       = TUNE_MAGIC;
    rec.mode        = (uint8_t)g_filter_mode;
    rec.limit_mask  = g_limit_mask;
    rec.kp          = g_mahony.getKp();
    rec.var_thresh  = g_mahony.getVarThreshold();
    rec.ema_alpha   = g_ema_alpha;
    rec.limit_front = g_limit_front;
    rec.limit_back  = g_limit_back;
    rec.limit_left  = g_limit_left;
    rec.limit_right = g_limit_right;
    EEPROM.put(TUNE_EEPROM_ADDR, rec);
    // Integer-only log (mode + mask). Float fields are deliberately not printed
    // here — this can run in the BLE-host task, whose stack the float-printf path
    // would overflow. The full values are always readable via the snapshot.
    Serial.printf("[CMD] Tuning+limits saved (mode=%u mask=0x%02X)\n",
                  (unsigned)rec.mode, (unsigned)rec.limit_mask);
}

// Load persisted tuning into the live state. No-op (returns false) on a fresh
// chip / version mismatch so the boot defaults stand.
static bool loadTuning() {
    PersistedTuning rec{};
    EEPROM.get(TUNE_EEPROM_ADDR, rec);
    if (rec.magic != TUNE_MAGIC) return false;
    g_filter_mode = (rec.mode <= (uint8_t)FilterMode::Accel) ? (FilterMode)rec.mode : FilterMode::Fusion;
    g_mahony.setKp(rec.kp);
    g_mahony.setVarThreshold(rec.var_thresh);
    g_ema_alpha   = (rec.ema_alpha > 0.0f && rec.ema_alpha <= 1.0f) ? rec.ema_alpha : EMA_ALPHA_DEFAULT;
    g_limit_mask  = rec.limit_mask;
    g_limit_front = rec.limit_front;
    g_limit_back  = rec.limit_back;
    g_limit_left  = rec.limit_left;
    g_limit_right = rec.limit_right;
    return true;
}

// ── Command hooks shared by the serial + BLE command parsers ─────────────────
// All of these receive already-parsed scalars (the callers parseDecimal/atoi out
// of a null-terminated buffer that was memcpy'd off the wire), so none of them
// touch the raw BLE receive buffer directly. They also avoid float-printf (%f),
// because a BLE command dispatches them in the BLE-host task whose small stack
// the libc soft-float formatter overflows — the original boot hard-fault. The
// live float values are always available via the readable snapshot instead.

void setFilterMode(FilterMode m) {
    g_filter_mode = m;
    const char* n = (m == FilterMode::Gyro) ? "gyro" : (m == FilterMode::Accel) ? "accel" : "fusion";
    Serial.printf("[CMD] Filter mode = %s\n", n);
}
void setKp(float kp)            { g_mahony.setKp(kp);            Serial.println("[CMD] Kp updated"); }
void setVarThreshold(float v)   { g_mahony.setVarThreshold(v);  Serial.println("[CMD] Motion gate updated"); }
void setEmaAlpha(float a) {
    if (a < 0.01f) a = 0.01f; if (a > 1.0f) a = 1.0f;
    g_ema_alpha = a;
    Serial.println("[CMD] EMA alpha updated");
}

// Mean + peak-to-peak spread of the rolling window for one axis (false = window
// not full yet). use_pitch picks pitch (front/back) vs roll (left/right). Pure
// integer/float arithmetic — no printf — so it is safe in the BLE-host task.
static bool limitWindowStats(bool use_pitch, float& mean_out, float& spread_out) {
    if (!g_lw_full) return false;
    const float* w = use_pitch ? g_lw_pitch : g_lw_roll;
    double sum = 0.0;
    float lo = w[0], hi = w[0];
    for (uint16_t i = 0; i < LIMIT_WIN; i++) {
        float v = w[i];
        sum += v;
        if (v < lo) lo = v;
        if (v > hi) hi = v;
    }
    mean_out   = (float)(sum / (double)LIMIT_WIN);
    spread_out = hi - lo;
    return true;
}

// Capture a tilt-range edge as the AVERAGE of the last ~0.5 s of zeroed angle
// (front/back = pitch, left/right = roll), rejecting the capture if the board
// wasn't held steady. The rolling window is filled by pollSensor() in the main
// loop, so this stays non-blocking and BLE-host-task safe.
void captureLimit(uint8_t edge) {
    if (!g_ema_ready) { Serial.println("[CMD] LIMIT ignored: no sample yet"); return; }
    const bool use_pitch = (edge == LIMIT_FRONT || edge == LIMIT_BACK);
    float mean, spread;
    if (!limitWindowStats(use_pitch, mean, spread)) {
        Serial.println("[CMD] LIMIT rejected: still settling, try again");
        return;
    }
    if (spread > LIMIT_MAX_SPREAD) {
        // Integer-only log (no %f — BLE-host task stack). The dashboard learns of
        // the rejection by re-reading the snapshot (this edge's mask bit is unset).
        Serial.printf("[CMD] LIMIT rejected: hold steadier (spread %d deg)\n", (int)(spread + 0.5f));
        return;
    }
    switch (edge) {
        case LIMIT_FRONT: g_limit_front = mean; break;
        case LIMIT_BACK:  g_limit_back  = mean; break;
        case LIMIT_LEFT:  g_limit_left  = mean; break;
        case LIMIT_RIGHT: g_limit_right = mean; break;
        default: return;
    }
    g_limit_mask |= edge;
    // Integer-only log (no %f — see note above). Exact captured degrees are in
    // the snapshot the dashboard reads back.
    Serial.printf("[CMD] Limit captured (avg, mask=0x%02X)\n", (unsigned)g_limit_mask);
}
void clearLimits() {
    g_limit_mask = 0;
    g_limit_front = g_limit_back = g_limit_left = g_limit_right = 0.0f;
    Serial.println("[CMD] Tilt limits cleared (Save to persist)");
}
void persistTuning() { saveTuning(); }

// Append a fixed-point decimal of `v` with `decimals` fraction digits to `out`
// using only integer math — deliberately avoids the floating-point printf path,
// whose deep newlib stack frames overflow the small BLE-host task stack this
// snapshot is built on (the boot hard-fault we hit was a FreeRTOS STKOF in that
// task, escalated to HardFault). Returns the new write cursor.
static char* appendFixed(char* p, char* end, float v, int decimals) {
    if (p >= end) return p;
    // round-half-away-from-zero in the smallest represented unit
    int32_t scale = 1;
    for (int i = 0; i < decimals; i++) scale *= 10;
    bool neg = v < 0.0f;
    float av = neg ? -v : v;
    int32_t scaled = (int32_t)(av * (float)scale + 0.5f);
    int32_t ip = scaled / scale;
    int32_t fp = scaled % scale;
    int n = snprintf(p, (size_t)(end - p), "%s%ld.%0*ld",
                     neg ? "-" : "", (long)ip, decimals, (long)fp);
    if (n < 0) return p;
    p += n;
    if (p > end) p = end;
    return p;
}

// Build the readable command-characteristic snapshot the dashboards parse:
//   "TUNE,mode,kp,var,ema,front,back,left,right,mask,health"
// Writes into `out` (size `cap`) and returns the string length. Float-printf
// free on purpose (see appendFixed) — this runs in the BLE-host task context.
// `health` is the trailing integer field (0 OK / 1 drift / 2 zero off); older
// clients that only read fields 0..9 ignore it (backward compatible).
int buildTuningSnapshot(char* out, int cap) {
    if (cap <= 0) return 0;
    const char* mode = (g_filter_mode == FilterMode::Gyro) ? "gyro"
                     : (g_filter_mode == FilterMode::Accel) ? "accel" : "fusion";
    char* p   = out;
    char* end = out + cap - 1;   // leave room for NUL
    int n = snprintf(p, (size_t)(end - p), "TUNE,%s,", mode);
    if (n > 0) { p += n; if (p > end) p = end; }
    p = appendFixed(p, end, g_mahony.getKp(),           3); if (p < end) *p++ = ',';
    p = appendFixed(p, end, g_mahony.getVarThreshold(), 4); if (p < end) *p++ = ',';
    p = appendFixed(p, end, g_ema_alpha,                3); if (p < end) *p++ = ',';
    p = appendFixed(p, end, g_limit_front,              2); if (p < end) *p++ = ',';
    p = appendFixed(p, end, g_limit_back,               2); if (p < end) *p++ = ',';
    p = appendFixed(p, end, g_limit_left,               2); if (p < end) *p++ = ',';
    p = appendFixed(p, end, g_limit_right,              2); if (p < end) *p++ = ',';
    n = snprintf(p, (size_t)(end - p), "%u,", (unsigned)g_limit_mask);
    if (n > 0) { p += n; if (p > end) p = end; }
    n = snprintf(p, (size_t)(end - p), "%u", (unsigned)g_calib_health);
    if (n > 0) { p += n; if (p > end) p = end; }
    *p = '\0';
    return (int)(p - out);
}

// ── On-demand gyro recalibration (GYROCAL) ──────────────────────────────────
// SteadySteps' filter panel offers a "fix drift" button that sends GYROCAL.
// The command may arrive on the BLE-host task, which must never block — so the
// handler only sets this flag; loop() performs the actual ~800 ms calibration
// (board must be held still, same as the boot-time pass). Streaming pauses for
// that window, then resumes with the fresh bias.
static volatile bool g_gyrocal_pending = false;

void requestGyroCal() {
    g_gyrocal_pending = true;
    Serial.println("[CMD] Gyro recalibration queued — keep the board still");
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
    g_latest_raw = e;   // unsmoothed fused estimate (gyro-dominated, low-lag)
    const float a = g_ema_alpha;
    if (!g_ema_ready) {
        g_latest_ema = e;
        g_ema_ready  = true;
    } else {
        g_latest_ema.roll  = a * e.roll  + (1.0f - a) * g_latest_ema.roll;
        g_latest_ema.pitch = a * e.pitch + (1.0f - a) * g_latest_ema.pitch;
        g_latest_ema.yaw   = a * e.yaw   + (1.0f - a) * g_latest_ema.yaw;
    }
    // Feed the rolling capture window with the ZEROED angle, so captureLimit()
    // can average a settled hold instead of one instantaneous sample.
    g_lw_roll[g_lw_idx]  = g_latest_ema.roll  - g_zero_offset.roll;
    g_lw_pitch[g_lw_idx] = g_latest_ema.pitch - g_zero_offset.pitch;
    if (++g_lw_idx >= LIMIT_WIN) { g_lw_idx = 0; g_lw_full = true; }
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
    } else {
        // No saved zero (fresh/erased chip). Only persist the boot pose if it
        // passes the plausibility gate (flat + still); poll a little longer to
        // give the board a chance to settle before giving up.
        uint32_t gate_start = millis();
        while (!bootPosePlausible() && millis() - gate_start < 2000) {
            pollSensor();
        }
        if (bootPosePlausible()) {
            g_zero_offset = g_latest_ema;
            saveZeroOffset(g_zero_offset);
            Serial.printf("[INIT] No saved zero — captured boot pose: roll=%.2f pitch=%.2f yaw=%.2f\n",
                          g_zero_offset.roll, g_zero_offset.pitch, g_zero_offset.yaw);
        } else {
            // Implausible boot pose — do NOT write a bad zero to NVM. Use a
            // temporary level reference; the operator runs ZERO once when set up.
            g_zero_offset = {0.0f, 0.0f, 0.0f};
            Serial.println("[INIT] Boot pose not level/still — zero deferred (not saved); run ZERO when set up.");
        }
    }

    // Restore persisted tuning + tilt limits, if any. Falls back to the boot
    // defaults (config.h) on a fresh/erased chip.
    if (loadTuning()) {
        Serial.printf("[INIT] Loaded saved tuning: mode=%u kp=%.2f var=%.4f ema=%.2f limitMask=0x%02X\n",
                      (unsigned)g_filter_mode, g_mahony.getKp(), g_mahony.getVarThreshold(),
                      g_ema_alpha, (unsigned)g_limit_mask);
    } else {
        Serial.println("[INIT] No saved tuning — using defaults.");
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
    // 0. Deferred gyro recalibration (GYROCAL command). Runs here — never on
    //    the BLE-host task — because it blocks ~800 ms sampling the still
    //    board. The BLE stack keeps servicing the link meanwhile (4 s
    //    supervision timeout gives plenty of margin).
    if (g_gyrocal_pending) {
        g_gyrocal_pending = false;
        Serial.println("[CMD] Recalibrating gyro — keep the board still...");
        if (g_imu.calibrateGyro()) {
            // Fresh driver bias supersedes the integral correction accumulated
            // against the old one — clear it so it can't re-introduce drift.
            g_mahony.resetBias();
            Serial.println("[CMD] Gyro recalibrated");
        } else {
            Serial.println("[CMD] Gyro recalibration failed (board moving?) — kept previous bias");
        }
        g_imu.resetWatchdog();
    }

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
        // Source of the streamed Euler depends on the live filter mode:
        //   fusion → EMA-smoothed fused estimate (default, smoothest)
        //   gyro   → unsmoothed fused estimate (gyro-dominated, lowest lag)
        //   accel  → orientation straight from the gravity vector (no gyro)
        const EulerAngles& src =
            (g_filter_mode == FilterMode::Gyro) ? g_latest_raw : g_latest_ema;
        float r, p;
        if (g_filter_mode == FilterMode::Accel) {
            // Tilt from gravity: roll about X, pitch about Y, in the body frame
            // the IMU driver already remaps into. atan2 keeps it well-defined.
            float ax = g_imu.getAccelX(), ay = g_imu.getAccelY(), az = g_imu.getAccelZ();
            constexpr float R2D = 57.2957795f;
            r = atan2f(ay, az) * R2D;
            p = atan2f(-ax, sqrtf(ay*ay + az*az)) * R2D;
        } else {
            r = src.roll;
            p = src.pitch;
        }
        r -= g_zero_offset.roll;
        p -= g_zero_offset.pitch;
        float y = g_latest_ema.yaw - g_zero_offset.yaw;
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

    // 3a. Calibration-health assessment (~4 Hz; self-throttles via its own dt).
    {
        static uint32_t last_health_ms = 0;
        if (now_ms - last_health_ms >= 250) { last_health_ms = now_ms; updateCalibHealth(now_ms); }
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
