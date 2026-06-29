#include "serial_command.h"
#include "config.h"
#include "imu_driver.h"
#include "ble_manager.h"
#include <cstring>
#include <cstdlib>
#include <Arduino.h>

// Defined in main.cpp
extern void zeroOrientation();
extern void clearZeroOrientation();
extern void setDriftLog(bool on);
extern void setSerialPrintEnabled(bool on);
extern void setSerialPrintDivider(uint32_t div);

// Runtime tuning + tilt-limit hooks (defined in the .ino). FilterMode lives in
// config.h, already included above. captureLimit takes a LIMIT_* bit.
extern void setFilterMode(FilterMode m);
extern void setKp(float kp);
extern void setVarThreshold(float v);
extern void setEmaAlpha(float a);
extern void captureLimit(uint8_t edge);
extern void clearLimits();
extern void persistTuning();
extern int  buildTuningSnapshot(char* out, int cap);

// LIMIT_* bits mirror the .ino definitions.
static constexpr uint8_t L_FRONT = 0x01, L_BACK = 0x02, L_LEFT = 0x04, L_RIGHT = 0x08;

// Lightweight decimal parser used instead of strtof/atof. The libc float-parse
// path drags in deep newlib stack frames that overflow the small BLE-host task
// stack these commands are dispatched on (the cause of the boot hard-fault).
// Handles optional sign, integer + fractional parts; no exponent (not needed
// for tuning values). Stops at the first non-numeric character.
static float parseDecimal(const char* s) {
    while (*s == ' ') s++;
    bool neg = false;
    if (*s == '+' || *s == '-') { neg = (*s == '-'); s++; }
    double v = 0.0;
    while (*s >= '0' && *s <= '9') { v = v * 10.0 + (*s - '0'); s++; }
    if (*s == '.') {
        s++;
        double frac = 0.1;
        while (*s >= '0' && *s <= '9') { v += (*s - '0') * frac; frac *= 0.1; s++; }
    }
    return (float)(neg ? -v : v);
}

// Shared command dispatcher used by BOTH the serial parser and the BLE command
// handler, so the two never drift. Returns true if the command was recognised.
// `cmd` must be a null-terminated, whitespace-trimmed string.
bool dispatchTuningCommand(const char* cmd) {
    if (strncasecmp(cmd, "MODE ", 5) == 0) {
        const char* m = cmd + 5;
        if      (strcasecmp(m, "fusion") == 0) setFilterMode(FilterMode::Fusion);
        else if (strcasecmp(m, "gyro")   == 0) setFilterMode(FilterMode::Gyro);
        else if (strcasecmp(m, "accel")  == 0) setFilterMode(FilterMode::Accel);
        else Serial.printf("[CMD] Unknown mode '%s'\n", m);
        return true;
    } else if (strncasecmp(cmd, "KP ", 3) == 0) {
        setKp(parseDecimal(cmd + 3));                   return true;
    } else if (strncasecmp(cmd, "VAR ", 4) == 0) {
        setVarThreshold(parseDecimal(cmd + 4));         return true;
    } else if (strncasecmp(cmd, "EMA ", 4) == 0) {
        setEmaAlpha(parseDecimal(cmd + 4));             return true;
    } else if (strncasecmp(cmd, "LIMIT ", 6) == 0) {
        const char* e = cmd + 6;
        if      (strcasecmp(e, "FRONT") == 0) captureLimit(L_FRONT);
        else if (strcasecmp(e, "BACK")  == 0) captureLimit(L_BACK);
        else if (strcasecmp(e, "LEFT")  == 0) captureLimit(L_LEFT);
        else if (strcasecmp(e, "RIGHT") == 0) captureLimit(L_RIGHT);
        else Serial.printf("[CMD] Unknown limit edge '%s'\n", e);
        return true;
    } else if (strcasecmp(cmd, "LIMITCLEAR") == 0) {
        clearLimits();                                  return true;
    } else if (strcasecmp(cmd, "SAVE") == 0) {
        persistTuning();                                return true;
    }
    return false;
}

void SerialCommand::begin() {
    buf_idx_ = 0;
}

void SerialCommand::poll() {
    while (Serial.available()) {
        char c = Serial.read();
        if (c == '\n' || c == '\r') {
            if (buf_idx_ > 0) {
                buffer_[buf_idx_] = '\0';
                processCommand(buffer_);
                buf_idx_ = 0;
            }
        } else if (buf_idx_ < BUF_SIZE - 1) {
            buffer_[buf_idx_++] = c;
        }
    }
}

void SerialCommand::processCommand(const char* cmd) {
    while (*cmd == ' ') cmd++;

    if (strcasecmp(cmd, "START") == 0) {
        if (streaming_) { *streaming_ = true; Serial.println("[CMD] Streaming started"); }
    } else if (strcasecmp(cmd, "STOP") == 0) {
        if (streaming_) { *streaming_ = false; Serial.println("[CMD] Streaming stopped"); }
    } else if (strcasecmp(cmd, "STATUS") == 0) {
        printStatus();
    } else if (strcasecmp(cmd, "HELP") == 0) {
        printHelp();
    } else if (strncasecmp(cmd, "RATE ", 5) == 0) {
        int hz = atoi(cmd + 5);
        if (hz < 1)  hz = 1;
        if (hz > 50) hz = 50;
        print_interval_ms_ = 1000 / (uint32_t)hz;
        Serial.printf("[CMD] Output rate set to %d Hz (%lu ms interval)\n",
                      hz, (unsigned long)print_interval_ms_);
    } else if (strcasecmp(cmd, "ZERO") == 0) {
        zeroOrientation();
    } else if (strcasecmp(cmd, "ZEROCLEAR") == 0) {
        clearZeroOrientation();
    } else if (strcasecmp(cmd, "DEBUG ON") == 0) {
        setDriftLog(true);
    } else if (strcasecmp(cmd, "DEBUG OFF") == 0) {
        setDriftLog(false);
    } else if (strcasecmp(cmd, "SERIAL ON") == 0) {
        setSerialPrintEnabled(true);
    } else if (strcasecmp(cmd, "SERIAL OFF") == 0) {
        setSerialPrintEnabled(false);
    } else if (strncasecmp(cmd, "SERIAL DIV ", 11) == 0) {
        int div = atoi(cmd + 11);
        if (div < 1) div = 1;
        setSerialPrintDivider((uint32_t)div);
    } else if (dispatchTuningCommand(cmd)) {
        // handled (MODE/KP/VAR/EMA/LIMIT/LIMITCLEAR/SAVE)
    } else {
        Serial.printf("[CMD] Unknown command: '%s'. Type HELP.\n", cmd);
    }
}

void SerialCommand::printHelp() {
    Serial.println("=== IMU Balance Board Commands ===");
    Serial.println("  START       Begin streaming angle data");
    Serial.println("  STOP        Pause streaming");
    Serial.println("  STATUS      Show current state");
    Serial.println("  RATE <hz>   Set output rate (1-50 Hz)");
    Serial.println("  ZERO        Capture current orientation as new zero (saved to NVM)");
    Serial.println("  ZEROCLEAR   Forget saved zero; next boot uses boot pose");
    Serial.println("  DEBUG ON    Enable drift diagnostic log (1 Hz)");
    Serial.println("  DEBUG OFF   Disable drift diagnostic log");
    Serial.println("  SERIAL ON   Enable per-frame angle print on UART");
    Serial.println("  SERIAL OFF  Disable per-frame angle print on UART");
    Serial.println("  SERIAL DIV <n> Print 1 of every n frames (default 5)");
    Serial.println("  MODE <m>    Filter source: fusion | gyro | accel");
    Serial.println("  KP <v>      Mahony proportional gain (live)");
    Serial.println("  VAR <v>     Motion-gate threshold g² (live)");
    Serial.println("  EMA <v>     Output smoothing alpha 0.01-1 (live)");
    Serial.println("  LIMIT <e>   Capture tilt edge: FRONT|BACK|LEFT|RIGHT");
    Serial.println("  LIMITCLEAR  Forget captured tilt limits");
    Serial.println("  SAVE        Persist tuning + tilt limits to NVM");
    Serial.println("  HELP        Show this help");
}

void SerialCommand::printStatus() {
    Serial.println("=== Status ===");
    Serial.printf("  Firmware:  v%s (LSM6DS3 + Mahony, BLE)\n", FIRMWARE_VERSION);
    Serial.printf("  Streaming: %s\n", (streaming_ && *streaming_) ? "yes" : "no");
    Serial.printf("  Rate:      %lu ms (%lu Hz)\n",
                  (unsigned long)print_interval_ms_,
                  (unsigned long)(1000 / print_interval_ms_));
    if (ble_ && ble_->isConnected()) {
        Serial.printf("  BLE:       advertising as %s\n", BLE_DEVICE_NAME);
        Serial.printf("  Central:   %s\n", ble_->hasClient() ? "connected" : "none");
    } else {
        Serial.println("  BLE:       not initialised");
    }
}
