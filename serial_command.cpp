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
