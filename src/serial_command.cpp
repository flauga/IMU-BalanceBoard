#include "serial_command.h"
#include "config.h"
#include "imu_driver.h"
#include "wifi_manager.h"
#include <WiFi.h>
#include <cstring>
#include <cstdlib>
#include <Arduino.h>

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
    Serial.println("  HELP        Show this help");
}

void SerialCommand::printStatus() {
    Serial.println("=== Status ===");
    Serial.printf("  Firmware:  v%s\n", FIRMWARE_VERSION);
    Serial.printf("  Streaming: %s\n", (streaming_ && *streaming_) ? "yes" : "no");
    Serial.printf("  Rate:      %lu ms (%lu Hz)\n",
                  (unsigned long)print_interval_ms_,
                  (unsigned long)(1000 / print_interval_ms_));
    Serial.printf("  Free heap: %lu bytes\n", (unsigned long)ESP.getFreeHeap());
    if (wifi_ && wifi_->isConnected()) {
        Serial.printf("  WiFi IP:   %s\n", WiFi.localIP().toString().c_str());
        Serial.printf("  WS port:   %u\n", WIFI_WS_PORT);
        Serial.printf("  WS clients:%u\n", wifi_->clientCount());
    } else {
        Serial.println("  WiFi:      not connected");
    }
}
