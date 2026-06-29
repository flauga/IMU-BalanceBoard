#pragma once

#include <Arduino.h>
#include "config.h"

class IMUDriver;
class BleManager;

// Shared command dispatcher for the runtime tuning + tilt-limit commands
// (MODE/KP/VAR/EMA/LIMIT/LIMITCLEAR/SAVE). Used by both the serial parser and
// the BLE command handler so they stay in lockstep. `cmd` must be a
// null-terminated, whitespace-trimmed string. Returns true if recognised.
bool dispatchTuningCommand(const char* cmd);

class SerialCommand {
public:
    void begin();
    void poll();

    void setIMU(IMUDriver* imu)          { imu_       = imu; }
    void setStreamingFlag(bool* flag)    { streaming_ = flag; }
    void setBle(BleManager* ble)         { ble_       = ble; }

    uint32_t getPrintIntervalMs() const  { return print_interval_ms_; }
    void     setPrintIntervalMs(uint32_t ms) { print_interval_ms_ = ms; }

private:
    static constexpr uint8_t BUF_SIZE = 64;
    char buffer_[BUF_SIZE];
    uint8_t buf_idx_ = 0;

    IMUDriver*   imu_       = nullptr;
    bool*        streaming_ = nullptr;
    BleManager*  ble_       = nullptr;

    uint32_t print_interval_ms_ = SERIAL_PRINT_INTERVAL_MS;

    void processCommand(const char* cmd);
    void printHelp();
    void printStatus();
};
