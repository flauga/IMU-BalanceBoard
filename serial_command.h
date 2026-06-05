#pragma once

#include <Arduino.h>
#include "config.h"

class IMUDriver;
class BleManager;

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
