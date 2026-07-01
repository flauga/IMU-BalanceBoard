#pragma once

#include <Arduino.h>
#include "config.h"

class SerialCommand;

// BLE manager built on the Silicon Labs BGAPI (sl_bt_*). PlatformIO's Silabs
// Arduino framework links libble_bgapi.a / libble_host.a as part of the Matter
// variant, so the calls resolve at link time without needing ArduinoBLE.
// User code receives stack events via the C linkage callback sl_bt_on_event(),
// which we forward to BleManager::instance()._onEvent().
class BleManager {
public:
    static BleManager& instance();   // singleton — needed for C-linkage callback

    bool begin();
    void poll();   // BGAPI is event-driven; poll() handles periodic upkeep.
    void sendFrame(uint32_t ms, float roll, float pitch, float yaw);

    // API parity with the ESP32 WifiManager. BLE has no implicit broadcast for
    // ad-hoc strings — kept as a no-op so SerialCommand compiles unchanged.
    void sendStatus(int8_t targetClient = -1);

    bool    isConnected() const { return _bleOk; }
    bool    hasClient()   const { return _hasCentral; }
    uint8_t clientCount() const { return _hasCentral ? 1 : 0; }

    void setStreamingFlag(bool* flag)   { _streaming = flag; }
    void setSerial(SerialCommand* ser)  { _serial = ser; }

    // Called from the global sl_bt_on_event() trampoline.
    void onBgapiEvent(void* evt);

private:
    BleManager() = default;

    // 16-byte binary frame format — identical to the ESP32 firmware's BIN mode.
    struct __attribute__((packed)) Frame {
        uint32_t ms;
        float    roll, pitch, yaw;
    };

    bool        _bleOk            = false;
    bool        _hasCentral       = false;
    uint8_t     _connectionHandle = 0xFF;
    bool        _notifyEnabled    = false;
    bool        _reqFastPending   = false;   // a fast-interval re-request is outstanding (anti ping-pong)
    bool*       _streaming        = nullptr;
    SerialCommand* _serial        = nullptr;

    uint16_t    _gattdbSession         = 0;
    uint16_t    _genericAccessSvc      = 0;
    uint16_t    _deviceNameChar        = 0;
    uint16_t    _imuSvc                = 0;
    uint16_t    _anglesChar            = 0;
    uint16_t    _cmdChar               = 0;
    uint16_t    _boardIdChar           = 0;

    // Stable per-board identity string (e.g. "MG24-AABBCCDDEEFF"), derived from
    // the radio's own Bluetooth address at boot. Read-only over BLE so a client
    // can tell which physical board it's connected to (per-user score keying).
    char        _boardId[20]           = {0};
    uint8_t     _boardIdLen            = 0;

    // Diagnostics for the once-per-second [STATS] line, mirroring ESP32.
    uint32_t _txFrames    = 0;
    uint32_t _lastTxMs    = 0;
    uint32_t _maxGapMs    = 0;
    uint32_t _lastStatsMs = 0;

    void _initGattDb();
    void _startAdvertising();
    void _handleCommandWrite(const uint8_t* data, uint8_t len);
    void _publishSnapshot();   // refresh the readable tuning/limit CSV snapshot
};
