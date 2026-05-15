#pragma once

#include <Arduino.h>
#include <WiFi.h>
#include "config.h"
#include <WebSocketsServer.h>
#include <atomic>

class SerialCommand;

class WifiManager {
public:
    bool begin();
    void poll();
    void sendFrame(uint32_t ms, float roll, float pitch, float yaw);

    // Send status JSON. targetClient >= 0 sends to that client only; -1 broadcasts.
    void sendStatus(int8_t targetClient = -1);

    bool    isConnected()  const { return _wifiOk; }
    bool    hasClient()    const { return _clientCount > 0; }
    uint8_t clientCount()  const { return (uint8_t)(_clientCount > 0 ? _clientCount : 0); }

    void setStreamingFlag(bool* flag)   { _streaming = flag; }
    void setSerial(SerialCommand* ser)  { _serial = ser; }

private:
    // Match the WebSocketsServer library's compile-time client max
    // (WEBSOCKETS_SERVER_CLIENT_MAX, default 5). The library bounds-checks
    // internally, but matching here keeps our per-client state array sized right.
    static constexpr uint8_t MAX_WS_CLIENTS = 5;

    WebSocketsServer* _ws          = nullptr;
    WiFiServer*       _http        = nullptr;  // synchronous HTTP, only serves DASHBOARD_HTML
    bool    _wifiOk                = false;
    int8_t  _clientCount           = 0;   // signed to prevent underflow
    bool*   _streaming             = nullptr;
    SerialCommand* _serial         = nullptr;
    uint32_t _lastReconnectMs      = 0;
    uint32_t _lastStatusMs         = 0;

    // Diagnostics for the once-per-second [STATS] line.
    uint32_t _txFrames             = 0;  // frames sent over WS in the last second
    uint32_t _lastTxMs             = 0;  // millis() of the previous sendFrame()
    uint32_t _maxGapMs             = 0;  // largest inter-send gap in the last second
    uint32_t _lastStatsMs          = 0;

    // Per-client: send frames as binary (16-byte little-endian struct) when true,
    // text CSV when false. Toggled by the client sending "BIN ON" / "BIN OFF".
    bool _binMode[MAX_WS_CLIENTS]  = { false };

    // ── Dedicated TX task ────────────────────────────────────────────────
    // sendFrame() is called from the Arduino loop task. Instead of writing
    // to the WS library inline (which puts the WS work on the same task
    // that can be preempted by WiFi driver bookkeeping), it publishes the
    // latest sample to a one-slot snapshot. A dedicated FreeRTOS task at
    // priority 5 on core 1 drains that snapshot at 50 Hz and runs the
    // WS library's loop continuously. Higher priority than Arduino loop
    // (prio 1) means this task gets CPU even when the loop is starving.
    struct TxSnapshot {
        uint32_t ms;
        float    roll, pitch, yaw;
    };
    TxSnapshot              _txSnap        = {};
    std::atomic<uint32_t>   _txSnapSeq{0};   // even = stable, odd = writing
    std::atomic<uint32_t>   _txSnapPubCount{0};   // incremented per publish
    uint32_t                _txSnapLastSent  = 0; // pub-count last consumed
    volatile bool           _wifiTxRun       = false;
    TaskHandle_t            _wifiTxHandle    = nullptr;

    bool _tryConnect();
    void _startServer();
    void _onEvent(uint8_t num, WStype_t type, uint8_t* payload, size_t length);
    void _pollHttp();
    void _txTaskLoop();
    static void _txTaskTrampoline(void* arg);
    void _broadcastSnapshot(const TxSnapshot& s);
};
