#pragma once

#include <Arduino.h>
#include <WiFi.h>
#include "config.h"
#include <WebSocketsServer.h>

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

    // Per-second stats counters. Reset every 1 s when printing.
    uint32_t _txFrames             = 0;  // frames sent over WS
    uint32_t _rxMsgs               = 0;  // text messages received
    uint32_t _lastStatsMs          = 0;
    uint32_t _lastTxMs             = 0;  // millis() at last sendFrame()
    uint32_t _maxGapMs             = 0;  // worst send-to-send gap in last second

    // Stall-attribution counters. All in microseconds, reset each second.
    uint32_t _maxSendFrameUs       = 0;  // worst single sendFrame() duration
    uint32_t _maxWsLoopUs          = 0;  // worst single _ws->loop() duration
    uint32_t _maxHttpPollUs        = 0;  // worst single _pollHttp() duration
    uint32_t _totWsLoopUs          = 0;  // cumulative _ws->loop() time
    uint32_t _wsLoopCalls          = 0;
    uint32_t _droppedFrames        = 0;  // frames skipped due to slow client / TCP backpressure

    // Send-watchdog: after a slow send (>SLOW_SEND_US), skip frames for a
    // cool-off period so the TCP send buffer can drain without blocking.
    static constexpr uint32_t SLOW_SEND_US  = 30000;   // 30 ms — anything over this is "stalled"
    static constexpr uint32_t COOLOFF_MS    = 100;     // drop frames for this long after a slow send
    uint32_t _coolOffUntilMs       = 0;

    // Per-client: send frames as binary (16-byte little-endian struct) when true,
    // text CSV when false. Toggled by the client sending "BIN ON" / "BIN OFF".
    bool _binMode[MAX_WS_CLIENTS]  = { false };

    bool _tryConnect();
    void _startServer();
    void _onEvent(uint8_t num, WStype_t type, uint8_t* payload, size_t length);
    void _pollHttp();
};
