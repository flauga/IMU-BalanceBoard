#pragma once

#include <Arduino.h>
#include <WiFi.h>
#include <ESPmDNS.h>
#include <WebSocketsServer.h>
#include <ESPAsyncWebServer.h>

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
    WebSocketsServer* _ws          = nullptr;
    AsyncWebServer*   _http        = nullptr;
    bool    _wifiOk                = false;
    int8_t  _clientCount           = 0;   // signed to prevent underflow
    bool*   _streaming             = nullptr;
    SerialCommand* _serial         = nullptr;
    uint32_t _lastReconnectMs      = 0;

    bool _tryConnect();
    void _startServer();
    void _onEvent(uint8_t num, WStype_t type, uint8_t* payload, size_t length);
};
