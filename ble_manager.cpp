#include "ble_manager.h"
#include "config.h"
#include "serial_command.h"

extern "C" {
#include "sl_bt_api.h"
#include "sl_bgapi.h"
}

#include <cstring>
#include <cstdlib>

// Defined in main.cpp — same hooks used by SerialCommand.
extern void zeroOrientation();
extern void clearZeroOrientation();
extern void setDriftLog(bool on);

// ────────────────────────────────────────────────────────────────────────────
// Singleton + global BGAPI event trampoline.
// The Silabs stack delivers events to a free function named sl_bt_on_event().
// We forward into the singleton so we can hold per-instance state cleanly.
// ────────────────────────────────────────────────────────────────────────────
BleManager& BleManager::instance() {
    static BleManager _inst;
    return _inst;
}

extern "C" void sl_bt_on_event(sl_bt_msg_t* evt) {
    BleManager::instance().onBgapiEvent(evt);
}

// ────────────────────────────────────────────────────────────────────────────
// 16-byte UUIDs in little-endian byte order (BGAPI's wire format).
// String form 6e400001-b5a3-f393-e0a9-e50e24dcca9e — emitted LSB first.
// ────────────────────────────────────────────────────────────────────────────
static const uint8_t kImuServiceUuid[16] = {
    0x9e, 0xca, 0xdc, 0x24, 0x0e, 0xe5, 0xa9, 0xe0,
    0x93, 0xf3, 0xa3, 0xb5, 0x01, 0x00, 0x40, 0x6e
};
static const uint8_t kAnglesCharUuid[16] = {
    0x9e, 0xca, 0xdc, 0x24, 0x0e, 0xe5, 0xa9, 0xe0,
    0x93, 0xf3, 0xa3, 0xb5, 0x02, 0x00, 0x40, 0x6e
};
static const uint8_t kCommandCharUuid[16] = {
    0x9e, 0xca, 0xdc, 0x24, 0x0e, 0xe5, 0xa9, 0xe0,
    0x93, 0xf3, 0xa3, 0xb5, 0x03, 0x00, 0x40, 0x6e
};

static const uint8_t kDeviceName[]    = BLE_DEVICE_NAME;
static const size_t  kDeviceNameLen   = sizeof(kDeviceName) - 1;
static uint8_t       kAdvertisingSet  = 0xff;

bool BleManager::begin() {
    // Stack initialization is performed by the framework. By the time setup()
    // runs the stack has already been booted, so all we do here is record the
    // "tried to begin" state — actual GATT DB creation happens in the boot
    // event handler below. We also seed _bleOk so STATUS reports correctly
    // before any central connects.
    _bleOk = true;
    Serial.println("[BLE] BleManager ready, waiting for sl_bt_evt_system_boot_id");
    return true;
}

void BleManager::poll() {
    // BGAPI is fully interrupt + event driven; there's no equivalent of
    // ArduinoBLE::poll(). All we do here is print the once-per-second
    // [STATS] line so log parsers built for the ESP32 keep working.
    extern volatile uint32_t g_sample_count;
    static uint32_t last_sample_count = 0;
    uint32_t now_ms = millis();
    if (now_ms - _lastStatsMs >= 1000) {
        uint32_t samples = g_sample_count - last_sample_count;
        last_sample_count = g_sample_count;
        Serial.printf("[STATS] tx=%lu/s samples=%lu/s maxGap=%lums clients=%u\n",
                      (unsigned long)_txFrames,
                      (unsigned long)samples,
                      (unsigned long)_maxGapMs,
                      (unsigned)clientCount());
        _txFrames    = 0;
        _maxGapMs    = 0;
        _lastStatsMs = now_ms;
    }
}

void BleManager::sendFrame(uint32_t ms, float roll, float pitch, float yaw) {
    if (!_hasCentral || !_notifyEnabled) return;

    Frame f{ms, roll, pitch, yaw};
    sl_status_t sc = sl_bt_gatt_server_send_notification(
        _connectionHandle,
        _anglesChar,
        sizeof(f),
        (const uint8_t*)&f);
    if (sc != SL_STATUS_OK) return;

    if (_lastTxMs != 0) {
        uint32_t gap = ms - _lastTxMs;
        if (gap > _maxGapMs) _maxGapMs = gap;
    }
    _lastTxMs = ms;
    _txFrames++;
}

void BleManager::sendStatus(int8_t /*targetClient*/) {
    // BLE central polls STATUS via the command characteristic and reads the
    // resulting UART log line. Kept as a no-op to preserve API parity with
    // the ESP32 WifiManager.
}

void BleManager::onBgapiEvent(void* evtPtr) {
    sl_bt_msg_t* evt = (sl_bt_msg_t*)evtPtr;
    switch (SL_BT_MSG_ID(evt->header)) {
        case sl_bt_evt_system_boot_id: {
            Serial.println("[BLE] stack booted");
            _initGattDb();
            _startAdvertising();
            Serial.println("========================================");
            Serial.printf( "  BLE device:  %s\n", BLE_DEVICE_NAME);
            Serial.printf( "  Service UUID:%s\n", BLE_IMU_SERVICE_UUID);
            Serial.println("========================================");
            Serial.println("[BLE] Advertising. Connect with any Web-Bluetooth client.");
            break;
        }
        case sl_bt_evt_connection_opened_id: {
            _hasCentral       = true;
            _connectionHandle = evt->data.evt_connection_opened.connection;
            Serial.println("[BLE] central connected");
            break;
        }
        case sl_bt_evt_connection_closed_id: {
            _hasCentral    = false;
            _notifyEnabled = false;
            Serial.println("[BLE] central disconnected — re-advertising");
            _startAdvertising();
            break;
        }
        case sl_bt_evt_gatt_server_attribute_value_id: {
            auto& w = evt->data.evt_gatt_server_attribute_value;
            if (w.attribute == _cmdChar) {
                _handleCommandWrite(w.value.data, w.value.len);
            }
            break;
        }
        case sl_bt_evt_gatt_server_characteristic_status_id: {
            auto& s = evt->data.evt_gatt_server_characteristic_status;
            if (s.characteristic == _anglesChar) {
                _notifyEnabled = (s.client_config_flags & sl_bt_gatt_notification) != 0;
                Serial.printf("[BLE] notifications %s\n",
                              _notifyEnabled ? "enabled" : "disabled");
            }
            break;
        }
        default:
            break;
    }
}

void BleManager::_initGattDb() {
    sl_status_t sc;
    sc = sl_bt_gattdb_new_session(&_gattdbSession);
    if (sc != SL_STATUS_OK) return;

    // Generic Access (0x1800) with Device Name characteristic (0x2A00).
    const uint8_t gaUuid[]      = { 0x00, 0x18 };
    sc = sl_bt_gattdb_add_service(_gattdbSession,
                                  sl_bt_gattdb_primary_service,
                                  SL_BT_GATTDB_ADVERTISED_SERVICE,
                                  sizeof(gaUuid), gaUuid,
                                  &_genericAccessSvc);
    const sl_bt_uuid_16_t nameUuid = { .data = { 0x00, 0x2A } };
    sl_bt_gattdb_add_uuid16_characteristic(_gattdbSession,
                                           _genericAccessSvc,
                                           SL_BT_GATTDB_CHARACTERISTIC_READ,
                                           0x00, 0x00,
                                           nameUuid,
                                           sl_bt_gattdb_fixed_length_value,
                                           kDeviceNameLen, kDeviceNameLen,
                                           kDeviceName,
                                           &_deviceNameChar);
    sl_bt_gattdb_start_service(_gattdbSession, _genericAccessSvc);

    // IMU service.
    sc = sl_bt_gattdb_add_service(_gattdbSession,
                                  sl_bt_gattdb_primary_service,
                                  SL_BT_GATTDB_ADVERTISED_SERVICE,
                                  sizeof(kImuServiceUuid), kImuServiceUuid,
                                  &_imuSvc);

    // Angles notify characteristic — 16 B payload, READ + NOTIFY.
    {
        uuid_128 uuid;
        memcpy(uuid.data, kAnglesCharUuid, 16);
        Frame initial{};
        sl_bt_gattdb_add_uuid128_characteristic(_gattdbSession,
                                                _imuSvc,
                                                SL_BT_GATTDB_CHARACTERISTIC_READ | SL_BT_GATTDB_CHARACTERISTIC_NOTIFY,
                                                0x00, 0x00,
                                                uuid,
                                                sl_bt_gattdb_fixed_length_value,
                                                sizeof(initial),
                                                sizeof(initial),
                                                (const uint8_t*)&initial,
                                                &_anglesChar);
    }

    // Command write characteristic — 64 B max, WRITE + WRITE_WITHOUT_RESPONSE.
    {
        uuid_128 uuid;
        memcpy(uuid.data, kCommandCharUuid, 16);
        uint8_t empty = 0;
        sl_bt_gattdb_add_uuid128_characteristic(_gattdbSession,
                                                _imuSvc,
                                                SL_BT_GATTDB_CHARACTERISTIC_WRITE |
                                                SL_BT_GATTDB_CHARACTERISTIC_WRITE_NO_RESPONSE,
                                                0x00, 0x00,
                                                uuid,
                                                sl_bt_gattdb_variable_length_value,
                                                64, sizeof(empty),
                                                &empty,
                                                &_cmdChar);
    }

    sl_bt_gattdb_start_service(_gattdbSession, _imuSvc);
    sl_bt_gattdb_commit(_gattdbSession);
}

void BleManager::_startAdvertising() {
    sl_status_t sc;
    static bool init_done = false;
    if (!init_done) {
        sc = sl_bt_advertiser_create_set(&kAdvertisingSet);
        if (sc != SL_STATUS_OK) return;
        sl_bt_advertiser_set_timing(kAdvertisingSet,
                                    160, 160,  // ~100 ms (units of 0.625 ms)
                                    0, 0);
        init_done = true;
    }
    sl_bt_legacy_advertiser_generate_data(kAdvertisingSet,
                                          sl_bt_advertiser_general_discoverable);
    sl_bt_legacy_advertiser_start(kAdvertisingSet,
                                  sl_bt_advertiser_connectable_scannable);
}

void BleManager::_handleCommandWrite(const uint8_t* data, uint8_t len) {
    char buf[65] = {0};
    if (len > 64) len = 64;
    memcpy(buf, data, len);
    buf[len] = '\0';

    // Strip trailing newline / cr.
    for (int i = (int)len - 1; i >= 0 && (buf[i] == '\n' || buf[i] == '\r'); i--) {
        buf[i] = '\0';
    }

    const char* cmd = buf;
    while (*cmd == ' ') cmd++;
    Serial.printf("[BLE] cmd: '%s'\n", cmd);

    if (strcasecmp(cmd, "START") == 0) {
        if (_streaming) { *_streaming = true; Serial.println("[CMD] Streaming started"); }
    } else if (strcasecmp(cmd, "STOP") == 0) {
        if (_streaming) { *_streaming = false; Serial.println("[CMD] Streaming stopped"); }
    } else if (strcasecmp(cmd, "ZERO") == 0) {
        zeroOrientation();
    } else if (strcasecmp(cmd, "ZEROCLEAR") == 0) {
        clearZeroOrientation();
    } else if (strcasecmp(cmd, "DEBUG ON") == 0) {
        setDriftLog(true);
    } else if (strcasecmp(cmd, "DEBUG OFF") == 0) {
        setDriftLog(false);
    } else if (strncasecmp(cmd, "RATE ", 5) == 0) {
        int hz = atoi(cmd + 5);
        if (hz < 1)  hz = 1;
        if (hz > 50) hz = 50;
        if (_serial) _serial->setPrintIntervalMs(1000 / (uint32_t)hz);
        Serial.printf("[CMD] Output rate set to %d Hz\n", hz);
    } else {
        Serial.printf("[CMD] Unknown BLE command: '%s'\n", cmd);
    }
}
