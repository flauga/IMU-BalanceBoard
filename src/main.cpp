#include <Arduino.h>
#include "config.h"
#include "types.h"
#include "imu_driver.h"
#include "quaternion_math.h"
#include "serial_command.h"
#include "wifi_manager.h"

// --- globals ---

static IMUDriver     g_imu;
static SerialCommand g_serial;
static WifiManager   g_wifi;

static uint32_t    g_last_print_ms = 0;
static EulerAngles g_euler = {0.0f, 0.0f, 0.0f};
static bool        g_streaming = true;

// --- Arduino entry points ---

void setup() {
    Serial.begin(SERIAL_BAUD_RATE);

    Serial.println();
    Serial.println("========================================");
    Serial.println("  IMU Balance Board Firmware v" FIRMWARE_VERSION);
    Serial.println("  ESP32 + BNO085 (I2C, on-chip fusion)");
    Serial.println("========================================");

    Serial.println("[INIT] Initializing BNO085 over I2C...");

    if (!g_imu.begin()) {
        Serial.println("[INIT] FATAL: BNO085 initialization failed. Halting.");
        while (true) { delay(1000); }
    }

    // Drain feature-response packets and wait for the first real quaternion.
    // The BNO085 sends acknowledgment packets before data begins flowing.
    {
        uint32_t t = millis();
        while (!g_imu.hasNewQuaternion() && millis() - t < 3000) {
            g_imu.checkReset();
            while (g_imu.update()) {}
        }
        if (g_imu.hasNewQuaternion()) {
            Serial.println("[INIT] First quaternion received.");
        } else {
            Serial.println("[INIT] WARNING: No quaternion within 3s — continuing anyway.");
        }
    }

    g_serial.setIMU(&g_imu);
    g_serial.setStreamingFlag(&g_streaming);
    g_serial.setWifi(&g_wifi);
    g_serial.begin();

    g_wifi.setStreamingFlag(&g_streaming);
    g_wifi.setSerial(&g_serial);
    g_wifi.begin();

    Serial.println("[INIT] READY. Type HELP for commands.");
    Serial.printf("[INIT] Free heap: %lu bytes\n", (unsigned long)ESP.getFreeHeap());
}

void loop() {
    // 1. Check for spontaneous BNO085 reset
    g_imu.checkReset();

    // 2. Drain all pending IMU events
    while (g_imu.update()) {}

    // 3. Process serial commands
    g_serial.poll();

    // 4. Convert latest quaternion to Euler angles
    if (g_imu.hasNewQuaternion()) {
        Quaternion q = {
            g_imu.getQuatW(),
            g_imu.getQuatX(),
            g_imu.getQuatY(),
            g_imu.getQuatZ()
        };
        g_euler = quat::toEuler(q);
        g_imu.clearNewQuaternion();
    }

    // 5. Periodic output at configured rate (serial + WiFi)
    uint32_t now_ms = millis();
    if (g_streaming && now_ms - g_last_print_ms >= g_serial.getPrintIntervalMs()) {
        g_last_print_ms = now_ms;
        Serial.printf("%lu,%.2f,%.2f,%.2f\n",
                      (unsigned long)now_ms,
                      g_euler.roll, g_euler.pitch, g_euler.yaw);
        g_wifi.sendFrame(now_ms, g_euler.roll, g_euler.pitch, g_euler.yaw);
    }

    // 6. Service WiFi WebSocket (must run every loop)
    g_wifi.poll();
}
