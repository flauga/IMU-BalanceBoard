#pragma once

#include <cstdint>

struct Quaternion {
    float w;  // real (scalar)
    float x;  // i
    float y;  // j
    float z;  // k
};

struct EulerAngles {
    float roll;   // ML tilt (degrees)
    float pitch;  // AP tilt (degrees)
    float yaw;    // heading (degrees)
};

enum class SystemState : uint8_t {
    INITIALIZING,
    CALIBRATING,
    READY
};
