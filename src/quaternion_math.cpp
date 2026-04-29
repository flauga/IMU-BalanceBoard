#include "quaternion_math.h"
#include <cmath>

namespace quat {

Quaternion identity() {
    return {1.0f, 0.0f, 0.0f, 0.0f};
}

Quaternion multiply(const Quaternion& a, const Quaternion& b) {
    return {
        a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z,
        a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
        a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
        a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w
    };
}

Quaternion conjugate(const Quaternion& q) {
    return {q.w, -q.x, -q.y, -q.z};
}

Quaternion normalize(const Quaternion& q) {
    float mag = sqrtf(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z);
    if (mag < 1e-10f) {
        return identity();
    }
    float inv = 1.0f / mag;
    return {q.w * inv, q.x * inv, q.y * inv, q.z * inv};
}

EulerAngles toEuler(const Quaternion& q) {
    // Roll (ML tilt)
    float sinr_cosp = 2.0f * (q.w * q.x + q.y * q.z);
    float cosr_cosp = 1.0f - 2.0f * (q.x * q.x + q.y * q.y);
    float roll = atan2f(sinr_cosp, cosr_cosp) * (180.0f / (float)M_PI);

    // Pitch (AP tilt)
    float sinp = 2.0f * (q.w * q.y - q.z * q.x);
    if (sinp > 1.0f) sinp = 1.0f;
    if (sinp < -1.0f) sinp = -1.0f;
    float pitch = asinf(sinp) * (180.0f / (float)M_PI);

    // Yaw (heading — gyro-integrated only, no magnetometer)
    float siny_cosp = 2.0f * (q.w * q.z + q.x * q.y);
    float cosy_cosp = 1.0f - 2.0f * (q.y * q.y + q.z * q.z);
    float yaw = atan2f(siny_cosp, cosy_cosp) * (180.0f / (float)M_PI);

    return {roll, pitch, yaw};
}

}  // namespace quat
