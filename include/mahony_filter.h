#pragma once

#include "types.h"
#include "config.h"

// Mahony explicit complementary filter on SO(3).
// Uses accelerometer as gravity reference; no magnetometer needed.
// Adaptive gain gates accel trust on magnitude deviation from 1g and
// on a sliding-window variance gate that rejects dynamic motion.
class MahonyFilter {
public:
    void reset();

    // Feed one sample. ax/ay/az in m/s², gx/gy/gz in rad/s, dt in seconds.
    void update(float ax, float ay, float az,
                float gx, float gy, float gz,
                float dt);

    EulerAngles getEuler()      const;
    Quaternion  getQuaternion() const { return {q0_, q1_, q2_, q3_}; }

    // Integral-feedback bias estimate (rad/s). Useful for drift debugging.
    float getBiasX() const { return bx_; }
    float getBiasY() const { return by_; }
    float getBiasZ() const { return bz_; }

    // Effective trust factor (0..1) of the most recent accel sample.
    // Updated each call to update(); used by diagnostic logs.
    float getLastTrust() const { return last_trust_; }

private:
    float q0_ = 1.0f, q1_ = 0.0f, q2_ = 0.0f, q3_ = 0.0f;
    float bx_ = 0.0f, by_ = 0.0f, bz_ = 0.0f;  // integral bias estimate

    float   var_win_[MAHONY_VAR_WINDOW] = {};
    float   var_sum_    = 0.0f;
    float   var_sum_sq_ = 0.0f;
    uint8_t var_idx_    = 0;
    bool    var_full_   = false;
    float   last_trust_ = 0.0f;

    float computeKpEff(float norm_g) const;
};
