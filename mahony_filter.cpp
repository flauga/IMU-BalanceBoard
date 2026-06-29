#include "mahony_filter.h"
#include <cmath>

void MahonyFilter::reset() {
    q0_ = 1.0f; q1_ = 0.0f; q2_ = 0.0f; q3_ = 0.0f;
    bx_ = 0.0f; by_ = 0.0f; bz_ = 0.0f;
    for (uint8_t i = 0; i < MAHONY_VAR_WINDOW; i++) var_win_[i] = 0.0f;
    var_sum_ = 0.0f; var_sum_sq_ = 0.0f;
    var_idx_ = 0; var_full_ = false;
    last_trust_ = 0.0f;
}

float MahonyFilter::computeKpEff(float norm_g) const {
    float mag_err   = fabsf(norm_g - 1.0f);
    float mag_trust = fmaxf(0.0f, 1.0f - MAHONY_ACCEL_GATE * mag_err);

    float var_trust = 1.0f;
    if (var_full_) {
        uint8_t n      = MAHONY_VAR_WINDOW;
        float mean     = var_sum_ / (float)n;
        float variance = (var_sum_sq_ / (float)n) - (mean * mean);
        if (variance < 0.0f) variance = 0.0f;
        var_trust = fmaxf(0.0f, 1.0f - variance / var_thresh_);
    }

    return kp_ * mag_trust * var_trust;
}

void MahonyFilter::update(float ax, float ay, float az,
                           float gx, float gy, float gz,
                           float dt) {
    float norm = sqrtf(ax*ax + ay*ay + az*az);

    // Update variance window with current |a| in g units
    float norm_g = (norm > 1e-6f) ? (norm / 9.80665f) : 0.0f;
    {
        float old_val       = var_win_[var_idx_];
        var_sum_           -= old_val;
        var_sum_sq_        -= old_val * old_val;
        var_win_[var_idx_]  = norm_g;
        var_sum_           += norm_g;
        var_sum_sq_        += norm_g * norm_g;
        var_idx_++;
        if (var_idx_ >= MAHONY_VAR_WINDOW) { var_idx_ = 0; var_full_ = true; }
    }

    // Gyro-only integration when accel is near zero (free-fall / invalid)
    if (norm < 1e-6f) {
        gx -= bx_; gy -= by_; gz -= bz_;
        float dq0 = 0.5f * (-q1_*gx - q2_*gy - q3_*gz);
        float dq1 = 0.5f * ( q0_*gx + q2_*gz - q3_*gy);
        float dq2 = 0.5f * ( q0_*gy - q1_*gz + q3_*gx);
        float dq3 = 0.5f * ( q0_*gz + q1_*gy - q2_*gx);
        q0_ += dq0*dt; q1_ += dq1*dt; q2_ += dq2*dt; q3_ += dq3*dt;
        float qn = 1.0f / sqrtf(q0_*q0_ + q1_*q1_ + q2_*q2_ + q3_*q3_);
        q0_ *= qn; q1_ *= qn; q2_ *= qn; q3_ *= qn;
        return;
    }

    float kp_eff   = computeKpEff(norm_g);
    // Trust ratio (0..1) — same gating applied to KI so the integral term
    // doesn't accumulate spurious bias during motion or accel anomalies.
    float trust    = (kp_ > 0.0f) ? (kp_eff / kp_) : 0.0f;
    last_trust_    = trust;
    float inv_norm = 1.0f / norm;
    ax *= inv_norm; ay *= inv_norm; az *= inv_norm;

    // Estimated gravity direction from current quaternion
    float vx = 2.0f * (q1_*q3_ - q0_*q2_);
    float vy = 2.0f * (q0_*q1_ + q2_*q3_);
    float vz = q0_*q0_ - q1_*q1_ - q2_*q2_ + q3_*q3_;

    // Error = cross product of measured vs estimated gravity
    float ex = ay*vz - az*vy;
    float ey = az*vx - ax*vz;
    float ez = ax*vy - ay*vx;

    // Integral feedback (gyro bias correction), gated by trust so dynamic
    // motion or accel-magnitude anomalies don't poison the bias estimate.
    // Clamped so a runaway from sensor anomalies can't dominate the gyro term.
    float ki_eff = MAHONY_KI * trust;
    bx_ += ki_eff * ex * dt;
    by_ += ki_eff * ey * dt;
    bz_ += ki_eff * ez * dt;
    if (bx_ >  MAHONY_BIAS_CLAMP) bx_ =  MAHONY_BIAS_CLAMP;
    if (bx_ < -MAHONY_BIAS_CLAMP) bx_ = -MAHONY_BIAS_CLAMP;
    if (by_ >  MAHONY_BIAS_CLAMP) by_ =  MAHONY_BIAS_CLAMP;
    if (by_ < -MAHONY_BIAS_CLAMP) by_ = -MAHONY_BIAS_CLAMP;
    if (bz_ >  MAHONY_BIAS_CLAMP) bz_ =  MAHONY_BIAS_CLAMP;
    if (bz_ < -MAHONY_BIAS_CLAMP) bz_ = -MAHONY_BIAS_CLAMP;

    // Apply correction to gyro rates
    gx = gx - bx_ + kp_eff * ex;
    gy = gy - by_ + kp_eff * ey;
    gz = gz - bz_ + kp_eff * ez;

    // Integrate quaternion
    float dq0 = 0.5f * (-q1_*gx - q2_*gy - q3_*gz);
    float dq1 = 0.5f * ( q0_*gx + q2_*gz - q3_*gy);
    float dq2 = 0.5f * ( q0_*gy - q1_*gz + q3_*gx);
    float dq3 = 0.5f * ( q0_*gz + q1_*gy - q2_*gx);
    q0_ += dq0*dt; q1_ += dq1*dt; q2_ += dq2*dt; q3_ += dq3*dt;

    float qn = 1.0f / sqrtf(q0_*q0_ + q1_*q1_ + q2_*q2_ + q3_*q3_);
    q0_ *= qn; q1_ *= qn; q2_ *= qn; q3_ *= qn;
}

EulerAngles MahonyFilter::getEuler() const {
    constexpr float RAD2DEG = 180.0f / 3.14159265358979323846f;

    float sinr_cosp = 2.0f * (q0_*q1_ + q2_*q3_);
    float cosr_cosp = 1.0f - 2.0f * (q1_*q1_ + q2_*q2_);
    float roll      = atan2f(sinr_cosp, cosr_cosp) * RAD2DEG;

    float sinp = 2.0f * (q0_*q2_ - q3_*q1_);
    if (sinp >  1.0f) sinp =  1.0f;
    if (sinp < -1.0f) sinp = -1.0f;
    float pitch = asinf(sinp) * RAD2DEG;

    float siny_cosp = 2.0f * (q0_*q3_ + q1_*q2_);
    float cosy_cosp = 1.0f - 2.0f * (q2_*q2_ + q3_*q3_);
    float yaw       = atan2f(siny_cosp, cosy_cosp) * RAD2DEG;

    return {roll, pitch, yaw};
}
