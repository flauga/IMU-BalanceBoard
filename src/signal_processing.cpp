#include "signal_processing.h"

#ifdef USE_SIGNAL_PROCESSING

ButterworthLP4::ButterworthLP4() {
    // Precomputed via scipy: butter(4, 10, btype='low', fs=100, output='sos')
    sections_[0] = {
        4.824343357716229e-03f,
        9.648686715432458e-03f,
        4.824343357716229e-03f,
        -1.048599576362612e+00f,
        2.961403575616696e-01f
    };
    sections_[1] = {
        1.000000000000000e+00f,
        2.000000000000000e+00f,
        1.000000000000000e+00f,
        -1.320913430819426e+00f,
        6.327387928852766e-01f
    };
    reset();
}

void ButterworthLP4::reset() {
    for (int s = 0; s < NUM_SECTIONS; s++) {
        state_[s].z1 = 0.0f;
        state_[s].z2 = 0.0f;
    }
}

float ButterworthLP4::processOneSample(float x) {
    float y = x;
    for (int s = 0; s < NUM_SECTIONS; s++) {
        float w   = y - sections_[s].a1 * state_[s].z1 - sections_[s].a2 * state_[s].z2;
        float out = sections_[s].b0 * w + sections_[s].b1 * state_[s].z1 + sections_[s].b2 * state_[s].z2;
        state_[s].z2 = state_[s].z1;
        state_[s].z1 = w;
        y = out;
    }
    return y;
}

void ButterworthLP4::forwardPass(float* data, uint16_t length) {
    for (uint16_t i = 0; i < length; i++) {
        data[i] = processOneSample(data[i]);
    }
}

void ButterworthLP4::filtfilt(float* data, uint16_t length) {
    if (length < 2) return;

    // Forward pass — initialize state to DC steady-state of first sample.
    // Correct formula: gain = (b0+b1+b2) / (1+a1+a2), then z1=z2=x0*gain.
    float x0 = data[0];
    for (int s = 0; s < NUM_SECTIONS; s++) {
        float num   = sections_[s].b0 + sections_[s].b1 + sections_[s].b2;
        float denom = 1.0f + sections_[s].a1 + sections_[s].a2;
        float ss    = (denom != 0.0f) ? (x0 * num / denom) : 0.0f;
        state_[s].z1 = ss;
        state_[s].z2 = ss;
    }
    forwardPass(data, length);

    // Reverse pass — initialize state from last sample of forward-filtered data.
    float xn = data[length - 1];
    for (int s = 0; s < NUM_SECTIONS; s++) {
        float num   = sections_[s].b0 + sections_[s].b1 + sections_[s].b2;
        float denom = 1.0f + sections_[s].a1 + sections_[s].a2;
        float ss    = (denom != 0.0f) ? (xn * num / denom) : 0.0f;
        state_[s].z1 = ss;
        state_[s].z2 = ss;
    }
    for (int32_t i = (int32_t)length - 1; i >= 0; i--) {
        data[i] = processOneSample(data[i]);
    }
}

#endif // USE_SIGNAL_PROCESSING
