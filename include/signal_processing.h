#pragma once

#ifdef USE_SIGNAL_PROCESSING

#include <cstdint>

struct BiquadCoeffs {
    float b0, b1, b2;
    float a1, a2;
};

struct BiquadState {
    float z1 = 0.0f;
    float z2 = 0.0f;
};

// 4th-order Butterworth lowpass at 10 Hz cutoff / 100 Hz sample rate.
// Enable by defining USE_SIGNAL_PROCESSING in platformio.ini build_flags.
class ButterworthLP4 {
public:
    ButterworthLP4();

    // Zero-phase filtering of an entire array (forward-reverse). In-place.
    void filtfilt(float* data, uint16_t length);

    void reset();

private:
    static constexpr int NUM_SECTIONS = 2;
    BiquadCoeffs sections_[NUM_SECTIONS];
    BiquadState  state_[NUM_SECTIONS];

    float processOneSample(float x);
    void  forwardPass(float* data, uint16_t length);
};

#endif // USE_SIGNAL_PROCESSING
