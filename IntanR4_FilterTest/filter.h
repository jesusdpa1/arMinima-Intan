#ifndef FILTER_H
#define FILTER_H

#include <Arduino.h>

// Filter types
enum FilterType {
    FILTER_NOTCH_50HZ,
    FILTER_NOTCH_60HZ,
    FILTER_HIGHPASS,
    FILTER_LOWPASS
};

// Filter structure to hold coefficients
struct FilterCoeff {
    float b0, b1, b2;  // Feed-forward coefficients
    float a1, a2;      // Feed-back coefficients
};

// Class for filter design and application
class Filter {
public:
    // Initialize the filter with sampling rate
    static void init(uint32_t samplingRate);

    // Calculate notch filter coefficients for 50Hz or 60Hz
    static FilterCoeff calculateNotchCoeff(FilterType type, float Q = 30.0f);

    // Calculate high-pass filter coefficients
    static FilterCoeff calculateHighPassCoeff(float cutoffFreq, float Q = 0.7071f);

    // Calculate low-pass filter coefficients
    static FilterCoeff calculateLowPassCoeff(float cutoffFreq, float Q = 0.7071f);

    // Apply filter to a single sample
    static float applySample(float input, float* x, float* y, const FilterCoeff& coeff);

    // Apply entire filter (convenience wrapper)
    static float apply(float input, float* inputBuffer, float* outputBuffer, const FilterCoeff& coeff);

    // Get the current sampling rate
    static uint32_t getSamplingRate();

    // Diagnostic function to print filter coefficients
    static void printFilterCoefficients(const FilterCoeff& coeff);

private:
    static uint32_t _samplingRate;
};

#endif // FILTER_H