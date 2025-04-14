#include "filter.h"
#include <math.h>

// Initialize static member
uint32_t Filter::_samplingRate = 0;

// Initialize the filter with sampling rate
void Filter::init(uint32_t samplingRate) {
    _samplingRate = samplingRate;
}

// Get the current sampling rate
uint32_t Filter::getSamplingRate() {
    return _samplingRate;
}

// Calculate notch filter coefficients for 50Hz or 60Hz
FilterCoeff Filter::calculateNotchCoeff(FilterType type, float Q) {
    FilterCoeff coeff;

    // Select the notch frequency
    float f0 = (type == FILTER_NOTCH_50HZ) ? 50.0f : 60.0f;

    // Normalize frequency to sampling rate
    float omega = 2.0f * M_PI * f0 / _samplingRate;
    float alpha = sin(omega) / (2.0f * Q);

    // Calculate coefficients for a notch filter
    float a0 = 1.0f + alpha;
    coeff.b0 = (1.0f + alpha) / a0;
    coeff.b1 = (-2.0f * cos(omega)) / a0;
    coeff.b2 = (1.0f + alpha) / a0;
    coeff.a1 = (-2.0f * cos(omega)) / a0;
    coeff.a2 = (1.0f - alpha) / a0;

    return coeff;
}

// Calculate high-pass filter coefficients
FilterCoeff Filter::calculateHighPassCoeff(float cutoffFreq, float Q) {
    FilterCoeff coeff;

    // Normalize frequency to sampling rate
    float omega = 2.0f * M_PI * cutoffFreq / _samplingRate;
    float alpha = sin(omega) / (2.0f * Q);

    // Calculate coefficients for a high-pass filter
    float a0 = 1.0f + alpha;
    coeff.b0 = (1.0f + cos(omega)) / (2.0f * a0);
    coeff.b1 = -(1.0f + cos(omega)) / a0;
    coeff.b2 = (1.0f + cos(omega)) / (2.0f * a0);
    coeff.a1 = (-2.0f * cos(omega)) / a0;
    coeff.a2 = (1.0f - alpha) / a0;

    return coeff;
}

// Calculate low-pass filter coefficients
FilterCoeff Filter::calculateLowPassCoeff(float cutoffFreq, float Q) {
    FilterCoeff coeff;

    // Normalize frequency to sampling rate
    float omega = 2.0f * M_PI * cutoffFreq / _samplingRate;
    float alpha = sin(omega) / (2.0f * Q);

    // Calculate coefficients for a low-pass filter
    float a0 = 1.0f + alpha;
    coeff.b0 = ((1.0f - cos(omega)) / 2.0f) / a0;
    coeff.b1 = (1.0f - cos(omega)) / a0;
    coeff.b2 = ((1.0f - cos(omega)) / 2.0f) / a0;
    coeff.a1 = (-2.0f * cos(omega)) / a0;
    coeff.a2 = (1.0f - alpha) / a0;

    return coeff;
}

// Apply filter to a single sample
float Filter::applySample(float input, float* x, float* y, const FilterCoeff& coeff) {
    // Update input buffer (x[0] is newest, x[2] is oldest)
    x[2] = x[1];
    x[1] = x[0];
    x[0] = input;

    // Calculate output
    float output = coeff.b0 * x[0] + coeff.b1 * x[1] + coeff.b2 * x[2] -
                   coeff.a1 * y[1] - coeff.a2 * y[2];

    // Update output buffer (y[0] is newest, y[2] is oldest)
    y[2] = y[1];
    y[1] = output;

    return output;
}

// Apply entire filter (convenience wrapper)
float Filter::apply(float input, float* inputBuffer, float* outputBuffer, const FilterCoeff& coeff) {
    return applySample(input, inputBuffer, outputBuffer, coeff);
}
