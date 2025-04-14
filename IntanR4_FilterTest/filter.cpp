// filter.cpp
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

// Calculate true notch filter coefficients
FilterCoeff Filter::calculateNotchCoeff(FilterType type, float Q) {
    FilterCoeff coeff;

    // Select the notch frequency
    float f0 = (type == FILTER_NOTCH_50HZ) ? 50.0f : 60.0f;

    // Normalized frequency
    float w0 = 2.0f * M_PI * f0 / _samplingRate;
    
    // Compute sine and cosine of normalized frequency
    float cosw0 = cos(w0);
    float sinw0 = sin(w0);

    // Compute alpha (bandwidth)
    float alpha = sinw0 / (2.0f * Q);

    // True notch filter coefficients
    // The key is to create a filter that effectively cancels out the specific frequency
    coeff.b0 = 1.0f / (1.0f + alpha);
    coeff.b1 = -2.0f * cosw0 / (1.0f + alpha);
    coeff.b2 = 1.0f / (1.0f + alpha);
    coeff.a1 = -2.0f * cosw0 / (1.0f + alpha);
    coeff.a2 = (1.0f - alpha) / (1.0f + alpha);

    return coeff;
}

// Apply filter to a single sample using Direct Form II Transposed
float Filter::applySample(float input, float* x, float* y, const FilterCoeff& coeff) {
    // Shift input buffer
    x[2] = x[1];
    x[1] = x[0];
    x[0] = input;

    // Shift output buffer
    y[2] = y[1];
    y[1] = y[0];

    // Calculate output
    y[0] = coeff.b0 * x[0] + 
           coeff.b1 * x[1] + 
           coeff.b2 * x[2] - 
           coeff.a1 * y[1] - 
           coeff.a2 * y[2];

    return y[0];
}

// Apply entire filter (convenience wrapper)
float Filter::apply(float input, float* inputBuffer, float* outputBuffer, const FilterCoeff& coeff) {
    return applySample(input, inputBuffer, outputBuffer, coeff);
}

// Diagnostic function to help understand filter behavior
void Filter::printFilterCoefficients(const FilterCoeff& coeff) {
    Serial.println("Notch Filter Coefficients:");
    Serial.print("b0: "); Serial.println(coeff.b0, 6);
    Serial.print("b1: "); Serial.println(coeff.b1, 6);
    Serial.print("b2: "); Serial.println(coeff.b2, 6);
    Serial.print("a1: "); Serial.println(coeff.a1, 6);
    Serial.print("a2: "); Serial.println(coeff.a2, 6);
}