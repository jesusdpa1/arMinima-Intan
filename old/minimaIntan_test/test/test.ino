#include "IntanShield.h"

// Configuration Variables
const bool FIRST_CHANNEL_POWER = true;
const Bandselect BANDWIDTH_SETTING = LowCutoff10Hz;

// Sampling Configuration
const unsigned long SAMPLING_INTERVAL_US = 250; // 2 kHz (500 microseconds between samples)
const float ADC_LSB = 0.195; // microvolts per LSB

// Create an instance of the IntanShield
IntanShield intanShield;

// Timing variables
unsigned long lastSampleTime = 0;

void setup() {
  // Initialize serial communication
  Serial.begin(250000);

  // Initialize the Intan Shield
  intanShield.begin();

  // Set bandwidth
  intanShield.setBandwidth(BANDWIDTH_SETTING);

  // Enable first channel, disable second channel
  intanShield.setChannelPower(FIRST_CHANNEL_POWER, false);

  // Perform initial calibration commands
  intanShield.sendConvertCommandH(FIRST_CHANNEL);
  
  // Initialization complete
  Serial.println("Intan Shield RHD2216 Initialized");
}

void loop() {
  // Check if it's time to sample
  unsigned long currentMicros = micros();
  
  if (currentMicros - lastSampleTime >= SAMPLING_INTERVAL_US) {
    // Update last sample time
    lastSampleTime = currentMicros;

    // Read raw data from first channel
    int16_t rawData = intanShield.readChannelData(FIRST_CHANNEL);

    // Apply 60 Hz notch filter
    intanShield.applyNotchFilter60(rawData);

    // Convert to microvolts
    float microvolts = rawData * ADC_LSB;

    // Send to serial plotter
    Serial.println(microvolts);
  }
}