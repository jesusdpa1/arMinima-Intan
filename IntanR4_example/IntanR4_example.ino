#include "IntanR4.h"
#include "filter.h"
#include "UIControl.h"

// Serial communication
const unsigned long SERIAL_BAUD_RATE = 250000;

// Sampling rate
const uint32_t SAMPLE_RATE_HZ = 2000;

void setup() {
  // Initialize UI control
  UIControl::init(SERIAL_BAUD_RATE);

  // Initialize the Intan system with sample rate
  Serial.print("Initializing Intan RHD2216 at ");
  Serial.print(SAMPLE_RATE_HZ);
  Serial.println(" Hz...");

  intanInit(SAMPLE_RATE_HZ);

  // Configure UI Control with the same sample rate
  UIControl::setSampleRate(SAMPLE_RATE_HZ);

  // Default config to apply notch filter and other defaults
  config.lowGainMode = false;
  config.averageEnergyMode = false;
  config.notchEnabled = true;   // Enable notch filter by default
  config.notch60Hz = true;      // Use 60Hz notch
  config.thresholdValue = 10;
  config.channel1Enabled = true;
  config.channel2Enabled = true;

  intanUpdateConfig(config);
  // Set 10Hz bandwidth (default)
  intanSetBandwidth(INTAN_BW_10HZ);

  Serial.println("Initialization complete.");
  Serial.println("Ready to receive commands.");
}

void loop() {
  // Process any serial commands
  UIControl::update();

  // Output data at regular intervals
  UIControl::outputData();

  // Small delay to ensure proper timing and serial processing
  delay(1);
}
