/*
 * Intan RHD2216 Filter Test
 *
 * This sketch continuously generates a synthetic 60Hz signal with noise,
 * applies the notch filter, and outputs the results in real-time.
 *
 * Create a copy of your IntanR4.h, IntanR4.cpp, filter.h, and filter.cpp files
 * in the same directory as this sketch.
 */

#include "IntanR4.h"
#include "filter.h"

// Serial communication
const unsigned long FILTER_TEST_BAUD_RATE = 250000;

// Sampling rate
const uint32_t FILTER_TEST_SAMPLE_RATE = 2000;

// Synthetic signal generation
float currentTime = 0.0f;
const float TIME_INCREMENT = 1.0f / FILTER_TEST_SAMPLE_RATE;
const float SIGNAL_FREQ = 60.0f;  // 60Hz signal
const float NOISE_FREQ_1 = 120.0f; // First noise component
const float NOISE_FREQ_2 = 180.0f; // Second noise component
const int16_t SIGNAL_AMPLITUDE = 1000;
const int16_t NOISE_AMPLITUDE_1 = 200;
const int16_t NOISE_AMPLITUDE_2 = 150;

// Filter state variables
float inBuffer[3] = {0, 0, 0};
float outBuffer[3] = {0, 0, 0};

// Output timing
unsigned long lastOutputTime = 0;
const unsigned long OUTPUT_INTERVAL_MS = 1; // 1ms between outputs (helps with serial timing)

// Buffer for serial output
char filterTestOutputBuffer[100];

void setup() {
  // Initialize serial communication
  Serial.begin(FILTER_TEST_BAUD_RATE);
  while (!Serial) {
    ; // Wait for serial port to connect
  }

  // Print welcome message
  Serial.println("Intan RHD2216 Filter Test");
  Serial.println("------------------------");
  Serial.println("Continuously generating and filtering a 60Hz signal");
  Serial.println("Original,Filtered");

  // Initialize the Intan system with sample rate
  intanInit(FILTER_TEST_SAMPLE_RATE);

  // Default config to apply notch filter
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

  // Initialize filter library
  Filter::init(FILTER_TEST_SAMPLE_RATE);
}

void loop() {
  // Only output at the specified interval to avoid overwhelming the serial port
  if (millis() - lastOutputTime >= OUTPUT_INTERVAL_MS) {
    lastOutputTime = millis();

    // Generate synthetic signal with 60Hz and noise
    int16_t originalSignal = generateTestSignal();

    // Apply the filter
    FilterCoeff coeff = (config.notch60Hz) ? notch60HzCoeff : notch50HzCoeff;
    float filteredSignal = Filter::apply((float)originalSignal, inBuffer, outBuffer, coeff);

    // Output data for plotting
    snprintf(filterTestOutputBuffer, sizeof(filterTestOutputBuffer), "%d,%d",
             originalSignal, (int16_t)filteredSignal);
    Serial.println(filterTestOutputBuffer);
  }

  // Check for any serial input (allows changing filter settings)
  if (Serial.available() > 0) {
    processFilterTestCommand();
  }
}

// Generate a synthetic signal with 60Hz frequency and noise
int16_t generateTestSignal() {
  // Calculate signal components
  float signal = SIGNAL_AMPLITUDE * sin(2.0f * PI * SIGNAL_FREQ * currentTime);
  float noise = NOISE_AMPLITUDE_1 * sin(2.0f * PI * NOISE_FREQ_1 * currentTime) +
                NOISE_AMPLITUDE_2 * sin(2.0f * PI * NOISE_FREQ_2 * currentTime);

  // Increment time
  currentTime += TIME_INCREMENT;

  // Return combined signal and noise
  return (int16_t)(signal + noise);
}

// Process a serial command
void processFilterTestCommand() {
  // Read command
  String command = Serial.readStringUntil('\n');
  command.trim();
  command.toLowerCase();

  // Process command
  if (command == "notch on") {
    config.notchEnabled = true;
    intanUpdateConfig(config);
    Serial.println("Notch filter ENABLED");
  }
  else if (command == "notch off") {
    config.notchEnabled = false;
    intanUpdateConfig(config);
    Serial.println("Notch filter DISABLED");
  }
  else if (command == "notch60 on") {
    config.notch60Hz = true;
    intanUpdateConfig(config);
    Serial.println("Notch filter set to 60Hz");
  }
  else if (command == "notch60 off") {
    config.notch60Hz = false;
    intanUpdateConfig(config);
    Serial.println("Notch filter set to 50Hz");
  }
  else if (command == "status") {
    printFilterTestStatus();
  }
  else if (command == "help") {
    printFilterTestHelp();
  }
}

// Print current status
void printFilterTestStatus() {
  Serial.println("Current Status:");
  Serial.println("--------------");
  Serial.print("Notch filter: ");
  Serial.println(config.notchEnabled ? "ENABLED" : "DISABLED");
  Serial.print("Notch frequency: ");
  Serial.println(config.notch60Hz ? "60Hz" : "50Hz");
  Serial.print("Sampling rate: ");
  Serial.print(FILTER_TEST_SAMPLE_RATE);
  Serial.println(" Hz");
}

// Print help message
void printFilterTestHelp() {
  Serial.println("Available commands:");
  Serial.println("  notch on/off   - Enable/disable notch filter");
  Serial.println("  notch60 on/off - Set notch filter to 60Hz (on) or 50Hz (off)");
  Serial.println("  status         - Display current settings");
  Serial.println("  help           - Display this help message");
}
