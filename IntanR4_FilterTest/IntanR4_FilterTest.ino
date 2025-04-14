/*
 * Intan RHD2216 Filter Test
 *
 * This sketch generates a composite signal with 10Hz and 60Hz components
 * to demonstrate notch filtering.
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

// Signal frequencies and amplitudes
const float LOW_FREQ = 1.0f;    // 10 Hz low-frequency component
const float HIGH_FREQ = 60.0f;   // 60 Hz high-frequency component

const int16_t LOW_AMPLITUDE = 500;   // Amplitude of low-frequency signal
const int16_t HIGH_AMPLITUDE = 1000; // Amplitude of high-frequency signal

// Filter state variables for 60Hz
float inBuffer60Hz[3] = {0, 0, 0};
float outBuffer60Hz[3] = {0, 0, 0};

// Output timing
unsigned long lastOutputTime = 0;
const unsigned long OUTPUT_INTERVAL_MS = 1; // 1ms between outputs (helps with serial timing)

// Buffer for serial output
char filterTestOutputBuffer[100];

// Configuration for notch filter
struct FilterConfig {
    bool notchEnabled = false;
    bool notch60HzEnabled = false;
} filterConfig;

void setup() {
  // Initialize serial communication
  Serial.begin(FILTER_TEST_BAUD_RATE);
  while (!Serial) {
    ; // Wait for serial port to connect
  }

  // Print welcome message
  Serial.println("Intan RHD2216 Filter Test");
  Serial.println("------------------------");
  Serial.println("Generating composite signal: 10Hz + 60Hz");
  Serial.println("Original,Filtered");

  // Initialize the Intan system with sample rate
  intanInit(FILTER_TEST_SAMPLE_RATE);

  // Default config 
  config.lowGainMode = false;
  config.averageEnergyMode = false;
  config.notchEnabled = false;
  config.notch60Hz = true;
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

    // Generate composite signal with 10Hz and 60Hz components
    int16_t originalSignal = generateTestSignal();
    int16_t filteredSignal = originalSignal;

    // Apply 60Hz filter if enabled
    if (filterConfig.notch60HzEnabled) {
      FilterCoeff coeff60Hz = notch60HzCoeff;
      float filteredFloat = Filter::apply(
        (float)filteredSignal, 
        inBuffer60Hz, 
        outBuffer60Hz, 
        coeff60Hz
      );
      filteredSignal = (int16_t)filteredFloat;
    }

    // Output data for plotting
    snprintf(filterTestOutputBuffer, sizeof(filterTestOutputBuffer), "%d,%d",
             originalSignal, filteredSignal);
    Serial.println(filterTestOutputBuffer);
  }

  // Check for any serial input (allows changing filter settings)
  if (Serial.available() > 0) {
    processFilterTestCommand();
  }
}

// Generate a composite signal with 10Hz and 60Hz components
int16_t generateTestSignal() {
  // Calculate low-frequency component (10 Hz)
  float lowFreqSignal = LOW_AMPLITUDE * sin(2.0f * PI * LOW_FREQ * currentTime);
  
  // Calculate high-frequency component (60 Hz)
  float highFreqSignal = HIGH_AMPLITUDE * sin(2.0f * PI * HIGH_FREQ * currentTime);
  
  // Combine the two signals
  float combinedSignal = lowFreqSignal + highFreqSignal;

  // Increment time
  currentTime += TIME_INCREMENT;

  // Return combined signal
  return (int16_t)combinedSignal;
}

// Process a serial command
void processFilterTestCommand() {
  // Read command
  String command = Serial.readStringUntil('\n');
  command.trim();
  command.toLowerCase();

  // Process command
  if (command == "notch on") {
    filterConfig.notchEnabled = true;
    Serial.println("Master Notch filter ENABLED");
  }
  else if (command == "notch off") {
    filterConfig.notchEnabled = false;
    filterConfig.notch60HzEnabled = false;
    // Reset filter buffers
    memset(inBuffer60Hz, 0, sizeof(inBuffer60Hz));
    memset(outBuffer60Hz, 0, sizeof(outBuffer60Hz));
    Serial.println("Master Notch filter DISABLED");
  }
  else if (command == "notch60 on") {
    filterConfig.notch60HzEnabled = true;
    Serial.println("60Hz Notch filter ENABLED");
  }
  else if (command == "notch60 off") {
    filterConfig.notch60HzEnabled = false;
    Serial.println("60Hz Notch filter DISABLED");
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
  Serial.println("Current Filter Status:");
  Serial.println("--------------");
  Serial.print("Master Notch: ");
  Serial.println(filterConfig.notchEnabled ? "ENABLED" : "DISABLED");
  Serial.print("60Hz Notch: ");
  Serial.println(filterConfig.notch60HzEnabled ? "ENABLED" : "DISABLED");
  Serial.print("Sampling rate: ");
  Serial.print(FILTER_TEST_SAMPLE_RATE);
  Serial.println(" Hz");
  Serial.println("Signal Components:");
  Serial.println("  - 10 Hz (Amplitude: 500)");
  Serial.println("  - 60 Hz (Amplitude: 1000)");
}

// Print help message
void printFilterTestHelp() {
  Serial.println("Available commands:");
  Serial.println("  notch on/off     - Enable/disable master notch filter");
  Serial.println("  notch60 on/off   - Enable/disable 60Hz notch filter");
  Serial.println("  status           - Display current settings");
  Serial.println("  help             - Display this help message");
}