#include "IntanR4.h"

// Serial communication
constexpr unsigned long SERIAL_BAUD_RATE = 250000;
String serialBuffer = "";
bool serialComplete = false;

// Sampling rate
constexpr uint32_t SAMPLE_RATE_HZ = 2000;

// Buffer for serial output
char outputBuffer[1000];

// Counter for data output rate limiting
uint32_t outputCounter = 0;
constexpr uint32_t OUTPUT_DIVIDER = 2; // Output every 5th sample (1kHz output rate at 5kHz sampling)

// Flag for checking if serial commands have been received
volatile bool serialInterruptOccurred = false;

void setup() {
  // Initialize serial communication
  Serial.begin(SERIAL_BAUD_RATE);
  while (!Serial) {
    ; // Wait for serial port to connect
  }

  // Print welcome message
  Serial.println("Intan RHD2216 Interface for Arduino R4 Minima");
  Serial.println("-------------------------------------------");
  Serial.println("Available commands:");
  Serial.println("  gain [high|low]    - Set gain mode");
  Serial.println("  notch [on|off]     - Enable/disable notch filter");
  Serial.println("  notch60 [on|off]   - Set notch filter to 60Hz (on) or 50Hz (off)");
  Serial.println("  threshold [value]  - Set threshold value (0-1023)");
  Serial.println("  channel1 [on|off]  - Enable/disable channel 1");
  Serial.println("  channel2 [on|off]  - Enable/disable channel 2");
  Serial.println("  status             - Display current settings");
  Serial.println("  reset              - Reset the system");

  // Initialize the Intan system with specified sample rate
  Serial.print("Initializing Intan RHD2216 at ");
  Serial.print(SAMPLE_RATE_HZ);
  Serial.println(" Hz...");

  // Initialize Intan with selected sample rate
  intanInit(SAMPLE_RATE_HZ);
  
  // Configure default settings
  config.lowGainMode = false;
  config.averageEnergyMode = false;
  config.notchEnabled = true;      // Enable notch filter by default
  config.notch60Hz = true;         // Use 60Hz notch
  config.thresholdValue = 10;
  config.channel1Enabled = true;
  config.channel2Enabled = true;

  // Apply configuration
  intanUpdateConfig(config);
  
  // Set default bandwidth
  intanSetBandwidth(IntanBandwidth::INTAN_BW_10HZ);

  Serial.println("Initialization complete.");
  Serial.println("Ready to receive commands.");
}

void loop() {
  // Check for available serial data
  if (Serial.available() > 0) {
    char inChar = static_cast<char>(Serial.read());
    
    // Add character to buffer unless it's a line ending
    if (inChar != '\n' && inChar != '\r') {
      serialBuffer += inChar;
    }
    
    // Check for complete command
    if (inChar == '\n' || inChar == '\r') {
      serialComplete = true;
    }
  }

  // Process serial commands
  if (serialComplete) {
    Serial.print("Processing command: ");
    Serial.println(serialBuffer);
    processCommand();
    serialBuffer = "";
    serialComplete = false;
  }

  // Manage data output
  outputCounter++;
  if (outputCounter >= OUTPUT_DIVIDER) {
    outputCounter = 0;

    // Read channel data
    int16_t data1 = intanReadChannelData(CHANNEL_1);
    int16_t data2 = intanReadChannelData(CHANNEL_2);
    
    // Read raw channel data
    int16_t rawData1 = intanReadRawChannelData(CHANNEL_1);
    int16_t rawData2 = intanReadRawChannelData(CHANNEL_2);
    
    // Calculate voltage in microvolts (LSB = 0.195 µV)
    float uV1 = data1 * 0.195f;
    float uV2 = data2 * 0.195f;
    
    // Format output including raw and processed data
    snprintf(outputBuffer, sizeof(outputBuffer), 
             "Raw1:%d,Proc1:%d,Raw2:%d,Proc2:%d,uV1:%.2f,uV2:%.2f",
             rawData1, data1, rawData2, data2, uV1, uV2);
    
    // Send via serial
    Serial.println(outputBuffer);
  }
}

// Process a command from the serial buffer
void processCommand() {
  // Convert to lowercase and trim whitespace
  serialBuffer.trim();
  serialBuffer.toLowerCase();

  // Command: gain [high|low]
  if (serialBuffer.startsWith("gain ")) {
    String param = serialBuffer.substring(5);
    param.trim();

    if (param == "high") {
      config.lowGainMode = false;
      intanUpdateConfig(config);
      Serial.println("Gain set to HIGH");
    }
    else if (param == "low") {
      config.lowGainMode = true;
      intanUpdateConfig(config);
      Serial.println("Gain set to LOW");
    }
    else {
      Serial.println("Invalid parameter. Use 'gain [high|low]'");
    }
  }

  // Command: notch [on|off]
  else if (serialBuffer.startsWith("notch ")) {
    String param = serialBuffer.substring(6);
    param.trim();

    if (param == "on") {
      config.notchEnabled = true;
      intanUpdateConfig(config);
      Serial.println("Notch filter ENABLED");
    }
    else if (param == "off") {
      config.notchEnabled = false;
      intanUpdateConfig(config);
      Serial.println("Notch filter DISABLED");
    }
    else {
      Serial.println("Invalid parameter. Use 'notch [on|off]'");
    }
  }

  // Command: notch60 [on|off]
  else if (serialBuffer.startsWith("notch60 ")) {
    String param = serialBuffer.substring(8);
    param.trim();

    if (param == "on") {
      config.notch60Hz = true;
      intanUpdateConfig(config);
      Serial.println("Notch filter set to 60Hz");
    }
    else if (param == "off") {
      config.notch60Hz = false;
      intanUpdateConfig(config);
      Serial.println("Notch filter set to 50Hz");
    }
    else {
      Serial.println("Invalid parameter. Use 'notch60 [on|off]'");
    }
  }

  // Command: threshold [value]
  else if (serialBuffer.startsWith("threshold ")) {
    String param = serialBuffer.substring(10);
    param.trim();

    int value = param.toInt();
    if (value >= 0 && value <= 1023) {
      config.thresholdValue = value;
      intanUpdateConfig(config);
      Serial.print("Threshold set to ");
      Serial.println(value);
    }
    else {
      Serial.println("Invalid threshold value. Use a value between 0 and 1023.");
    }
  }

  // Command: channel1 [on|off]
  else if (serialBuffer.startsWith("channel1 ")) {
    String param = serialBuffer.substring(9);
    param.trim();

    if (param == "on") {
      config.channel1Enabled = true;
      intanUpdateConfig(config);
      Serial.println("Channel 1 ENABLED");
    }
    else if (param == "off") {
      config.channel1Enabled = false;
      intanUpdateConfig(config);
      Serial.println("Channel 1 DISABLED");
    }
    else {
      Serial.println("Invalid parameter. Use 'channel1 [on|off]'");
    }
  }

  // Command: channel2 [on|off]
  else if (serialBuffer.startsWith("channel2 ")) {
    String param = serialBuffer.substring(9);
    param.trim();

    if (param == "on") {
      config.channel2Enabled = true;
      intanUpdateConfig(config);
      Serial.println("Channel 2 ENABLED");
    }
    else if (param == "off") {
      config.channel2Enabled = false;
      intanUpdateConfig(config);
      Serial.println("Channel 2 DISABLED");
    }
    else {
      Serial.println("Invalid parameter. Use 'channel2 [on|off]'");
    }
  }

  // Command: status
  else if (serialBuffer == "status") {
    // Display current settings
    Serial.println("Current Status:");
    Serial.println("--------------");

    Serial.print("Gain mode: ");
    Serial.println(config.lowGainMode ? "LOW" : "HIGH");

    Serial.print("Notch filter: ");
    Serial.println(config.notchEnabled ? "ENABLED" : "DISABLED");

    Serial.print("Notch frequency: ");
    Serial.println(config.notch60Hz ? "60Hz" : "50Hz");

    Serial.print("Threshold: ");
    Serial.println(config.thresholdValue);

    Serial.print("Channel 1: ");
    Serial.println(config.channel1Enabled ? "ENABLED" : "DISABLED");

    Serial.print("Channel 2: ");
    Serial.println(config.channel2Enabled ? "ENABLED" : "DISABLED");

    Serial.print("Sampling rate: ");
    Serial.print(SAMPLE_RATE_HZ);
    Serial.println(" Hz");
  }

  // Command: reset
  else if (serialBuffer == "reset") {
    Serial.println("Resetting system...");

    // Reset to default configuration
    config.lowGainMode = false;
    config.averageEnergyMode = false;
    config.notchEnabled = true;  // Changed to true to match default in .cpp
    config.notch60Hz = true;     // Changed to true to match default in .cpp
    config.thresholdValue = 10;
    config.channel1Enabled = true;
    config.channel2Enabled = false;

    // Reset the Intan interface
    intanReset();
    intanUpdateConfig(config);

    Serial.println("Reset complete.");
  }

  // Unknown command
  else {
    Serial.print("Unknown command: ");
    Serial.println(serialBuffer);
    Serial.println("Type 'status' for a list of current settings.");
  }
}