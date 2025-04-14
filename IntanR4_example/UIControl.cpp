#include "UIControl.h"
#include "IntanR4.h"
#include "filter.h"

// Initialize static members
unsigned long UIControl::serialBaudRate = 250000;
String UIControl::serialBuffer = "";
bool UIControl::serialComplete = false;
uint32_t UIControl::outputCounter = 0;
uint32_t UIControl::outputDivider = 2;
uint32_t UIControl::sampleRateHz = 2000;
char UIControl::outputBuffer[1000];

// Initialize the UI control with a baud rate
void UIControl::init(unsigned long baudRate) {
    serialBaudRate = baudRate;
    
    // Initialize serial communication
    Serial.begin(serialBaudRate);
    while (!Serial) {
        ; // Wait for serial port to connect
    }
    
    // Set up serial event
    serialBuffer = "";
    serialComplete = false;
    
    // Print welcome message
    printWelcomeMessage();
}

// Process any incoming serial commands
void UIControl::update() {
    // Add a small delay to allow for serial processing
    delayMicroseconds(500);
    
    // Check for any available serial data
    while (Serial.available() > 0) {
        char inChar = (char)Serial.read();
        
        // Add character to buffer unless it's a line ending
        if (inChar != '\n' && inChar != '\r') {
            serialBuffer += inChar;
        }
        
        // If we get a newline, set the complete flag
        if (inChar == '\n' || inChar == '\r') {
            serialComplete = true;
        }
        
        // Small delay to allow processing
        delayMicroseconds(100);
    }

    // Process any serial commands with higher priority
    if (serialComplete) {
        Serial.print("Processing command: ");
        Serial.println(serialBuffer);
        processCommand(serialBuffer);
        serialBuffer = "";
        serialComplete = false;
    }
}

// Output data to serial port
void UIControl::outputData() {
    outputCounter++;
    // Only output data every OUTPUT_DIVIDER samples
    if (outputCounter >= outputDivider) {
        outputCounter = 0;

        // Read channel data
        int16_t data1 = intanReadChannelData(CHANNEL_1);
        int16_t data2 = intanReadChannelData(CHANNEL_2);
        
        // Read raw data
        int16_t rawData1 = intanReadRawChannelData(CHANNEL_1);
        
        // Calculate voltage in microvolts (LSB = 0.195 µV)
        float uV1 = data1 * 0.195f;
        float uV2 = data2 * 0.195f;
        
        // Format the output string
        snprintf(outputBuffer, sizeof(outputBuffer), "%d,%d,%d,%.2f,%.2f",
                rawData1, data1, data2, uV1, uV2);
        
        // Send via serial
        Serial.println(outputBuffer);
        
        // Small delay after sending data to ensure it's processed
        delayMicroseconds(100);
    }
}

// Set the sampling rate for data output dividing
void UIControl::setSampleRate(uint32_t sampleRate) {
    sampleRateHz = sampleRate;
    
    // Adjust output divider based on sample rate to maintain reasonable output rate
    if (sampleRate <= 2000) {
        outputDivider = 2;  // Output at half the sample rate for low rates
    } else if (sampleRate <= 5000) {
        outputDivider = 5;  // Output at 1/5 the sample rate for medium rates
    } else {
        outputDivider = 10; // Output at 1/10 the sample rate for high rates
    }
}

// Process the buffer contents as a command
void UIControl::processCommand(String command) {
    // Convert to lowercase and trim whitespace
    command.trim();
    command.toLowerCase();

    // Command: gain [high|low]
    if (command.startsWith("gain ")) {
        String param = command.substring(5);
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
    else if (command.startsWith("notch ")) {
        String param = command.substring(6);
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
    else if (command.startsWith("notch60 ")) {
        String param = command.substring(8);
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
    else if (command.startsWith("threshold ")) {
        String param = command.substring(10);
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
    else if (command.startsWith("channel1 ")) {
        String param = command.substring(9);
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
    else if (command.startsWith("channel2 ")) {
        String param = command.substring(9);
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
    else if (command == "status") {
        printStatus();
    }

    // Command: reset
    else if (command == "reset") {
        Serial.println("Resetting system...");

        // Reset to default configuration
        config.lowGainMode = false;
        config.averageEnergyMode = false;
        config.notchEnabled = true;
        config.notch60Hz = true;
        config.thresholdValue = 10;
        config.channel1Enabled = true;
        config.channel2Enabled = true;

        // Reset the Intan interface
        intanReset();
        intanUpdateConfig(config);

        Serial.println("Reset complete.");
    }
    
    // Command: testfilter - test notch filter with synthetic signal
    else if (command == "testfilter") {
        runFilterTest();
    }

    // Unknown command
    else {
        Serial.print("Unknown command: ");
        Serial.println(command);
        Serial.println("Type 'status' for a list of current settings.");
    }
}

// Run filter test with synthetic signal
void UIControl::runFilterTest() {
    Serial.println("Running filter test with synthetic 60Hz signal...");
    
    // Create a synthetic 60Hz signal with some noise
    const int numSamples = 200;
    int16_t testSignal[numSamples];
    
    // Generate a 60Hz sine wave with noise
    for (int i = 0; i < numSamples; i++) {
        float t = (float)i / sampleRateHz;
        // 60Hz component (1000 amplitude)
        float signal = 1000.0f * sin(2.0f * PI * 60.0f * t);
        // Additional noise (200 amplitude)
        float noise = 200.0f * sin(2.0f * PI * 120.0f * t) + 
                     150.0f * sin(2.0f * PI * 180.0f * t);
        testSignal[i] = (int16_t)(signal + noise);
    }
    
    // Create buffers for filtering
    float inBuf[3] = {0, 0, 0};
    float outBuf[3] = {0, 0, 0};
    int16_t filteredSignal[numSamples];
    
    // Get the appropriate filter coefficients
    FilterCoeff coeff = (config.notch60Hz) ? notch60HzCoeff : notch50HzCoeff;
    
    // Apply the filter and output the results
    Serial.println("Original,Filtered");
    for (int i = 0; i < numSamples; i++) {
        float input = (float)testSignal[i];
        float output = Filter::apply(input, inBuf, outBuf, coeff);
        filteredSignal[i] = (int16_t)output;
        
        // Output for plotting
        Serial.print(testSignal[i]);
        Serial.print(",");
        Serial.println(filteredSignal[i]);
    }
    
    Serial.println("Filter test complete. You can plot the data to verify filter performance.");
}

// Print welcome message and command list
void UIControl::printWelcomeMessage() {
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
    Serial.println("  testfilter         - Test notch filter with synthetic signal");
}

// Print current status
void UIControl::printStatus() {
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
    Serial.print(sampleRateHz);
    Serial.println(" Hz");
}