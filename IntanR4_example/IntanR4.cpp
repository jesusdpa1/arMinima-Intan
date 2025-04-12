#include "IntanR4.h"

// Default configuration values
IntanConfig config = {
    .lowGainMode = false,      // Normal gain mode
    .averageEnergyMode = false, // Raw data mode
    .notchEnabled = true,      // Notch filter enabled
    .notch60Hz = true,         // 60Hz notch filter
    .thresholdValue = 500,     // Default threshold
    .channel1Enabled = true,   // Channel 1 enabled
    .channel2Enabled = false   // Channel 2 disabled
};

// Global variables
enum NotchFilterType { NOTCH_60HZ, NOTCH_50HZ, NOTCH_NONE };
NotchFilterType notchFilter;

// Sample rate in Hz
uint32_t sampleRateHz = 5000;

// Buffers for channel data
volatile int16_t channelData[2] = {0, 0};          // Raw data from chip
volatile int16_t filteredChannelData[2] = {0, 0};  // Data after filtering
volatile int16_t finalChannelData[2] = {0, 0};     // Final data for output

// Accumulator variables
volatile int32_t accumulator[2] = {0, 0};          // Running accumulation
volatile int32_t finalAccumulator[2] = {0, 0};     // Final accumulation
volatile int16_t ampMax[2] = {-32768, -32768};     // Maximum value seen
volatile int16_t ampMin[2] = {32767, 32767};       // Minimum value seen
volatile int16_t amplitude[2] = {0, 0};            // Current amplitude

// Notch filter state variables
volatile float inSamples[2][3] = {{0.0f}};          // Input samples for filter
volatile float outSamples[2][3] = {{0.0f}};         // Output samples from filter

// ISR state variables
volatile uint8_t currentChannel = 0;               // Current channel being sampled
volatile uint8_t accumulatorCount = 0;             // Counter for accumulator
volatile bool channelPower[2] = {true, false};      // Whether channels are powered
volatile int32_t thresholdValue = 500;             // Threshold for activity detection
volatile bool detectionState[2] = {false, false};   // Activity detection state

// Timer for sampling
FspTimer samplingTimer;

// Implement the static callback method
void IntanCallback::timerCallback(timer_callback_args_t *p_args) {
    (void)p_args; // Unused parameter
    intanProcessTimerEvent();
}

// Initialize the Intan interface
void intanInit(uint32_t sampleRate) {
    // Save the sample rate
    sampleRateHz = sampleRate;

    // Configure SPI pins
    pinMode(INTAN_CS_PIN, OUTPUT);
    digitalWrite(INTAN_CS_PIN, HIGH);  // Deselect chip initially

    // Initialize SPI
    SPI.begin();

    // Set up the timer for sampling at the specified rate
    uint8_t timerType = GPT_TIMER;
    int8_t timerNum = FspTimer::get_available_timer(timerType);

    if (timerNum < 0 || timerType != GPT_TIMER) {
        // Try to get a timer even if it's used for PWM
        timerNum = FspTimer::get_available_timer(timerType, true);

        if (timerNum < 0) {
            // If still no success, force use of a PWM timer
            FspTimer::force_use_of_pwm_reserved_timer();
            timerNum = FspTimer::get_available_timer(timerType, true);
        }
    }

    // Configure the timer for the desired sampling rate
    // We sample both channels in rotation, so timer freq = 2 * sample rate
    float timerFreq = sampleRate * 2.0f;

    if (samplingTimer.begin(TIMER_MODE_PERIODIC, timerType, timerNum, timerFreq, 0.0f, IntanCallback::timerCallback)) {
        Serial.print("Timer configured for sampling at ");
        Serial.print(sampleRate);
        Serial.println(" Hz per channel");
    } else {
        Serial.println("Timer setup failed!");
    }

    // Initialize registers with default settings
    intanInitializeRegisters();

    // Calibrate the ADC
    intanCalibrate();

    // Reset DSP filters
    intanSendConvertCommandH(CHANNEL_1);
    intanSendConvertCommandH(CHANNEL_2);

    // Prime the pipeline with first conversions
    intanSendConvertCommand(CHANNEL_1);
    intanSendConvertCommand(CHANNEL_2);

    // Set up pin modes for status pins
    pinMode(4, OUTPUT); // Channel 1 activity output
    pinMode(5, OUTPUT); // Channel 2 activity output

    // Apply the initial configuration
    intanUpdateConfig(config);

    // Start the timer
    samplingTimer.enable_interrupt();
    samplingTimer.start();
}

// Update configuration
void intanUpdateConfig(IntanConfig newConfig) {
    config = newConfig;

    // Update channel power
    channelPower[0] = config.channel1Enabled;
    channelPower[1] = config.channel2Enabled;
    intanSetChannelPower(config.channel1Enabled, config.channel2Enabled);

    // Update threshold
    thresholdValue = config.thresholdValue;

    // Update notch filter
    if (!config.notchEnabled) {
        notchFilter = NOTCH_NONE;
    } else if (config.notch60Hz) {
        notchFilter = NOTCH_60HZ;
    } else {
        notchFilter = NOTCH_50HZ;
    }
}

// Reset the Intan interface
void intanReset() {
    // Stop the timer
    samplingTimer.stop();
    samplingTimer.disable_interrupt();

    // Reset all variables
    for (int i = 0; i < 2; i++) {
        channelData[i] = 0;
        filteredChannelData[i] = 0;
        finalChannelData[i] = 0;
        accumulator[i] = 0;
        finalAccumulator[i] = 0;
        ampMax[i] = -32768;
        ampMin[i] = 32767;
        amplitude[i] = 0;
        for (int j = 0; j < 3; j++) {
            inSamples[i][j] = 0.0f;
            outSamples[i][j] = 0.0f;
        }
    }

    // Re-initialize registers
    intanInitializeRegisters();

    // Restart the timer
    samplingTimer.enable_interrupt();
    samplingTimer.start();
}

// Send a READ command to the RHD chip
uint16_t intanSendReadCommand(uint8_t regnum) {
    uint16_t cmd = READ_CMD_MASK | (regnum << 8);

    digitalWrite(INTAN_CS_PIN, LOW);
    SPI.beginTransaction(SPISettings(20000000, MSBFIRST, SPI_MODE0));
    uint16_t result = SPI.transfer16(cmd);
    SPI.endTransaction();
    digitalWrite(INTAN_CS_PIN, HIGH);

    return result;
}

// Send a CONVERT command to the RHD chip
uint16_t intanSendConvertCommand(uint8_t channelnum) {
    uint16_t cmd = CONVERT_CMD_MASK | (channelnum << 8);

    digitalWrite(INTAN_CS_PIN, LOW);
    SPI.beginTransaction(SPISettings(20000000, MSBFIRST, SPI_MODE0));
    uint16_t result = SPI.transfer16(cmd);
    SPI.endTransaction();
    digitalWrite(INTAN_CS_PIN, HIGH);

    return result;
}

// Send a CONVERT command with high-pass filter reset
uint16_t intanSendConvertCommandH(uint8_t channelnum) {
    uint16_t cmd = CONVERT_HP_CMD_MASK | (channelnum << 8);

    digitalWrite(INTAN_CS_PIN, LOW);
    SPI.beginTransaction(SPISettings(20000000, MSBFIRST, SPI_MODE0));
    uint16_t result = SPI.transfer16(cmd);
    SPI.endTransaction();
    digitalWrite(INTAN_CS_PIN, HIGH);

    return result;
}

// Send a WRITE command to the RHD chip
uint16_t intanSendWriteCommand(uint8_t regnum, uint8_t data) {
    uint16_t cmd = WRITE_CMD_MASK | (regnum << 8) | data;

    digitalWrite(INTAN_CS_PIN, LOW);
    SPI.beginTransaction(SPISettings(20000000, MSBFIRST, SPI_MODE0));
    uint16_t result = SPI.transfer16(cmd);
    SPI.endTransaction();
    digitalWrite(INTAN_CS_PIN, HIGH);

    return result;
}

// Calibrate the RHD ADC
void intanCalibrate() {
    // Send calibration command
    digitalWrite(INTAN_CS_PIN, LOW);
    SPI.beginTransaction(SPISettings(20000000, MSBFIRST, SPI_MODE0));
    SPI.transfer16(CALIBRATE_CMD);
    SPI.endTransaction();
    digitalWrite(INTAN_CS_PIN, HIGH);

    // Wait for calibration to complete (9 dummy commands)
    for (int i = 0; i < 9; i++) {
        intanSendReadCommand(40); // Use read command as a dummy
        delayMicroseconds(10);
    }
}

// Read the latest data from a channel
int16_t intanReadChannelData(uint8_t channelnum) {
    if (channelnum == CHANNEL_1) {
        return finalChannelData[0];
    } else if (channelnum == CHANNEL_2) {
        return finalChannelData[1];
    }
    return 0;
}

// Read the latest accumulator value for a channel
int32_t intanReadAccumulatorData(uint8_t channelnum) {
    if (channelnum == CHANNEL_1) {
        return finalAccumulator[0];
    } else if (channelnum == CHANNEL_2) {
        return finalAccumulator[1];
    }
    return 0;
}

// Apply 60Hz notch filter to a channel
void intanApplyNotchFilter60Hz(uint8_t channel) {
    // Update input buffer
    inSamples[channel][0] = inSamples[channel][1];
    inSamples[channel][1] = inSamples[channel][2];
    inSamples[channel][2] = channelData[channel];

    // Apply filter coefficients for 60Hz notch
    outSamples[channel][2] = 0.9696f * inSamples[channel][0]
                          - 1.8030f * inSamples[channel][1]
                          + 0.9696f * inSamples[channel][2]
                          - 0.9391f * outSamples[channel][0]
                          + 1.8029f * outSamples[channel][1];

    // Update output buffer
    outSamples[channel][0] = outSamples[channel][1];
    outSamples[channel][1] = outSamples[channel][2];

    // Update filtered data
    filteredChannelData[channel] = (int16_t)outSamples[channel][2];
}

// Apply 50Hz notch filter to a channel
void intanApplyNotchFilter50Hz(uint8_t channel) {
    // Update input buffer
    inSamples[channel][0] = inSamples[channel][1];
    inSamples[channel][1] = inSamples[channel][2];
    inSamples[channel][2] = channelData[channel];

    // Apply filter coefficients for 50Hz notch
    outSamples[channel][2] = 0.9696f * inSamples[channel][0]
                          - 1.8443f * inSamples[channel][1]
                          + 0.9696f * inSamples[channel][2]
                          - 0.9391f * outSamples[channel][0]
                          + 1.8442f * outSamples[channel][1];

    // Update output buffer
    outSamples[channel][0] = outSamples[channel][1];
    outSamples[channel][1] = outSamples[channel][2];

    // Update filtered data
    filteredChannelData[channel] = (int16_t)outSamples[channel][2];
}

// Pass through data without notch filtering
void intanApplyNotchFilterNone(uint8_t channel) {
    // No filtering, just pass through
    filteredChannelData[channel] = channelData[channel];
}

// Process a timer event - called by ISR
void intanProcessTimerEvent() {
    // Read data from the current channel
    channelData[currentChannel] = intanSendConvertCommand(currentChannel == 0 ? CHANNEL_1 : CHANNEL_2);

    // Apply the appropriate notch filter
    switch (notchFilter) {
        case NOTCH_60HZ:
            intanApplyNotchFilter60Hz(currentChannel);
            break;

        case NOTCH_50HZ:
            intanApplyNotchFilter50Hz(currentChannel);
            break;

        case NOTCH_NONE:
        default:
            intanApplyNotchFilterNone(currentChannel);
            break;
    }

    // Zero out channel if it's disabled
    if (!channelPower[currentChannel]) {
        filteredChannelData[currentChannel] = 0;
    }

    // Store the final data
    finalChannelData[currentChannel] = filteredChannelData[currentChannel];

    // Accumulator logic
    // Track min/max of data for amplitude calculation
    if (filteredChannelData[currentChannel] > ampMax[currentChannel]) {
        ampMax[currentChannel] = filteredChannelData[currentChannel];
    }

    if (filteredChannelData[currentChannel] < ampMin[currentChannel]) {
        ampMin[currentChannel] = filteredChannelData[currentChannel];
    }

    // Calculate amplitude as max-min
    amplitude[currentChannel] = ampMax[currentChannel] - ampMin[currentChannel];

    // Add to accumulator
    accumulator[currentChannel] += amplitude[currentChannel];

    // Toggle between channels
    currentChannel = 1 - currentChannel;

    // Accumulator period processing
    if (currentChannel == 0) {
        accumulatorCount++;

        // Process accumulator data every 10 samples (at 5kHz, this is every 2ms)
        if (accumulatorCount >= 10) {
            accumulatorCount = 0;

            // Copy accumulator data to final values
            finalAccumulator[0] = accumulator[0];
            finalAccumulator[1] = accumulator[1];

            // Check for threshold crossing
            for (int ch = 0; ch < 2; ch++) {
                if (accumulator[ch] > thresholdValue && !detectionState[ch]) {
                    // Signal went above threshold
                    detectionState[ch] = true;
                    digitalWrite(ch + 4, HIGH); // Set output pin high
                }
                else if (accumulator[ch] < (thresholdValue / 2) && detectionState[ch]) {
                    // Signal returned below 50% of threshold
                    detectionState[ch] = false;
                    digitalWrite(ch + 4, LOW); // Set output pin low
                }
            }

            // Reset accumulator and min/max values
            for (int ch = 0; ch < 2; ch++) {
                accumulator[ch] = 0;
                ampMax[ch] = -32768;
                ampMin[ch] = 32767;
                amplitude[ch] = 0;
            }
        }
    }
}

// Set the bandwidth cutoff frequency
void intanSetBandwidth(IntanBandwidth bandwidth) {
    uint8_t R12, RL, RLDAC1, R13, ADCaux3en, RLDAC3, RLDAC2;

    switch(bandwidth) {
        case INTAN_BW_10HZ:
            // 10Hz lower cutoff
            RL = 0;
            RLDAC1 = 5;
            ADCaux3en = 0;
            RLDAC3 = 0;
            RLDAC2 = 1;
            break;

        case INTAN_BW_1HZ:
            // 1Hz lower cutoff
            RL = 0;
            RLDAC1 = 44;
            ADCaux3en = 0;
            RLDAC3 = 0;
            RLDAC2 = 6;
            break;

        case INTAN_BW_0_1HZ:
            // 0.1Hz lower cutoff
            RL = 0;
            RLDAC1 = 16;
            ADCaux3en = 0;
            RLDAC3 = 1;
            RLDAC2 = 60;
            break;
    }

    // Calculate register values
    R12 = ((RL << 7) | RLDAC1);
    R13 = (ADCaux3en << 7) | (RLDAC3 << 6) | RLDAC2;

    // Send commands to update registers
    intanSendWriteCommand(12, R12);
    intanSendWriteCommand(13, R13);
}

// Initialize registers with specific settings
void intanInitializeRegisters() {
    // R0: ADC Configuration and Amplifier Fast Settle
    // D[7] - D[6]: ADC reference BW = 3
    // D[5]: amp fast settle = 0
    // D[4]: amp Vref enable = 0
    // D[3] - D[2]: ADC comparator bias = 3
    // D[1] - D[0]: ADC comparator select = 2
    intanSendWriteCommand(0, 0b11011110);

    // R1: Supply Sensor and ADC Buffer Bias Current
    // D[7]: X - set to 0
    // D[6]: VDD sense enable = 0
    // D[5] - D[0]: ADC buffer bias = 32
    intanSendWriteCommand(1, 0b00100000);

    // R2: MUX Bias Current
    // D[7] - D[6]: X - set to 0
    // D[5] - D[0]: MUX bias current = 40
    intanSendWriteCommand(2, 0b00101000);

    // R3: MUX Load, Temperature Sensor, and Auxiliary Digital Output
    // D[7] - D[5]: MUX load = 0
    // D[4]: tempS2 = 0
    // D[3]: tempS1 = 0
    // D[2]: tempen = 0
    // D[1]: digout HiZ = 0
    // D[0]: digout = 0
    intanSendWriteCommand(3, 0b00000000);

    // R4: ADC Output Format and DSP Offset Removal
    // D[7]: weak MISO = 1
    // D[6]: twoscomp = 1
    // D[5]: absmode = 0
    // D[4]: DSPen = 1
    // D[3] - D[0]: DSP cutoff frequency variable = 8
    intanSendWriteCommand(4, 0b11011000);

    // R5: Impedance Check Control
    intanSendWriteCommand(5, 0b00000000);

    // R6: Impedance Check DAC
    intanSendWriteCommand(6, 0b00000000);

    // R7: Impedance Check Amplifier Select
    intanSendWriteCommand(7, 0b00000000);

    // R8-R11: Amplifier Bandwidth High-Frequency Select
    // Configure for 10kHz upper cutoff
    intanSendWriteCommand(8, 30);  // RH1 DAC1 = 30
    intanSendWriteCommand(9, 5);   // RH1 DAC2 = 5
    intanSendWriteCommand(10, 43); // RH2 DAC1 = 43
    intanSendWriteCommand(11, 6);  // RH2 DAC2 = 6

    // R12-R13: Amplifier Bandwidth Low-Frequency Select
    // Set default to 10Hz cutoff
    intanSetBandwidth(INTAN_BW_10HZ);

    // R14-R17: Individual Amplifier Power (all off initially)
    intanSendWriteCommand(14, 0);
    intanSendWriteCommand(15, 0);
    intanSendWriteCommand(16, 0);
    intanSendWriteCommand(17, 0);
}

// Set channel power
void intanSetChannelPower(bool ch1Power, bool ch2Power) {
    // Read current settings
    intanSendReadCommand(14);
    intanSendReadCommand(14);
    uint8_t reg14 = intanSendReadCommand(14);

    intanSendReadCommand(15);
    intanSendReadCommand(15);
    uint8_t reg15 = intanSendReadCommand(15);

    // Update channel 1 (CHANNEL_1) power
    if (ch1Power) {
        if (CHANNEL_1 < 8) {
            reg14 |= (1 << CHANNEL_1);
        } else {
            reg15 |= (1 << (CHANNEL_1 - 8));
        }
    } else {
        if (CHANNEL_1 < 8) {
            reg14 &= ~(1 << CHANNEL_1);
        } else {
            reg15 &= ~(1 << (CHANNEL_1 - 8));
        }
    }

    // Update channel 2 (CHANNEL_2) power
    if (ch2Power) {
        if (CHANNEL_2 < 8) {
            reg14 |= (1 << CHANNEL_2);
        } else {
            reg15 |= (1 << (CHANNEL_2 - 8));
        }
    } else {
        if (CHANNEL_2 < 8) {
            reg14 &= ~(1 << CHANNEL_2);
        } else {
            reg15 &= ~(1 << (CHANNEL_2 - 8));
        }
    }

    // Write updated register values
    intanSendWriteCommand(14, reg14);
    intanSendWriteCommand(15, reg15);

    // Update the channel power state
    channelPower[0] = ch1Power;
    channelPower[1] = ch2Power;
}
