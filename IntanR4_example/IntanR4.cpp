#include "IntanR4.h"
#include "FspTimer.h"
#include "filter.h"  // Include the filter header

// Number of channels (can be increased as needed)
#define NUM_CHANNELS 2

// Default configuration values
IntanConfig config = {
    .lowGainMode = false,      // Normal gain mode
    .averageEnergyMode = false, // Raw data mode
    .notchEnabled = true,      // Notch filter enabled
    .thresholdValue = 10,      // Default threshold
    .channel1Enabled = true,   // Channel 1 enabled
    .channel2Enabled = false   // Channel 2 disabled
};

// Filter coefficient for 60Hz notch
FilterCoeff notch60HzCoeff;

// Sample rate in Hz
uint32_t sampleRateHz = 2000;

// Channel mapping - defines which physical channels on the chip are being used
const uint8_t CHANNEL_MAP[NUM_CHANNELS] = {0, 15};  // Using channels 0 and 15

// Buffers for channel data
volatile int16_t channelData[NUM_CHANNELS] = {0};          // Raw data from chip
volatile int16_t filteredChannelData[NUM_CHANNELS] = {0};  // Data after filtering
volatile int16_t finalChannelData[NUM_CHANNELS] = {0};     // Final data for output

// Notch filter state variables
volatile float inSamples[NUM_CHANNELS][3] = {{0}};          // Input samples for filter
volatile float outSamples[NUM_CHANNELS][3] = {{0}};         // Output samples from filter

// Accumulator variables
volatile int32_t accumulator[NUM_CHANNELS] = {0};          // Running accumulation
volatile int32_t finalAccumulator[NUM_CHANNELS] = {0};     // Final accumulation
volatile int16_t ampMax[NUM_CHANNELS] = {-32768, -32768};     // Maximum value seen
volatile int16_t ampMin[NUM_CHANNELS] = {32767, 32767};       // Minimum value seen
volatile int16_t amplitude[NUM_CHANNELS] = {0, 0};            // Current amplitude

// ISR state variables
volatile uint8_t currentChannel = 0;               // Current channel being sampled
volatile uint8_t accumulatorCount = 0;             // Counter for accumulator
volatile bool channelPower[NUM_CHANNELS] = {true, false};   // Whether channels are powered
volatile int32_t thresholdValue = 10;              // Threshold for activity detection
volatile bool detectionState[NUM_CHANNELS] = {false, false};   // Activity detection state

// Timer for sampling
FspTimer samplingTimer;

// Implement the static callback method
void IntanCallback::timerCallback(timer_callback_args_t *p_args) {
    (void)p_args; // Unused parameter
    intanProcessTimerEvent();
}

// Initialize filters
void intanInitFilters(uint32_t sampleRate) {
    // Initialize the filter module with the sample rate
    Filter::init(sampleRate);

    // Precalculate 60Hz notch filter coefficients
    notch60HzCoeff = Filter::calculateNotchCoeff(FILTER_NOTCH_60HZ);
}

// Apply 60Hz notch filter
void intanApplyNotchFilter(uint8_t channel) {
    // Convert 16-bit int to float for processing
    float inputSample = (float)channelData[channel];

    // Apply the filter
    float outputSample = Filter::apply(
        inputSample,
        (float*)&inSamples[channel],
        (float*)&outSamples[channel],
        notch60HzCoeff
    );

    // Convert back to 16-bit int
    filteredChannelData[channel] = (int16_t)outputSample;
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

// Read the raw (unfiltered) data from a channel
int16_t intanReadRawChannelData(uint8_t channelnum) {
    if (channelnum == CHANNEL_1) {
        return channelData[0];
    } else if (channelnum == CHANNEL_2) {
        return channelData[1];
    }
    return 0;
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
    intanSendWriteCommand(0, 0b11011110);

    // R1: Supply Sensor and ADC Buffer Bias Current
    intanSendWriteCommand(1, 0b00100000);

    // R2: MUX Bias Current
    intanSendWriteCommand(2, 0b00101000);

    // R3: MUX Load, Temperature Sensor, and Auxiliary Digital Output
    intanSendWriteCommand(3, 0b00000000);

    // R4: ADC Output Format and DSP Offset Removal
    intanSendWriteCommand(4, 0b11011000);

    // R5: Impedance Check Control
    intanSendWriteCommand(5, 0b00000000);

    // R6: Impedance Check DAC
    intanSendWriteCommand(6, 0b00000000);

    // R7: Impedance Check Amplifier Select
    intanSendWriteCommand(7, 0b00000000);

    // R8-R11: Amplifier Bandwidth High-Frequency Select
    intanSendWriteCommand(8, 30);  // RH1 DAC1 = 30
    intanSendWriteCommand(9, 5);   // RH1 DAC2 = 5
    intanSendWriteCommand(10, 43); // RH2 DAC1 = 43
    intanSendWriteCommand(11, 6);  // RH2 DAC2 = 6

    // R12-R13: Amplifier Bandwidth Low-Frequency Select
    intanSetBandwidth(INTAN_BW_10HZ);

    // R14-R17: Individual Amplifier Power (all off initially)
    intanSendWriteCommand(14, 0);
    intanSendWriteCommand(15, 0);
    intanSendWriteCommand(16, 0);
    intanSendWriteCommand(17, 0);
}

// Set channel power
void intanSetChannelPower(bool* channelPowerSettings, int numChannels) {
    // Read current settings from the chip
    uint8_t reg14 = intanSendReadCommand(14);
    uint8_t reg15 = intanSendReadCommand(15);

    // Update the channel power array
    for (int i = 0; i < numChannels && i < NUM_CHANNELS; i++) {
        channelPower[i] = channelPowerSettings[i];

        // Get the physical channel number
        uint8_t physicalChannel = CHANNEL_MAP[i];

        // Update the appropriate register based on the channel number
        if (physicalChannel < 8) {
            if (channelPowerSettings[i]) {
                reg14 |= (1 << physicalChannel);
            } else {
                reg14 &= ~(1 << physicalChannel);
            }
        } else {
            if (channelPowerSettings[i]) {
                reg15 |= (1 << (physicalChannel - 8));
            } else {
                reg15 &= ~(1 << (physicalChannel - 8));
            }
        }
    }

    // Write updated register values
    intanSendWriteCommand(14, reg14);
    intanSendWriteCommand(15, reg15);
}

// Compatibility wrapper for legacy code
void intanSetChannelPower(bool ch1Power, bool ch2Power) {
    bool channelPowerSettings[NUM_CHANNELS] = {ch1Power, ch2Power};
    intanSetChannelPower(channelPowerSettings, NUM_CHANNELS);
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

    // Initialize filters with the same sample rate
    intanInitFilters(sampleRate);

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
    // We sample all channels in rotation, so timer freq = NUM_CHANNELS * sample rate
    float timerFreq = sampleRate * NUM_CHANNELS;

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

    // Reset DSP filters for all channels
    for (int i = 0; i < NUM_CHANNELS; i++) {
        intanSendConvertCommandH(CHANNEL_MAP[i]);
    }

    // Prime the pipeline with first conversions
    for (int i = 0; i < NUM_CHANNELS; i++) {
        intanSendConvertCommand(CHANNEL_MAP[i]);
    }

    // Set up pin modes for status pins
    pinMode(4, OUTPUT); // Channel 1 activity output
    pinMode(5, OUTPUT); // Channel 2 activity output

    // Apply the initial configuration
    intanUpdateConfig(config);

    // Start the timer
    samplingTimer.setup_overflow_irq();
    samplingTimer.open();
    samplingTimer.start();
}

// Update configuration
void intanUpdateConfig(IntanConfig newConfig) {
    config = newConfig;

    // Create an array of channel power settings
    bool channelPowerSettings[NUM_CHANNELS] = {
        newConfig.channel1Enabled,
        newConfig.channel2Enabled
        // Add more channels as needed
    };

    // Update channel power
    intanSetChannelPower(channelPowerSettings, NUM_CHANNELS);

    // Update threshold
    thresholdValue = newConfig.thresholdValue;

    Serial.print("Config updated - Notch filter: ");
    Serial.println(config.notchEnabled ? "ENABLED" : "DISABLED");
}

// Process a timer event - called by ISR
void intanProcessTimerEvent() {
    // Get the physical channel number for the current logical channel
    uint8_t physicalChannel = CHANNEL_MAP[currentChannel];

    // Read data from the current channel
    channelData[currentChannel] = intanSendConvertCommand(physicalChannel);

    // Apply notch filter if enabled
    if (config.notchEnabled) {
        intanApplyNotchFilter(currentChannel);
    } else {
        // If filter is disabled, just pass through the original data
        filteredChannelData[currentChannel] = channelData[currentChannel];
        
        // Reset filter buffers to prevent artifact buildup
        for (int j = 0; j < 3; j++) {
            inSamples[currentChannel][j] = 0.0f;
            outSamples[currentChannel][j] = 0.0f;
        }
    }

    // Zero out channel if it's disabled
    if (!channelPower[currentChannel]) {
        filteredChannelData[currentChannel] = 0;
    }

    // Store the final data
    finalChannelData[currentChannel] = filteredChannelData[currentChannel];

    // Accumulator logic
    if (filteredChannelData[currentChannel] > ampMax[currentChannel]) {
        ampMax[currentChannel] = filteredChannelData[currentChannel];
    }

    if (filteredChannelData[currentChannel] < ampMin[currentChannel]) {
        ampMin[currentChannel] = filteredChannelData[currentChannel];
    }

    amplitude[currentChannel] = ampMax[currentChannel] - ampMin[currentChannel];
    accumulator[currentChannel] += amplitude[currentChannel];
    currentChannel = (currentChannel + 1) % NUM_CHANNELS;

    // Accumulator period processing
    if (currentChannel == 0) {
        accumulatorCount++;
        if (accumulatorCount >= 10) {
            accumulatorCount = 0;
            for (int ch = 0; ch < NUM_CHANNELS; ch++) {
                finalAccumulator[ch] = accumulator[ch];
            }
            for (int ch = 0; ch < NUM_CHANNELS; ch++) {
                if (accumulator[ch] > thresholdValue && !detectionState[ch]) {
                    detectionState[ch] = true;
                    digitalWrite(ch + 4, HIGH);
                }
                else if (accumulator[ch] < (thresholdValue / 2) && detectionState[ch]) {
                    detectionState[ch] = false;
                    digitalWrite(ch + 4, LOW);
                }
            }
            for (int ch = 0; ch < NUM_CHANNELS; ch++) {
                accumulator[ch] = 0;
                ampMax[ch] = -32768;
                ampMin[ch] = 32767;
                amplitude[ch] = 0;
            }
        }
    }
}

// Reset the Intan interface
void intanReset() {
    // Stop the timer
    samplingTimer.stop();

    // Reset all variables
    for (int i = 0; i < NUM_CHANNELS; i++) {
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
        channelPower[i] = (i == 0); // Enable first channel, disable others
        detectionState[i] = false;
    }

    // Reset configuration to default
    config = {
        .lowGainMode = false,
        .averageEnergyMode = false,
        .notchEnabled = true,
        .thresholdValue = 10,
        .channel1Enabled = true,
        .channel2Enabled = false
    };

    // Reset threshold
    thresholdValue = 10;

    // Re-initialize registers
    intanInitializeRegisters();

    // Recalibrate
    intanCalibrate();

    // Reset DSP filters for all channels
    for (int i = 0; i < NUM_CHANNELS; i++) {
        intanSendConvertCommandH(CHANNEL_MAP[i]);
    }

    // Prime the pipeline with first conversions
    for (int i = 0; i < NUM_CHANNELS; i++) {
        intanSendConvertCommand(CHANNEL_MAP[i]);
    }

    // Restart the timer
    samplingTimer.setup_overflow_irq();
    samplingTimer.open();
    samplingTimer.start();

    // Update configuration
    intanUpdateConfig(config);
}