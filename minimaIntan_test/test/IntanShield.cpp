#include "IntanShield.h"

IntanShield::IntanShield() {
    // Initialize filter history arrays
    memset(inputHistory, 0, sizeof(inputHistory));
    memset(outputHistory, 0, sizeof(outputHistory));
}

void IntanShield::applyNotchFilter60(int16_t& channelData) {
    // Shift input history
    inputHistory[0][0] = inputHistory[0][1];
    inputHistory[0][1] = inputHistory[0][2];
    inputHistory[0][2] = channelData;

    // Compute filtered output
    float output = 
        0.9696 * inputHistory[0][0] - 
        1.803 * inputHistory[0][1] + 
        0.9696 * inputHistory[0][2] - 
        0.9391 * outputHistory[0][0] + 
        1.8029 * outputHistory[0][1];

    // Shift output history
    outputHistory[0][0] = outputHistory[0][1];
    outputHistory[0][1] = output;

    // Update channel data
    channelData = static_cast<int16_t>(output);
}

uint16_t IntanShield::sendReadCommand(uint8_t regnum) {
    uint16_t mask = (0b1100 << 12) | (regnum << 8);
    digitalWrite(chipSelectPin, LOW);
    uint16_t response = SPI.transfer16(mask);
    digitalWrite(chipSelectPin, HIGH);
    return response;
}

uint16_t IntanShield::sendWriteCommand(uint8_t regnum, uint8_t data) {
    uint16_t mask = (0b1000 << 12) | (regnum << 8) | data;
    digitalWrite(chipSelectPin, LOW);
    uint16_t response = SPI.transfer16(mask);
    digitalWrite(chipSelectPin, HIGH);
    return response;
}

uint16_t IntanShield::sendConvertCommand(uint8_t channelnum) {
    uint16_t mask = channelnum << 8;
    digitalWrite(chipSelectPin, LOW);
    uint16_t response = SPI.transfer16(mask);
    digitalWrite(chipSelectPin, HIGH);
    return response;
}

uint16_t IntanShield::sendConvertCommandH(uint8_t channelnum) {
    uint16_t mask = (channelnum << 8) | 0x01;
    digitalWrite(chipSelectPin, LOW);
    uint16_t response = SPI.transfer16(mask);
    digitalWrite(chipSelectPin, HIGH);
    return response;
}

void IntanShield::begin() {
    // Initialize SPI
    pinMode(chipSelectPin, OUTPUT);
    digitalWrite(chipSelectPin, HIGH);
    
    SPI.begin();
    SPI.beginTransaction(SPISettings(24000000, MSBFIRST, SPI_MODE0));
    
    // Perform initial configuration with default 10 Hz low cutoff
    configureRegisters(LowCutoff10Hz);
    
    // Perform calibration
    calibrate();
    
    // Initial convert commands
    sendConvertCommand(FIRST_CHANNEL);
    sendConvertCommand(SECOND_CHANNEL);
}

void IntanShield::calibrate() {
    // Calibration sequence
    digitalWrite(chipSelectPin, LOW);
    SPI.transfer16(0b0101010100000000);
    digitalWrite(chipSelectPin, HIGH);
    
    // Dummy reads to complete calibration
    for (int i = 0; i < 9; i++) {
        sendReadCommand(40);
    }
}

void IntanShield::configureRegisters(Bandselect bandSetting) {
    // R0: ADC Configuration and Amplifier Fast Settle
    // D[7]: weak MISO = 1 (enable weak pull-up/down)
    // Other bits configured for optimal performance
    sendWriteCommand(0, 0b11011110);

    // R1: Supply Sensor and ADC Buffer Bias Current
    sendWriteCommand(1, 0b00100000);

    // R2: MUX Bias Current
    sendWriteCommand(2, 0b00101000);

    // R3: MUX Load, Temperature Sensor, and Auxiliary Digital Output
    sendWriteCommand(3, 0b00000000);

    // R4: ADC Output Format and DSP Offset Removal
    // Configured for two's complement, DSP offset removal
    sendWriteCommand(4, 0b11011000);

    // R5-R7: Impedance Check Control (set to 0)
    sendWriteCommand(5, 0b00000000);
    sendWriteCommand(6, 0b00000000);
    sendWriteCommand(7, 0b00000000);

    // R8-R11: On-Chip Amplifier Bandwidth High-Frequency Select
    // Adjusted for 10-2000 Hz bandwidth
    sendWriteCommand(8, 45);   // RH1 DAC1 
    sendWriteCommand(9, 7);    // RH1 DAC2 
    sendWriteCommand(10, 55);  // RH2 DAC1 
    sendWriteCommand(11, 8);   // RH2 DAC2 

    // R12-R13: On-Chip Amplifier Bandwidth Low-Frequency Select
    uint8_t R12, RL, RLDAC1, R13, ADCaux3en, RLDAC3, RLDAC2;

    // For 10 Hz lower cutoff
    RL = 0;
    RLDAC1 = 5;
    ADCaux3en = 0;
    RLDAC3 = 0;
    RLDAC2 = 1;

    R12 = ((RL << 7) | RLDAC1);
    R13 = (ADCaux3en << 7) | (RLDAC3 << 6) | RLDAC2;

    sendWriteCommand(12, R12);
    sendWriteCommand(13, R13);

    // R14-R17: Individual Amplifier Power (initially all off)
    sendWriteCommand(14, 0b00000000);
    sendWriteCommand(15, 0b00000000);
    sendWriteCommand(16, 0);
    sendWriteCommand(17, 0);
}

void IntanShield::setBandwidth(Bandselect bandSetting) {
    configureRegisters(bandSetting);
}

int16_t IntanShield::readChannelData(uint8_t channelnum) {
    return sendConvertCommand(channelnum);
}

void IntanShield::setChannelPower(bool channel1, bool channel2) {
    uint8_t reg14 = 0, reg15 = 0;
    
    if (channel1) {
        if (FIRST_CHANNEL < 8) {
            reg14 |= (1 << FIRST_CHANNEL);
        } else {
            reg15 |= (1 << (FIRST_CHANNEL - 8));
        }
    }
    
    if (channel2) {
        if (SECOND_CHANNEL < 8) {
            reg14 |= (1 << SECOND_CHANNEL);
        } else {
            reg15 |= (1 << (SECOND_CHANNEL - 8));
        }
    }
    
    // Write to power control registers
    sendWriteCommand(14, reg14);
    sendWriteCommand(15, reg15);
}

uint8_t IntanShield::scaleForDAC(int16_t rawData) {
    int temp = rawData >> 8;
    temp *= DAC_SCALE_COEFFICIENT;
    
    // Clip to 8-bit range
    temp = constrain(temp, -128, 127);
    
    return static_cast<uint8_t>(temp + 128);
}

uint8_t IntanShield::scaleForDAC_ACC(int16_t rawData) {
    int temp = rawData >> 8;
    temp *= DAC_SCALE_COEFFICIENT;
    
    // Clip to unsigned 8-bit range
    temp = constrain(temp, 0, 255);
    
    return static_cast<uint8_t>(temp);
}

// Placeholder implementations for other methods
long IntanShield::readAccumulatorData(uint8_t channelnum) {
    return readChannelData(channelnum);
}

long IntanShield::readThreshold() {
    return 200; // Default threshold
}

bool IntanShield::lowGainMode() {
    return false; // Default low gain mode
}