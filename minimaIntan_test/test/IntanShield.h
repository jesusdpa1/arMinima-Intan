#ifndef INTAN_SHIELD_H
#define INTAN_SHIELD_H

#include <SPI.h>
#include <Wire.h>

// SPI Pins Configuration
const int chipSelectPin = 10; // Chip Select pin
const int sclkPin = 13;       // SPI Clock
const int mosiPin = 11;       // MOSI
const int misoPin = 12;       // MISO

// Constants and Definitions
#define FIRST_CHANNEL 0
#define SECOND_CHANNEL 15
#define DAC_SCALE_COEFFICIENT 12

// Bandwidth Selection Enum
enum Bandselect { 
    LowCutoff10Hz, 
    LowCutoff1Hz, 
    LowCutoff100mHz 
};

class IntanShield {
public:
    IntanShield();

    // Configuration and Initialization
    void begin();
    void calibrate();
    
    // SPI Communication Methods
    uint16_t sendReadCommand(uint8_t regnum);
    uint16_t sendWriteCommand(uint8_t regnum, uint8_t data);
    uint16_t sendConvertCommand(uint8_t channelnum);
    uint16_t sendConvertCommandH(uint8_t channelnum);

    // Data Processing Methods
    int16_t readChannelData(uint8_t channelnum);
    uint8_t scaleForDAC(int16_t rawdata);
    uint8_t scaleForDAC_ACC(int16_t rawdata);
    
    // Configuration Methods
    void setChannelPower(bool channel1, bool channel2);
    void setBandwidth(Bandselect bandSetting);
    
    // Utility Methods
    long readAccumulatorData(uint8_t channelnum);
    long readThreshold();
    bool lowGainMode();

    // Notch Filter - now public
    void applyNotchFilter60(int16_t& channelData);

private:
    // Internal configuration methods
    void configureRegisters(Bandselect bandSetting);

    // Filter history variables
    float inputHistory[1][3];
    float outputHistory[1][3];

    // Notch filter internal implementation
    void updateFilterHistory(float* inputHist, float* outputHist, int16_t& channelData);
};

#endif // INTAN_SHIELD_H