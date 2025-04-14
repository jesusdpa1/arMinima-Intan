#ifndef INTAN_R4_H
#define INTAN_R4_H

#include <SPI.h>
#include <Wire.h>
#include <FspTimer.h>
#include "filter.h"  // Include the filter header

#if defined(ARDUINO) && ARDUINO >= 100
    #include "Arduino.h"
#else
    #include "WProgram.h"
#endif

// Number of channels supported (can be increased as needed)
#define NUM_CHANNELS 2

// Configuration settings structure
struct IntanConfig {
    bool lowGainMode;         // When enabled, scale signals down by a factor of 4
    bool averageEnergyMode;   // Whether to display energy over time or raw data
    bool notchEnabled;        // Whether notch filter is enabled
    bool notch60Hz;           // If true, 60Hz; if false, 50Hz
    int thresholdValue;       // Threshold for detecting activity
    bool channel1Enabled;     // Enable/disable first channel
    bool channel2Enabled;     // Enable/disable second channel
};

// Default configuration
extern IntanConfig config;

// Timer callback class for R4 Minima
class IntanCallback {
public:
    static void timerCallback(timer_callback_args_t *p_args);
};

// SPI Pins for Arduino R4 Minima
const int INTAN_CS_PIN = 10;
const int INTAN_SCLK_PIN = 13;
const int INTAN_MOSI_PIN = 11;
const int INTAN_MISO_PIN = 12;

// Define constants for SPI commands
#define WRITE_CMD_MASK     0x8000  // 1000 0000 0000 0000
#define CONVERT_CMD_MASK   0x0000  // 0000 0000 0000 0000
#define CONVERT_HP_CMD_MASK 0x0001 // 0000 0000 0000 0001
#define CALIBRATE_CMD      0x5500  // 0101 0101 0000 0000
#define READ_CMD_MASK      0xC000  // 1100 0000 0000 0000

// Define channels
#define CHANNEL_1          0
#define CHANNEL_2          15

// Get/Set current notch filter type
enum NotchFilterType { NOTCH_60HZ, NOTCH_50HZ, NOTCH_NONE };
extern NotchFilterType notchFilter;

// Filter coefficients for 50Hz and 60Hz notch filters
extern FilterCoeff notch50HzCoeff;
extern FilterCoeff notch60HzCoeff;

// Function declarations
void intanInit(uint32_t sampleRate = 5000);
void intanInitFilters(uint32_t sampleRate);
void intanUpdateConfig(IntanConfig newConfig);
void intanReset();
uint16_t intanSendReadCommand(uint8_t regnum);
uint16_t intanSendConvertCommand(uint8_t channelnum);
uint16_t intanSendConvertCommandH(uint8_t channelnum);
uint16_t intanSendWriteCommand(uint8_t regnum, uint8_t data);
void intanCalibrate();
int16_t intanReadRawChannelData(uint8_t channelnum);
int16_t intanReadChannelData(uint8_t channelnum);
int32_t intanReadAccumulatorData(uint8_t channelnum);
void intanApplyNotchFilter(uint8_t channel, NotchFilterType filterType);
void intanProcessTimerEvent();

// Enumerations for bandwidths
enum IntanBandwidth {
    INTAN_BW_10HZ = 0,    // 10Hz low cutoff
    INTAN_BW_1HZ = 1,     // 1Hz low cutoff
    INTAN_BW_0_1HZ = 2    // 0.1Hz low cutoff
};

// Set bandwidth cutoff
void intanSetBandwidth(IntanBandwidth bandwidth);

// Initialize registers with specific settings
void intanInitializeRegisters();

// Set channel power - two versions for flexibility
void intanSetChannelPower(bool ch1Power, bool ch2Power); // Legacy version
void intanSetChannelPower(bool* channelPowerSettings, int numChannels); // New scalable version

#endif // INTAN_R4_H
