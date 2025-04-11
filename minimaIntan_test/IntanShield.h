#ifndef INTAN_SHIELD_H
#define INTAN_SHIELD_H

#include <SPI.h>
#include <Wire.h>

#if defined(ARDUINO) && ARDUINO >= 100
    #include "Arduino.h"
#else
    #include "WProgram.h"
#endif

// SPI Pins
const int chipSelectPin = 10; // Chip Select pin
const int sclkPin = 13;       // SPI Clock
const int mosiPin = 11;       // MOSI
const int misoPin = 12;       // MISO

// Function declarations - all sound-related functions removed
void InitIntanShield();
void TimerISR();
uint16_t SendReadCommand(uint8_t regnum);
uint16_t SendConvertCommand(uint8_t channelnum);
uint16_t SendConvertCommandH(uint8_t channelnum);
uint16_t SendWriteCommand(uint8_t regnum, uint8_t data);
uint8_t ScaleForDAC(int rawdata);
uint8_t ScaleForDAC_ACC(int rawdata);
void Calibrate();
int ReadChannelData(uint8_t channelnum);
long ReadAccumulatorData(uint8_t channelnum);
void NotchFilter60();
void NotchFilter50();
void NotchFilterNone();
void SetAmpPwr(bool Ch1, bool Ch2);
bool LowGainMode();
long ReadThreshold();

// Replace AVR-specific prescaler bits with direct values
const byte PS_128 = 0x07; // Equivalent to (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0)
const byte PS_16 = 0x04;  // Equivalent to (1 << ADPS2)

#define FIRSTCHANNEL 0
#define SECONDCHANNEL 15
#define DAC_SCALE_COEFFICIENT 12

#endif // INTAN_SHIELD_H