#ifndef UICONTROL_H
#define UICONTROL_H

#include <Arduino.h>
#include "IntanR4.h"
#include "filter.h"

class UIControl {
public:
    // Initialize the UI control with a baud rate
    static void init(unsigned long baudRate);

    // Process any incoming serial commands
    static void update();

    // Output data to serial port
    static void outputData();

    // Set the sampling rate for data output dividing
    static void setSampleRate(uint32_t sampleRate);

    // Process the buffer contents as a command
    static void processCommand(String command);

    // Run filter test with synthetic signal
    static void runFilterTest();

    // Print welcome message and command list
    static void printWelcomeMessage();

    // Print current status
    static void printStatus();

private:
    static unsigned long serialBaudRate;
    static String serialBuffer;
    static bool serialComplete;
    static uint32_t outputCounter;
    static uint32_t outputDivider;
    static uint32_t sampleRateHz;
    static char outputBuffer[1000];
};

#endif // UICONTROL_H
