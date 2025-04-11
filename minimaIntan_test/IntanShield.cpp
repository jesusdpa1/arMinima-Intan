#include "IntanShield.h"

/* Global variables */

enum Notchselect { Notch60, Notch50, NoNotch };
Notchselect notch_setting; // notch_setting has 3 possible values. It is set by Analog Pins 2 and 3 (aka Digital Pins 16 and 17) and updated every 0.5 ms

volatile bool audio_enable = false; // audio_enable is set by Digital Pin 6 and updated every 0.5 ms - kept for logic but sound functionality removed

volatile bool low_gain_mode; // low_gain_mode is either true or false. It is set by Digital Pin 7 and updated every 0.5 ms
                             // When enabled, scale channel data down by a factor of 4. By default, channel data begins to clip when input voltages are 1.1 mV peak-to-peak. When low_gain_mode is enabled, this increases to 4.4 mV peak-to-peak
                             // Should be enabled when signals appear too strong and are clipping (more common with EKG than EMG). Should be disabled when signals appear too weak and are hard to see

volatile long accumulator[2] = {0L, 0L}; // Add up the amplitudes of the signal over a short period to determine how strong the signal is
volatile long amplitude[2] = {0, 0}; // Determine the amplitude of a signal, sample by sample, to determine how strong the signal is
volatile long ampmax[2] = {0, 0}; // Keep track of the maximum amplitude of every accumulation period
volatile long ampmin[2] = {255, 255}; // Keep track of the minimum amplitude of every accumulation period
volatile long finalaccumulator[2] = {0L, 0L}; // Retain the final values of the accumulator

volatile long high_threshold; // Set the high threshold of each channel - how strong must a signal be to trigger a sound - set by potentiometer

volatile bool reset[2] = {true, true}; // Keep track of recent history of signal strength.
                                       // If a signal was recently strong enough to trigger a sound or digital out pulse, reset = false
                                       // Strong signals thereafter will not trigger new sounds or pulses until the signal becomes weak enough to fall below the low threshold

volatile int i = 0; // Keep track of which channel's turn it is to be sampled. The ISR runs every 0.5 ms, and we sample 1 channel per ISR

volatile int16_t channel_data[2] = {0, 0}; // Store the 16-bit channel data from two channels of the RHD
volatile int16_t final_channel_data[2] = {0, 0}; // Store the final 8-bit version of the channel data
volatile float in[2][3] = {{0}}; // Keep track of past values of the original signal for notch filtering
volatile float out[2][3] = {{0}}; // Keep track of the past values of the processed signal for notch filtering
volatile bool firstchannelpower; // Adapt FirstChannelPwr from the .ino file (which is easier for the user to change), into a global variable
volatile bool secondchannelpower; // Adapt SecondChannelPwr from the .ino file (which is easier for the user to change), into a global variable

// Add initialization function for SPI
void InitIntanShield() {
    // Configure SPI pins
    pinMode(chipSelectPin, OUTPUT);
    pinMode(sclkPin, OUTPUT);
    pinMode(mosiPin, OUTPUT);
    pinMode(misoPin, INPUT);
    
    digitalWrite(chipSelectPin, HIGH); // Start with CS high (inactive)
    
    // Initialize SPI
    SPI.begin();
    SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
}

// Create a custom ISR for Arduino R4 Minima using the Arduino API instead of direct register manipulation
// This needs to be set up in your main sketch with proper timer configuration
void TimerISR() {
    digitalWrite(2, HIGH); // Optional but helpful - monitor how much time each interrupt cycle takes

    // Update notch_setting in case the user changed the state of the DIP switch
    if (digitalRead(16) == LOW) {
        notch_setting = NoNotch;
    }
    else {
        if (digitalRead(17) == HIGH) {
            notch_setting = Notch60; 
        }
        else {
            notch_setting = Notch50;
        }
    }

    // Update audio_enable in case the user changed the state of the DIP switch
    if (digitalRead(6) == HIGH)
        audio_enable = true;
    else
        audio_enable = false;

    // Update low_gain_mode in case the user changed the state of the DIP switch
    if (digitalRead(7) == HIGH)
        low_gain_mode = true;
    else
        low_gain_mode = false;
  
    static int loopcounter = 0; // Variable to keep track of how many times the ISR has been run since beginning of the accumulation period

    /* Reading channel data from the RHD, and filtering/formatting the data */
    static int testcount = 0;
    
    // Read the channel data from whichever sample in the pipeline corresponds to this 1/2000th of a second
    if (i == 0) {
        channel_data[0] = SendConvertCommand(FIRSTCHANNEL);
        if (!firstchannelpower)
            channel_data[0] = 0;
    }

    if (i == 1) {
        channel_data[1] = SendConvertCommand(SECONDCHANNEL);
        if (!secondchannelpower)
            channel_data[1] = 0;
    }
    
    // Notch filter calculation - if no notch filter is desired, NotchFilterNone should still be called to scale the channel data
    switch(notch_setting) {
        case Notch60:
            NotchFilter60();
            break;
        case Notch50:
            NotchFilter50();
            break;
        case NoNotch:
            NotchFilterNone();
            break;
    }
    
    final_channel_data[i] = channel_data[i]; // Save channel data into a variable that is not directly manipulated by the NotchFilter functions

    // Accumulation period logic
    if (loopcounter < 20) {
        // Find the maximum absolute measurement from the data for this cycle's channel
        ampmax[i] = max(channel_data[i], ampmax[i]);

        // Find the minimum absolute measurement from the data for this cycle's channel
        ampmin[i] = min(channel_data[i], ampmin[i]);

        // Find the "amplitude" of the data as the largest possible difference between max and min
        amplitude[i] = max(ampmax[i] - ampmin[i], amplitude[i]);

        // To read lower-frequency signals, also consider large offsets that approach DC
        amplitude[i] = max(amplitude[i], ampmax[i]-128);
        amplitude[i] = max(amplitude[i], 128-ampmin[i]);

        // Add the amplitude to the accumulator bin
        accumulator[i] += amplitude[i];

        if (i == 1) {
            // When both channels have been updated, increase loopcounter by one.
            loopcounter++;
        }
    }
    else if (i == 1) {
        // If this is the end of the accumulator period, determine if digital output should be changed
        
        /* Read potentiometer value from ADC to determine what the threshold value should be */
        high_threshold = 30 * analogRead(A0) + 200;
        if (low_gain_mode)
            high_threshold = high_threshold * 4;
        
        for (int x = 0; x < 2; x++) {
            // For each channel
            if (accumulator[x] > high_threshold && reset[x] == true) {
                // If the signal is strong enough to surpass the high threshold, and was preceded by at least one weak accumulation period
                digitalWrite(x + 4, HIGH); // Write HIGH to the Digital Pin correponding to the channel
                
                // Note: Sound functionality removed - digital output is still maintained

                // Flag that this channel's last accumulation was strong
                reset[x] = false;
            }
            else if (accumulator[x] < 0.5 * high_threshold) {
                // If the signal is lower than 30% of the high threshold, it is considered low enough enough to reset the channel
                digitalWrite(x + 4, LOW);
                reset[x] = true;
            }
            
            // Keep data in final output accumulator
            finalaccumulator[x] = accumulator[x];
      
            // Clear the accumulator bin to 0 to prepare for the next accumulation period
            accumulator[x] = 0L; 
        } 

        // Set the loop counter to 0 to prepare for the next accumulation period  
        loopcounter = 0;
        for (int x = 0; x < 2; x++) {
            // Reinitialize the accumulator calculation variables for all channels
            amplitude[x] = 0;
            accumulator[x] = 0;
            ampmax[x] = 0;
            ampmin[x] = 255;
        }
    }

    if (i == 1)
        // If we just read the data from the SECONDCHANNEL, read FIRSTCHANNEL on the next iteration
        i = 0;
    else
        // If we just read the data from the FIRSTCHANNEL, read SECONDCHANNEL on the next iteration
        i++;
      
    // Optional but helpful - monitor how much time each interrupt cycle takes
    digitalWrite(2, LOW);
}

uint16_t SendReadCommand(uint8_t regnum) {
    // Sends a READ command to the RHD chip through the SPI interface
    uint16_t mask = regnum << 8;
    mask = 0b1100000000000000 | mask;
    digitalWrite(chipSelectPin, LOW);
    uint16_t out = SPI.transfer16(mask);
    digitalWrite(chipSelectPin, HIGH);
    return out;
}

uint16_t SendConvertCommand(uint8_t channelnum) {
    // Sends a CONVERT command to the RHD chip through the SPI interface
    uint16_t mask = channelnum << 8;
    mask = 0b0000000000000000 | mask;
    digitalWrite(chipSelectPin, LOW);
    uint16_t out = SPI.transfer16(mask);
    digitalWrite(chipSelectPin, HIGH);
    return out;
}

uint16_t SendConvertCommandH(uint8_t channelnum) {
    // Sends a CONVERT command to the RHD chip through the SPI interface, with the LSB set to 1
    uint16_t mask = channelnum << 8;
    mask = 0b0000000000000001 | mask;
    digitalWrite(chipSelectPin, LOW);
    uint16_t out = SPI.transfer16(mask);
    digitalWrite(chipSelectPin, HIGH);
    return out;
}

uint16_t SendWriteCommand(uint8_t regnum, uint8_t data) {
    // Sends a WRITE command to the RHD chip through the SPI interface
    uint16_t mask = regnum << 8;
    mask = 0b1000000000000000 | mask | data;
    digitalWrite(chipSelectPin, LOW);
    uint16_t out = SPI.transfer16(mask);
    digitalWrite(chipSelectPin, HIGH);
    return out;
}

void Calibrate() {
    // Sends a CALIBRATE command to the RHD chip through the SPI interface
    digitalWrite(chipSelectPin, LOW);
    SPI.transfer16(0b0101010100000000);
    digitalWrite(chipSelectPin, HIGH);
    int i = 0;
    for (i = 0; i < 9; i++) {
        // Use the READ command as a dummy command
        SendReadCommand(40);
    }
}

uint8_t ScaleForDAC(int rawdata) {
    // Converts raw 16-bit data from the chip to 8-bit data suitable for sending over the DAC
    
    // Save rawdata into a temporary variable
    int temp = rawdata;
    
    temp = rawdata >> 8;
    
    // Multiply the data by a scaling coefficient
    temp = temp * DAC_SCALE_COEFFICIENT;
    
    // If low_gain_mode is enabled, divide output by 4
    if (low_gain_mode == true)
        temp = temp / 4;
    
    // If the output is below the DAC's output range, set it to the lowest value
    if (temp < -128)
        temp = -128;
    
    // If the output is above the DAC's output range, set it to the highest value
    else if (temp > 127)
        temp = 127;
    
    // Return the scaled output, with an offset to make it a range of unsigned integers from 0 to 255
    return (uint8_t) (temp + 128);
}

uint8_t ScaleForDAC_ACC(int rawdata) {
    // Converts raw 16-bit data from the chip to 8-bit data suitable for sending over the DAC
    
    // Save rawdata into a temporary variable
    int temp = rawdata;
    
    temp = rawdata >> 8;
    
    // Multiply the data by a scaling coefficient
    temp = temp * DAC_SCALE_COEFFICIENT;
    
    // If low_gain_mode is enabled, divide output by 4
    if (low_gain_mode == true)
        temp = temp / 4;
    
    // If the output is below the DAC's output range, set it to the lowest value
    if (temp < 0)
        temp = 0;
    
    // If the output is above the DAC's output range, set it to the highest value
    else if (temp > 255)
        temp = 255;
    
    // Return the scaled output
    return (uint8_t) (temp);
}

int ReadChannelData(uint8_t channelnum) {
    // Function called by the main loop of the sketch to access the global sample variable
    if (channelnum == FIRSTCHANNEL)
        return (int) final_channel_data[0];
    else if (channelnum == SECONDCHANNEL)
        return (int) final_channel_data[1];
    
    return 0; // Default return if channel not found
}

long ReadAccumulatorData(uint8_t channelnum) {
    // Function called by the main loop of the sketch to access the global accumulator variable
    if (channelnum == FIRSTCHANNEL)
        return (long) finalaccumulator[0];
    else if (channelnum == SECONDCHANNEL)
        return (long) finalaccumulator[1];
    
    return 0; // Default return if channel not found
}

void NotchFilter60() {
    // Function that uses an IIR notch filter to remove 60 Hz noise
  
    // Updating the previous values for the input arrays as a new sample comes in
    in[i][0] = in[i][1];
    in[i][1] = in[i][2];
    in[i][2] = channel_data[i];

    // Performing the IIR notch filter algorithm
    out[i][2] = 0.9696 * in[i][0] - 1.803 * in[i][1] + 0.9696 * in[i][2] - 0.9391 * out[i][0] + 1.8029 * out[i][1];

    // Update the previous values for the output arrays
    out[i][0] = out[i][1];
    out[i][1] = out[i][2];

    // Save the output of the IIR notch filter algorithm to the global variable channel_data
    channel_data[i] = out[i][2];
}

void NotchFilter50() {
    // Function that uses an IIR notch filter to remove 50 Hz noise
    
    // Updating the previous values for the input arrays as a new sample comes in
    in[i][0] = in[i][1];
    in[i][1] = in[i][2];
    in[i][2] = channel_data[i];

    // Performing the IIR notch filter algorithm
    out[i][2] = 0.9696 * in[i][0] - 1.8443 * in[i][1] + 0.9696 * in[i][2] - 0.9391 * out[i][0] + 1.8442 * out[i][1];

    // Update the previous values for the output arrays
    out[i][0] = out[i][1];
    out[i][1] = out[i][2];

    // Save the output of the IIR notch filter algorithm to the global variable channel_data
    channel_data[i] = out[i][2];
}

void NotchFilterNone() {
    // Does nothing, only used as a placeholder
}

void SetAmpPwr(bool Ch1, bool Ch2) {
    // Function that takes amplifier channel power information from the .ino file
    
    // 2 8-bit variables to hold the values read from Registers 14 and 15
    uint8_t previousreg14;
    uint8_t previousreg15;
    
    // Use READ commands to assign values from the chip to the 8-bit variables
    SendReadCommand(14);
    SendReadCommand(14);
    previousreg14 = SendReadCommand(14);
    SendReadCommand(15);
    SendReadCommand(15);
    previousreg15 = SendReadCommand(15);
    
    // If Ch1 is true, set firstchannelpower as true and send a WRITE command
    if (Ch1) {
        firstchannelpower = true;
        if (FIRSTCHANNEL < 8) {
            // If FIRSTCHANNEL is an amplifier channel between 0 and 7
            SendWriteCommand(14, (1<<FIRSTCHANNEL | previousreg14));
            previousreg14 = 1 << FIRSTCHANNEL | previousreg14;
        }
        else if (FIRSTCHANNEL >= 8){
            // If FIRSTCHANNEL is an amplifier channel between 8 and 15
            SendWriteCommand(15, (1<<abs(FIRSTCHANNEL-8) | previousreg15));
            previousreg15 = 1 << abs(FIRSTCHANNEL-8) | previousreg15;
        }
    }
    else {
        // If Ch1 is false, set firstchannelpower as false
        firstchannelpower = false;
    }
    
    // If Ch2 is true, set secondchannelpower as true and send a WRITE command
    if (Ch2) {
        secondchannelpower = true;
        if (SECONDCHANNEL < 8)
            // If SECONDCHANNEL is an amplifier channel between 0 and 7
            SendWriteCommand(14, (1<<SECONDCHANNEL | previousreg14));
        else if (SECONDCHANNEL >= 8)
            // If SECONDCHANNEL is an amplifier channel between 8 and 15
            SendWriteCommand(15, (1<<abs(SECONDCHANNEL-8) | previousreg15));
    }
    else {
        // If Ch2 is false, set secondchannelpower as false
        secondchannelpower = false;
    }
}

bool LowGainMode() {
    // Simple function that returns the global variable low_gain_mode
    return low_gain_mode;
}

long ReadThreshold() {
    // Simple function that returns the global variable high_threshold
    return high_threshold;
}