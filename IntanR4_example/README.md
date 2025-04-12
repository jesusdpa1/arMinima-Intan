# Intan RHD2216 Library for Arduino R4 Minima

This library provides an interface between the Arduino R4 Minima and the Intan RHD2216 digital electrophysiology chip. The implementation is designed for the Renesas RA4M1 ARM Cortex-M4 microcontroller on the R4 Minima.

## Features

- **High-Speed Sampling**: 5kHz per channel sampling rate
- **Software-Controlled Parameters**: All settings configurable via serial commands
- **Notch Filtering**: Software-based 50Hz and 60Hz notch filters
- **Bandwidth Control**: Fixed 10Hz lower cutoff with configurable options
- **Channel Control**: Enable/disable individual channels programmatically
- **Activity Detection**: Digital outputs triggered when signal exceeds threshold
- **Serial Output**: Real-time data streaming at 250kbps

## Hardware Connections

Connect the RHD2216 chip to the Arduino R4 Minima as follows:

- **SPI Connections**:
  - CS: Pin 10
  - SCLK: Pin 13
  - MOSI: Pin 11
  - MISO: Pin 12

- **Status Output Pins**:
  - Digital Pin 4: Channel 1 activity indicator
  - Digital Pin 5: Channel 2 activity indicator

## Available Commands

The library provides a serial command interface to control all settings:

| Command | Description |
|---------|-------------|
| `gain [high\|low]` | Set gain mode (high = normal gain, low = 1/4 gain) |
| `notch [on\|off]` | Enable/disable notch filter |
| `notch60 [on\|off]` | Set notch filter to 60Hz (on) or 50Hz (off) |
| `threshold [value]` | Set threshold value (0-1023) |
| `channel1 [on\|off]` | Enable/disable channel 1 |
| `channel2 [on\|off]` | Enable/disable channel 2 |
| `status` | Display current settings |
| `reset` | Reset to default settings |

## Serial Output Format

The serial output provides both raw ADC values and voltage in microvolts:

```
<raw_ch1>,<raw_ch2>,<uV_ch1>,<uV_ch2>
```

Where:
- `raw_ch1`, `raw_ch2`: Raw 16-bit ADC values from channels 1 and 2
- `uV_ch1`, `uV_ch2`: Voltage in microvolts (LSB = 0.195 µV)

## Implementation Details

The library uses the Arduino R4 Minima's FspTimer API for precise timing, which is tailored to the Renesas RA4M1 microcontroller architecture. This is a significant departure from AVR-based Arduino implementations that use direct register manipulation for timing.

The RHD2216 communication uses the SPI protocol with a 20MHz clock speed for fast data transfer. The library handles all the necessary SPI command formatting, including:

- READ commands (0xC000 | register << 8)
- WRITE commands (0x8000 | register << 8 | data)
- CONVERT commands (0x0000 | channel << 8)
- CALIBRATE commands (0x5500)

## License

This library is provided as open-source software under the MIT License.

## Acknowledgments

This implementation is based on knowledge gained from studying the Intan STM32 firmware framework and adapting it for the Arduino R4 Minima platform.
