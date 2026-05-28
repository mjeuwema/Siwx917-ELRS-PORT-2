# ExpressLRS 4.0 Receiver - SiWx917 + LR1121

An ExpressLRS 4.0 compatible receiver implementation for the Silicon Labs SiWx917 SoC with Semtech LR1121 radio module.

## Hardware

- **MCU:** Silicon Labs SiWx917 (Wi-Fi + BLE SoC)
- **Radio:** Waveshare Core1121-HF (Semtech LR1121 sub-GHz transceiver)
- **RF Switch:** PE4259 antenna switch
  - DIO5 → VDD (power enable, active high)
  - DIO6 → CTRL (path select: LOW=TX/RF1, HIGH=RX/RF2)

### Connections (GSPI)

| SiWx917 Pin | LR1121 Pin | Function |
|-------------|------------|----------|
| GPIO_25     | SCK        | SPI Clock |
| GPIO_26     | MISO       | SPI Data Out |
| GPIO_27     | MOSI       | SPI Data In |
| GPIO_28     | NSS        | SPI Chip Select |
| GPIO_29     | BUSY       | Radio Busy |
| GPIO_46     | DIO9       | Radio IRQ (ELRS DIO1 signal) |
| GPIO_30     | NRESET     | Radio Reset |

### Gemini / Diversity Prototype Wiring

The first Gemini bring-up should keep both LR1121 modules on the proven
16 MHz GSPI bus and add separate control lines for the second radio. This
matches the ELRS dual-radio model while avoiding a second, unproven SPI driver
path in the timing-critical packet loop.

| Signal | Radio 1 | Radio 2 prototype | Notes |
|--------|---------|-------------------|-------|
| SCK    | GPIO_25 | GPIO_25 shared    | Shared GSPI clock |
| MISO   | GPIO_26 | GPIO_26 shared    | Both LR1121 MISO pins must tri-state when NSS is high |
| MOSI   | GPIO_27 | GPIO_27 shared    | Shared GSPI MOSI |
| NSS    | GPIO_28 | GPIO_50           | Separate chip-select per radio |
| BUSY   | GPIO_29 | GPIO_51           | Separate BUSY per radio |
| DIO9   | GPIO_46 | GPIO_47           | Separate IRQ per radio |
| NRESET | GPIO_30 | GPIO_49           | Separate reset is better for debug |

Avoid GPIO_10/11 (LED/button), GPIO_52/54/55 (USART0 serial), and ULP/UULP GPIOs
for the second radio IRQ path. On BRD2708A, the breakout pads expose convenient
HP GPIOs GPIO_50, GPIO_51, GPIO_48, GPIO_47, GPIO_49, GPIO_46, and GPIO_15; the
prototype map keeps GPIO_46 for Radio 1 DIO9 and uses GPIO_50/51/47/49 for
Radio 2.

## Features

- ExpressLRS 4.0 protocol (900MHz ISM band)
- FHSS with 80 channels
- Multiple packet rates (50Hz - 1000Hz)
- Wi-Fi configuration portal
- NVM3 persistent storage for binding/settings
- CRSF output for flight controller

## Prerequisites

### Required Software

1. **Simplicity Studio 5** (v5.9 or later)
   - Download: https://www.silabs.com/developers/simplicity-studio

2. **WiSeConnect 3 SDK** (2025.6.2 or compatible)
   - Install via Simplicity Studio Package Manager
   - Required components: Wi-Fi, GSPI, NVM3, FreeRTOS

3. **GNU ARM Toolchain** (included with Simplicity Studio)

### SDK Installation

The build system expects the Silicon Labs SDK installed via Simplicity Studio. After installation, the SDK is typically located at:
```
Windows: C:\Users\<username>\.silabs\slt\installs\
macOS:   ~/.silabs/slt/installs/
Linux:   ~/.silabs/slt/installs/
```

## Building

### Option 1: Simplicity Studio (Recommended)

1. Open Simplicity Studio
2. File → Import → MCU Project
3. Select the `wifi_gspi_merged.slcp` file
4. Build using the IDE

### Option 2: Command Line (CMake)

After importing the project once in Simplicity Studio (to generate SDK paths):

```bash
cd cmake_gcc
cmake --preset base
cd build
cmake --build .
```

Output: `cmake_gcc/build/base/wifi_gspi_merged.rps`

### Regenerating Project Files

If SDK paths are incorrect, regenerate using Simplicity Studio:

1. Open the `.slcp` project file
2. Right-click → Force Generation
3. This updates `cmake_gcc/wifi_gspi_merged.cmake` with correct SDK paths

## Flashing

Using Simplicity Commander:
```bash
commander flash cmake_gcc/build/base/wifi_gspi_merged.rps
```

Or via Simplicity Studio debug interface.

## Configuration

### Binding Phrase

Set your binding phrase in `src/elrs_config.c`:
```c
.uid = { 0xBE, 0x93, 0x67, 0x27, 0xD6, 0x9C }  // "matthew"
```

To generate UID from a binding phrase, use the same MD5-based algorithm as ExpressLRS:
```python
import hashlib
uid = hashlib.md5(b"your_phrase").digest()[:6]
```

### Wi-Fi Configuration

Default AP settings in `config/sl_net_default_values.h`:
- SSID: `ELRS_RX`
- Password: `expresslrs`
- IP: `10.0.0.1`

Connect to configure binding phrase, packet rate, and other settings via web interface.

## Project Structure

```
wifi_gspi_merged/
├── src/
│   ├── elrs_protocol/     # ELRS protocol implementation
│   │   ├── elrs_rx.c      # Main RX state machine
│   │   ├── fhss.c         # Frequency hopping
│   │   ├── crc.c          # CRC calculations
│   │   ├── ota.c          # Over-the-air packet handling
│   │   ├── LR1121Driver.c # ELRS-compatible LR1121 driver
│   │   └── ...
│   ├── elrs_web/          # Web configuration interface
│   ├── lr1121_driver.c    # Low-level SPI/DMA driver
│   ├── lr1121_elrs_init.c # Radio initialization
│   ├── elrs_config.c      # Persistent configuration
│   └── ...
├── config/                # Hardware configuration headers
├── autogen/               # Generated files (linker script, etc.)
├── cmake_gcc/             # CMake build system
└── wifi_gspi_merged.slcp  # Simplicity Studio project
```

## Status

**Work in Progress** - Currently debugging packet CRC validation.

- [x] GSPI communication with LR1121
- [x] Radio initialization and configuration
- [x] FHSS channel hopping
- [x] Preamble detection and RxDone interrupts
- [x] RF switch configuration (PE4259)
- [ ] CRC validation on received packets
- [ ] ELRS binding/sync
- [ ] CRSF output to FC
- [ ] Telemetry uplink

## License

This project incorporates code from:
- [ExpressLRS](https://github.com/ExpressLRS/ExpressLRS) - GPLv3
- Silicon Labs SDK - Silicon Labs License

## Acknowledgments

- ExpressLRS team for the protocol implementation
- Silicon Labs for the SiWx917 SDK
- Semtech for LR1121 documentation
