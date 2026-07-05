# Radio Common Library for nRF24L01+

A centralized radio configuration and communication library for ESP32-based scoreboard timer modules using nRF24L01+ radio modules.

## Overview

This library provides a common interface for nRF24L01+ radio communication across all scoreboard timer modules:

- Controller (master)
- Play Clock (display)
- Repeater (range extender)

## Features

- **Centralized Configuration**: All radio settings in one place
- **Network Standardization**: Consistent channel, address, and data rate settings
- **Platform Support**: ESP32 with ESP-IDF, with hooks for other platforms
- **Debug Support**: Comprehensive logging and register dump functionality
- **Error Handling**: Robust error checking and status reporting

## Quick Start

### Installation

Add as a Git submodule alongside your ESP-IDF project's `main/` directory:

```bash
git submodule add https://github.com/nevalions/scoreboard-clock-radio-common-nrf24.git radio-common
```

This is an ESP-IDF component only (`idf_component_register` in `radio-common/CMakeLists.txt`) —
there is no standalone/generic CMake target to `add_subdirectory()`/`target_link_libraries()`
against. Point ESP-IDF at the component directory in your project's top-level
`CMakeLists.txt`:

```cmake
set(EXTRA_COMPONENT_DIRS ../radio-common)
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(your_project)
```

Then require it from your `main/CMakeLists.txt`:

```cmake
idf_component_register(
    SRCS "main.c"
    INCLUDE_DIRS "."
    REQUIRES radio-common
)
```

### Basic Usage

```c
#include "radio_common.h"

RadioCommon radio;

// Initialize radio with your desired pins
if (!radio_common_init(&radio, YOUR_CE_PIN, YOUR_CSN_PIN)) {
    ESP_LOGE("APP", "Radio initialization failed");
    return;
}

// Configure with standard settings
if (!radio_common_configure(&radio)) {
    ESP_LOGE("APP", "Radio configuration failed");
    return;
}

// Set to TX mode for sending
nrf24_write_register(&radio, NRF24_REG_CONFIG, RADIO_CONFIG_TX_MODE);
```

## Hardware Configuration

### Default Pinout

- **SPI Interface**: SPI3 (MOSI: GPIO23, MISO: GPIO19, SCK: GPIO18)
- **Control Pins**: CE and CSN pins (configured by application)
- **Status LED**: GPIO2

**Note**: The library no longer defines default CE/CSN pins. These must be configured by your application when calling `radio_common_init()`.

### SPI Configuration

- **SPI Host**: SPI3_HOST (configurable in radio_config.h)
- **Clock Speed**: 1 MHz (suitable for nRF24L01+)
- **SPI Mode**: 0 (CPOL=0, CPHA=0)
- **Bit Order**: MSB first
- **CS Control**: Manual (CSN pin controlled by library)

### Radio Settings

- **Channel**: 76 default; runtime-agile across `RADIO_CHANNEL_CANDIDATES` {76, 82, 78, 74, 49, 24} — controller surveys occupancy (RPD) and picks the quietest, receivers hop the list until they hear frames
- **Data Rate**: 250 kbps (`RADIO_RF_SETUP` alias; fall back to 1 Mbps if clone modules fail at 250 kbps)
- **Power Level**: 0 dBm
- **Network Address**: 0xE7E7E7E7E7
- **Payload Size**: 6 bytes, fixed (`RADIO_PAYLOAD_SIZE`) — dynamic payloads (DYNPD/FEATURE)
  are never enabled
- **Auto-ACK**: Disabled on all pipes (`EN_AA` = 0x00) — broadcast; ACKs from multiple
  receivers would collide
- **Retries**: Disabled (`SETUP_RETR` = 0x00) — fire-and-forget
- **CRC**: 1-byte
- **Time field encoding** (payload bytes 0-1, big-endian): 0-99 whole seconds;
  255 null/clear; 256+d final-countdown deciseconds (d = 0-49, i.e. 4.9-0.1 s).
  Bit 15 flags "warn at 10 s" (football buzzer); receivers mask it off before
  interpreting the value

## Usage Examples

### Sending Data

```c
uint8_t payload[RADIO_PAYLOAD_SIZE] = { /* your 6 bytes of data */ };

// Write payload to TX buffer
nrf24_write_payload(&radio, payload, sizeof(payload));

// Pulse CE to transmit
gpio_write(radio.ce_pin, 1);
delay_us(15);
gpio_write(radio.ce_pin, 0);
```

### Receiving Data

```c
// Set to RX mode and start listening
nrf24_write_register(&radio, NRF24_REG_CONFIG, RADIO_CONFIG_RX_MODE);
gpio_write(radio.ce_pin, 1);

// Check for received data
if (nrf24_get_status(&radio) & NRF24_STATUS_RX_DR) {
    uint8_t payload[RADIO_PAYLOAD_SIZE];
    nrf24_read_payload(&radio, payload, sizeof(payload));

    // Process received data...

    // Clear RX flag
    nrf24_write_register(&radio, NRF24_REG_STATUS, NRF24_STATUS_RX_DR);
}
```

### Module-Specific Extensions

Each module can extend the base `RadioCommon` structure:

```c
// Controller-specific extensions
typedef struct {
    RadioCommon base;
    uint32_t last_success_time;
    uint32_t last_failure_time;
    uint16_t success_count;
    uint16_t failure_count;
    bool link_good;
} ControllerRadio;

// Play Clock-specific extensions
typedef struct {
    RadioCommon base;
    uint32_t last_message_time;
    bool link_alive;
} PlayClockRadio;
```

## API Reference

### Core Functions

| Function                         | Purpose                      |
| -------------------------------- | ---------------------------- |
| `radio_common_init()`            | Initialize radio hardware    |
| `radio_common_configure()`       | Apply standard configuration |
| `radio_common_set_addresses()`   | Set TX/RX addresses          |
| `radio_common_dump_registers()`  | Debug register dump          |
| `radio_common_is_connected()`    | Check hardware connectivity  |
| `radio_common_validate_config()` | Validate radio settings      |

### Low-Level Functions

| Function                 | Purpose              |
| ------------------------ | -------------------- |
| `nrf24_read_register()`  | Read radio register  |
| `nrf24_write_register()` | Write radio register |
| `nrf24_get_status()`     | Get status register  |
| `nrf24_read_payload()`   | Read RX payload      |
| `nrf24_write_payload()`  | Write TX payload     |
| `nrf24_flush_rx()`       | Flush RX buffer      |
| `nrf24_flush_tx()`       | Flush TX buffer      |
| `nrf24_power_up()`       | Power up radio       |
| `nrf24_power_down()`     | Power down radio     |

## Platform Support

### ESP32 (Primary)

Full support with ESP-IDF framework including:
- SPI3 driver integration with DMA support
- GPIO control for CE/CSN pins
- FreeRTOS delays and timing
- ESP logging system integration
- Hardware abstraction for easy porting

### Other Platforms (unused/untested)

A generic, non-ESP32 code path exists in `src/radio_common.c` behind weak symbols, but it is
currently unused and untested — nothing in this repo builds or exercises it. Implementing
support would mean providing these weak symbols:

- `radio_common_platform_init()` - Platform-specific initialization
- `radio_common_platform_transfer()` - SPI data transfer
- `radio_common_delay_ms/us()` - Timing functions

## Debugging

### Enable Debug Logging

```c
#define RADIO_DEBUG_ENABLED 1
```

### Common Debug Commands

```c
// Dump all radio registers
radio_common_dump_registers(&radio);

// Test hardware connectivity
if (!radio_common_is_connected(&radio)) {
    ESP_LOGE("APP", "Radio not connected");
}

// Validate current configuration
if (!radio_common_validate_config(&radio)) {
    ESP_LOGE("APP", "Radio configuration invalid");
}
```

## Building

### ESP-IDF Component (only supported build)

This module is an ESP-IDF component only (`idf_component_register` in `CMakeLists.txt`).
There is no standalone CMake build target and no test suite. It is consumed by adding it as
a component directory inside an ESP-IDF project; ESP-IDF's own build system registers and
builds it automatically.

### Standalone Library (planned / not implemented)

A standalone (non-ESP-IDF) CMake build is not currently provided. The non-ESP32 code path
(`radio_common_platform_init`, `radio_common_platform_transfer`, etc., guarded by weak
symbols) exists in `src/radio_common.c` for future portability but is currently unused and
untested — no build wires it up, and there is no test suite exercising it.

## Troubleshooting

### Common Issues

**Radio not responding**

- Check CE and CSN pin connections
- Verify SPI wiring (MOSI, MISO, SCK)
- Ensure power supply is stable (3.3V)

**Communication failures**

- Verify both devices use the same channel and address
- Check for interference on the selected channel
- Ensure payload sizes match on TX/RX

**Range issues**

- Check antenna connection
- Verify power level settings
- Consider adding repeater modules

## License

This library is released under the same license as the scoreboard clock project.
