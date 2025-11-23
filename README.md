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

Add as Git submodule to your ESP-IDF project:
```bash
git submodule add https://github.com/nevalions/scoreboard-clock-radio-common-nrf24.git radio-common
```

Update your `CMakeLists.txt`:
```cmake
add_subdirectory(radio-common)
target_link_libraries(your_target radio_common)
```

### Basic Usage

```c
#include "radio_common.h"

RadioCommon radio;

// Initialize radio
if (!radio_common_init(&radio, GPIO_NUM_5, GPIO_NUM_4)) {
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
- **SPI Interface**: SPI2 (MOSI: GPIO23, MISO: GPIO19, SCK: GPIO18)
- **Control Pins**: CE (GPIO5), CSN (GPIO4)
- **Status LED**: GPIO2

### Radio Settings
- **Channel**: 76 (2.476 GHz)
- **Data Rate**: 1 Mbps
- **Power Level**: 0 dBm
- **Network Address**: 0xE7E7E7E7E7
- **Payload Size**: 32 bytes
- **Auto-ACK**: Enabled
- **Retries**: Up to 3, 750µs delay

## Usage Examples

### Sending Data

```c
uint8_t payload[32] = { /* your data */ };

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
    uint8_t payload[32];
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

| Function | Purpose |
|----------|---------|
| `radio_common_init()` | Initialize radio hardware |
| `radio_common_configure()` | Apply standard configuration |
| `radio_common_set_addresses()` | Set TX/RX addresses |
| `radio_common_dump_registers()` | Debug register dump |
| `radio_common_is_connected()` | Check hardware connectivity |
| `radio_common_validate_config()` | Validate radio settings |

### Low-Level Functions

| Function | Purpose |
|----------|---------|
| `nrf24_read_register()` | Read radio register |
| `nrf24_write_register()` | Write radio register |
| `nrf24_get_status()` | Get status register |
| `nrf24_read_payload()` | Read RX payload |
| `nrf24_write_payload()` | Write TX payload |
| `nrf24_flush_rx()` | Flush RX buffer |
| `nrf24_flush_tx()` | Flush TX buffer |
| `nrf24_power_up()` | Power up radio |
| `nrf24_power_down()` | Power down radio |

## Platform Support

### ESP32 (Primary)
Full support with ESP-IDF framework including SPI driver integration, GPIO control, FreeRTOS delays, and ESP logging system.

### Other Platforms
Generic platform support using weak symbols. Implement these functions:
- `radio_common_platform_init()`
- `radio_common_platform_transfer()`
- `radio_common_delay_ms/us()`

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

### Standalone Library
```bash
mkdir build && cd build
cmake ..
make
```

### ESP-IDF Component
The library automatically detects ESP-IDF environment and registers as a component.

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

For development guidelines and contribution instructions, see [AGENTS.md](AGENTS.md).

## License

This library is released under the same license as the scoreboard clock project.