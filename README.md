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

## Radio Configuration

- **Channel**: 76 (2.476 GHz)
- **Data Rate**: 1 Mbps
- **Power Level**: 0 dBm
- **Network Address**: 0xE7E7E7E7E7
- **Payload Size**: 32 bytes
- **Auto-ACK**: Enabled
- **Retries**: Up to 3, 750µs delay

## Hardware Configuration

- **SPI Interface**: SPI2 (MOSI: GPIO23, MISO: GPIO19, SCK: GPIO18)
- **Control Pins**: CE (GPIO5), CSN (GPIO4)
- **Status LED**: GPIO2

## Usage

### Basic Initialization

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

// Set to TX mode
nrf24_write_register(&radio, NRF24_REG_CONFIG, RADIO_CONFIG_TX_MODE);
```

### Sending Data

```c
uint8_t payload[32] = { /* your data */ };

// Write payload
nrf24_write_payload(&radio, payload, sizeof(payload));

// Pulse CE to transmit
gpio_write(radio.ce_pin, 1);
delay_us(15);
gpio_write(radio.ce_pin, 0);
```

### Receiving Data

```c
// Set to RX mode
nrf24_write_register(&radio, NRF24_REG_CONFIG, RADIO_CONFIG_RX_MODE);
gpio_write(radio.ce_pin, 1);

// Check for data
if (nrf24_get_status(&radio) & NRF24_STATUS_RX_DR) {
    uint8_t payload[32];
    nrf24_read_payload(&radio, payload, sizeof(payload));
    
    // Process data...
    
    // Clear RX flag
    nrf24_write_register(&radio, NRF24_REG_STATUS, NRF24_STATUS_RX_DR);
}
```

## Integration with Modules

### Adding to Existing Module

1. Add as Git submodule:
   ```bash
   git submodule add https://github.com/nevalions/scoreboard-clock-radio-common-nrf24.git radio-common
   ```

2. Update CMakeLists.txt:
   ```cmake
   add_subdirectory(radio-common)
   target_link_libraries(your_target radio_common)
   ```

3. Include in your code:
   ```c
   #include "radio_common.h"
   ```

### Module-Specific Extensions

Each module can extend the base `RadioCommon` structure with module-specific fields:

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

- `radio_common_init()` - Initialize radio hardware
- `radio_common_configure()` - Apply standard configuration
- `radio_common_set_addresses()` - Set TX/RX addresses
- `radio_common_dump_registers()` - Debug register dump
- `radio_common_is_connected()` - Check hardware connectivity
- `radio_common_validate_config()` - Validate radio settings

### Low-Level Functions

- `nrf24_read_register()` - Read radio register
- `nrf24_write_register()` - Write radio register
- `nrf24_get_status()` - Get status register
- `nrf24_read_payload()` - Read RX payload
- `nrf24_write_payload()` - Write TX payload
- `nrf24_flush_rx()` - Flush RX buffer
- `nrf24_flush_tx()` - Flush TX buffer
- `nrf24_power_up()` - Power up radio
- `nrf24_power_down()` - Power down radio

## Platform Support

### ESP32 (Primary)

Full support with ESP-IDF framework:
- SPI driver integration
- GPIO control
- FreeRTOS delays
- ESP logging system

### Other Platforms

Generic platform support with weak symbols:
- Implement `radio_common_platform_init()`
- Implement `radio_common_platform_transfer()`
- Implement `radio_common_delay_ms/us()`

## Debugging

### Enable Debug Logging

```c
#define RADIO_DEBUG_ENABLED 1
```

### Register Dump

```c
radio_common_dump_registers(&radio);
```

### Connection Test

```c
if (!radio_common_is_connected(&radio)) {
    ESP_LOGE("APP", "Radio not connected");
}
```

## Building

### As Standalone Library

```bash
mkdir build && cd build
cmake ..
make
```

### As ESP-IDF Component

The library automatically detects ESP-IDF environment and registers as a component.

## License

This library is released under the same license as the scoreboard clock project.

## Contributing

When making changes:
1. Maintain backward compatibility
2. Update documentation
3. Test on all supported platforms
4. Follow existing code style