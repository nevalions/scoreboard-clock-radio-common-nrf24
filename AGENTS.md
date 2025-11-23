# Development Guidelines

This document contains development guidelines and build instructions for the radio-common library.

## Build Commands

### Standard Build
```bash
mkdir build && cd build
cmake ..
make
```

### Debug Build
```bash
mkdir build && cd build
cmake -DCMAKE_BUILD_TYPE=Debug ..
make
```

### Clean Build
```bash
rm -rf build
```

## Code Style Guidelines

### Language & Standards
- **Language**: C (C99 standard)
- **Headers**: Use `#pragma once` for include guards

### Naming Conventions
- **Functions**: `snake_case` (e.g., `radio_common_init`)
- **Variables**: `snake_case` (e.g., `ce_pin`)
- **Constants**: `UPPER_SNAKE_CASE` (e.g., `RADIO_CHANNEL`)
- **Types**: `PascalCase` (e.g., `RadioCommon`)

### Formatting
- **Indentation**: 4 spaces (no tabs)

### Platform & Hardware
- **Primary Platform**: ESP32 with ESP-IDF
- **Generic Platform**: Support via weak symbols
- **SPI**: Manual CSN control, 1MHz clock speed
- **GPIO**: Use `gpio_num_t` enum for pin definitions

### Error Handling & Logging
- **Return Values**: Use `bool` for success/failure
- **Error Logging**: Use `ESP_LOGE` for errors
- **Info Logging**: Use `ESP_LOGI` for informational messages
- **Debug Logging**: Use `ESP_LOGD` for debug output

### Memory Management
- **Struct Initialization**: Use `memset` to zero-initialize
- **Pointer Validation**: Check pointers before dereferencing

### Testing & Validation
- Always validate radio configuration after initialization
- Test connectivity with `radio_common_is_connected()`
- Use register dumps for debugging hardware issues