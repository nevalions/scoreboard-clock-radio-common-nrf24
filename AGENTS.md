# AGENTS.md

**Purpose**: This library synchronizes radio settings across all scoreboard timer submodules. It is designed as a Git submodule used by both the parent project and individual modules that require radio communication.

## Build Commands
- **Build**: `mkdir build && cd build && cmake .. && make`
- **Debug Build**: `cmake -DCMAKE_BUILD_TYPE=Debug .. && make`
- **Clean**: `rm -rf build`

## Code Style Guidelines
- **Language**: C (C99 standard)
- **Indentation**: 4 spaces (no tabs)
- **Naming**: 
  - Functions: `snake_case` (e.g., `radio_common_init`)
  - Variables: `snake_case` (e.g., `ce_pin`)
  - Constants: `UPPER_SNAKE_CASE` (e.g., `RADIO_CHANNEL`)
  - Types: `PascalCase` (e.g., `RadioCommon`)
- **Headers**: Use `#pragma once` for include guards
- **Platform Support**: ESP32 primary, generic platform via weak symbols
- **Error Handling**: Return `bool` for success/failure, use ESP_LOGE for errors
- **Logging**: Use ESP_LOGI/INFO, ESP_LOGE/ERROR, ESP_LOGD/DEBUG macros
- **Memory**: Initialize structs with memset, validate pointers before use
- **SPI**: Manual CSN control, 1MHz clock speed
- **GPIO**: Use gpio_num_t enum for pin definitions