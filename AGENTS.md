# Development Guidelines

This document contains comprehensive development guidelines, coding standards, and contribution workflow for the radio-common library.

## Table of Contents

- [Build System](#build-system)
- [Code Style Guidelines](#code-style-guidelines)
- [Development Workflow](#development-workflow)
- [Testing Guidelines](#testing-guidelines)
- [Platform Support](#platform-support)
- [API Design Principles](#api-design-principles)
- [Debugging & Validation](#debugging--validation)
- [Release Process](#release-process)

## Build System

This module is an ESP-IDF component only: `CMakeLists.txt` is a single
`idf_component_register(...)` call (`SRCS "src/radio_common.c"`, `INCLUDE_DIRS "include"`).
There is **no standalone CMake build** (no plain `cmake .. && make` target), **no
`ENABLE_TESTING`/`ctest` build**, and **no test suite** in this repo — any instructions
implying otherwise are aspirational/planned, not implemented.

### ESP-IDF Integration (only supported build)
Consuming projects add this directory via `EXTRA_COMPONENT_DIRS` (see `repeater/CMakeLists.txt`
for a working example) and `REQUIRES radio-common` in their component's
`idf_component_register()`. ESP-IDF's build system then builds it as a component
automatically — no manual `add_subdirectory`/`target_link_libraries` step.

### Clean Build
```bash
idf.py fullclean
```
(run from the consuming ESP-IDF project; this component has no independent build directory
of its own)

## Code Style Guidelines

### Language & Standards
- **Language**: C (C99 standard)
- **Headers**: Use `#pragma once` for include guards
- **Compiler**: GCC with `-Wall -Wextra -Werror` flags

### Naming Conventions
- **Functions**: `snake_case` (e.g., `radio_common_init`)
- **Variables**: `snake_case` (e.g., `ce_pin`)
- **Constants**: `UPPER_SNAKE_CASE` (e.g., `RADIO_CHANNEL`)
- **Types**: `PascalCase` (e.g., `RadioCommon`)
- **File Names**: `snake_case` (e.g., `radio_common.c`)

### Formatting
- **Indentation**: 4 spaces (no tabs)
- **Line Length**: Maximum 100 characters
- **Braces**: K&R style
- **Spacing**: Around operators, after commas

### Documentation
- **Public Functions**: Doxygen-style comments
- **Complex Logic**: Inline comments explaining the "why"
- **TODO/FIXME**: Use standard tags with owner and date

### Example Function Documentation
```c
/**
 * @brief Initialize the radio hardware with specified pins
 * 
 * @param radio Pointer to RadioCommon structure to initialize
 * @param ce_pin Chip Enable pin number
 * @param csn_pin Chip Select Not pin number
 * @return true if initialization successful, false otherwise
 * 
 * @note This function configures SPI and GPIO pins but does not
 *       configure radio parameters. Use radio_common_configure()
 *       after successful initialization.
 */
bool radio_common_init(RadioCommon* radio, gpio_num_t ce_pin, gpio_num_t csn_pin);
```

## Development Workflow

### Setting Up Development Environment

1. **Clone Repository**
   ```bash
   git clone https://github.com/nevalions/scoreboard-clock-radio-common-nrf24.git
   cd radio-common
   ```

2. **Install Dependencies**
   ```bash
   # ESP-IDF is required — there is no non-ESP-IDF build path
   # Follow the ESP-IDF installation guide
   ```

3. **Initial Build**

   Build via a consuming ESP-IDF project (e.g. `repeater/`), which points
   `EXTRA_COMPONENT_DIRS` at this directory:
   ```bash
   cd ../repeater   # or another module that depends on radio-common
   idf.py build
   ```

### Making Changes

1. **Create Feature Branch**
   ```bash
   git checkout -b feature/your-feature-name
   ```

2. **Make Changes**
   - Follow code style guidelines
   - Add tests for new functionality
   - Update documentation

3. **Test Changes**

   No automated test suite exists (see Testing Guidelines). Verify by building a
   consuming ESP-IDF module and checking behavior on real hardware:
   ```bash
   cd ../repeater   # or another module that depends on radio-common
   idf.py build flash monitor
   ```

4. **Commit Changes**
   ```bash
   git add .
   git commit -m "feat: add new feature description"
   ```

### Commit Message Format

Use conventional commits format:
- `feat:` New features
- `fix:` Bug fixes
- `docs:` Documentation changes
- `style:` Code style changes (formatting, etc.)
- `refactor:` Code refactoring
- `test:` Adding or updating tests
- `chore:` Maintenance tasks

## Testing Guidelines (planned / not implemented)

There is currently **no automated test suite** for this module — no Unity tests, no `ctest`
target, nothing under a `test/` directory. The guidelines below describe an aspirational
testing approach for future work, not anything that exists in the repo today.

### Unit Tests (not implemented)
- Test all public functions
- Test error conditions and edge cases
- Use consistent test naming: `test_function_name_scenario`

### Integration Tests (not implemented)
- Test with actual hardware when possible
- Validate radio communication between devices
- Test platform-specific functionality

### Validation Checklist
- [ ] Code compiles without warnings
- [ ] Documentation is updated
- [ ] Hardware validation (if applicable)

### Manual Verification (current practice)
In the absence of automated tests, changes are currently verified by building the consuming
ESP-IDF module (e.g. `repeater`, `controller`) and checking behavior via serial monitor /
`radio_common_dump_registers()` against real hardware.

## Platform Support

### ESP32 (Primary Platform)
- **Framework**: ESP-IDF
- **SPI Driver**: ESP-IDF SPI driver
- **GPIO**: ESP-IDF GPIO driver
- **Logging**: ESP_LOG macros
- **Delays**: FreeRTOS delays

### Generic Platform Support (unused/untested)
`src/radio_common.c` defines weak-symbol default implementations for the non-ESP32 path, but
nothing in this repo currently overrides or exercises them — they exist for future
portability only. The actual weak symbols, matching `include/radio_common.h`, are:

```c
__attribute__((weak)) bool radio_common_platform_init(RadioCommon *radio) {
    // Default implementation - should be overridden
    return false;
}

__attribute__((weak)) uint8_t radio_common_platform_transfer(RadioCommon *radio,
                                                              uint8_t data) {
    // Default implementation - should be overridden; single byte in, single byte out
    return 0xFF;
}

__attribute__((weak)) void radio_common_delay_ms(uint32_t ms) { /* ... */ }
__attribute__((weak)) void radio_common_delay_us(uint32_t us) { /* ... */ }
```

### Adding New Platform Support
1. Implement required weak symbols
2. Add platform-specific tests (no test infrastructure exists yet — see Testing Guidelines)
3. Update documentation
4. Add build configuration options

## API Design Principles

### Function Design
- **Single Responsibility**: Each function does one thing well
- **Consistent Return Types**: Use `bool` for success/failure
- **Error Handling**: Return false on error, log details
- **Parameter Validation**: Check pointers and ranges

### Structure Design
- **Opaque Pointers**: Hide implementation details when possible
- **Extensibility**: Allow for future expansion without breaking API
- **Memory Management**: Clear ownership and lifecycle

### Example API Design
```c
// Good: Clear parameters, consistent return type
bool radio_common_send_packet(RadioCommon* radio, const uint8_t* data, size_t length);

// Avoid: Multiple responsibilities, unclear parameters
bool radio_common_operation(RadioCommon* radio, int mode, void* data, int flags);
```

## Debugging & Validation

### Debug Logging
```c
// Enable debug logging
#define RADIO_DEBUG_ENABLED 1

// Use appropriate log levels
ESP_LOGE(TAG, "Critical error: %s", error_message);
ESP_LOGW(TAG, "Warning: %s", warning_message);
ESP_LOGI(TAG, "Info: %s", info_message);
ESP_LOGD(TAG, "Debug: %s", debug_message);
```

### Register Dump
```c
void radio_common_dump_registers(const RadioCommon* radio) {
    ESP_LOGI(TAG, "Radio Register Dump:");
    for (int i = 0; i < 0x1F; i++) {
        uint8_t value = nrf24_read_register(radio, i);
        ESP_LOGI(TAG, "Reg 0x%02X: 0x%02X", i, value);
    }
}
```

### Validation Functions
The real `radio_common_validate_config()` (in `src/radio_common.c`) checks TX/RX addresses
are non-zero, the configured channel is ≤125, and the address width register holds a valid
value (0x01-0x03, i.e. 3-5 bytes) — it does not check power state or data rate:

```c
bool radio_common_validate_config(RadioCommon *radio) {
    if (!radio || !radio->initialized) {
        ESP_LOGE(TAG, "Radio not initialized");
        return false;
    }

    if (!RADIO_ADDRESS_IS_VALID(radio->tx_address)) {
        ESP_LOGE(TAG, "Invalid TX address");
        return false;
    }
    if (!RADIO_ADDRESS_IS_VALID(radio->rx_address)) {
        ESP_LOGE(TAG, "Invalid RX address");
        return false;
    }

    uint8_t channel = nrf24_read_register(radio, NRF24_REG_RF_CH);
    if (channel > 125) {
        ESP_LOGE(TAG, "Invalid channel: %d", channel);
        return false;
    }

    uint8_t addr_width = nrf24_read_register(radio, NRF24_REG_SETUP_AW);
    if (addr_width < 0x01 || addr_width > 0x03) {
        ESP_LOGE(TAG, "Invalid address width: %d", addr_width + 2);
        return false;
    }

    return true;
}
```

## Release Process

### Version Management
- Use semantic versioning (MAJOR.MINOR.PATCH)
- Update version in CMakeLists.txt and header files
- Tag releases in Git

### Pre-Release Checklist
- [ ] All tests pass
- [ ] Documentation is updated
- [ ] CHANGELOG is updated
- [ ] Version numbers are updated
- [ ] Build artifacts are created

### Release Steps
1. Update version numbers
2. Update CHANGELOG
3. Create Git tag
4. Create GitHub release
5. Update documentation website (if applicable)

### Breaking Changes
- Increment MAJOR version
- Document breaking changes in CHANGELOG
- Provide migration guide
- Consider deprecation warnings

## Contributing

### Before Contributing
1. Read this document thoroughly
2. Set up development environment
3. Review existing code and tests
4. Discuss significant changes in issues first

### Documentation Guidelines
- **README.md**: Contains user-facing documentation and quick start guides
- **AGENTS.md**: Contains development guidelines and internal documentation
- **Important**: Do not reference or add AGENTS.md to README.md - this is an internal development document

### Pull Request Process
1. Fork repository
2. Create feature branch
3. Make changes with tests
4. Ensure all tests pass
5. Submit pull request with clear description
6. Respond to code review feedback

### Code Review Guidelines
- Review for functionality and style
- Check for proper error handling
- Verify test coverage
- Ensure documentation is updated

## Performance Considerations

### SPI Communication
- Minimize SPI transactions
- Use appropriate clock speeds (1MHz default)
- Batch register operations when possible

### Memory Usage
- Avoid dynamic allocation in critical paths
- Use stack allocation for small structures
- Consider memory constraints on embedded targets

### Timing
- Respect radio timing constraints
- Use appropriate delays for state changes
- Consider interrupt latency in time-critical operations