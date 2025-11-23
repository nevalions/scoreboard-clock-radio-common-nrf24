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

### Test Build
```bash
mkdir build && cd build
cmake -DENABLE_TESTING=ON ..
make
ctest
```

### Clean Build
```bash
rm -rf build
```

### ESP-IDF Integration
The library automatically detects ESP-IDF environment and registers as a component. No additional configuration needed.

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
   # Ubuntu/Debian
   sudo apt-get install cmake build-essential
   
   # ESP-IDF (if developing for ESP32)
   # Follow ESP-IDF installation guide
   ```

3. **Initial Build**
   ```bash
   mkdir build && cd build
   cmake ..
   make
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
   ```bash
   make
   ctest  # if tests are available
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

## Testing Guidelines

### Unit Tests
- Test all public functions
- Test error conditions and edge cases
- Use consistent test naming: `test_function_name_scenario`

### Integration Tests
- Test with actual hardware when possible
- Validate radio communication between devices
- Test platform-specific functionality

### Validation Checklist
- [ ] Code compiles without warnings
- [ ] All tests pass
- [ ] Static analysis passes (if configured)
- [ ] Documentation is updated
- [ ] Hardware validation (if applicable)

### Test Structure
```c
void test_radio_common_init_success(void) {
    RadioCommon radio;
    
    // Test successful initialization
    TEST_ASSERT_TRUE(radio_common_init(&radio, GPIO_NUM_5, GPIO_NUM_4));
    TEST_ASSERT_EQUAL(GPIO_NUM_5, radio.ce_pin);
    TEST_ASSERT_EQUAL(GPIO_NUM_4, radio.csn_pin);
}

void test_radio_common_init_null_pointer(void) {
    // Test null pointer handling
    TEST_ASSERT_FALSE(radio_common_init(NULL, GPIO_NUM_5, GPIO_NUM_4));
}
```

## Platform Support

### ESP32 (Primary Platform)
- **Framework**: ESP-IDF
- **SPI Driver**: ESP-IDF SPI driver
- **GPIO**: ESP-IDF GPIO driver
- **Logging**: ESP_LOG macros
- **Delays**: FreeRTOS delays

### Generic Platform Support
Implement weak symbols for platform abstraction:

```c
__attribute__((weak)) bool radio_common_platform_init(RadioCommon* radio) {
    // Default implementation - should be overridden
    return false;
}

__attribute__((weak)) bool radio_common_platform_transfer(
    RadioCommon* radio, const uint8_t* tx_data, uint8_t* rx_data, size_t length) {
    // Default implementation - should be overridden
    return false;
}
```

### Adding New Platform Support
1. Implement required weak symbols
2. Add platform-specific tests
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
```c
bool radio_common_validate_config(const RadioCommon* radio) {
    // Validate critical settings
    uint8_t config = nrf24_read_register(radio, NRF24_REG_CONFIG);
    uint8_t rf_setup = nrf24_read_register(radio, NRF24_REG_RF_SETUP);
    
    // Check power state
    if (!(config & NRF24_CONFIG_PWR_UP)) {
        ESP_LOGE(TAG, "Radio not powered up");
        return false;
    }
    
    // Check data rate
    uint8_t data_rate = rf_setup & NRF24_RF_SETUP_RF_DR;
    if (data_rate != RADIO_DATA_RATE_1MBPS) {
        ESP_LOGE(TAG, "Invalid data rate: 0x%02X", data_rate);
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