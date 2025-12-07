#pragma once

#if defined(ESP_PLATFORM) || defined(ESP32) || defined(CONFIG_IDF_TARGET_ESP32)
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#endif

#include "radio_config.h"

// =============================================================================
// COMMON RADIO STRUCTURES
// =============================================================================

// Basic radio communication structure
typedef struct {
  bool initialized;
  gpio_num_t ce_pin;
  gpio_num_t csn_pin;

  // Radio configuration
  uint8_t tx_address[5];
  uint8_t rx_address[5];

  // SPI device handle (always included for ESP32)
  void *spi;
} RadioCommon;

// =============================================================================
// LOW-LEVEL RADIO FUNCTIONS
// =============================================================================

// Basic register operations
uint8_t nrf24_read_register(RadioCommon *radio, uint8_t reg);
bool nrf24_write_register(RadioCommon *radio, uint8_t reg, uint8_t value);
uint8_t nrf24_get_status(RadioCommon *radio);

// Power management
void nrf24_power_up(RadioCommon *radio);
void nrf24_power_down(RadioCommon *radio);

// Payload operations
bool nrf24_read_payload(RadioCommon *radio, uint8_t *data, uint8_t length);
bool nrf24_write_payload(RadioCommon *radio, uint8_t *data, uint8_t length);
bool nrf24_write_ack_payload(RadioCommon *radio, uint8_t pipe,
                             const uint8_t *data, uint8_t length);

// Buffer management
void nrf24_flush_rx(RadioCommon *radio);
void nrf24_flush_tx(RadioCommon *radio);

// =============================================================================
// COMMON RADIO INITIALIZATION
// =============================================================================

// Initialize radio with common settings
bool radio_common_init(RadioCommon *radio, gpio_num_t ce_pin,
                       gpio_num_t csn_pin);

// Configure radio with standard settings
bool radio_common_configure(RadioCommon *radio);

// Set radio addresses
bool radio_common_set_addresses(RadioCommon *radio, const uint8_t *tx_addr,
                                const uint8_t *rx_addr);

// =============================================================================
// UTILITY FUNCTIONS
// =============================================================================

// Debug function to dump all registers
void radio_common_dump_registers(RadioCommon *radio);

// Check if radio responds to basic communication
bool radio_common_is_connected(RadioCommon *radio);

// Validate radio configuration
bool radio_common_validate_config(RadioCommon *radio);

// =============================================================================
// PLATFORM-SPECIFIC FUNCTIONS
// =============================================================================

#if defined(ESP_PLATFORM) || defined(ESP32) || defined(CONFIG_IDF_TARGET_ESP32)
// ESP32 specific SPI functions
bool radio_common_spi_init(RadioCommon *radio);
uint8_t radio_common_spi_transfer(RadioCommon *radio, uint8_t data);
#else
// Generic platform functions (to be implemented by user)
bool radio_common_platform_init(RadioCommon *radio);
uint8_t radio_common_platform_transfer(RadioCommon *radio, uint8_t data);
void radio_common_delay_ms(uint32_t ms);
void radio_common_delay_us(uint32_t us);
#endif
