#pragma once

#include <stdbool.h>
#include <stdint.h>

// Use ESP-IDF types when available
#if defined(ESP_IDF_VER) || defined(__ESP32__) || defined(ESP_PLATFORM) // || defined(CONFIG_IDF_TARGET_ESP32)
#include "driver/gpio.h"
#include "driver/spi_master.h"
#else
// GPIO pin definitions (compatible with ESP32)
typedef enum {
    GPIO_NUM_0 = 0,
    GPIO_NUM_1 = 1,
    GPIO_NUM_2 = 2,
    GPIO_NUM_3 = 3,
    GPIO_NUM_4 = 4,
    GPIO_NUM_5 = 5,
    GPIO_NUM_18 = 18,
    GPIO_NUM_19 = 19,
    GPIO_NUM_23 = 23
} gpio_num_t;

// SPI host definitions
typedef enum {
    SPI1_HOST = 0,
    SPI2_HOST = 1,
    SPI3_HOST = 2
} spi_host_device_t;
#endif

// Logging macros (will be overridden by ESP-IDF when available)
#ifndef ESP_LOGD
#define ESP_LOGD(tag, format, ...) printf("[D] %s: " format "\n", tag, ##__VA_ARGS__)
#endif
#ifndef ESP_LOGI
#define ESP_LOGI(tag, format, ...) printf("[I] %s: " format "\n", tag, ##__VA_ARGS__)
#endif
#ifndef ESP_LOGE
#define ESP_LOGE(tag, format, ...) printf("[E] %s: " format "\n", tag, ##__VA_ARGS__)
#endif
#ifndef ESP_LOGW
#define ESP_LOGW(tag, format, ...) printf("[W] %s: " format "\n", tag, ##__VA_ARGS__)
#endif

// =============================================================================
// NETWORK CONFIGURATION
// =============================================================================

// Radio network settings
#define RADIO_CHANNEL 20
#define RADIO_ADDRESS {0xE7, 0xE7, 0xE7, 0xE7, 0xE7}
#define RADIO_PAYLOAD_SIZE 6

// Radio performance settings
#define RADIO_DATA_RATE_1MBPS 0x06  // RF_SETUP value for 1Mbps
#define RADIO_POWER_0DBM 0x06       // RF_SETUP value for 0dBm

// =============================================================================
// HARDWARE CONFIGURATION
// =============================================================================

// ESP32 GPIO pins for nRF24L01+
#define RADIO_CE_PIN GPIO_NUM_5
#define RADIO_CSN_PIN GPIO_NUM_4

// ESP32 SPI pins
#define RADIO_SPI_HOST SPI2_HOST
#define RADIO_MOSI_PIN GPIO_NUM_23
#define RADIO_MISO_PIN GPIO_NUM_19
#define RADIO_SCK_PIN GPIO_NUM_18

// Status LED (common across modules)
#define RADIO_STATUS_LED_PIN GPIO_NUM_2

// =============================================================================
// nRF24L01+ COMMANDS
// =============================================================================

#define NRF24_CMD_R_REGISTER 0x00
#define NRF24_CMD_W_REGISTER 0x20
#define NRF24_CMD_RX_PAYLOAD 0x61
#define NRF24_CMD_TX_PAYLOAD 0xA0
#define NRF24_CMD_FLUSH_TX 0xE1
#define NRF24_CMD_FLUSH_RX 0xE2
#define NRF24_CMD_REUSE_TX_PL 0xE3
#define NRF24_CMD_ACTIVATE 0x50
#define NRF24_CMD_R_RX_PL_WID 0x60
#define NRF24_CMD_W_TX_PAYLOAD_NOACK 0xB0
#define NRF24_CMD_W_ACK_PAYLOAD 0xA8
#define NRF24_CMD_NOP 0xFF

// =============================================================================
// nRF24L01+ REGISTERS
// =============================================================================

#define NRF24_REG_CONFIG 0x00
#define NRF24_REG_EN_AA 0x01
#define NRF24_REG_EN_RXADDR 0x02
#define NRF24_REG_SETUP_AW 0x03
#define NRF24_REG_SETUP_RETR 0x04
#define NRF24_REG_RF_CH 0x05
#define NRF24_REG_RF_SETUP 0x06
#define NRF24_REG_STATUS 0x07
#define NRF24_REG_OBSERVE_TX 0x08
#define NRF24_REG_CD 0x09
#define NRF24_REG_RX_ADDR_P0 0x0A
#define NRF24_REG_RX_ADDR_P1 0x0B
#define NRF24_REG_RX_ADDR_P2 0x0C
#define NRF24_REG_RX_ADDR_P3 0x0D
#define NRF24_REG_RX_ADDR_P4 0x0E
#define NRF24_REG_RX_ADDR_P5 0x0F
#define NRF24_REG_TX_ADDR 0x10
#define NRF24_REG_RX_PW_P0 0x11
#define NRF24_REG_RX_PW_P1 0x12
#define NRF24_REG_RX_PW_P2 0x13
#define NRF24_REG_RX_PW_P3 0x14
#define NRF24_REG_RX_PW_P4 0x15
#define NRF24_REG_RX_PW_P5 0x16
#define NRF24_REG_FIFO_STATUS 0x17
#define NRF24_REG_DYNPD 0x1C
#define NRF24_REG_FEATURE 0x1D

// =============================================================================
// nRF24L01+ CONFIGURATION BITS
// =============================================================================

// CONFIG register bits
#define NRF24_CONFIG_MASK_RX_DR 0x40
#define NRF24_CONFIG_MASK_TX_DS 0x20
#define NRF24_CONFIG_MASK_MAX_RT 0x10
#define NRF24_CONFIG_EN_CRC 0x08
#define NRF24_CONFIG_CRCO 0x04
#define NRF24_CONFIG_PWR_UP 0x02
#define NRF24_CONFIG_PRIM_RX 0x01

// STATUS register bits
#define NRF24_STATUS_RX_DR 0x40
#define NRF24_STATUS_TX_DS 0x20
#define NRF24_STATUS_MAX_RT 0x10

// RF_SETUP register bits
#define NRF24_RF_SETUP_PLL_LOCK 0x10
#define NRF24_RF_SETUP_RF_DR 0x08
#define NRF24_RF_SETUP_RF_PWR 0x06
#define NRF24_RF_SETUP_LNA_HCURR 0x01

// FIFO_STATUS register bits
#define NRF24_FIFO_STATUS_TX_REUSE 0x40
#define NRF24_FIFO_STATUS_TX_FULL 0x20
#define NRF24_FIFO_STATUS_TX_EMPTY 0x10
#define NRF24_FIFO_STATUS_RX_FULL 0x02
#define NRF24_FIFO_STATUS_RX_EMPTY 0x01

// =============================================================================
// COMMON RADIO CONFIGURATION VALUES
// =============================================================================

// Standard radio configuration
#define RADIO_CONFIG_CRC_ENABLED (NRF24_CONFIG_EN_CRC)
#define RADIO_CONFIG_TX_MODE (RADIO_CONFIG_CRC_ENABLED | NRF24_CONFIG_PWR_UP)
#define RADIO_CONFIG_RX_MODE (RADIO_CONFIG_CRC_ENABLED | NRF24_CONFIG_PWR_UP | NRF24_CONFIG_PRIM_RX)

// Auto retransmission settings (500us delay, up to 15 retries)
#define RADIO_SETUP_RETR 0x4F

// Address width (5 bytes)
#define RADIO_SETUP_AW 0x03

// Enable auto-acknowledgment on pipe 0
#define RADIO_EN_AA_PIPE0 0x01

// Enable pipe 0
#define RADIO_EN_RXADDR_PIPE0 0x01

// Clear all status flags
#define RADIO_STATUS_CLEAR_ALL (NRF24_STATUS_RX_DR | NRF24_STATUS_TX_DS | NRF24_STATUS_MAX_RT)

// =============================================================================
// COMMON MACROS
// =============================================================================

// Convert frequency to channel
#define RADIO_FREQ_TO_CHANNEL(freq) ((uint8_t)(((freq) - 2400.0) / 1.0))

// Convert channel to frequency
#define RADIO_CHANNEL_TO_FREQ(ch) (2400.0 + (ch))

// Check if address is valid (5 bytes, not all zeros)
#define RADIO_ADDRESS_IS_VALID(addr) ((addr)[0] != 0x00 || (addr)[1] != 0x00 || \
                                     (addr)[2] != 0x00 || (addr)[3] != 0x00 || \
                                     (addr)[4] != 0x00)

// =============================================================================
// DEBUG AND LOGGING
// =============================================================================

// Enable/disable debug logging
#ifndef RADIO_DEBUG_ENABLED
#define RADIO_DEBUG_ENABLED 1
#endif

// Debug logging macro
#if RADIO_DEBUG_ENABLED
#define RADIO_DEBUG_LOG(tag, format, ...) ESP_LOGD(tag, format, ##__VA_ARGS__)
#else
#define RADIO_DEBUG_LOG(tag, format, ...)
#endif

// =============================================================================
// ERROR CODES
// =============================================================================

typedef enum {
    RADIO_OK = 0,
    RADIO_ERROR_NOT_INITIALIZED,
    RADIO_ERROR_SPI_FAILED,
    RADIO_ERROR_INVALID_PARAMETER,
    RADIO_ERROR_TIMEOUT,
    RADIO_ERROR_MAX_RETRIES,
    RADIO_ERROR_NOT_CONNECTED
} radio_error_t;