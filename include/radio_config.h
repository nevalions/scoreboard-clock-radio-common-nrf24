#pragma once

#include <stdbool.h>
#include <stdint.h>

// Use ESP-IDF types when available
#if defined(ESP_IDF_VER) || defined(__ESP32__) ||                              \
    defined(ESP_PLATFORM) // || defined(CONFIG_IDF_TARGET_ESP32)
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
typedef enum { SPI1_HOST = 0, SPI2_HOST = 1, SPI3_HOST = 2 } spi_host_device_t;
#endif

// Logging macros (will be overridden by ESP-IDF when available)
#if defined(ESP_IDF_VER) || defined(__ESP32__) || defined(ESP_PLATFORM)
// ESP-IDF environment - use its logging macros
#include "esp_log.h"
#else
// Non-ESP-IDF environment - define our own logging macros
#ifndef ESP_LOGD
#define ESP_LOGD(tag, format, ...)                                             \
  printf("[D] %s: " format "\n", tag, ##__VA_ARGS__)
#endif
#ifndef ESP_LOGI
#define ESP_LOGI(tag, format, ...)                                             \
  printf("[I] %s: " format "\n", tag, ##__VA_ARGS__)
#endif
#ifndef ESP_LOGE
#define ESP_LOGE(tag, format, ...)                                             \
  printf("[E] %s: " format "\n", tag, ##__VA_ARGS__)
#endif
#ifndef ESP_LOGW
#define ESP_LOGW(tag, format, ...)                                             \
  printf("[W] %s: " format "\n", tag, ##__VA_ARGS__)
#endif
#endif

// =============================================================================
// NETWORK CONFIGURATION
// =============================================================================

// Radio network settings
// Channel 76 = 2476 MHz: above WiFi channel 11's occupied band (tops out at
// 2473 MHz) and below the BLE advertising channel at 2480 MHz. The previous
// channel 20 (2420 MHz) sat inside WiFi channel 1's occupied bandwidth.
#define RADIO_CHANNEL 76
#define RADIO_ADDRESS {0xE7, 0xE7, 0xE7, 0xE7, 0xE7}
#define RADIO_PAYLOAD_SIZE 6

// Channel candidates for venue agility. The controller surveys these (RPD)
// and picks the quietest; receivers hop through the list until they hear
// frames. First entry must equal RADIO_CHANNEL (the boot default).
// Chosen to dodge the WiFi 1/6/11 lobes and the BLE advertising channels
// (2402/2426/2480 MHz): 76/82/78/74 above WiFi ch 11, 49 in the 6-11 gap,
// 24 in the 1-6 gap
#define RADIO_CHANNEL_CANDIDATES {76, 82, 78, 74, 49, 24}
#define RADIO_CHANNEL_CANDIDATE_COUNT 6

// RPD samples per channel in an occupancy survey (1ms apart: ~40ms/channel,
// ~250ms for the full candidate list)
#define RADIO_SURVEY_SAMPLES 40

// Time field encoding (16-bit big-endian in payload bytes 0-1):
//   0-99    whole seconds
//   255     null/clear sentinel (display off)
//   256+d   final-countdown deciseconds, d = 0-49 (4.9 s ... 0.1 s), shown
//           by displays as two digits without a decimal point ("49" = 4.9 s;
//           the sub-5 s warning color and countdown context disambiguate)
#define RADIO_TIME_NULL 255
#define RADIO_TIME_DECISECONDS_BASE 256
#define RADIO_TIME_DECISECONDS_MAX 49
#define RADIO_TIME_IS_DECISECONDS(v)                                          \
  ((v) >= RADIO_TIME_DECISECONDS_BASE &&                                      \
   (v) <= RADIO_TIME_DECISECONDS_BASE + RADIO_TIME_DECISECONDS_MAX)
#define RADIO_TIME_DECISECONDS(v)                                             \
  ((uint16_t)((v) - RADIO_TIME_DECISECONDS_BASE))

// Bit 15 of the time field flags "warn at 10s": set by the controller for
// sports whose rules call for a 10-second buzzer (football). Receivers
// strip the flag before interpreting the value; the null sentinel is sent
// without it
#define RADIO_TIME_FLAG_WARN10 0x8000
#define RADIO_TIME_VALUE_MASK 0x7FFF

// RF_SETUP register values: one combined write, not two settings.
// Bits: RF_DR_LOW=1, RF_DR_HIGH=0 (250 kbps), RF_PWR=11 (0 dBm, max power)
#define RADIO_RF_SETUP_250KBPS_0DBM 0x26
// Bits: RF_DR_LOW=0, RF_DR_HIGH=0 (1 Mbps), RF_PWR=11 (0 dBm, max power)
#define RADIO_RF_SETUP_1MBPS_0DBM 0x06

// Active RF_SETUP: 250 kbps buys ~9 dB of receiver sensitivity (-94 vs
// -85 dBm) — the cheapest range margin available. Counterfeit nRF24 clones
// sometimes fail at 250 kbps: if the bench link is dead after flashing,
// point this alias back at RADIO_RF_SETUP_1MBPS_0DBM.
#define RADIO_RF_SETUP RADIO_RF_SETUP_250KBPS_0DBM

// =============================================================================
// HARDWARE CONFIGURATION
// =============================================================================

// nRF24L01+ CE/CSN pins are application-specific: each module defines its
// own and passes them to radio_common_init()

// ESP32 SPI pins
#define RADIO_SPI_HOST SPI3_HOST
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
#define NRF24_RF_SETUP_RF_DR_LOW 0x20
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
#define RADIO_CONFIG_RX_MODE                                                   \
  (RADIO_CONFIG_CRC_ENABLED | NRF24_CONFIG_PWR_UP | NRF24_CONFIG_PRIM_RX)

// Auto retransmission disabled: this is a broadcast protocol — multiple
// receivers share one address, so ACKs would collide and retries buy nothing
#define RADIO_SETUP_RETR 0x00

// Address width (5 bytes)
#define RADIO_SETUP_AW 0x03

// Auto-ACK disabled on all pipes (broadcast; see RADIO_SETUP_RETR note)
#define RADIO_EN_AA_NONE 0x00

// Enable pipe 0
#define RADIO_EN_RXADDR_PIPE0 0x01

// Clear all status flags
#define RADIO_STATUS_CLEAR_ALL                                                 \
  (NRF24_STATUS_RX_DR | NRF24_STATUS_TX_DS | NRF24_STATUS_MAX_RT)

// =============================================================================
// COMMON MACROS
// =============================================================================

// Convert frequency to channel
#define RADIO_FREQ_TO_CHANNEL(freq) ((uint8_t)(((freq) - 2400.0) / 1.0))

// Convert channel to frequency
#define RADIO_CHANNEL_TO_FREQ(ch) (2400.0 + (ch))

// Check if address is valid (5 bytes, not all zeros)
#define RADIO_ADDRESS_IS_VALID(addr)                                           \
  ((addr)[0] != 0x00 || (addr)[1] != 0x00 || (addr)[2] != 0x00 ||              \
   (addr)[3] != 0x00 || (addr)[4] != 0x00)

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
