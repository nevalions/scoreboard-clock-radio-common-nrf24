// Radio common includes
#include "../include/radio_common.h"
#include <string.h>
#include <stdio.h>



// Define TAG if ESP32 headers not available
#ifndef TAG
static const char* TAG = "RADIO_COMMON";
#endif

// =============================================================================
// PLATFORM-SPECIFIC IMPLEMENTATIONS
// =============================================================================

#if defined(ESP_PLATFORM) || defined(ESP32) || defined(CONFIG_IDF_TARGET_ESP32)
// ESP32 specific SPI implementation
static uint8_t spi_transfer(RadioCommon* radio, uint8_t data) {
    if (!radio->spi) {
        ESP_LOGE(TAG, "SPI device handle is NULL!");
        return 0xFF;
    }
    
    uint8_t rx_data;
    spi_transaction_t trans = {
        .length = 8,
        .tx_buffer = &data,
        .rx_buffer = &rx_data
    };
    
    esp_err_t ret = spi_device_transmit((spi_device_handle_t)radio->spi, &trans);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "SPI transfer failed: %s", esp_err_to_name(ret));
        return 0xFF;
    }
    
    RADIO_DEBUG_LOG(TAG, "SPI TX: 0x%02X, RX: 0x%02X", data, rx_data);
    return rx_data;
}

static void gpio_write(gpio_num_t pin, bool level) {
    gpio_set_level(pin, level ? 1 : 0);
}

static void delay_ms(uint32_t ms) {
    vTaskDelay(pdMS_TO_TICKS(ms));
}

#else
// Generic platform implementations (weak symbols)
__attribute__((weak)) bool radio_common_platform_init(RadioCommon* radio) {
    ESP_LOGE(TAG, "Platform-specific init not implemented");
    return false;
}

__attribute__((weak)) uint8_t radio_common_platform_transfer(RadioCommon* radio, uint8_t data) {
    ESP_LOGE(TAG, "Platform-specific transfer not implemented");
    return 0xFF;
}

__attribute__((weak)) void radio_common_delay_ms(uint32_t ms) {
    // Generic delay - should be implemented by platform
    for (volatile uint32_t i = 0; i < ms * 1000; i++);
}

__attribute__((weak)) void radio_common_delay_us(uint32_t us) {
    // Generic microsecond delay - should be implemented by platform
    for (volatile uint32_t i = 0; i < us; i++);
}

// Use generic implementations
static uint8_t spi_transfer(RadioCommon* radio, uint8_t data) {
    return radio_common_platform_transfer(radio, data);
}

static void gpio_write(gpio_num_t pin, bool level) {
    // Platform-specific GPIO write should be implemented
    (void)pin;
    (void)level;
}

static void delay_ms(uint32_t ms) {
    radio_common_delay_ms(ms);
}

static void delay_us(uint32_t us) {
    radio_common_delay_us(us);
}
#endif

// =============================================================================
// LOW-LEVEL RADIO FUNCTIONS
// =============================================================================

uint8_t nrf24_read_register(RadioCommon* radio, uint8_t reg) {
    if (!radio || !radio->initialized) {
        return 0xFF;
    }
    
    gpio_write(radio->csn_pin, 0);
    spi_transfer(radio, NRF24_CMD_R_REGISTER | reg);
    uint8_t value = spi_transfer(radio, NRF24_CMD_NOP);
    gpio_write(radio->csn_pin, 1);
    
    RADIO_DEBUG_LOG(TAG, "Read register 0x%02X = 0x%02X", reg, value);
    return value;
}

bool nrf24_write_register(RadioCommon* radio, uint8_t reg, uint8_t value) {
    if (!radio || !radio->initialized) {
        return false;
    }
    
    gpio_write(radio->csn_pin, 0);
    uint8_t status = spi_transfer(radio, NRF24_CMD_W_REGISTER | reg);
    spi_transfer(radio, value);
    gpio_write(radio->csn_pin, 1);
    
    RADIO_DEBUG_LOG(TAG, "Write register 0x%02X = 0x%02X, status = 0x%02X", reg, value, status);
    return true;
}

uint8_t nrf24_get_status(RadioCommon* radio) {
    if (!radio || !radio->initialized) {
        return 0xFF;
    }
    
    gpio_write(radio->csn_pin, 0);
    uint8_t status = spi_transfer(radio, NRF24_CMD_NOP);
    gpio_write(radio->csn_pin, 1);
    
    return status;
}

void nrf24_power_up(RadioCommon* radio) {
    if (!radio || !radio->initialized) return;
    
    uint8_t config = nrf24_read_register(radio, NRF24_REG_CONFIG);
    nrf24_write_register(radio, NRF24_REG_CONFIG, config | NRF24_CONFIG_PWR_UP);
    delay_ms(5); // Power-up delay
}

void nrf24_power_down(RadioCommon* radio) {
    if (!radio || !radio->initialized) return;
    
    uint8_t config = nrf24_read_register(radio, NRF24_REG_CONFIG);
    nrf24_write_register(radio, NRF24_REG_CONFIG, config & ~NRF24_CONFIG_PWR_UP);
}

bool nrf24_read_payload(RadioCommon* radio, uint8_t* data, uint8_t length) {
    if (!radio || !radio->initialized || !data || length == 0) {
        return false;
    }
    
    gpio_write(radio->csn_pin, 0);
    spi_transfer(radio, NRF24_CMD_RX_PAYLOAD);
    for (uint8_t i = 0; i < length; i++) {
        data[i] = spi_transfer(radio, NRF24_CMD_NOP);
    }
    gpio_write(radio->csn_pin, 1);
    
    return true;
}

bool nrf24_write_payload(RadioCommon* radio, uint8_t* data, uint8_t length) {
    if (!radio || !radio->initialized || !data || length == 0) {
        return false;
    }
    
    gpio_write(radio->csn_pin, 0);
    spi_transfer(radio, NRF24_CMD_TX_PAYLOAD);
    for (uint8_t i = 0; i < length; i++) {
        spi_transfer(radio, data[i]);
    }
    gpio_write(radio->csn_pin, 1);
    
    return true;
}

void nrf24_flush_rx(RadioCommon* radio) {
    if (!radio || !radio->initialized) return;
    
    gpio_write(radio->csn_pin, 0);
    spi_transfer(radio, NRF24_CMD_FLUSH_RX);
    gpio_write(radio->csn_pin, 1);
    
    RADIO_DEBUG_LOG(TAG, "RX buffer flushed");
}

void nrf24_flush_tx(RadioCommon* radio) {
    if (!radio || !radio->initialized) return;
    
    gpio_write(radio->csn_pin, 0);
    spi_transfer(radio, NRF24_CMD_FLUSH_TX);
    gpio_write(radio->csn_pin, 1);
    
    RADIO_DEBUG_LOG(TAG, "TX buffer flushed");
}

// =============================================================================
// COMMON RADIO INITIALIZATION
// =============================================================================

#if defined(ESP_PLATFORM) || defined(ESP32) || defined(CONFIG_IDF_TARGET_ESP32)
bool radio_common_spi_init(RadioCommon* radio) {
    if (!radio) return false;
    
    // Configure SPI bus
    spi_bus_config_t bus_cfg = {
        .mosi_io_num = RADIO_MOSI_PIN,
        .miso_io_num = RADIO_MISO_PIN,
        .sclk_io_num = RADIO_SCK_PIN,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1
    };

    spi_device_interface_config_t dev_cfg = {
        .clock_speed_hz = 1000000,  // 1MHz
        .mode = 0,
        .spics_io_num = -1,  // Use manual CSN control
        .queue_size = 7
    };

    ESP_LOGI(TAG, "Initializing SPI bus with MOSI=%d, MISO=%d, SCK=%d", RADIO_MOSI_PIN, RADIO_MISO_PIN, RADIO_SCK_PIN);
    esp_err_t ret = spi_bus_initialize(RADIO_SPI_HOST, &bus_cfg, SPI_DMA_CH_AUTO);
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {  // ESP_ERR_INVALID_STATE means already initialized
        ESP_LOGE(TAG, "SPI bus initialization failed: %s", esp_err_to_name(ret));
        return false;
    }
    ESP_LOGI(TAG, "SPI bus initialized successfully");
    
    ret = spi_bus_add_device(RADIO_SPI_HOST, &dev_cfg, (spi_device_handle_t*)&radio->spi);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "SPI device addition failed: %s", esp_err_to_name(ret));
        return false;
    }
    ESP_LOGI(TAG, "SPI device added successfully, handle=%p", radio->spi);
    
    ESP_LOGI(TAG, "SPI initialized successfully");
    return true;
}
#endif

bool radio_common_init(RadioCommon* radio, gpio_num_t ce_pin, gpio_num_t csn_pin) {
    if (!radio) {
        ESP_LOGE(TAG, "Invalid radio pointer");
        return false;
    }
    
    // Initialize structure
    memset(radio, 0, sizeof(RadioCommon));
    radio->ce_pin = ce_pin;
    radio->csn_pin = csn_pin;
    
    // Set default addresses
    uint8_t default_addr[] = RADIO_ADDRESS;
    memcpy(radio->tx_address, default_addr, 5);
    memcpy(radio->rx_address, default_addr, 5);
    
#if defined(ESP_PLATFORM) || defined(ESP32) || defined(CONFIG_IDF_TARGET_ESP32)
    // Initialize GPIO pins
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << ce_pin) | (1ULL << csn_pin),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&io_conf);
    
    // Initialize SPI
    ESP_LOGI(TAG, "About to call radio_common_spi_init");
    if (!radio_common_spi_init(radio)) {
        ESP_LOGE(TAG, "SPI initialization failed");
        return false;
    }
    ESP_LOGI(TAG, "radio_common_spi_init completed");
#else
    // Use platform-specific initialization
    ESP_LOGI(TAG, "About to call radio_common_platform_init");
    if (!radio_common_platform_init(radio)) {
        ESP_LOGE(TAG, "Platform initialization failed");
        return false;
    }
    ESP_LOGI(TAG, "radio_common_platform_init completed");
#endif
    
    // Set initial pin states
    gpio_write(ce_pin, 0);
    gpio_write(csn_pin, 1);
    
    // Test basic communication
    uint8_t status = nrf24_get_status(radio);
    ESP_LOGI(TAG, "Initial radio status: 0x%02X", status);
    
    radio->initialized = true;
    ESP_LOGI(TAG, "Radio initialized successfully");
    return true;
}

bool radio_common_configure(RadioCommon* radio) {
    if (!radio || !radio->initialized) {
        ESP_LOGE(TAG, "Radio not initialized");
        return false;
    }
    
    // Power down first
    nrf24_write_register(radio, NRF24_REG_CONFIG, 0);
    delay_ms(10);
    
    // Configure basic settings
    if (!nrf24_write_register(radio, NRF24_REG_RF_CH, RADIO_CHANNEL)) {
        ESP_LOGE(TAG, "Failed to set channel");
        return false;
    }
    
    if (!nrf24_write_register(radio, NRF24_REG_RF_SETUP, RADIO_DATA_RATE_1MBPS)) {
        ESP_LOGE(TAG, "Failed to set RF setup");
        return false;
    }
    
    if (!nrf24_write_register(radio, NRF24_REG_SETUP_AW, RADIO_SETUP_AW)) {
        ESP_LOGE(TAG, "Failed to set address width");
        return false;
    }
    
    if (!nrf24_write_register(radio, NRF24_REG_SETUP_RETR, RADIO_SETUP_RETR)) {
        ESP_LOGE(TAG, "Failed to set retransmission");
        return false;
    }
    
    if (!nrf24_write_register(radio, NRF24_REG_EN_AA, RADIO_EN_AA_PIPE0)) {
        ESP_LOGE(TAG, "Failed to enable auto-ACK");
        return false;
    }
    
    if (!nrf24_write_register(radio, NRF24_REG_EN_RXADDR, RADIO_EN_RXADDR_PIPE0)) {
        ESP_LOGE(TAG, "Failed to enable RX pipe");
        return false;
    }
    
    if (!nrf24_write_register(radio, NRF24_REG_RX_PW_P0, RADIO_PAYLOAD_SIZE)) {
        ESP_LOGE(TAG, "Failed to set payload size");
        return false;
    }
    
    // Set addresses
    if (!radio_common_set_addresses(radio, radio->tx_address, radio->rx_address)) {
        ESP_LOGE(TAG, "Failed to set addresses");
        return false;
    }
    
    // Clear status flags
    nrf24_write_register(radio, NRF24_REG_STATUS, RADIO_STATUS_CLEAR_ALL);
    
    // Flush buffers
    nrf24_flush_rx(radio);
    nrf24_flush_tx(radio);
    
    // Power up the radio
    nrf24_power_up(radio);
    
    ESP_LOGI(TAG, "Radio configured successfully");
    return true;
}

bool radio_common_set_addresses(RadioCommon* radio, const uint8_t* tx_addr, const uint8_t* rx_addr) {
    if (!radio || !radio->initialized || !tx_addr || !rx_addr) {
        return false;
    }
    
    // Store addresses
    memcpy(radio->tx_address, tx_addr, 5);
    memcpy(radio->rx_address, rx_addr, 5);
    
    // Set TX address
    gpio_write(radio->csn_pin, 0);
    spi_transfer(radio, NRF24_CMD_W_REGISTER | NRF24_REG_TX_ADDR);
    for (int i = 0; i < 5; i++) {
        spi_transfer(radio, tx_addr[i]);
    }
    gpio_write(radio->csn_pin, 1);
    
    // Set RX address P0 (required for auto-ACK)
    gpio_write(radio->csn_pin, 0);
    spi_transfer(radio, NRF24_CMD_W_REGISTER | NRF24_REG_RX_ADDR_P0);
    for (int i = 0; i < 5; i++) {
        spi_transfer(radio, rx_addr[i]);
    }
    gpio_write(radio->csn_pin, 1);
    
    ESP_LOGI(TAG, "Addresses set successfully");
    return true;
}

// =============================================================================
// UTILITY FUNCTIONS
// =============================================================================

void radio_common_dump_registers(RadioCommon* radio) {
    if (!radio || !radio->initialized) {
        ESP_LOGE(TAG, "Radio not initialized");
        return;
    }
    
    ESP_LOGI(TAG, "=== nRF24L01+ Register Dump ===");
    ESP_LOGI(TAG, "SPI handle: %p", radio->spi);
    
    // Read and display key registers
    uint8_t config = nrf24_read_register(radio, NRF24_REG_CONFIG);
    uint8_t status = nrf24_get_status(radio);
    uint8_t rf_setup = nrf24_read_register(radio, NRF24_REG_RF_SETUP);
    uint8_t rf_ch = nrf24_read_register(radio, NRF24_REG_RF_CH);
    uint8_t setup_aw = nrf24_read_register(radio, NRF24_REG_SETUP_AW);
    uint8_t setup_retr = nrf24_read_register(radio, NRF24_REG_SETUP_RETR);
    uint8_t en_aa = nrf24_read_register(radio, NRF24_REG_EN_AA);
    uint8_t en_rxaddr = nrf24_read_register(radio, NRF24_REG_EN_RXADDR);
    uint8_t fifo_status = nrf24_read_register(radio, NRF24_REG_FIFO_STATUS);
    
    ESP_LOGI(TAG, "CONFIG:    0x%02X (PWR_UP:%d, PRIM_RX:%d, CRC:%d)", 
             config, 
             (config & NRF24_CONFIG_PWR_UP) ? 1 : 0,
             (config & NRF24_CONFIG_PRIM_RX) ? 1 : 0,
             (config & NRF24_CONFIG_EN_CRC) ? 1 : 0);
    
    ESP_LOGI(TAG, "STATUS:    0x%02X (TX_DS:%d, RX_DR:%d, MAX_RT:%d)", 
             status,
             (status & NRF24_STATUS_TX_DS) ? 1 : 0,
             (status & NRF24_STATUS_RX_DR) ? 1 : 0,
             (status & NRF24_STATUS_MAX_RT) ? 1 : 0);
    
    ESP_LOGI(TAG, "RF_SETUP:  0x%02X (RF_PWR:%d, RF_DR:%d)", 
             rf_setup,
             (rf_setup & NRF24_RF_SETUP_RF_PWR) >> 1,
             (rf_setup & NRF24_RF_SETUP_RF_DR) ? 2 : 1);
    
    ESP_LOGI(TAG, "RF_CH:     0x%02X (Channel: %d, Freq: %.3f GHz)", 
             rf_ch, rf_ch, RADIO_CHANNEL_TO_FREQ(rf_ch));
    
    ESP_LOGI(TAG, "SETUP_AW:  0x%02X (Addr width: %d bytes)", setup_aw, setup_aw + 2);
    ESP_LOGI(TAG, "SETUP_RETR: 0x%02X (ARD:%d us, ARC:%d)", 
             setup_retr, 
             ((setup_retr & 0xF0) >> 4) * 250 + 250,
             setup_retr & 0x0F);
    ESP_LOGI(TAG, "EN_AA:     0x%02X", en_aa);
    ESP_LOGI(TAG, "EN_RXADDR: 0x%02X", en_rxaddr);
    ESP_LOGI(TAG, "FIFO_STATUS: 0x%02X", fifo_status);
    
    // Read addresses
    uint8_t tx_addr[5];
    gpio_write(radio->csn_pin, 0);
    spi_transfer(radio, NRF24_CMD_R_REGISTER | NRF24_REG_TX_ADDR);
    for (int i = 0; i < 5; i++) {
        tx_addr[i] = spi_transfer(radio, NRF24_CMD_NOP);
    }
    gpio_write(radio->csn_pin, 1);
    
    ESP_LOGI(TAG, "TX_ADDR:   %02X:%02X:%02X:%02X:%02X", 
             tx_addr[0], tx_addr[1], tx_addr[2], tx_addr[3], tx_addr[4]);
    
    // Check if module responds (basic connectivity test)
    if (config == 0x00 || config == 0xFF) {
        ESP_LOGE(TAG, "Radio module may not be connected (CONFIG: 0x%02X)", config);
    } else {
        ESP_LOGI(TAG, "Radio module appears to be connected");
    }
    
    ESP_LOGI(TAG, "=== End Register Dump ===");
}

bool radio_common_is_connected(RadioCommon* radio) {
    if (!radio || !radio->initialized) {
        return false;
    }
    
    uint8_t config = nrf24_read_register(radio, NRF24_REG_CONFIG);
    
    // If config is all zeros or all ones, likely not connected
    if (config == 0x00 || config == 0xFF) {
        return false;
    }
    
    // Try to write and read back a test value
    uint8_t original = nrf24_read_register(radio, NRF24_REG_SETUP_AW);
    if (!nrf24_write_register(radio, NRF24_REG_SETUP_AW, 0x03)) {
        return false;
    }
    uint8_t read_back = nrf24_read_register(radio, NRF24_REG_SETUP_AW);
    nrf24_write_register(radio, NRF24_REG_SETUP_AW, original);  // Restore
    
    return (read_back == 0x03);
}

bool radio_common_validate_config(RadioCommon* radio) {
    if (!radio || !radio->initialized) {
        ESP_LOGE(TAG, "Radio not initialized");
        return false;
    }
    
    // Validate addresses
    if (!RADIO_ADDRESS_IS_VALID(radio->tx_address)) {
        ESP_LOGE(TAG, "Invalid TX address");
        return false;
    }
    
    if (!RADIO_ADDRESS_IS_VALID(radio->rx_address)) {
        ESP_LOGE(TAG, "Invalid RX address");
        return false;
    }
    
    // Validate channel
    uint8_t channel = nrf24_read_register(radio, NRF24_REG_RF_CH);
    if (channel > 125) {  // Valid channels: 0-125 (2.400-2.525 GHz)
        ESP_LOGE(TAG, "Invalid channel: %d", channel);
        return false;
    }
    
    // Validate address width
    uint8_t addr_width = nrf24_read_register(radio, NRF24_REG_SETUP_AW);
    if (addr_width < 0x01 || addr_width > 0x03) {  // Valid: 3-5 bytes
        ESP_LOGE(TAG, "Invalid address width: %d", addr_width + 2);
        return false;
    }
    
    ESP_LOGI(TAG, "Radio configuration is valid");
    return true;
}