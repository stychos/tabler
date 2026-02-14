#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "esp_log.h"

#include "rc522_registers.h"
#include "include/rc522_types.h"
#include "include/rc522.h"

static const char *TAG = "rc522";

// Forward declaration
rc522_status_t rc522_calculate_crc(rc522_handle_t *h, const uint8_t *data,
                                    uint8_t len, uint8_t *result);

// ---------------------------------------------------------------------------
// Low-level SPI register access
// ---------------------------------------------------------------------------

static void rc522_write_reg(rc522_handle_t *h, uint8_t reg, uint8_t val)
{
    uint8_t tx[2] = { (reg << 1) & 0x7E, val };
    spi_transaction_t t = {
        .length = 16,
        .tx_buffer = tx,
    };
    spi_device_polling_transmit(h->spi, &t);
}

static uint8_t rc522_read_reg(rc522_handle_t *h, uint8_t reg)
{
    uint8_t tx[2] = { ((reg << 1) & 0x7E) | 0x80, 0x00 };
    uint8_t rx[2] = { 0 };
    spi_transaction_t t = {
        .length = 16,
        .tx_buffer = tx,
        .rx_buffer = rx,
    };
    spi_device_polling_transmit(h->spi, &t);
    return rx[1];
}

static void rc522_write_reg_multi(rc522_handle_t *h, uint8_t reg,
                                   const uint8_t *data, uint8_t len)
{
    uint8_t addr = (reg << 1) & 0x7E;
    uint8_t tx[65];
    tx[0] = addr;
    memcpy(&tx[1], data, len);
    spi_transaction_t t = {
        .length = (uint32_t)(len + 1) * 8,
        .tx_buffer = tx,
    };
    spi_device_polling_transmit(h->spi, &t);
}

static void rc522_set_bits(rc522_handle_t *h, uint8_t reg, uint8_t mask)
{
    rc522_write_reg(h, reg, rc522_read_reg(h, reg) | mask);
}

static void rc522_clear_bits(rc522_handle_t *h, uint8_t reg, uint8_t mask)
{
    rc522_write_reg(h, reg, rc522_read_reg(h, reg) & ~mask);
}

// ---------------------------------------------------------------------------
// Transceive - send data to card and receive response
// ---------------------------------------------------------------------------

rc522_status_t rc522_transceive(rc522_handle_t *h, uint8_t command,
                                 const uint8_t *send_data, uint8_t send_len,
                                 uint8_t *recv_data, uint8_t *recv_len,
                                 uint8_t *valid_bits, uint8_t rx_align,
                                 bool check_crc)
{
    uint8_t irq_wait = 0;
    if (command == PCD_CMD_TRANSCEIVE) {
        irq_wait = 0x30; // RxIRq + IdleIRq
    } else if (command == PCD_CMD_MF_AUTHENT) {
        irq_wait = 0x10; // IdleIRq
    }

    rc522_write_reg(h, RC522_REG_COM_I_EN, irq_wait | 0x80); // IRqInv
    rc522_clear_bits(h, RC522_REG_COM_IRQ, 0x80);             // Clear all IRQ bits
    rc522_set_bits(h, RC522_REG_FIFO_LEVEL, 0x80);            // FlushBuffer

    rc522_write_reg(h, RC522_REG_COMMAND, PCD_CMD_IDLE);       // Stop any active command

    // Write data to FIFO
    rc522_write_reg_multi(h, RC522_REG_FIFO_DATA, send_data, send_len);

    // Set bit framing for rx_align and valid_bits
    uint8_t bit_framing = (rx_align << 4) | (valid_bits ? (*valid_bits & 0x07) : 0);
    rc522_write_reg(h, RC522_REG_BIT_FRAMING, bit_framing);

    // Execute command
    rc522_write_reg(h, RC522_REG_COMMAND, command);

    if (command == PCD_CMD_TRANSCEIVE) {
        rc522_set_bits(h, RC522_REG_BIT_FRAMING, 0x80); // StartSend
    }

    // Wait for completion
    uint16_t timeout = 2000;
    uint8_t irq;
    while (true) {
        irq = rc522_read_reg(h, RC522_REG_COM_IRQ);
        if (irq & irq_wait) break;        // One of our expected IRQs
        if (irq & 0x01) {                 // TimerIRq
            return RC522_ERR_TIMEOUT;
        }
        if (--timeout == 0) {
            return RC522_ERR_TIMEOUT;
        }
    }

    // Check for errors
    uint8_t error = rc522_read_reg(h, RC522_REG_ERROR);
    if (error & 0x13) { // BufferOvfl | ParityErr | ProtocolErr
        return RC522_ERR_INTERNAL;
    }

    if (error & 0x08) { // CollErr
        return RC522_ERR_COLLISION;
    }

    // Read data from FIFO
    if (recv_data && recv_len) {
        uint8_t n = rc522_read_reg(h, RC522_REG_FIFO_LEVEL);
        if (n > *recv_len) {
            return RC522_ERR_NO_ROOM;
        }
        *recv_len = n;
        for (uint8_t i = 0; i < n; i++) {
            recv_data[i] = rc522_read_reg(h, RC522_REG_FIFO_DATA);
        }
        if (valid_bits) {
            *valid_bits = rc522_read_reg(h, RC522_REG_CONTROL) & 0x07;
        }
    }

    // Optional CRC check
    if (check_crc && recv_data && recv_len && *recv_len >= 2) {
        uint8_t crc[2];
        rc522_status_t s = rc522_calculate_crc(h, recv_data, *recv_len - 2, crc);
        if (s != RC522_OK) return s;
        if (crc[0] != recv_data[*recv_len - 2] || crc[1] != recv_data[*recv_len - 1]) {
            return RC522_ERR_CRC;
        }
    }

    return RC522_OK;
}

// ---------------------------------------------------------------------------
// Hardware CRC_A calculation
// ---------------------------------------------------------------------------

rc522_status_t rc522_calculate_crc(rc522_handle_t *h, const uint8_t *data,
                                    uint8_t len, uint8_t *result)
{
    rc522_write_reg(h, RC522_REG_COMMAND, PCD_CMD_IDLE);
    rc522_write_reg(h, RC522_REG_DIV_IRQ, 0x04);          // Clear CRCIRq
    rc522_set_bits(h, RC522_REG_FIFO_LEVEL, 0x80);        // FlushBuffer
    rc522_write_reg_multi(h, RC522_REG_FIFO_DATA, data, len);
    rc522_write_reg(h, RC522_REG_COMMAND, PCD_CMD_CALC_CRC);

    uint16_t timeout = 5000;
    while (true) {
        uint8_t irq = rc522_read_reg(h, RC522_REG_DIV_IRQ);
        if (irq & 0x04) break; // CRCIRq
        if (--timeout == 0) return RC522_ERR_TIMEOUT;
    }
    rc522_write_reg(h, RC522_REG_COMMAND, PCD_CMD_IDLE);

    result[0] = rc522_read_reg(h, RC522_REG_CRC_RESULT_LSB);
    result[1] = rc522_read_reg(h, RC522_REG_CRC_RESULT_MSB);
    return RC522_OK;
}

// ---------------------------------------------------------------------------
// Antenna control
// ---------------------------------------------------------------------------

static void rc522_antenna_on(rc522_handle_t *h)
{
    uint8_t val = rc522_read_reg(h, RC522_REG_TX_CONTROL);
    if ((val & 0x03) != 0x03) {
        rc522_write_reg(h, RC522_REG_TX_CONTROL, val | 0x03);
    }
}

// ---------------------------------------------------------------------------
// Init
// ---------------------------------------------------------------------------

rc522_status_t rc522_init(const rc522_config_t *config, rc522_handle_t *handle)
{
    memset(handle, 0, sizeof(*handle));
    handle->rst_pin = config->rst;

    // Configure RST pin
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << config->rst),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&io_conf);

    // Hard reset
    gpio_set_level(config->rst, 0);
    vTaskDelay(pdMS_TO_TICKS(10));
    gpio_set_level(config->rst, 1);
    vTaskDelay(pdMS_TO_TICKS(50));

    // Init SPI bus
    spi_bus_config_t bus_cfg = {
        .miso_io_num = config->miso,
        .mosi_io_num = config->mosi,
        .sclk_io_num = config->sck,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 64,
    };
    esp_err_t err = spi_bus_initialize(config->spi_host, &bus_cfg, SPI_DMA_DISABLED);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "SPI bus init failed: %s", esp_err_to_name(err));
        return RC522_ERR_INTERNAL;
    }

    spi_device_interface_config_t dev_cfg = {
        .clock_speed_hz = config->clock_speed_hz,
        .mode = 0,
        .spics_io_num = config->cs,
        .queue_size = 1,
    };
    err = spi_bus_add_device(config->spi_host, &dev_cfg, &handle->spi);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "SPI add device failed: %s", esp_err_to_name(err));
        return RC522_ERR_INTERNAL;
    }

    // Soft reset
    rc522_write_reg(handle, RC522_REG_COMMAND, PCD_CMD_SOFT_RESET);
    vTaskDelay(pdMS_TO_TICKS(50));

    // Timer: TPrescaler = 0xD3E -> ~25ms timeout @ 13.56MHz
    rc522_write_reg(handle, RC522_REG_T_MODE, 0x8D);        // TAuto=1, prescaler hi
    rc522_write_reg(handle, RC522_REG_T_PRESCALER, 0x3E);   // prescaler lo
    rc522_write_reg(handle, RC522_REG_T_RELOAD_HI, 0x00);
    rc522_write_reg(handle, RC522_REG_T_RELOAD_LO, 0x1E);   // reload = 30

    rc522_write_reg(handle, RC522_REG_TX_ASK, 0x40);        // 100% ASK modulation
    rc522_write_reg(handle, RC522_REG_MODE, 0x3D);          // CRC preset 0x6363

    rc522_antenna_on(handle);

    // Read firmware version
    handle->firmware_version = rc522_read_reg(handle, RC522_REG_VERSION);
    ESP_LOGI(TAG, "MFRC522 firmware: 0x%02X", handle->firmware_version);

    if (handle->firmware_version == 0x00 || handle->firmware_version == 0xFF) {
        ESP_LOGE(TAG, "RC522 not detected (version=0x%02X)", handle->firmware_version);
        return RC522_ERR_INTERNAL;
    }

    return RC522_OK;
}

uint8_t rc522_firmware_version(const rc522_handle_t *handle)
{
    return handle->firmware_version;
}
