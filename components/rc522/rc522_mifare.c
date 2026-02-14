#include <string.h>
#include "esp_log.h"

#include "rc522_registers.h"
#include "include/rc522_types.h"
#include "include/rc522.h"

// Defined in rc522.c
extern rc522_status_t rc522_transceive(rc522_handle_t *h, uint8_t command,
                                        const uint8_t *send_data, uint8_t send_len,
                                        uint8_t *recv_data, uint8_t *recv_len,
                                        uint8_t *valid_bits, uint8_t rx_align,
                                        bool check_crc);
extern rc522_status_t rc522_calculate_crc(rc522_handle_t *h, const uint8_t *data,
                                           uint8_t len, uint8_t *result);

// Thin SPI wrappers (same as in rc522_picc.c)
static uint8_t read_reg(rc522_handle_t *h, uint8_t reg)
{
    uint8_t tx[2] = { ((reg << 1) & 0x7E) | 0x80, 0x00 };
    uint8_t rx[2] = { 0 };
    spi_transaction_t t = { .length = 16, .tx_buffer = tx, .rx_buffer = rx };
    spi_device_polling_transmit(h->spi, &t);
    return rx[1];
}

static const char *TAG = "rc522_mifare";

// ---------------------------------------------------------------------------
// MIFARE Authenticate (Key A or Key B)
// ---------------------------------------------------------------------------

rc522_status_t rc522_mifare_auth(rc522_handle_t *handle, uint8_t cmd,
                                  uint8_t block_addr, const uint8_t key[6],
                                  const rc522_uid_t *uid)
{
    if (cmd != PICC_CMD_MF_AUTH_KEY_A && cmd != PICC_CMD_MF_AUTH_KEY_B) {
        return RC522_ERR_INVALID;
    }

    // Build auth buffer: auth_cmd + block + 6-byte key + first 4 UID bytes
    uint8_t buffer[12];
    buffer[0] = cmd;
    buffer[1] = block_addr;
    memcpy(&buffer[2], key, 6);
    memcpy(&buffer[8], uid->bytes, 4);

    uint8_t dummy_len = 0;
    rc522_status_t status = rc522_transceive(handle, PCD_CMD_MF_AUTHENT,
                                              buffer, 12, NULL, &dummy_len,
                                              NULL, 0, false);
    if (status != RC522_OK) {
        ESP_LOGE(TAG, "Auth failed: %d", status);
        return RC522_ERR_AUTH_FAILED;
    }

    // Check MFCrypto1On bit
    if (!(read_reg(handle, RC522_REG_STATUS2) & 0x08)) {
        ESP_LOGE(TAG, "MFCrypto1On not set after auth");
        return RC522_ERR_AUTH_FAILED;
    }

    return RC522_OK;
}

// ---------------------------------------------------------------------------
// MIFARE Read (16 bytes from a block)
// ---------------------------------------------------------------------------

rc522_status_t rc522_mifare_read(rc522_handle_t *handle, uint8_t block_addr,
                                  uint8_t *buf, uint8_t *buf_len)
{
    if (buf == NULL || buf_len == NULL || *buf_len < 18) {
        return RC522_ERR_INVALID;
    }

    uint8_t cmd_buf[4];
    cmd_buf[0] = PICC_CMD_MF_READ;
    cmd_buf[1] = block_addr;

    rc522_status_t status = rc522_calculate_crc(handle, cmd_buf, 2, &cmd_buf[2]);
    if (status != RC522_OK) return status;

    status = rc522_transceive(handle, PCD_CMD_TRANSCEIVE,
                               cmd_buf, 4, buf, buf_len,
                               NULL, 0, true);
    return status;
}

// ---------------------------------------------------------------------------
// MIFARE Write (16 bytes to a block) - two-phase protocol
// ---------------------------------------------------------------------------

rc522_status_t rc522_mifare_write(rc522_handle_t *handle, uint8_t block_addr,
                                   const uint8_t *data, uint8_t len)
{
    if (data == NULL || len != 16) {
        return RC522_ERR_INVALID;
    }

    // Phase 1: send WRITE command + block address
    uint8_t cmd_buf[4];
    cmd_buf[0] = PICC_CMD_MF_WRITE;
    cmd_buf[1] = block_addr;

    rc522_status_t status = rc522_calculate_crc(handle, cmd_buf, 2, &cmd_buf[2]);
    if (status != RC522_OK) return status;

    uint8_t ack = 0;
    uint8_t ack_len = 1;
    uint8_t valid_bits = 0;

    status = rc522_transceive(handle, PCD_CMD_TRANSCEIVE,
                               cmd_buf, 4, &ack, &ack_len,
                               &valid_bits, 0, false);
    if (status != RC522_OK) return status;

    // MIFARE ACK is 4 bits = 0x0A
    if (valid_bits != 4 || (ack & 0x0F) != 0x0A) {
        return RC522_ERR_INTERNAL;
    }

    // Phase 2: send the 16 data bytes + CRC
    uint8_t write_buf[18];
    memcpy(write_buf, data, 16);

    status = rc522_calculate_crc(handle, write_buf, 16, &write_buf[16]);
    if (status != RC522_OK) return status;

    ack = 0;
    ack_len = 1;
    valid_bits = 0;

    status = rc522_transceive(handle, PCD_CMD_TRANSCEIVE,
                               write_buf, 18, &ack, &ack_len,
                               &valid_bits, 0, false);
    if (status != RC522_OK) return status;

    if (valid_bits != 4 || (ack & 0x0F) != 0x0A) {
        return RC522_ERR_INTERNAL;
    }

    return RC522_OK;
}
