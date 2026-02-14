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

static const char *TAG = "rc522_ul";

// ---------------------------------------------------------------------------
// Ultralight / NTAG Read (4 consecutive pages = 16 bytes)
// ---------------------------------------------------------------------------

rc522_status_t rc522_ultralight_read(rc522_handle_t *handle, uint8_t page_addr,
                                      uint8_t *buf, uint8_t *buf_len)
{
    if (buf == NULL || buf_len == NULL || *buf_len < 18) {
        return RC522_ERR_INVALID;
    }

    // Same READ command as Classic (0x30) — returns 16 bytes (4 pages)
    uint8_t cmd_buf[4];
    cmd_buf[0] = PICC_CMD_MF_READ;
    cmd_buf[1] = page_addr;

    rc522_status_t status = rc522_calculate_crc(handle, cmd_buf, 2, &cmd_buf[2]);
    if (status != RC522_OK) return status;

    status = rc522_transceive(handle, PCD_CMD_TRANSCEIVE,
                               cmd_buf, 4, buf, buf_len,
                               NULL, 0, true);
    return status;
}

// ---------------------------------------------------------------------------
// Ultralight / NTAG Write (4 bytes to a single page)
// ---------------------------------------------------------------------------

rc522_status_t rc522_ultralight_write(rc522_handle_t *handle, uint8_t page_addr,
                                       const uint8_t *data)
{
    if (data == NULL) return RC522_ERR_INVALID;

    // Single-phase write: 0xA2 + page_addr + 4 data bytes + CRC
    uint8_t cmd_buf[8];
    cmd_buf[0] = PICC_CMD_UL_WRITE;
    cmd_buf[1] = page_addr;
    memcpy(&cmd_buf[2], data, 4);

    rc522_status_t status = rc522_calculate_crc(handle, cmd_buf, 6, &cmd_buf[6]);
    if (status != RC522_OK) return status;

    uint8_t ack = 0;
    uint8_t ack_len = 1;
    uint8_t valid_bits = 0;

    status = rc522_transceive(handle, PCD_CMD_TRANSCEIVE,
                               cmd_buf, 8, &ack, &ack_len,
                               &valid_bits, 0, false);
    if (status != RC522_OK) return status;

    // Expect 4-bit ACK = 0x0A
    if (valid_bits != 4 || (ack & 0x0F) != 0x0A) {
        ESP_LOGE(TAG, "Write NAK: bits=%d ack=0x%02X", valid_bits, ack);
        return RC522_ERR_INTERNAL;
    }

    return RC522_OK;
}
