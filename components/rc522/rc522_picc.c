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

// Forward declarations for internal register access (defined in rc522.c)
// We re-implement thin wrappers here to avoid exposing them in the public header.
static void write_reg(rc522_handle_t *h, uint8_t reg, uint8_t val)
{
    uint8_t tx[2] = { (reg << 1) & 0x7E, val };
    spi_transaction_t t = { .length = 16, .tx_buffer = tx };
    spi_device_polling_transmit(h->spi, &t);
}

static uint8_t read_reg(rc522_handle_t *h, uint8_t reg)
{
    uint8_t tx[2] = { ((reg << 1) & 0x7E) | 0x80, 0x00 };
    uint8_t rx[2] = { 0 };
    spi_transaction_t t = { .length = 16, .tx_buffer = tx, .rx_buffer = rx };
    spi_device_polling_transmit(h->spi, &t);
    return rx[1];
}

static void clear_bits(rc522_handle_t *h, uint8_t reg, uint8_t mask)
{
    write_reg(h, reg, read_reg(h, reg) & ~mask);
}

static const char *TAG = "rc522_picc";

// ---------------------------------------------------------------------------
// REQA - check if a new card is present
// ---------------------------------------------------------------------------

rc522_status_t rc522_is_new_card_present(rc522_handle_t *handle)
{
    // Reset baud rate and clear CollReg bits
    write_reg(handle, RC522_REG_TX_MODE, 0x00);
    write_reg(handle, RC522_REG_RX_MODE, 0x00);
    write_reg(handle, RC522_REG_MOD_WIDTH, 0x26);

    uint8_t cmd = PICC_CMD_REQA;
    uint8_t atqa[2] = { 0 };
    uint8_t atqa_len = sizeof(atqa);
    uint8_t valid_bits = 7; // REQA is a short frame (7 bits)

    rc522_status_t status = rc522_transceive(handle, PCD_CMD_TRANSCEIVE,
                                              &cmd, 1, atqa, &atqa_len,
                                              &valid_bits, 0, false);
    if (status != RC522_OK) return status;
    if (atqa_len != 2) return RC522_ERR_INTERNAL;
    return RC522_OK;
}

// ---------------------------------------------------------------------------
// Anti-collision + Select (supports cascade levels 1-3)
// ---------------------------------------------------------------------------

rc522_status_t rc522_read_card_uid(rc522_handle_t *handle, rc522_uid_t *uid)
{
    memset(uid, 0, sizeof(*uid));

    uint8_t cascade_cmds[] = { PICC_CMD_SEL_CL1, PICC_CMD_SEL_CL2, PICC_CMD_SEL_CL3 };
    uint8_t uid_index = 0;
    bool uid_complete = false;

    for (int level = 0; level < 3 && !uid_complete; level++) {
        uint8_t buffer[9]; // 2 SEL bytes + 4 UID bytes + BCC
        uint8_t buf_len;
        uint8_t valid_bits;
        uint8_t tx_last_bits;

        // Anti-collision: NVB = 0x20 (only SEL + NVB, no UID bits)
        buffer[0] = cascade_cmds[level];
        buffer[1] = 0x20; // NVB: 2 bytes transmitted
        buf_len = 9;
        valid_bits = 0;

        // Clear collision register
        clear_bits(handle, RC522_REG_COLL, 0x80); // ValuesAfterColl

        rc522_status_t status = rc522_transceive(handle, PCD_CMD_TRANSCEIVE,
                                                  buffer, 2, &buffer[2], &buf_len,
                                                  &valid_bits, 0, false);
        if (status != RC522_OK) return status;

        if (buf_len != 5) return RC522_ERR_INTERNAL;

        // Verify BCC (XOR of the 4 UID bytes)
        uint8_t bcc = buffer[2] ^ buffer[3] ^ buffer[4] ^ buffer[5];
        if (bcc != buffer[6]) return RC522_ERR_CRC;

        // SELECT: NVB = 0x70 (all 7 bytes: SEL + NVB + 4 UID + BCC)
        buffer[1] = 0x70;
        // Append CRC_A
        uint8_t crc[2];
        status = rc522_calculate_crc(handle, buffer, 7, crc);
        if (status != RC522_OK) return status;
        buffer[7] = crc[0];
        buffer[8] = crc[1];

        uint8_t sak_buf[3]; // SAK + CRC
        uint8_t sak_len = sizeof(sak_buf);
        tx_last_bits = 0;

        status = rc522_transceive(handle, PCD_CMD_TRANSCEIVE,
                                   buffer, 9, sak_buf, &sak_len,
                                   &tx_last_bits, 0, true);
        if (status != RC522_OK) return status;

        if (buffer[2] == PICC_CMD_CT) {
            // Cascade tag - UID continues at next level
            memcpy(&uid->bytes[uid_index], &buffer[3], 3);
            uid_index += 3;
        } else {
            // Final level
            memcpy(&uid->bytes[uid_index], &buffer[2], 4);
            uid_index += 4;
            uid_complete = true;
        }

        uid->sak = sak_buf[0];
    }

    uid->len = uid_index;
    return RC522_OK;
}

// ---------------------------------------------------------------------------
// HLTA - halt the card
// ---------------------------------------------------------------------------

rc522_status_t rc522_halt(rc522_handle_t *handle)
{
    uint8_t buffer[4];
    buffer[0] = PICC_CMD_HLTA;
    buffer[1] = 0;

    rc522_status_t status = rc522_calculate_crc(handle, buffer, 2, &buffer[2]);
    if (status != RC522_OK) return status;

    uint8_t dummy_len = 0;
    // HLTA: we don't expect a response (card goes silent), timeout is normal
    status = rc522_transceive(handle, PCD_CMD_TRANSCEIVE,
                               buffer, 4, NULL, &dummy_len,
                               NULL, 0, false);
    // Timeout is the expected response from HLTA
    if (status == RC522_ERR_TIMEOUT) return RC522_OK;
    return (status == RC522_OK) ? RC522_ERR_INTERNAL : status;
}

// ---------------------------------------------------------------------------
// Stop crypto1 (clear Status2Reg MFCrypto1On bit)
// ---------------------------------------------------------------------------

void rc522_stop_crypto1(rc522_handle_t *handle)
{
    clear_bits(handle, RC522_REG_STATUS2, 0x08);
}
