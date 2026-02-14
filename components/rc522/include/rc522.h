#pragma once

#include "rc522_types.h"

// Initialise the MFRC522 over SPI; fills in *handle
rc522_status_t rc522_init(const rc522_config_t *config, rc522_handle_t *handle);

// Return firmware version byte read at init (0x91 = v1, 0x92 = v2)
uint8_t rc522_firmware_version(const rc522_handle_t *handle);

// --- PICC layer ---

// Send REQA; returns RC522_OK if a card answered
rc522_status_t rc522_is_new_card_present(rc522_handle_t *handle);

// Run anti-collision + select; fills uid struct
rc522_status_t rc522_read_card_uid(rc522_handle_t *handle, rc522_uid_t *uid);

// HLTA - put card to sleep
rc522_status_t rc522_halt(rc522_handle_t *handle);

// Clear MFCrypto1On flag
void rc522_stop_crypto1(rc522_handle_t *handle);

// MIFARE auth command constants (for use with rc522_mifare_auth)
#define RC522_AUTH_KEY_A  0x60
#define RC522_AUTH_KEY_B  0x61

// --- MIFARE Classic layer ---

// Authenticate a sector using Key A or Key B
rc522_status_t rc522_mifare_auth(rc522_handle_t *handle, uint8_t cmd,
                                  uint8_t block_addr, const uint8_t key[6],
                                  const rc522_uid_t *uid);

// Read 16 bytes from a block (caller provides buffer[16])
rc522_status_t rc522_mifare_read(rc522_handle_t *handle, uint8_t block_addr,
                                  uint8_t *buf, uint8_t *buf_len);

// Write 16 bytes to a block
rc522_status_t rc522_mifare_write(rc522_handle_t *handle, uint8_t block_addr,
                                   const uint8_t *data, uint8_t len);
