#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "driver/spi_master.h"
#include "driver/gpio.h"

// Status codes returned by driver functions
typedef enum {
    RC522_OK = 0,
    RC522_ERR_TIMEOUT,
    RC522_ERR_CRC,
    RC522_ERR_COLLISION,
    RC522_ERR_INTERNAL,
    RC522_ERR_INVALID,
    RC522_ERR_NO_ROOM,
    RC522_ERR_AUTH_FAILED,
} rc522_status_t;

// UID structure - supports 4, 7, or 10 byte UIDs
#define RC522_UID_MAX_LEN 10

typedef struct {
    uint8_t bytes[RC522_UID_MAX_LEN];
    uint8_t len;
    uint8_t sak;    // Select Acknowledge
} rc522_uid_t;

// Driver handle (internal state)
typedef struct {
    spi_device_handle_t spi;
    gpio_num_t rst_pin;
    uint8_t firmware_version;
} rc522_handle_t;

// Configuration for rc522_init()
typedef struct {
    gpio_num_t miso;
    gpio_num_t mosi;
    gpio_num_t sck;
    gpio_num_t cs;
    gpio_num_t rst;
    spi_host_device_t spi_host;
    int clock_speed_hz;
} rc522_config_t;

// Default configuration macro
#define RC522_CONFIG_DEFAULT() { \
    .miso = GPIO_NUM_19,         \
    .mosi = GPIO_NUM_23,         \
    .sck  = GPIO_NUM_18,         \
    .cs   = GPIO_NUM_5,          \
    .rst  = GPIO_NUM_22,         \
    .spi_host = SPI3_HOST,       \
    .clock_speed_hz = 5000000,   \
}
