#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "rc522.h"

#define APP_STATE_TOTAL_BLOCKS      64
#define APP_STATE_BLOCK_SIZE        16
#define APP_STATE_SECTORS           16
#define APP_STATE_BLOCKS_PER_SECTOR 4

typedef struct {
    bool        has_tag;
    rc522_uid_t uid;
    uint8_t     data[APP_STATE_TOTAL_BLOCKS][APP_STATE_BLOCK_SIZE];
    bool        block_read[APP_STATE_TOTAL_BLOCKS];
    bool        block_dirty[APP_STATE_TOTAL_BLOCKS];
} app_state_t;

void            app_state_init(void);
app_state_t    *app_state_get(void);
void            app_state_clear(void);
