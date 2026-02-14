#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "rc522.h"

// Classic storage (max Classic 4K: 256 blocks x 16 bytes)
#define APP_STATE_MAX_BLOCKS      256
#define APP_STATE_BLOCK_SIZE      16

// UL/NTAG storage (max NTAG216: 231 pages x 4 bytes)
#define APP_STATE_MAX_PAGES       231
#define APP_STATE_PAGE_SIZE       4

typedef struct {
    bool              has_tag;
    rc522_uid_t       uid;
    rc522_picc_type_t picc_type;
    union {
        struct {
            uint8_t data[APP_STATE_MAX_BLOCKS][APP_STATE_BLOCK_SIZE];
            bool    read[APP_STATE_MAX_BLOCKS];
            bool    dirty[APP_STATE_MAX_BLOCKS];
        } blocks;
        struct {
            uint8_t data[APP_STATE_MAX_PAGES][APP_STATE_PAGE_SIZE];
            bool    read[APP_STATE_MAX_PAGES];
            bool    dirty[APP_STATE_MAX_PAGES];
        } pages;
    };
} app_state_t;

#define APP_STATE_IS_CLASSIC(st) \
    ((st)->picc_type == RC522_PICC_TYPE_CLASSIC_1K || \
     (st)->picc_type == RC522_PICC_TYPE_CLASSIC_4K)

#define APP_STATE_IS_UL(st) \
    ((st)->picc_type == RC522_PICC_TYPE_ULTRALIGHT   || \
     (st)->picc_type == RC522_PICC_TYPE_ULTRALIGHT_C || \
     (st)->picc_type == RC522_PICC_TYPE_NTAG213      || \
     (st)->picc_type == RC522_PICC_TYPE_NTAG215      || \
     (st)->picc_type == RC522_PICC_TYPE_NTAG216)

void            app_state_init(void);
app_state_t    *app_state_get(void);
void            app_state_clear(void);
