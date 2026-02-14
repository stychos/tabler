#include <string.h>
#include <stdio.h>
#include <ctype.h>
#include <stdlib.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "esp_system.h"
#include "esp_chip_info.h"
#include "esp_flash.h"
#include "esp_timer.h"
#include "soc/soc.h"

#include "shell.h"
#include "app_state.h"

#define UART_NUM       UART_NUM_0
#define UART_BUF_SIZE  256
#define CMD_BUF_SIZE   64
#define PROMPT         ">_ "

// ANSI color codes
#define ANSI_RESET   "\033[0m"
#define ANSI_BOLD    "\033[1m"
#define ANSI_WHITE   "\033[97m"
#define ANSI_YELLOW  "\033[33m"
#define ANSI_GRAY    "\033[90m"
#define ANSI_CYAN    "\033[36m"
#define ANSI_GREEN   "\033[32m"
#define ANSI_RED     "\033[31m"
#define ANSI_INVERSE "\033[7m"

static rc522_handle_t *s_rc522;

// Default MIFARE key (factory default)
static const uint8_t DEFAULT_KEY[6] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };

// ---------------------------------------------------------------------------
// UART helpers
// ---------------------------------------------------------------------------

static void shell_uart_init(void)
{
    uart_config_t cfg = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    // Let any pending log/printf output drain before taking over UART
    fflush(stdout);
    vTaskDelay(pdMS_TO_TICKS(100));

    uart_param_config(UART_NUM, &cfg);
    uart_driver_install(UART_NUM, UART_BUF_SIZE * 2, 0, 0, NULL, 0);
}

static int shell_getchar(int timeout_ms)
{
    uint8_t c;
    int len = uart_read_bytes(UART_NUM, &c, 1, pdMS_TO_TICKS(timeout_ms));
    return (len > 0) ? c : -1;
}

static void shell_putchar(char c)
{
    uart_write_bytes(UART_NUM, &c, 1);
}

static void shell_puts(const char *s)
{
    uart_write_bytes(UART_NUM, s, strlen(s));
}

static void shell_printf(const char *fmt, ...)
{
    char buf[128];
    va_list args;
    va_start(args, fmt);
    vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);
    shell_puts(buf);
}

// Read a line with echo and backspace handling
static int shell_readline(char *buf, int max_len)
{
    int pos = 0;
    while (pos < max_len - 1) {
        int c = shell_getchar(portMAX_DELAY);
        if (c < 0) continue;

        if (c == '\r' || c == '\n') {
            shell_puts("\r\n");
            break;
        }
        if (c == 0x7F || c == '\b') { // Backspace / DEL
            if (pos > 0) {
                pos--;
                shell_puts("\b \b");
            }
            continue;
        }
        if (c >= 0x20 && c < 0x7F) {
            buf[pos++] = (char)c;
            shell_putchar((char)c);
        }
    }
    buf[pos] = '\0';
    return pos;
}

// Check for cancel key (ESC) without blocking
static bool shell_check_cancel(void)
{
    int c = shell_getchar(0);
    return (c == 0x1B);
}

// ---------------------------------------------------------------------------
// Classic 4K sector math helpers
// ---------------------------------------------------------------------------

static int classic_sector_count(const app_state_t *st)
{
    return (st->picc_type == RC522_PICC_TYPE_CLASSIC_4K) ? 40 : 16;
}

static int classic_sector_base(int sector)
{
    if (sector < 32) return sector * 4;
    return 128 + (sector - 32) * 16;
}

static int classic_sector_size(int sector)
{
    return (sector < 32) ? 4 : 16;
}

static bool classic_is_trailer(int addr)
{
    // For blocks 0-127: trailer is every 4th block (addr % 4 == 3)
    // For blocks 128-255: trailer is every 16th block (addr % 16 == 15)
    if (addr < 128) return (addr % 4) == 3;
    return (addr % 16) == 15;
}

// Which sector does a block belong to?
static int classic_block_to_sector(int addr)
{
    if (addr < 128) return addr / 4;
    return 32 + (addr - 128) / 16;
}

// ---------------------------------------------------------------------------
// Shared display helpers
// ---------------------------------------------------------------------------

static void print_sector(const app_state_t *st, int sector)
{
    int base = classic_sector_base(sector);
    int size = classic_sector_size(sector);
    for (int r = 0; r < size; r++) {
        int addr = base + r;
        shell_printf(ANSI_BOLD ANSI_WHITE "  Block %02d: " ANSI_RESET, addr);

        if (!st->blocks.read[addr]) {
            shell_puts(ANSI_GRAY);
            for (int i = 0; i < APP_STATE_BLOCK_SIZE; i++)
                shell_puts("-- ");
            shell_puts(ANSI_RESET "\r\n");
            continue;
        }

        if (addr == 0) {
            shell_puts(ANSI_GRAY);           // manufacturer block
        } else if (r == size - 1) {
            shell_puts(ANSI_YELLOW);          // sector trailer
        }

        for (int i = 0; i < APP_STATE_BLOCK_SIZE; i++)
            shell_printf("%02X ", st->blocks.data[addr][i]);

        shell_puts(ANSI_RESET "\r\n");
    }
}

static void print_pages(const app_state_t *st, int group)
{
    const rc522_picc_info_t *info = rc522_get_picc_info(st->picc_type);
    int total = info->total_pages;
    int start = group * 16;
    if (start >= total) {
        shell_puts(ANSI_RED "Invalid page group.\r\n" ANSI_RESET);
        return;
    }
    int end = start + 16;
    if (end > total) end = total;

    for (int p = start; p < end; p++) {
        shell_printf(ANSI_BOLD ANSI_WHITE "  Page %3d: " ANSI_RESET, p);

        if (!st->pages.read[p]) {
            shell_puts(ANSI_GRAY "-- -- -- --" ANSI_RESET "\r\n");
            continue;
        }

        // Color: gray for UID/lock pages 0-3, yellow for last 5 config pages
        if (p <= 3) {
            shell_puts(ANSI_GRAY);
        } else if (p >= total - 5) {
            shell_puts(ANSI_YELLOW);
        }

        for (int i = 0; i < APP_STATE_PAGE_SIZE; i++)
            shell_printf("%02X ", st->pages.data[p][i]);

        shell_puts(ANSI_RESET "\r\n");
    }
}

// ---------------------------------------------------------------------------
// Commands
// ---------------------------------------------------------------------------

static void cmd_info(const char *args)
{
    esp_chip_info_t chip;
    esp_chip_info(&chip);

    uint32_t flash_size = 0;
    esp_flash_get_size(NULL, &flash_size);

    shell_printf(ANSI_CYAN "Chip:     " ANSI_RESET "ESP32 rev %d.%d, %d core(s)\r\n",
                 chip.revision / 100, chip.revision % 100, chip.cores);
    shell_printf(ANSI_CYAN "Flash:    " ANSI_RESET "%lu KB\r\n", (unsigned long)(flash_size / 1024));
    shell_printf(ANSI_CYAN "Heap:     " ANSI_RESET "%lu bytes free\r\n", (unsigned long)esp_get_free_heap_size());
    shell_printf(ANSI_CYAN "IDF:      " ANSI_RESET "%s\r\n", esp_get_idf_version());
    shell_printf(ANSI_CYAN "RC522 FW: " ANSI_RESET "0x%02X\r\n", rc522_firmware_version(s_rc522));
    shell_printf(ANSI_CYAN "Uptime:   " ANSI_RESET "%lld s\r\n", esp_timer_get_time() / 1000000LL);

    app_state_t *st = app_state_get();
    if (!st->has_tag) {
        shell_puts(ANSI_CYAN "Tag:      " ANSI_RESET "No tag in memory\r\n");
        return;
    }

    // Print UID and type
    const rc522_picc_info_t *info = rc522_get_picc_info(st->picc_type);
    shell_printf(ANSI_CYAN "Tag:      " ANSI_RESET "%s\r\n", info->name);
    shell_puts(ANSI_CYAN "Tag UID:  " ANSI_RESET);
    for (int i = 0; i < st->uid.len; i++) {
        shell_printf("%02X", st->uid.bytes[i]);
        if (i < st->uid.len - 1) shell_putchar(':');
    }
    shell_printf("  SAK: 0x%02X\r\n", st->uid.sak);

    // Parse args: -sN for sector, -pN for page group
    if (APP_STATE_IS_CLASSIC(st)) {
        int sector = 0;
        int max_sectors = classic_sector_count(st);
        if (args && args[0]) {
            const char *p = args;
            while (*p && isspace((unsigned char)*p)) p++;
            if (p[0] == '-' && p[1] == 's') {
                p += 2;
                while (*p && isspace((unsigned char)*p)) p++;
                int s = atoi(p);
                if (s >= 0 && s < max_sectors)
                    sector = s;
                else {
                    shell_printf(ANSI_RED "Invalid sector %d (0-%d)\r\n" ANSI_RESET,
                                 s, max_sectors - 1);
                    return;
                }
            }
        }
        shell_printf("\r\n  Sector %d:\r\n", sector);
        print_sector(st, sector);
    } else if (APP_STATE_IS_UL(st)) {
        int group = 0;
        int total = info->total_pages;
        int max_groups = (total + 15) / 16;
        if (args && args[0]) {
            const char *p = args;
            while (*p && isspace((unsigned char)*p)) p++;
            if (p[0] == '-' && p[1] == 'p') {
                p += 2;
                while (*p && isspace((unsigned char)*p)) p++;
                int g = atoi(p);
                if (g >= 0 && g < max_groups)
                    group = g;
                else {
                    shell_printf(ANSI_RED "Invalid page group %d (0-%d)\r\n" ANSI_RESET,
                                 g, max_groups - 1);
                    return;
                }
            }
        }
        shell_printf("\r\n  Pages %d-%d:\r\n", group * 16,
                     ((group + 1) * 16 < total ? (group + 1) * 16 - 1 : total - 1));
        print_pages(st, group);
    } else {
        shell_puts(ANSI_GRAY "  Detection only — no memory data.\r\n" ANSI_RESET);
    }
}

// ---------------------------------------------------------------------------
// cmd_read — auto-detect type and read accordingly
// ---------------------------------------------------------------------------

static void cmd_read(void)
{
    shell_puts("Present tag... (ESC to cancel)\r\n");

    while (!shell_check_cancel()) {
        rc522_status_t status = rc522_is_new_card_present(s_rc522);
        if (status != RC522_OK) {
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }

        rc522_uid_t uid;
        status = rc522_read_card_uid(s_rc522, &uid);
        if (status != RC522_OK) {
            shell_puts(ANSI_RED "Failed to read UID.\r\n" ANSI_RESET);
            rc522_halt(s_rc522);
            rc522_stop_crypto1(s_rc522);
            continue;
        }

        // Detect type
        rc522_picc_type_t picc_type = rc522_detect_picc_type(s_rc522, uid.sak);
        const rc522_picc_info_t *info = rc522_get_picc_info(picc_type);

        shell_printf(ANSI_GREEN "Card detected! " ANSI_RESET "%s  UID: ", info->name);
        for (int i = 0; i < uid.len; i++) {
            shell_printf("%02X", uid.bytes[i]);
            if (i < uid.len - 1) shell_putchar(':');
        }
        shell_puts("\r\n");

        app_state_t *st = app_state_get();
        app_state_clear();
        st->uid = uid;
        st->picc_type = picc_type;

        // --- DESFire / Plus: detect only ---
        if (picc_type == RC522_PICC_TYPE_DESFIRE || picc_type == RC522_PICC_TYPE_PLUS) {
            st->has_tag = true;
            shell_puts(ANSI_YELLOW "Detection only — read/write not supported for this type.\r\n" ANSI_RESET);
            rc522_halt(s_rc522);
            rc522_stop_crypto1(s_rc522);
            return;
        }

        // --- Classic 1K / 4K ---
        if (APP_STATE_IS_CLASSIC(st)) {
            int sectors = classic_sector_count(st);
            int read_count = 0;
            int fail_count = 0;

            for (int sector = 0; sector < sectors; sector++) {
                // Yield periodically to prevent interrupt WDT on large reads
                if (sector > 0 && (sector % 8) == 0)
                    taskYIELD();

                int base_block = classic_sector_base(sector);
                int blk_count = classic_sector_size(sector);

                status = rc522_mifare_auth(s_rc522, RC522_AUTH_KEY_A,
                                            base_block, DEFAULT_KEY, &uid);
                if (status != RC522_OK) {
                    shell_printf(ANSI_RED "  Sector %d: auth failed\r\n" ANSI_RESET, sector);
                    fail_count += blk_count;
                    continue;
                }

                for (int b = 0; b < blk_count; b++) {
                    int addr = base_block + b;
                    uint8_t buf[18];
                    uint8_t buf_len = sizeof(buf);
                    status = rc522_mifare_read(s_rc522, addr, buf, &buf_len);
                    if (status != RC522_OK) {
                        shell_printf(ANSI_RED "  Block %02d: read failed\r\n" ANSI_RESET, addr);
                        fail_count++;
                        continue;
                    }
                    memcpy(st->blocks.data[addr], buf, APP_STATE_BLOCK_SIZE);
                    st->blocks.read[addr] = true;
                    st->blocks.dirty[addr] = false;
                    read_count++;
                }
            }

            st->has_tag = true;
            shell_printf(ANSI_GREEN "Read %d block(s)" ANSI_RESET, read_count);
            if (fail_count > 0)
                shell_printf(", " ANSI_RED "%d failed" ANSI_RESET, fail_count);
            shell_puts(".\r\n");
        }

        // --- Ultralight / NTAG ---
        else if (APP_STATE_IS_UL(st)) {
            int total_pages = info->total_pages;
            int read_count = 0;
            int fail_count = 0;

            // Read 4 pages at a time
            for (int p = 0; p < total_pages; p += 4) {
                if (p > 0 && (p % 32) == 0)
                    taskYIELD();
                uint8_t buf[18];
                uint8_t buf_len = sizeof(buf);
                status = rc522_ultralight_read(s_rc522, p, buf, &buf_len);
                if (status != RC522_OK) {
                    shell_printf(ANSI_RED "  Page %d: read failed\r\n" ANSI_RESET, p);
                    int remaining = total_pages - p;
                    fail_count += (remaining < 4 ? remaining : 4);
                    continue;
                }

                // Copy up to 4 pages from the 16-byte response
                for (int i = 0; i < 4 && (p + i) < total_pages; i++) {
                    memcpy(st->pages.data[p + i], &buf[i * 4], APP_STATE_PAGE_SIZE);
                    st->pages.read[p + i] = true;
                    st->pages.dirty[p + i] = false;
                    read_count++;
                }
            }

            st->has_tag = true;
            shell_printf(ANSI_GREEN "Read %d page(s)" ANSI_RESET, read_count);
            if (fail_count > 0)
                shell_printf(", " ANSI_RED "%d failed" ANSI_RESET, fail_count);
            shell_puts(".\r\n");
        }

        else {
            st->has_tag = true;
            shell_puts(ANSI_YELLOW "Unknown tag type — stored UID only.\r\n" ANSI_RESET);
        }

        rc522_halt(s_rc522);
        rc522_stop_crypto1(s_rc522);
        return;
    }

    shell_puts("Cancelled.\r\n");
}

// ---------------------------------------------------------------------------
// cmd_write — type-aware write
// ---------------------------------------------------------------------------

static void cmd_write(const char *args)
{
    app_state_t *st = app_state_get();
    if (!st->has_tag) {
        shell_puts(ANSI_RED "No tag in memory. Use 'read' first.\r\n" ANSI_RESET);
        return;
    }

    // Parse -f flag (force write trailers)
    bool force = false;
    if (args && args[0]) {
        const char *p = args;
        while (*p && isspace((unsigned char)*p)) p++;
        if (p[0] == '-' && p[1] == 'f') force = true;
    }

    // --- Classic ---
    if (APP_STATE_IS_CLASSIC(st)) {
        int total = (st->picc_type == RC522_PICC_TYPE_CLASSIC_4K) ? 256 : 64;
        int write_count = 0;
        for (int i = 1; i < total; i++) {
            if (!st->blocks.read[i]) continue;
            if (!force && classic_is_trailer(i)) continue;
            write_count++;
        }
        if (write_count == 0) {
            shell_puts("No blocks to write.\r\n");
            return;
        }

        if (force)
            shell_puts(ANSI_YELLOW "Force mode: sector trailers WILL be written!\r\n" ANSI_RESET);

        shell_printf("Will write %d block(s). Present target tag... (ESC to cancel)\r\n",
                     write_count);

        while (!shell_check_cancel()) {
            rc522_status_t status = rc522_is_new_card_present(s_rc522);
            if (status != RC522_OK) {
                vTaskDelay(pdMS_TO_TICKS(100));
                continue;
            }

            rc522_uid_t uid;
            status = rc522_read_card_uid(s_rc522, &uid);
            if (status != RC522_OK) {
                shell_puts(ANSI_RED "Failed to read UID.\r\n" ANSI_RESET);
                rc522_halt(s_rc522);
                rc522_stop_crypto1(s_rc522);
                continue;
            }

            shell_puts(ANSI_GREEN "Target detected! " ANSI_RESET "Writing...\r\n");

            int written = 0;
            int last_sector = -1;

            for (int addr = 1; addr < total; addr++) {
                if (!st->blocks.read[addr]) continue;
                if (!force && classic_is_trailer(addr)) {
                    shell_printf(ANSI_YELLOW "  Block %02d: SKIP (trailer)\r\n" ANSI_RESET, addr);
                    continue;
                }

                int sector = classic_block_to_sector(addr);

                if (sector != last_sector) {
                    int sector_base = classic_sector_base(sector);
                    status = rc522_mifare_auth(s_rc522, RC522_AUTH_KEY_A,
                                                sector_base, DEFAULT_KEY, &uid);
                    if (status != RC522_OK) {
                        shell_printf(ANSI_RED "  Auth failed for sector %d\r\n" ANSI_RESET, sector);
                        break;
                    }
                    last_sector = sector;
                }

                status = rc522_mifare_write(s_rc522, addr,
                                             st->blocks.data[addr], APP_STATE_BLOCK_SIZE);
                if (status != RC522_OK) {
                    shell_printf(ANSI_RED "  Block %02d: FAIL\r\n" ANSI_RESET, addr);
                } else {
                    shell_printf(ANSI_GREEN "  Block %02d: OK\r\n" ANSI_RESET, addr);
                    st->blocks.dirty[addr] = false;
                    written++;
                }
            }

            shell_printf(ANSI_GREEN "Wrote %d block(s).\r\n" ANSI_RESET, written);
            rc522_halt(s_rc522);
            rc522_stop_crypto1(s_rc522);
            return;
        }

        shell_puts("Cancelled.\r\n");
        return;
    }

    // --- Ultralight / NTAG ---
    if (APP_STATE_IS_UL(st)) {
        const rc522_picc_info_t *info = rc522_get_picc_info(st->picc_type);
        int total = info->total_pages;
        // Writable range: pages 4 through (total - 5), skip UID/lock and config
        int first_writable = 4;
        int last_writable = total - 5;
        if (last_writable < first_writable) {
            shell_puts("No writable pages.\r\n");
            return;
        }

        int write_count = 0;
        for (int p = first_writable; p <= last_writable; p++) {
            if (st->pages.read[p]) write_count++;
        }
        if (write_count == 0) {
            shell_puts("No pages to write.\r\n");
            return;
        }

        shell_printf("Will write %d page(s). Present target tag... (ESC to cancel)\r\n",
                     write_count);

        while (!shell_check_cancel()) {
            rc522_status_t status = rc522_is_new_card_present(s_rc522);
            if (status != RC522_OK) {
                vTaskDelay(pdMS_TO_TICKS(100));
                continue;
            }

            rc522_uid_t uid;
            status = rc522_read_card_uid(s_rc522, &uid);
            if (status != RC522_OK) {
                shell_puts(ANSI_RED "Failed to read UID.\r\n" ANSI_RESET);
                rc522_halt(s_rc522);
                rc522_stop_crypto1(s_rc522);
                continue;
            }

            shell_puts(ANSI_GREEN "Target detected! " ANSI_RESET "Writing...\r\n");

            int written = 0;
            for (int p = first_writable; p <= last_writable; p++) {
                if (!st->pages.read[p]) continue;

                status = rc522_ultralight_write(s_rc522, p, st->pages.data[p]);
                if (status != RC522_OK) {
                    shell_printf(ANSI_RED "  Page %3d: FAIL\r\n" ANSI_RESET, p);
                } else {
                    shell_printf(ANSI_GREEN "  Page %3d: OK\r\n" ANSI_RESET, p);
                    st->pages.dirty[p] = false;
                    written++;
                }
            }

            shell_printf(ANSI_GREEN "Wrote %d page(s).\r\n" ANSI_RESET, written);
            rc522_halt(s_rc522);
            rc522_stop_crypto1(s_rc522);
            return;
        }

        shell_puts("Cancelled.\r\n");
        return;
    }

    shell_puts(ANSI_RED "Write not supported for this tag type.\r\n" ANSI_RESET);
}

// ---------------------------------------------------------------------------
// Hex editor
// ---------------------------------------------------------------------------

static int hex_val(int c)
{
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'a' && c <= 'f') return c - 'a' + 10;
    if (c >= 'A' && c <= 'F') return c - 'A' + 10;
    return -1;
}

// ---------------------------------------------------------------------------
// Classic hex editor
// ---------------------------------------------------------------------------

static void cmd_edit_classic(void)
{
    app_state_t *st = app_state_get();
    int max_sectors = classic_sector_count(st);
    int total_blocks = (st->picc_type == RC522_PICC_TYPE_CLASSIC_4K) ? 256 : 64;

    static uint8_t edit_buf[APP_STATE_MAX_BLOCKS][APP_STATE_BLOCK_SIZE];
    static bool    dirty[APP_STATE_MAX_BLOCKS];
    memcpy(edit_buf, st->blocks.data, total_blocks * APP_STATE_BLOCK_SIZE);
    memset(dirty, 0, sizeof(dirty));

    int cur_sector = 0;
    int cur_row    = 0;  // row within sector display
    int cur_byte   = 0;
    int cur_nibble = 0;

    // Skip block 0 cursor landing
    if (cur_sector == 0) cur_row = 1;

    int sector_size = classic_sector_size(cur_sector);

    #define CL_EDITOR_LINES (1 + sector_size)

    #define CL_DRAW_EDITOR() do { \
        sector_size = classic_sector_size(cur_sector); \
        int _base = classic_sector_base(cur_sector); \
        shell_printf(ANSI_BOLD ANSI_CYAN \
            "Editing sector %d" ANSI_RESET \
            "  [/] switch sector  ESC quit  Enter save\r\n", cur_sector); \
        for (int _r = 0; _r < sector_size; _r++) { \
            int _addr = _base + _r; \
            shell_printf(ANSI_BOLD ANSI_WHITE "Block %02d: " ANSI_RESET, _addr); \
            bool _is_trailer = (_r == sector_size - 1); \
            bool _is_mfr = (_addr == 0); \
            if (_is_mfr) shell_puts(ANSI_GRAY); \
            else if (_is_trailer) shell_puts(ANSI_YELLOW); \
            if (!st->blocks.read[_addr]) { \
                for (int _i = 0; _i < APP_STATE_BLOCK_SIZE; _i++) { \
                    if (_r == cur_row && _i == cur_byte) { \
                        shell_puts(ANSI_RESET ANSI_INVERSE "-- " ANSI_RESET); \
                        if (_is_mfr) shell_puts(ANSI_GRAY); \
                        else if (_is_trailer) shell_puts(ANSI_YELLOW); \
                    } else { \
                        shell_puts("-- "); \
                    } \
                } \
            } else { \
                for (int _i = 0; _i < APP_STATE_BLOCK_SIZE; _i++) { \
                    if (_r == cur_row && _i == cur_byte) { \
                        char _hx[4]; \
                        snprintf(_hx, sizeof(_hx), "%02X ", edit_buf[_addr][_i]); \
                        if (cur_nibble == 0) { \
                            shell_puts(ANSI_RESET ANSI_INVERSE); \
                            shell_putchar(_hx[0]); \
                            shell_puts(ANSI_RESET); \
                            if (_is_mfr) shell_puts(ANSI_GRAY); \
                            else if (_is_trailer) shell_puts(ANSI_YELLOW); \
                            shell_putchar(_hx[1]); \
                            shell_putchar(' '); \
                        } else { \
                            shell_puts(ANSI_RESET); \
                            if (_is_mfr) shell_puts(ANSI_GRAY); \
                            else if (_is_trailer) shell_puts(ANSI_YELLOW); \
                            shell_putchar(_hx[0]); \
                            shell_puts(ANSI_RESET ANSI_INVERSE); \
                            shell_putchar(_hx[1]); \
                            shell_puts(ANSI_RESET); \
                            if (_is_mfr) shell_puts(ANSI_GRAY); \
                            else if (_is_trailer) shell_puts(ANSI_YELLOW); \
                            shell_putchar(' '); \
                        } \
                    } else { \
                        shell_printf("%02X ", edit_buf[_addr][_i]); \
                    } \
                } \
            } \
            shell_puts(ANSI_RESET "\r\n"); \
        } \
    } while(0)

    #define CL_MOVE_UP() do { \
        char _esc[16]; \
        snprintf(_esc, sizeof(_esc), "\033[%dA", CL_EDITOR_LINES); \
        shell_puts(_esc); \
        shell_puts("\r"); \
    } while(0)

    CL_DRAW_EDITOR();

    while (true) {
        int c = shell_getchar(portMAX_DELAY);
        if (c < 0) continue;

        if (c == 0x1B) {
            int c2 = shell_getchar(50);
            if (c2 < 0) {
                shell_puts(ANSI_YELLOW "Discarded.\r\n" ANSI_RESET);
                return;
            }
            if (c2 == '[') {
                int c3 = shell_getchar(50);
                if (c3 == 'A') { // Up
                    if (cur_row > 0) {
                        cur_row--;
                        int addr = classic_sector_base(cur_sector) + cur_row;
                        if (addr == 0) cur_row++;
                    }
                } else if (c3 == 'B') { // Down
                    if (cur_row < sector_size - 1) cur_row++;
                } else if (c3 == 'C') { // Right
                    cur_nibble++;
                    if (cur_nibble > 1) {
                        cur_nibble = 0;
                        cur_byte++;
                        if (cur_byte > 15) {
                            cur_byte = 0;
                            if (cur_row < sector_size - 1) {
                                cur_row++;
                            } else {
                                cur_byte = 15;
                                cur_nibble = 1;
                            }
                        }
                    }
                } else if (c3 == 'D') { // Left
                    cur_nibble--;
                    if (cur_nibble < 0) {
                        cur_nibble = 1;
                        cur_byte--;
                        if (cur_byte < 0) {
                            cur_byte = 0;
                            cur_nibble = 0;
                            if (cur_row > 0) {
                                cur_row--;
                                int addr = classic_sector_base(cur_sector) + cur_row;
                                if (addr == 0) {
                                    cur_row++;
                                } else {
                                    cur_byte = 15;
                                    cur_nibble = 1;
                                }
                            }
                        }
                    }
                }
                CL_MOVE_UP();
                CL_DRAW_EDITOR();
                continue;
            }
            continue;
        }

        // Enter - save to app_state
        if (c == '\r' || c == '\n') {
            for (int i = 0; i < total_blocks; i++) {
                if (dirty[i]) {
                    memcpy(st->blocks.data[i], edit_buf[i], APP_STATE_BLOCK_SIZE);
                    st->blocks.dirty[i] = true;
                }
            }
            shell_puts(ANSI_GREEN "Saved to RAM. Use 'write' to flash to card.\r\n" ANSI_RESET);
            return;
        }

        // [ / ] - switch sector
        if (c == '[') {
            if (cur_sector > 0) {
                CL_MOVE_UP();
                // Erase old lines (sector size may change for 4K)
                for (int i = 0; i < CL_EDITOR_LINES; i++)
                    shell_puts("\033[2K\r\n");
                for (int i = 0; i < CL_EDITOR_LINES; i++)
                    shell_puts("\033[A");
                shell_puts("\r");

                cur_sector--;
                sector_size = classic_sector_size(cur_sector);
                if (cur_row >= sector_size) cur_row = sector_size - 1;
                if (cur_sector == 0 && cur_row == 0) cur_row = 1;
                CL_DRAW_EDITOR();
            }
            continue;
        }
        if (c == ']') {
            if (cur_sector < max_sectors - 1) {
                CL_MOVE_UP();
                for (int i = 0; i < CL_EDITOR_LINES; i++)
                    shell_puts("\033[2K\r\n");
                for (int i = 0; i < CL_EDITOR_LINES; i++)
                    shell_puts("\033[A");
                shell_puts("\r");

                cur_sector++;
                sector_size = classic_sector_size(cur_sector);
                if (cur_row >= sector_size) cur_row = sector_size - 1;
                CL_DRAW_EDITOR();
            }
            continue;
        }

        // Hex digit input
        int hv = hex_val(c);
        if (hv >= 0) {
            int addr = classic_sector_base(cur_sector) + cur_row;
            if (addr == 0) continue;

            uint8_t val = edit_buf[addr][cur_byte];
            if (cur_nibble == 0) {
                val = (uint8_t)((hv << 4) | (val & 0x0F));
            } else {
                val = (uint8_t)((val & 0xF0) | hv);
            }
            edit_buf[addr][cur_byte] = val;
            dirty[addr] = true;

            cur_nibble++;
            if (cur_nibble > 1) {
                cur_nibble = 0;
                cur_byte++;
                if (cur_byte > 15) {
                    cur_byte = 0;
                    if (cur_row < sector_size - 1) {
                        cur_row++;
                    } else {
                        cur_byte = 15;
                        cur_nibble = 1;
                    }
                }
            }

            CL_MOVE_UP();
            CL_DRAW_EDITOR();
            continue;
        }
    }

    #undef CL_EDITOR_LINES
    #undef CL_DRAW_EDITOR
    #undef CL_MOVE_UP
}

// ---------------------------------------------------------------------------
// UL/NTAG page-based hex editor
// ---------------------------------------------------------------------------

static void cmd_edit_ul(void)
{
    app_state_t *st = app_state_get();
    const rc522_picc_info_t *info = rc522_get_picc_info(st->picc_type);
    int total_pages = info->total_pages;
    int max_groups = (total_pages + 15) / 16;

    static uint8_t edit_buf[APP_STATE_MAX_PAGES][APP_STATE_PAGE_SIZE];
    static bool    dirty[APP_STATE_MAX_PAGES];
    memcpy(edit_buf, st->pages.data, total_pages * APP_STATE_PAGE_SIZE);
    memset(dirty, 0, sizeof(dirty));

    int cur_group  = 0;
    int cur_row    = 4;  // start at page 4 (skip UID/lock pages 0-3)
    int cur_byte   = 0;
    int cur_nibble = 0;

    // Clamp: if first group has fewer than 5 pages, start at what's available
    if (cur_row >= 16 || cur_row >= total_pages) cur_row = 0;

    #define UL_PAGE_COUNT(grp) ({ \
        int _start = (grp) * 16; \
        int _end = _start + 16; \
        if (_end > total_pages) _end = total_pages; \
        _end - _start; \
    })

    #define UL_EDITOR_LINES (1 + UL_PAGE_COUNT(cur_group))

    #define UL_DRAW_EDITOR() do { \
        int _start = cur_group * 16; \
        int _count = UL_PAGE_COUNT(cur_group); \
        shell_printf(ANSI_BOLD ANSI_CYAN \
            "Editing pages %d-%d" ANSI_RESET \
            "  [/] switch group  ESC quit  Enter save\r\n", \
            _start, _start + _count - 1); \
        for (int _r = 0; _r < _count; _r++) { \
            int _p = _start + _r; \
            shell_printf(ANSI_BOLD ANSI_WHITE "Page %3d: " ANSI_RESET, _p); \
            bool _is_uid = (_p <= 3); \
            bool _is_cfg = (_p >= total_pages - 5); \
            if (_is_uid) shell_puts(ANSI_GRAY); \
            else if (_is_cfg) shell_puts(ANSI_YELLOW); \
            if (!st->pages.read[_p]) { \
                for (int _i = 0; _i < APP_STATE_PAGE_SIZE; _i++) { \
                    if (_r == cur_row && _i == cur_byte) { \
                        shell_puts(ANSI_RESET ANSI_INVERSE "-- " ANSI_RESET); \
                        if (_is_uid) shell_puts(ANSI_GRAY); \
                        else if (_is_cfg) shell_puts(ANSI_YELLOW); \
                    } else { \
                        shell_puts("-- "); \
                    } \
                } \
            } else { \
                for (int _i = 0; _i < APP_STATE_PAGE_SIZE; _i++) { \
                    if (_r == cur_row && _i == cur_byte) { \
                        char _hx[4]; \
                        snprintf(_hx, sizeof(_hx), "%02X ", edit_buf[_p][_i]); \
                        if (cur_nibble == 0) { \
                            shell_puts(ANSI_RESET ANSI_INVERSE); \
                            shell_putchar(_hx[0]); \
                            shell_puts(ANSI_RESET); \
                            if (_is_uid) shell_puts(ANSI_GRAY); \
                            else if (_is_cfg) shell_puts(ANSI_YELLOW); \
                            shell_putchar(_hx[1]); \
                            shell_putchar(' '); \
                        } else { \
                            shell_puts(ANSI_RESET); \
                            if (_is_uid) shell_puts(ANSI_GRAY); \
                            else if (_is_cfg) shell_puts(ANSI_YELLOW); \
                            shell_putchar(_hx[0]); \
                            shell_puts(ANSI_RESET ANSI_INVERSE); \
                            shell_putchar(_hx[1]); \
                            shell_puts(ANSI_RESET); \
                            if (_is_uid) shell_puts(ANSI_GRAY); \
                            else if (_is_cfg) shell_puts(ANSI_YELLOW); \
                            shell_putchar(' '); \
                        } \
                    } else { \
                        shell_printf("%02X ", edit_buf[_p][_i]); \
                    } \
                } \
            } \
            shell_puts(ANSI_RESET "\r\n"); \
        } \
    } while(0)

    #define UL_MOVE_UP() do { \
        char _esc[16]; \
        snprintf(_esc, sizeof(_esc), "\033[%dA", UL_EDITOR_LINES); \
        shell_puts(_esc); \
        shell_puts("\r"); \
    } while(0)

    UL_DRAW_EDITOR();

    while (true) {
        int c = shell_getchar(portMAX_DELAY);
        if (c < 0) continue;

        int page_count = UL_PAGE_COUNT(cur_group);

        if (c == 0x1B) {
            int c2 = shell_getchar(50);
            if (c2 < 0) {
                shell_puts(ANSI_YELLOW "Discarded.\r\n" ANSI_RESET);
                return;
            }
            if (c2 == '[') {
                int c3 = shell_getchar(50);
                if (c3 == 'A') { // Up
                    if (cur_row > 0) {
                        cur_row--;
                        // Skip UID/lock pages 0-3
                        int p = cur_group * 16 + cur_row;
                        if (p <= 3) cur_row++;
                    }
                } else if (c3 == 'B') { // Down
                    if (cur_row < page_count - 1) cur_row++;
                } else if (c3 == 'C') { // Right
                    cur_nibble++;
                    if (cur_nibble > 1) {
                        cur_nibble = 0;
                        cur_byte++;
                        if (cur_byte > 3) {
                            cur_byte = 0;
                            if (cur_row < page_count - 1) {
                                cur_row++;
                            } else {
                                cur_byte = 3;
                                cur_nibble = 1;
                            }
                        }
                    }
                } else if (c3 == 'D') { // Left
                    cur_nibble--;
                    if (cur_nibble < 0) {
                        cur_nibble = 1;
                        cur_byte--;
                        if (cur_byte < 0) {
                            cur_byte = 0;
                            cur_nibble = 0;
                            if (cur_row > 0) {
                                cur_row--;
                                int p = cur_group * 16 + cur_row;
                                if (p <= 3) {
                                    cur_row++;
                                } else {
                                    cur_byte = 3;
                                    cur_nibble = 1;
                                }
                            }
                        }
                    }
                }
                UL_MOVE_UP();
                UL_DRAW_EDITOR();
                continue;
            }
            continue;
        }

        // Enter - save
        if (c == '\r' || c == '\n') {
            for (int i = 0; i < total_pages; i++) {
                if (dirty[i]) {
                    memcpy(st->pages.data[i], edit_buf[i], APP_STATE_PAGE_SIZE);
                    st->pages.dirty[i] = true;
                }
            }
            shell_puts(ANSI_GREEN "Saved to RAM. Use 'write' to flash to card.\r\n" ANSI_RESET);
            return;
        }

        // [ / ] - switch page group
        if (c == '[') {
            if (cur_group > 0) {
                UL_MOVE_UP();
                for (int i = 0; i < UL_EDITOR_LINES; i++)
                    shell_puts("\033[2K\r\n");
                for (int i = 0; i < UL_EDITOR_LINES; i++)
                    shell_puts("\033[A");
                shell_puts("\r");

                cur_group--;
                int new_count = UL_PAGE_COUNT(cur_group);
                if (cur_row >= new_count) cur_row = new_count - 1;
                // Skip UID/lock pages
                int p = cur_group * 16 + cur_row;
                if (p <= 3) cur_row = 4 - cur_group * 16;
                if (cur_row < 0) cur_row = 0;
                UL_DRAW_EDITOR();
            }
            continue;
        }
        if (c == ']') {
            if (cur_group < max_groups - 1) {
                UL_MOVE_UP();
                for (int i = 0; i < UL_EDITOR_LINES; i++)
                    shell_puts("\033[2K\r\n");
                for (int i = 0; i < UL_EDITOR_LINES; i++)
                    shell_puts("\033[A");
                shell_puts("\r");

                cur_group++;
                int new_count = UL_PAGE_COUNT(cur_group);
                if (cur_row >= new_count) cur_row = new_count - 1;
                UL_DRAW_EDITOR();
            }
            continue;
        }

        // Hex digit input
        int hv = hex_val(c);
        if (hv >= 0) {
            int p = cur_group * 16 + cur_row;
            // Skip UID/lock pages 0-3
            if (p <= 3) continue;

            uint8_t val = edit_buf[p][cur_byte];
            if (cur_nibble == 0) {
                val = (uint8_t)((hv << 4) | (val & 0x0F));
            } else {
                val = (uint8_t)((val & 0xF0) | hv);
            }
            edit_buf[p][cur_byte] = val;
            dirty[p] = true;

            cur_nibble++;
            if (cur_nibble > 1) {
                cur_nibble = 0;
                cur_byte++;
                if (cur_byte > 3) {
                    cur_byte = 0;
                    if (cur_row < page_count - 1) {
                        cur_row++;
                    } else {
                        cur_byte = 3;
                        cur_nibble = 1;
                    }
                }
            }

            UL_MOVE_UP();
            UL_DRAW_EDITOR();
            continue;
        }
    }

    #undef UL_PAGE_COUNT
    #undef UL_EDITOR_LINES
    #undef UL_DRAW_EDITOR
    #undef UL_MOVE_UP
}

static void cmd_edit(void)
{
    app_state_t *st = app_state_get();
    if (!st->has_tag) {
        shell_puts(ANSI_RED "No tag in memory. Use 'read' first.\r\n" ANSI_RESET);
        return;
    }

    if (APP_STATE_IS_CLASSIC(st)) {
        cmd_edit_classic();
    } else if (APP_STATE_IS_UL(st)) {
        cmd_edit_ul();
    } else {
        shell_puts(ANSI_RED "Edit not supported for this tag type.\r\n" ANSI_RESET);
    }
}

// ---------------------------------------------------------------------------
// Other commands
// ---------------------------------------------------------------------------

static void cmd_clear(void)
{
    app_state_clear();
    shell_puts(ANSI_GREEN "State cleared.\r\n" ANSI_RESET);
}

static void cmd_reboot(void)
{
    shell_puts("Rebooting...\r\n");
    vTaskDelay(pdMS_TO_TICKS(200));
    esp_restart();
}

static void cmd_help(void)
{
    shell_puts(ANSI_BOLD "Commands:\r\n" ANSI_RESET);
    shell_puts("  " ANSI_CYAN "info" ANSI_RESET "   [-sN|-pN]  Device info + sector/page display\r\n");
    shell_puts("  " ANSI_CYAN "read" ANSI_RESET "             Read tag (auto-detects type)\r\n");
    shell_puts("  " ANSI_CYAN "edit" ANSI_RESET "             Hex editor for tag data\r\n");
    shell_puts("  " ANSI_CYAN "write" ANSI_RESET "  [-f]      Write blocks/pages to tag (-f includes trailers)\r\n");
    shell_puts("  " ANSI_CYAN "clear" ANSI_RESET "            Clear stored tag\r\n");
    shell_puts("  " ANSI_CYAN "reboot" ANSI_RESET "           Restart device\r\n");
    shell_puts("  " ANSI_CYAN "help" ANSI_RESET "             This message\r\n");
}

// ---------------------------------------------------------------------------
// Main shell loop
// ---------------------------------------------------------------------------

void shell_run(rc522_handle_t *rc522)
{
    s_rc522 = rc522;
    shell_uart_init();

    char cmd[CMD_BUF_SIZE];

    shell_puts("\r\n");

    while (true) {
        shell_puts(PROMPT);
        shell_readline(cmd, sizeof(cmd));

        // Trim leading/trailing whitespace
        char *p = cmd;
        while (*p && isspace((unsigned char)*p)) p++;
        char *end = p + strlen(p) - 1;
        while (end > p && isspace((unsigned char)*end)) *end-- = '\0';

        if (*p == '\0') continue;

        // Parse command and arguments
        char *arg = p;
        while (*arg && !isspace((unsigned char)*arg)) arg++;
        if (*arg) {
            *arg = '\0';
            arg++;
            while (*arg && isspace((unsigned char)*arg)) arg++;
        }

        if (strcmp(p, "info") == 0)        cmd_info(arg);
        else if (strcmp(p, "read") == 0)   cmd_read();
        else if (strcmp(p, "edit") == 0)   cmd_edit();
        else if (strcmp(p, "write") == 0)  cmd_write(arg);
        else if (strcmp(p, "clear") == 0)  cmd_clear();
        else if (strcmp(p, "reboot") == 0) cmd_reboot();
        else if (strcmp(p, "help") == 0)   cmd_help();
        else {
            shell_printf(ANSI_RED "Unknown command: '%s'. Type 'help'.\r\n" ANSI_RESET, p);
        }
    }
}
