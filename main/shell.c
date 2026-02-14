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
    char buf[256];
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

// Check for cancel key (ESC or space) without blocking
static bool shell_check_cancel(void)
{
    int c = shell_getchar(0);
    return (c == 0x1B);
}

// ---------------------------------------------------------------------------
// Shared display helper
// ---------------------------------------------------------------------------

static void print_sector(const app_state_t *st, int sector)
{
    int base = sector * APP_STATE_BLOCKS_PER_SECTOR;
    for (int r = 0; r < APP_STATE_BLOCKS_PER_SECTOR; r++) {
        int addr = base + r;
        // Block number in bold white
        shell_printf(ANSI_BOLD ANSI_WHITE "  Block %02d: " ANSI_RESET, addr);

        if (!st->block_read[addr]) {
            // Unread block — gray dashes
            shell_puts(ANSI_GRAY);
            for (int i = 0; i < APP_STATE_BLOCK_SIZE; i++)
                shell_puts("-- ");
            shell_puts(ANSI_RESET "\r\n");
            continue;
        }

        // Pick color for the hex line
        if (addr == 0) {
            shell_puts(ANSI_GRAY);           // manufacturer block
        } else if (r == 3) {
            shell_puts(ANSI_YELLOW);          // sector trailer
        }

        for (int i = 0; i < APP_STATE_BLOCK_SIZE; i++)
            shell_printf("%02X ", st->data[addr][i]);

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

    // Print UID
    shell_puts(ANSI_CYAN "Tag UID:  " ANSI_RESET);
    for (int i = 0; i < st->uid.len; i++) {
        shell_printf("%02X", st->uid.bytes[i]);
        if (i < st->uid.len - 1) shell_putchar(':');
    }
    shell_printf("  SAK: 0x%02X\r\n", st->uid.sak);

    // Parse -sN argument
    int sector = 0; // default
    if (args && args[0]) {
        const char *p = args;
        while (*p && isspace((unsigned char)*p)) p++;
        if (p[0] == '-' && p[1] == 's') {
            p += 2;
            while (*p && isspace((unsigned char)*p)) p++;
            int s = atoi(p);
            if (s >= 0 && s < APP_STATE_SECTORS)
                sector = s;
            else {
                shell_printf(ANSI_RED "Invalid sector %d (0-%d)\r\n" ANSI_RESET,
                             s, APP_STATE_SECTORS - 1);
                return;
            }
        }
    }

    shell_printf("\r\n  Sector %d:\r\n", sector);
    print_sector(st, sector);
}

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

        shell_puts(ANSI_GREEN "Card detected! " ANSI_RESET "UID: ");
        for (int i = 0; i < uid.len; i++) {
            shell_printf("%02X", uid.bytes[i]);
            if (i < uid.len - 1) shell_putchar(':');
        }
        shell_puts("\r\n");

        // Read all 16 sectors (64 blocks)
        app_state_t *st = app_state_get();
        st->uid = uid;
        int read_count = 0;
        int fail_count = 0;

        for (int sector = 0; sector < APP_STATE_SECTORS; sector++) {
            int base_block = sector * APP_STATE_BLOCKS_PER_SECTOR;

            status = rc522_mifare_auth(s_rc522, RC522_AUTH_KEY_A,
                                        base_block, DEFAULT_KEY, &uid);
            if (status != RC522_OK) {
                shell_printf(ANSI_RED "  Sector %d: auth failed\r\n" ANSI_RESET, sector);
                fail_count += APP_STATE_BLOCKS_PER_SECTOR;
                continue;
            }

            for (int b = 0; b < APP_STATE_BLOCKS_PER_SECTOR; b++) {
                int addr = base_block + b;
                uint8_t buf[18];
                uint8_t buf_len = sizeof(buf);
                status = rc522_mifare_read(s_rc522, addr, buf, &buf_len);
                if (status != RC522_OK) {
                    shell_printf(ANSI_RED "  Block %02d: read failed\r\n" ANSI_RESET, addr);
                    fail_count++;
                    continue;
                }
                memcpy(st->data[addr], buf, APP_STATE_BLOCK_SIZE);
                st->block_read[addr] = true;
                st->block_dirty[addr] = false;
                read_count++;
            }
        }

        st->has_tag = true;
        shell_printf(ANSI_GREEN "Read %d block(s)" ANSI_RESET, read_count);
        if (fail_count > 0)
            shell_printf(", " ANSI_RED "%d failed" ANSI_RESET, fail_count);
        shell_puts(".\r\n");

        rc522_halt(s_rc522);
        rc522_stop_crypto1(s_rc522);
        return;
    }

    shell_puts("Cancelled.\r\n");
}

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

    // Count writable blocks (read + skip block 0, skip trailers unless -f)
    int write_count = 0;
    for (int i = 1; i < APP_STATE_TOTAL_BLOCKS; i++) {
        if (!st->block_read[i]) continue;
        if (!force && (i % APP_STATE_BLOCKS_PER_SECTOR == 3)) continue;
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

        for (int addr = 1; addr < APP_STATE_TOTAL_BLOCKS; addr++) {
            if (!st->block_read[addr]) continue;
            // Skip trailers unless force mode
            if (!force && (addr % APP_STATE_BLOCKS_PER_SECTOR == 3)) {
                shell_printf(ANSI_YELLOW "  Block %02d: SKIP (trailer)\r\n" ANSI_RESET, addr);
                continue;
            }

            int sector = addr / APP_STATE_BLOCKS_PER_SECTOR;

            // Auth per sector
            if (sector != last_sector) {
                int sector_base = sector * APP_STATE_BLOCKS_PER_SECTOR;
                status = rc522_mifare_auth(s_rc522, RC522_AUTH_KEY_A,
                                            sector_base, DEFAULT_KEY, &uid);
                if (status != RC522_OK) {
                    shell_printf(ANSI_RED "  Auth failed for sector %d\r\n" ANSI_RESET, sector);
                    break;
                }
                last_sector = sector;
            }

            status = rc522_mifare_write(s_rc522, addr,
                                         st->data[addr], APP_STATE_BLOCK_SIZE);
            if (status != RC522_OK) {
                shell_printf(ANSI_RED "  Block %02d: FAIL\r\n" ANSI_RESET, addr);
            } else {
                shell_printf(ANSI_GREEN "  Block %02d: OK\r\n" ANSI_RESET, addr);
                st->block_dirty[addr] = false;
                written++;
            }
        }

        shell_printf(ANSI_GREEN "Wrote %d block(s).\r\n" ANSI_RESET, written);

        rc522_halt(s_rc522);
        rc522_stop_crypto1(s_rc522);
        return;
    }

    shell_puts("Cancelled.\r\n");
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

static void cmd_edit(void)
{
    app_state_t *st = app_state_get();
    if (!st->has_tag) {
        shell_puts(ANSI_RED "No tag in memory. Use 'read' first.\r\n" ANSI_RESET);
        return;
    }

    // Work on a local copy so ESC can discard
    uint8_t edit_buf[APP_STATE_TOTAL_BLOCKS][APP_STATE_BLOCK_SIZE];
    bool    dirty[APP_STATE_TOTAL_BLOCKS];
    memcpy(edit_buf, st->data, sizeof(edit_buf));
    memset(dirty, 0, sizeof(dirty));

    int cur_sector = 0;
    int cur_block  = 0;  // 0-3 within sector
    int cur_byte   = 0;  // 0-15
    int cur_nibble = 0;  // 0=high, 1=low

    // Skip block 0 cursor landing if sector 0
    if (cur_sector == 0) cur_block = 1;

    // -- Draw functions (inline via macros for sector redraw) --
    // Header + 4 block lines = 6 lines total (header, blank, 4 data)

    #define EDITOR_LINES 5  // header + 4 block lines

    // Draw the editor
    #define DRAW_EDITOR() do { \
        int _base = cur_sector * APP_STATE_BLOCKS_PER_SECTOR; \
        shell_printf(ANSI_BOLD ANSI_CYAN \
            "Editing sector %d" ANSI_RESET \
            "  [/] switch sector  ESC quit  Enter save\r\n", cur_sector); \
        for (int _r = 0; _r < APP_STATE_BLOCKS_PER_SECTOR; _r++) { \
            int _addr = _base + _r; \
            shell_printf(ANSI_BOLD ANSI_WHITE "Block %02d: " ANSI_RESET, _addr); \
            bool _is_trailer = (_r == 3); \
            bool _is_mfr = (_addr == 0); \
            if (_is_mfr) shell_puts(ANSI_GRAY); \
            else if (_is_trailer) shell_puts(ANSI_YELLOW); \
            if (!st->block_read[_addr]) { \
                for (int _i = 0; _i < APP_STATE_BLOCK_SIZE; _i++) { \
                    if (_r == cur_block && _i == cur_byte) { \
                        shell_puts(ANSI_RESET ANSI_INVERSE "-- " ANSI_RESET); \
                        if (_is_mfr) shell_puts(ANSI_GRAY); \
                        else if (_is_trailer) shell_puts(ANSI_YELLOW); \
                    } else { \
                        shell_puts("-- "); \
                    } \
                } \
            } else { \
                for (int _i = 0; _i < APP_STATE_BLOCK_SIZE; _i++) { \
                    if (_r == cur_block && _i == cur_byte) { \
                        char _hx[4]; \
                        snprintf(_hx, sizeof(_hx), "%02X ", edit_buf[_addr][_i]); \
                        /* highlight the active nibble */ \
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

    // Move cursor up to redraw in place
    #define MOVE_UP() do { \
        char _esc[12]; \
        snprintf(_esc, sizeof(_esc), "\033[%dA", EDITOR_LINES); \
        shell_puts(_esc); \
        shell_puts("\r"); \
    } while(0)

    // Initial draw
    DRAW_EDITOR();

    // Input loop
    while (true) {
        int c = shell_getchar(portMAX_DELAY);
        if (c < 0) continue;

        // ESC - could be escape key or start of arrow sequence
        if (c == 0x1B) {
            int c2 = shell_getchar(50);
            if (c2 < 0) {
                // Plain ESC - discard and exit
                shell_puts(ANSI_YELLOW "Discarded.\r\n" ANSI_RESET);
                return;
            }
            if (c2 == '[') {
                int c3 = shell_getchar(50);
                if (c3 == 'A') { // Up
                    if (cur_block > 0) {
                        cur_block--;
                        // Skip block 0 (manufacturer)
                        int addr = cur_sector * APP_STATE_BLOCKS_PER_SECTOR + cur_block;
                        if (addr == 0) cur_block++;
                    }
                } else if (c3 == 'B') { // Down
                    if (cur_block < 3) cur_block++;
                } else if (c3 == 'C') { // Right
                    cur_nibble++;
                    if (cur_nibble > 1) {
                        cur_nibble = 0;
                        cur_byte++;
                        if (cur_byte > 15) {
                            cur_byte = 0;
                            if (cur_block < 3) {
                                cur_block++;
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
                            if (cur_block > 0) {
                                cur_block--;
                                int addr = cur_sector * APP_STATE_BLOCKS_PER_SECTOR + cur_block;
                                if (addr == 0) {
                                    cur_block++;
                                } else {
                                    cur_byte = 15;
                                    cur_nibble = 1;
                                }
                            }
                        }
                    }
                }
                // Redraw
                MOVE_UP();
                DRAW_EDITOR();
                continue;
            }
            continue;
        }

        // Enter - save to app_state
        if (c == '\r' || c == '\n') {
            for (int i = 0; i < APP_STATE_TOTAL_BLOCKS; i++) {
                if (dirty[i]) {
                    memcpy(st->data[i], edit_buf[i], APP_STATE_BLOCK_SIZE);
                    st->block_dirty[i] = true;
                }
            }
            shell_puts(ANSI_GREEN "Saved to RAM. Use 'write' to flash to card.\r\n" ANSI_RESET);
            return;
        }

        // [ / ] - switch sector
        if (c == '[') {
            if (cur_sector > 0) {
                cur_sector--;
                // Keep cursor position but clamp
                if (cur_sector == 0 && cur_block == 0) cur_block = 1;
                MOVE_UP();
                DRAW_EDITOR();
            }
            continue;
        }
        if (c == ']') {
            if (cur_sector < APP_STATE_SECTORS - 1) {
                cur_sector++;
                MOVE_UP();
                DRAW_EDITOR();
            }
            continue;
        }

        // Hex digit input
        int hv = hex_val(c);
        if (hv >= 0) {
            int addr = cur_sector * APP_STATE_BLOCKS_PER_SECTOR + cur_block;

            // Block 0 is not editable
            if (addr == 0) continue;

            // Warn on sector trailer
            if (cur_block == 3 && cur_byte == 0 && cur_nibble == 0) {
                // Show warning once at start of trailer edit
                // (displayed below editor area, won't persist)
            }

            uint8_t val = edit_buf[addr][cur_byte];
            if (cur_nibble == 0) {
                val = (uint8_t)((hv << 4) | (val & 0x0F));
            } else {
                val = (uint8_t)((val & 0xF0) | hv);
            }
            edit_buf[addr][cur_byte] = val;
            dirty[addr] = true;

            // Advance cursor
            cur_nibble++;
            if (cur_nibble > 1) {
                cur_nibble = 0;
                cur_byte++;
                if (cur_byte > 15) {
                    cur_byte = 0;
                    if (cur_block < 3) {
                        cur_block++;
                    } else {
                        cur_byte = 15;
                        cur_nibble = 1;
                    }
                }
            }

            MOVE_UP();
            DRAW_EDITOR();
            continue;
        }
    }

    #undef EDITOR_LINES
    #undef DRAW_EDITOR
    #undef MOVE_UP
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
    shell_puts("  " ANSI_CYAN "info" ANSI_RESET "   [-sN]  Device info + sector display\r\n");
    shell_puts("  " ANSI_CYAN "read" ANSI_RESET "          Read tag (sector 1)\r\n");
    shell_puts("  " ANSI_CYAN "edit" ANSI_RESET "          Hex editor for tag data\r\n");
    shell_puts("  " ANSI_CYAN "write" ANSI_RESET "  [-f]   Write all blocks to tag (-f includes trailers)\r\n");
    shell_puts("  " ANSI_CYAN "clear" ANSI_RESET "         Clear stored tag\r\n");
    shell_puts("  " ANSI_CYAN "reboot" ANSI_RESET "        Restart device\r\n");
    shell_puts("  " ANSI_CYAN "help" ANSI_RESET "          This message\r\n");
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
