#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs_flash.h"
#include "esp_system.h"
#include "esp_chip_info.h"
#include "esp_flash.h"
#include "esp_log.h"

#include "rc522.h"
#include "app_state.h"
#include "shell.h"

static const char *TAG = "main";

static const char *reset_reason_str(esp_reset_reason_t reason)
{
    switch (reason) {
        case ESP_RST_POWERON:  return "Power-on";
        case ESP_RST_SW:       return "Software";
        case ESP_RST_PANIC:    return "Panic";
        case ESP_RST_INT_WDT:  return "Interrupt watchdog";
        case ESP_RST_TASK_WDT: return "Task watchdog";
        case ESP_RST_WDT:      return "Other watchdog";
        case ESP_RST_DEEPSLEEP:return "Deep sleep";
        case ESP_RST_BROWNOUT: return "Brownout";
        default:               return "Unknown";
    }
}

void app_main(void)
{
    printf("\r\n");
    printf("========================================\r\n");
    printf("  TABLER - RFID Tag Manager\r\n");
    printf("========================================\r\n");
    printf("\r\n");

    // Device info
    esp_chip_info_t chip;
    esp_chip_info(&chip);
    uint32_t flash_size = 0;
    esp_flash_get_size(NULL, &flash_size);

    printf("Chip:    ESP32 rev %d.%d, %d core(s)\r\n",
           chip.revision / 100, chip.revision % 100, chip.cores);
    printf("Flash:   %lu KB\r\n", (unsigned long)(flash_size / 1024));
    printf("Heap:    %lu bytes free\r\n", (unsigned long)esp_get_free_heap_size());
    printf("IDF:     %s\r\n", esp_get_idf_version());
    printf("Reset:   %s\r\n", reset_reason_str(esp_reset_reason()));
    printf("\r\n");

    // Init NVS
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        nvs_flash_erase();
        nvs_flash_init();
    }

    // Init RC522
    rc522_config_t rc522_cfg = RC522_CONFIG_DEFAULT();
    rc522_handle_t rc522;

    rc522_status_t rc_status = rc522_init(&rc522_cfg, &rc522);
    if (rc_status != RC522_OK) {
        ESP_LOGE(TAG, "RC522 init failed (status=%d)", rc_status);
        printf("RC522 init FAILED. Check wiring.\r\n");
        printf("Continuing without reader...\r\n\r\n");
    } else {
        uint8_t fw = rc522_firmware_version(&rc522);
        printf("RC522:   Firmware 0x%02X %s\r\n\r\n",
               fw, (fw == 0x91) ? "(v1.0)" :
                   (fw == 0x92) ? "(v2.0)" : "(unknown)");
    }

    // Init app state
    app_state_init();

    // Enter shell
    shell_run(&rc522);
}
