idf_component_register(
    SRCS "main.c" "shell.c" "app_state.c"
    INCLUDE_DIRS "."
    REQUIRES rc522 driver nvs_flash esp_timer spi_flash
)
