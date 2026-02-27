idf_component_register(
        SRCS "main.c" "ble.c" "ble_gatt_svr.c" "i2c.c"
        INCLUDE_DIRS "../includes"
        PRIV_REQUIRES lsm6dso32 bmp5
        REQUIRES driver esp_timer freertos log nvs_flash bt
)
