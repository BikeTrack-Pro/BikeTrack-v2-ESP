idf_component_register(
        SRCS "main.c"
        INCLUDE_DIRS "."
        PRIV_REQUIRES lsm6dso32
        REQUIRES driver esp_timer freertos log
)
