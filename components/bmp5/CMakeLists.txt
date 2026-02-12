idf_component_register(
        SRCS "BMP5_SensorAPI/bmp5.c" "bmp5_esp32_interface.c"
        INCLUDE_DIRS "BMP5_SensorAPI" "."
        PRIV_REQUIRES driver
)