#ifndef BMP5_ESP32_INTERFACE_H
#define BMP5_ESP32_INTERFACE_H

#include <bmp5_defs.h>
#include "driver/i2c_master.h"


/**
 * @brief I2C Lese-Funktion für Bosch BMP5 API
 */
BMP5_INTF_RET_TYPE bmp5_i2c_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr);

/**
 * @brief I2C Schreib-Funktion für Bosch BMP5 API
 */
BMP5_INTF_RET_TYPE bmp5_i2c_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len, void *intf_ptr);

/**
 * @brief Delay-Funktion in Mikrosekunden
 */
void bmp5_delay_us(uint32_t period, void *intf_ptr);

#endif // BMP5_ESP32_INTERFACE_H
