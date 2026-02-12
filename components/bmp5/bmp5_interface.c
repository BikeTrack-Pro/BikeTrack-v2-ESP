#include "bmp5_interface.h"

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_rom_sys.h>
#include <string.h>

static const int I2C_TIMEOUT_MS = 1000;


BMP5_INTF_RET_TYPE bmp5_i2c_read(const uint8_t reg_addr, uint8_t *reg_data, const uint32_t len, void *intf_ptr) {
    if (intf_ptr == NULL || reg_data == NULL) return BMP5_E_NULL_PTR;

    const esp_err_t err = i2c_master_transmit_receive(intf_ptr, &reg_addr, 1, reg_data, len, I2C_TIMEOUT_MS);
    return (err == ESP_OK) ? BMP5_OK : BMP5_E_COM_FAIL;
}

BMP5_INTF_RET_TYPE bmp5_i2c_write(const uint8_t reg_addr, const uint8_t *reg_data, const uint32_t len, void *intf_ptr) {
    if (intf_ptr == NULL || reg_data == NULL) return BMP5_E_NULL_PTR;
    if (len == 0) return BMP5_E_NULL_PTR;

    uint8_t buffer[len + 1];
    buffer[0] = reg_addr;
    memcpy(&buffer[1], reg_data, len);

    const esp_err_t err = i2c_master_transmit(intf_ptr, buffer, len + 1, I2C_TIMEOUT_MS);
    return (err == ESP_OK) ? BMP5_OK : BMP5_E_COM_FAIL;
}

void bmp5_delay_us(const uint32_t period, void *intf_ptr) {
    if (period >= 1000) {
        vTaskDelay(pdMS_TO_TICKS(period / 1000));
    } else if (period > 0) {
        esp_rom_delay_us(period);
    }
}
