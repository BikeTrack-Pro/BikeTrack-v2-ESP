#include "bmp5_esp32_interface.h"

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_rom_sys.h>
#include <string.h>

#define I2C_TIMEOUT_MS 1000


BMP5_INTF_RET_TYPE bmp5_i2c_read(const uint8_t reg_addr, uint8_t *reg_data, const uint32_t len, void *intf_ptr) {
    if (intf_ptr == NULL || reg_data == NULL) return BMP5_E_NULL_PTR;

    // Cast des intf_ptr auf das neue Driver-Handle
    i2c_master_dev_handle_t dev_handle = (i2c_master_dev_handle_t) intf_ptr;

    const esp_err_t err = i2c_master_transmit_receive(dev_handle, &reg_addr, 1, reg_data, len, I2C_TIMEOUT_MS);
    return (err == ESP_OK) ? BMP5_OK : BMP5_E_COM_FAIL;
}


BMP5_INTF_RET_TYPE bmp5_i2c_write(const uint8_t reg_addr, const uint8_t *reg_data, const uint32_t len, void *intf_ptr) {
    if (intf_ptr == NULL || reg_data == NULL) return BMP5_E_NULL_PTR;

    i2c_master_dev_handle_t dev_handle = (i2c_master_dev_handle_t) intf_ptr;

    // Effizienz: Wir kombinieren Reg-Adresse und Daten, um einen einzigen I2C-Transfer zu nutzen.
    // Da BMP5-Register-Writes meist klein sind, ist ein Stack-Buffer am schnellsten.
    uint8_t buffer[len + 1];
    buffer[0] = reg_addr;
    memcpy(&buffer[1], reg_data, len);

    const esp_err_t err = i2c_master_transmit(dev_handle, buffer, len + 1, I2C_TIMEOUT_MS);
    return (err == ESP_OK) ? BMP5_OK : BMP5_E_COM_FAIL;
}


void bmp5_delay_us(const uint32_t period, void *intf_ptr) {
    (void) intf_ptr; // Nicht benötigt für Delay

    if (period >= 1000) {
        // Bei langen Delays FreeRTOS den Vortritt lassen (CPU-schonend)
        vTaskDelay(pdMS_TO_TICKS(period / 1000));
    } else if (period > 0) {
        // Bei sehr kurzen Delays präzises Busy-Waiting
        esp_rom_delay_us(period);
    }
}
