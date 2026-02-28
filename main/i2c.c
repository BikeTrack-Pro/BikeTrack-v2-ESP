#include <esp_log.h>

#include "i2c.h"


static const char *TAG = "[I2C]";

static i2c_master_bus_handle_t i2c_bus_h = NULL;


/**
 * @brief Initializes the I2C manager by configuring and creating a new I2C master bus.
 *
 * This function checks if the I2C bus has already been initialized. If not, it configures
 * the I2C master bus with predefined settings, including clock source, GPIO pins for
 * SCL and SDA, the I2C port number, and glitch ignore count. It then creates the I2C bus
 * and initializes the bus handle.
 *
 * @return
 *     - ESP_OK: If the I2C manager is successfully initialized or already initialized.
 *     - An error code of type esp_err_t: If initialization fails.
 */
esp_err_t init_i2c_bus(void) {
    if (i2c_bus_h != NULL) return ESP_OK;

    const i2c_master_bus_config_t bus_config = {
        .flags.enable_internal_pullup = true, // TODO: Use external Pull-Ups on PCB
        .clk_source = I2C_MASTER_CLK_SRC,
        .scl_io_num = I2C_MASTER_SCL,
        .sda_io_num = I2C_MASTER_SDA,
        .i2c_port = I2C_MASTER_NUM,
        .glitch_ignore_cnt = 7,
    };

    const esp_err_t ret = i2c_new_master_bus(&bus_config, &i2c_bus_h);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "I2C Bus initialized on SDA:%d SCL:%d", I2C_MASTER_SDA, I2C_MASTER_SCL);
    } else {
        ESP_LOGE(TAG, "I2C Bus Init failed: %s", esp_err_to_name(ret));
    }
    return ret;
}


/**
 * @brief Deinitializes the I2C manager by deleting the I2C master bus and releasing resources.
 *
 * This function checks if the I2C bus handle is valid. If so, it deletes the I2C master bus
 * and resets the bus handle to NULL. If the bus is successfully deleted, a log message is
 * generated indicating successful deinitialization.
 *
 * @return
 *     - ESP_OK: If the I2C bus is successfully deleted or the bus handle was already NULL.
 *     - An error code of type esp_err_t: If deinitialization fails.
 */
esp_err_t deinit_i2c_bus(void) {
    if (i2c_bus_h == NULL) return ESP_OK;

    const esp_err_t ret = i2c_del_master_bus(i2c_bus_h);
    if (ret == ESP_OK) {
        i2c_bus_h = NULL;
        ESP_LOGI(TAG, "I2C Bus was deleted successfully");
    }
    return ret;
}


/**
 * @brief Retrieves the handle for the I2C master bus, initializing the I2C manager if necessary.
 *
 * This function ensures that the I2C bus handle is properly initialized before returning it.
 * If the handle is not initialized, the function logs a warning message and attempts to
 * initialize the I2C manager by invoking `init_i2c_manager()`. Once initialized, the
 * I2C bus handle is returned to the caller.
 *
 * @return
 *     - A valid i2c_master_bus_handle_t instance: If the bus handle is successfully initialized and retrieved.
 *     - NULL: If the initialization fails.
 */
i2c_master_bus_handle_t get_i2c_bus_handle(void) {
    if (i2c_bus_h == NULL) {
        ESP_LOGW(TAG, "Bus handle requested but not initialized! Initializing now...");
        init_i2c_bus();
    }
    return i2c_bus_h;
}
