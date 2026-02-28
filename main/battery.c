#include <driver/i2c_master.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <driver/gpio.h>
#include <driver/i2c.h>
#include <esp_log.h>

#include "battery.h"
#include "max17048.h"
#include "ble.h"
#include "i2c.h"

#define MAX17048_ADDR   0x36
#define INTERRUPT_PIN   CONFIG_BIKETRACK_BATTERY_INTERRUPT_PIN
#define I2C_SCL_SPEED   CONFIG_BIKETRACK_BATTERY_I2C_SCL_SPEED
#define MAX_INTERVAL    CONFIG_BIKETRACK_BATTERY_TASK_INTERVAL_MS
#define STACK_SIZE      CONFIG_BIKETRACK_BATTERY_TASK_STACK_SIZE
#define PRIORITY        CONFIG_BIKETRACK_BATTERY_TASK_PRIORITY
#define CORE            CONFIG_BIKETRACK_BATTERY_TASK_CORE


static const char *TAG = "[BAT]";

static i2c_master_dev_handle_t max17048_h = NULL;
static i2c_master_bus_handle_t i2c_bus_h = NULL;
static TaskHandle_t battery_task_h = NULL;


/**
 * @brief Interrupt Service Routine (ISR) for handling battery-related events.
 *
 * This function is triggered when a configured GPIO interrupt occurs, typically
 * when the battery alert signal from the hardware (e.g., MAX17048 fuel gauge) is active.
 * It notifies the battery task to process the event and determines whether
 * a context switch is required to prioritize the battery task.
 *
 * @param _ Unused parameter, provided to match the ISR callback signature.
 */
static void IRAM_ATTR battery_isr_handler(void *_) {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    vTaskNotifyGiveFromISR(battery_task_h, &xHigherPriorityTaskWoken);
    if (xHigherPriorityTaskWoken)
    portYIELD_FROM_ISR();
}


/**
 * @brief Writes a 16-bit value to a specified register of the MAX17048 fuel gauge sensor.
 *
 * This function sends a 3-byte data packet over I2C, where the first byte represents
 * the destination register address, and the two bytes in a row represent the 16-bit value
 * to be written (MSB first). It communicates with the hardware through the I2C master interface.
 *
 * @param reg The 8-bit register address where the data will be written.
 * @param val The 16-bit value to be written to the specified register.
 * @return esp_err_t Returns ESP_OK on a successful writing or an appropriate error code on failure.
 */
static esp_err_t max17048_write_reg(const uint8_t reg, const uint16_t val) {
    const uint8_t data[3] = {reg, (uint8_t) (val >> 8), (uint8_t) (val & 0xFF)};
    return i2c_master_transmit(max17048_h, data, 3, -1);
}


/**
 * @brief Probes the MAX17048 fuel gauge to check if it is reachable on the I2C bus.
 *
 * This function sends a probe command to the MAX17048 device at its configured
 * I2C address. It verifies the device's presence by ensuring there are no errors
 * in the communication. If the probe is successful, the device is considered
 * reachable; otherwise, an error is logged, and the function returns false.
 *
 * @return true if the device is reachable, false otherwise.
 */
bool probe_max17048() {
    const esp_err_t probe_err = i2c_master_probe(i2c_bus_h, MAX17048_ADDR, pdMS_TO_TICKS(100));
    if (probe_err != ESP_OK) {
        ESP_LOGE(TAG, "Hardware-Check failed: Chip is not reachable! (Code: %s)", esp_err_to_name(probe_err));
        return false;
    }
    ESP_LOGI(TAG, "Probe for MAX17048-chip was successful");
    return true;
}


/**
 * @brief Adds the MAX17048 fuel gauge to the I2C bus.
 *
 * This function initializes the MAX17048 device by creating an I2C device configuration
 * and registering the device with the I2C bus. The configuration includes the I2C
 * address and clock speed necessary for communication. The function is part of the
 * setup process for enabling communication with the MAX17048 chip deside the library.
 *
 * It uses a pre-configured I2C bus handle and updates a handle for the device to
 * enable further operations on the MAX17048.
 */
void add_max17048_to_bus() {
    const i2c_device_config_t dev_cfg = {
        .device_address = MAX17048_ADDR,
        .scl_speed_hz = I2C_SCL_SPEED,
    };

    i2c_master_bus_add_device(i2c_bus_h, &dev_cfg, &max17048_h);
}


/**
 * @brief Initializes the MAX17048 fuel gauge chip for battery monitoring.
 *
 * This function performs the necessary initialization steps for the MAX17048 chip.
 * It first probes the chip on the I2C bus to ensure communication. If the probe is
 * successful, the function retrieves the default configuration, adjusts the necessary
 * parameters such as I2C bus handle, device address, and I2C communication speed,
 * and initializes the chip using the provided configuration. Finally, the chip is
 * added to the I2C bus to allow for further communication aside from the max17048-library.
 *
 * @return true if the initialization is successful, false otherwise.
 */
bool init_max17048() {
    if (probe_max17048() == false) return false;

    max17048_config_t max_config;
    max17048_get_default_config(&max_config);
    max_config.i2c_bus_handle = i2c_bus_h;
    max_config.device_address = MAX17048_ADDR;
    max_config.i2c_freq_hz = I2C_SCL_SPEED;

    const esp_err_t init_rslt = max17048_init_on_bus_with_config(&max_config);
    if (init_rslt != ESP_OK) {
        ESP_LOGE(TAG, "MAX17048-Chip could not be initialised! (Code: %s)", esp_err_to_name(init_rslt));
        return false;
    }
    ESP_LOGI(TAG, "MAX17048-Chip initialised successfully");

    add_max17048_to_bus();

    return true;
}


/**
 * @brief Configures the MAX17048 fuel gauge chip and sets up its interrupt.
 *
 * This function performs two tasks:
 * - Configures a GPIO pin as an interrupt input for the MAX17048 ALERT signal.
 *   The interrupt is triggered on the falling edge of the signal, as the ALERT pin is active-low.
 * - Configures the MAX17048 chip to generate an interrupt on every 1% change in the battery state of charge (SOC).
 *
 * @return True if the configuration is successful, false otherwise. If the GPIO setup or
 *         writing to the MAX17048 configuration register fails, the function returns false.
 */
bool configure_max17048() {
    // Set up INTERRUPT-GPIO for Active-Low ALERT
    const gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << INTERRUPT_PIN),
        .intr_type = GPIO_INTR_NEGEDGE,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = 1, // TODO: Pull-Up on PCB
    };
    gpio_config(&io_conf);
    gpio_isr_handler_add(INTERRUPT_PIN, battery_isr_handler, NULL);

    // Configure the chip to send an Interrupt on every 1% battery change (Bit 2 (ALSC) on CONFIG Register (0x0C)).
    return max17048_write_reg(0x0C, 0x1C70 | (1 << 2)) == ESP_OK;
}


/**
 * @brief Updates the battery status by reading the state of charge (SoC) and voltage.
 *
 * This function:
 * - Retrieves the battery's state of charge and voltage from the MAX17048 fuel gauge (logs the values for debugging purposes).
 * - Clears any battery alert signals by resetting the STATUS register.
 * - Updates the BLE layer with the new battery level.
 *
 * The SoC and voltage values are obtained using the MAX17048-library functions.
 * The STATUS-Register at address 0x1A is reset to 0x0000 to clear any hardware alerts.
 * BLE notifications for the updated battery level may also be triggered via this process.
 */
static void update_battery_status(void) {
    float soc = 0, voltage = 0;

    max17048_get_soc(&soc);
    max17048_get_voltage(&voltage);
    ESP_LOGD(TAG, "Battery Update: %.2f%% (%.2fV)", soc, voltage);

    // Reset the STATUS Register (0x1A) to clear the Alert
    max17048_write_reg(0x1A, 0x0000);

    // Send datat to possibly trigger BLE update
    ble_set_battery_level(soc);
}


/**
 * @brief Continuously manages battery monitoring and status updates.
 *
 * This task performs the key battery management operations,
 * such as periodically updating the battery status or responding
 * to interrupts signaling significant battery events. It uses
 * a FreeRTOS notification-based mechanism to trigger updates based
 * on external signals or timeouts.
 *
 * @param _ Unused parameter, provided to match the FreeRTOS task function signature.
 */
_Noreturn void battery_task(void *_) {
    // Initially update battery state
    update_battery_status();

    while (true) {
        const uint32_t thread_notification = ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(MAX_INTERVAL));
        if (thread_notification > 0) {
            ESP_LOGI(TAG, "Battery task triggered by interrupt (ALERT)");
        }

        // When an interrupt is received or the maximum interval is reached, the battery status is updated.
        update_battery_status();
    }
}


/**
 * @brief Initializes and starts the battery monitoring task.
 *
 * This function sets up the battery monitoring subsystem by performing the following steps:
 * - Acquires a handle to the I2C bus required for communication with the MAX17048 chip.
 * - Initializes the MAX17048 fuel gauge chip. If initialization fails, an error is logged, and the function exits.
 * - Configures the MAX17048 chip to enable battery status updates and interrupt handling. If configuration fails, an error is logged, and the function exits.
 * - Creates a FreeRTOS task (`battery_task`) pinned to the specified core to handle periodic updates and interrupts for battery status.
 *
 * The task is responsible for monitoring the battery level and updating the status based on hardware signals or periodic intervals.
 */
void start_battery_task(void) {
    esp_log_level_set("MAX17048_COMP", ESP_LOG_WARN);

    i2c_bus_h = get_i2c_bus_handle();

    if (init_max17048() == false) {
        ESP_LOGE(TAG, "Could not initialize MAX17048-Chip!");
        return;
    }

    if (configure_max17048() == false) {
        ESP_LOGE(TAG, "Could not configure MAX17048-Chip!");
        return;
    }

    xTaskCreatePinnedToCore(battery_task, "battery_task", STACK_SIZE, NULL, PRIORITY, &battery_task_h, CORE);
}
