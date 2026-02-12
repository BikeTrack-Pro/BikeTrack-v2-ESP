#include "lsm6dso32.h"

#include <esp_log.h>
#include <string.h>


// SPI settings
#define MAX_SPI_BUFFER_SIZE     (CONFIG_BIKETRACK_IMU_WATERMARK * 7 + 100)
#define SPI_CLOCK_SPEED_HZ      CONFIG_BIKETRACK_SPI_CLK_SPEED
#define PIN_NUM_MISO            CONFIG_BIKETRACK_SPI_MISO
#define PIN_NUM_MOSI            CONFIG_BIKETRACK_SPI_MOSI
#define PIN_NUM_CLK             CONFIG_BIKETRACK_SPI_CLK
#define PIN_NUM_CS              CONFIG_BIKETRACK_SPI_CS
#define SPI_HOST_ID             SPI2_HOST

// IMU settings
#define INT1_PIN                CONFIG_BIKETRACK_IMU_INT1
#define FIFO_WATERMARK          CONFIG_BIKETRACK_IMU_WATERMARK


static DMA_ATTR uint8_t spi_tx_buf[MAX_SPI_BUFFER_SIZE];
static DMA_ATTR uint8_t spi_rx_buf[MAX_SPI_BUFFER_SIZE];

static spi_device_handle_t spi_handle;

/**
 * @brief Retrieves the SPI device handle used for communication.
 *
 * This function returns the handle of the SPI device currently being used
 * for communication with the sensor. The handle is a static internal reference
 * to the SPI device configured during initialization.
 *
 * @return The SPI device handle. Returns a valid handle if properly initialized,
 * or an undefined value if the handle is uninitialized.
 */
spi_device_handle_t get_spi_handle(void) {
    return spi_handle;
}

/**
 * @brief Writes data to the specified register of the sensor via SPI.
 *
 * This function transmits a block of data to the target device using
 * the SPI interface. The provided register address and data are encapsulated
 * into a single SPI transaction, and the write operation is performed.
 *
 * @param handle The SPI device handle used for communication.
 * @param reg The address of the register to which the data will be written.
 *            The write operation clears the most significant bit of the register address.
 * @param bufPtr Pointer to the buffer containing the data to be written.
 * @param len The number of bytes to write from the buffer.
 *
 * @return 0 on success or -1 on failure.
 */
int32_t platform_write(void *handle, const uint8_t reg, const uint8_t *bufPtr, const uint16_t len) {
    if (len + 1 > MAX_SPI_BUFFER_SIZE) return -1;

    spi_tx_buf[0] = reg & 0x7F;
    memcpy(&spi_tx_buf[1], bufPtr, len);

    spi_transaction_t transaction = {
        .length = 8 * (len + 1),
        .tx_buffer = spi_tx_buf,
        .rx_buffer = NULL,
    };

    return (spi_device_transmit(handle, &transaction) == ESP_OK) ? 0 : -1;
}


/**
 * @brief Reads data from the specified register of the sensor via SPI.
 *
 * This function performs a read operation from the target device using the SPI interface.
 * The register address is transmitted as part of the SPI transaction, and the data read
 * from the device is copied into the provided buffer.
 *
 * @param handle The SPI device handle used for communication.
 * @param reg The address of the register to be read. The read operation sets the most significant bit of the register address.
 * @param bufPtr Pointer to the buffer where the read data will be stored.
 * @param len The number of bytes to read from the register.
 *
 * @return 0 on success or -1 on failure.
 */
int32_t platform_read(void *handle, const uint8_t reg, uint8_t *bufPtr, const uint16_t len) {
    if (len + 1 > MAX_SPI_BUFFER_SIZE) return -1;

    memset(spi_tx_buf, 0, len + 1);
    spi_tx_buf[0] = reg | 0x80;

    spi_transaction_t transaction = {
        .length = 8 * (len + 1),
        .tx_buffer = spi_tx_buf,
        .rx_buffer = spi_rx_buf,
    };

    const esp_err_t ret = spi_device_transmit(handle, &transaction);
    if (ret == ESP_OK) {
        memcpy(bufPtr, &spi_rx_buf[1], len);
        return 0;
    }
    return -1;
}


/**
 * @brief Delays the execution for a specified number of milliseconds.
 *
 * This function introduces a blocking delay for the specified duration
 * by converting the millisecond input into RTOS ticks and invoking the
 * `vTaskDelay` function. It is typically used to synchronize tasks
 * or create timing-based events within a FreeRTOS environment.
 *
 * @param ms The number of milliseconds to delay. The specified duration
 *           is converted to the corresponding number of RTOS ticks.
 */
void platform_delay(const uint32_t ms) {
    vTaskDelay(pdMS_TO_TICKS(ms));
}


/**
 * @brief Initializes the SPI bus and adds the SPI device for communication.
 *
 * This function configures and initializes the SPI bus with specific parameters
 * such as MISO, MOSI, and SCLK pin assignments, and sets up an SPI device configuration
 * for the sensor. The SPI bus is then initialized, and the sensor device is added to the bus.
 *
 * The implementation uses default values for certain parameters, e.g., the maximum transfer size.
 * It also logs the initialization status and pin assignments.
 *
 * @note Before calling this function, ensure that the necessary pin definitions (e.g., `PIN_NUM_MISO`)
 *       and configurations are set correctly in the project.
 * @note The SPI bus and device are initialized with DMA enabled automatically using `SPI_DMA_CH_AUTO`.
 *
 * @return void
 */
void platform_init(void) {
    const spi_bus_config_t busCfg = {
        .miso_io_num = PIN_NUM_MISO,
        .mosi_io_num = PIN_NUM_MOSI,
        .sclk_io_num = PIN_NUM_CLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 0 // Default (4092 Bytes)
    };

    const spi_device_interface_config_t devCfg = {
        .clock_speed_hz = SPI_CLOCK_SPEED_HZ,
        .mode = 3,
        .spics_io_num = PIN_NUM_CS,
        .queue_size = 7,
    };

    ESP_ERROR_CHECK(spi_bus_initialize(SPI_HOST_ID, &busCfg, SPI_DMA_CH_AUTO));
    ESP_ERROR_CHECK(spi_bus_add_device(SPI_HOST_ID, &devCfg, &spi_handle));
}


/**
 * @brief Configures the FIFO settings on the LSM6DSO32 IMU sensor.
 *
 * This function sets up the FIFO (First-In, First-Out) functionality of the
 * LSM6DSO32 sensor. It initializes the FIFO mode, sets a watermark, configures
 * only accelerometer data to be batched into the FIFO, and routes the FIFO
 * threshold interrupt to the INT1 pin. This setup ensures continuous data
 * streaming for the FIFO while excluding gyroscope data.
 *
 * @param dev_ctx Pointer to the sensor's device context, which contains the
 *                function pointers for hardware-specific read, write, and
 *                delay operations.
 */
void platform_fifo_setup(const stmdev_ctx_t *dev_ctx) {
    // FIFO in Bypass (Reset)
    lsm6dso32_fifo_mode_set(dev_ctx, LSM6DSO32_BYPASS_MODE);

    // Set watermark (ex. 120 Samples, because dividable by 3 for X, Y, Z)
    lsm6dso32_fifo_watermark_set(dev_ctx, FIFO_WATERMARK);

    //
    lsm6dso32_fifo_stop_on_wtm_set(dev_ctx, PROPERTY_DISABLE);

    // We are only interested in the Accelerometer-data in the FIFO (max update rate)
    // IMPORTANT: Because of this, we know that only XL data is in FIFO
    lsm6dso32_fifo_gy_batch_set(dev_ctx, LSM6DSO32_GY_NOT_BATCHED);
    lsm6dso32_fifo_xl_batch_set(dev_ctx, LSM6DSO32_XL_BATCHED_AT_6667Hz);

    // Route interrupt to INT1
    lsm6dso32_pin_int1_route_t int1_route;
    lsm6dso32_pin_int1_route_get(dev_ctx, &int1_route);
    int1_route.int1_ctrl.int1_fifo_th = PROPERTY_ENABLE;
    lsm6dso32_pin_int1_route_set(dev_ctx, &int1_route);
}


/**
 * @brief Reads multiple FIFO samples from the device into a buffer.
 *
 * This function retrieves data directly from the FIFO of the sensor
 * using the specified device context. It calculates the total number
 * of bytes to read based on the number of samples and processes them
 * in a bulk operation.
 *
 * Each sample consists of 7 bytes: 6 bytes for the 3-axis accelerometer
 * (X, Y, Z - 2 bytes each) and 1 byte for a data tag.
 *
 * @param dev_ctx A pointer to the device context structure. This should
 *                be properly initialized with platform-specific read,
 *                write, and delay functions.
 * @param buffer  A pointer to the buffer where the read data will be
 *                stored. The buffer should be large enough to hold
 *                `num_samples * 7` bytes.
 * @param num_samples The number of samples to read from the FIFO.
 *                    Each sample is 7 bytes in size.
 *
 * @return ESP_OK on success, or an error code indicating the specific
 *         failure.
 */
esp_err_t platform_fifo_bulk_read(const stmdev_ctx_t *dev_ctx, uint8_t *buffer, const uint16_t num_samples) {
    // Per Sample = 3 (X,Y,Z) * 2 Bytes (16 Bit) + 1 TAG-Byte
    const uint16_t total_bytes = num_samples * 7;

    return dev_ctx->read_reg(dev_ctx->handle, LSM6DSO32_FIFO_DATA_OUT_TAG, buffer, total_bytes);
}


/**
 * @brief Configures the accelerometer full-scale range.
 *
 * This function sets the full-scale range of the accelerometer for the
 * LSM6DSO32 sensor. The new scale value is passed as a parameter and
 * is applied to the sensor through the provided device context.
 *
 * @param dev_ctx Pointer to the device context (stmdev_ctx_t) used
 *                for communication with the sensor. Must be properly initialized.
 * @param value   The desired accelerometer full-scale range
 *                (lsm6dso32_fs_xl_t) to be configured in the sensor.
 *
 * @return 0 if the configuration is successful. Returns a negative
 * value in case of an error.
 */
int32_t configure_xl(const stmdev_ctx_t *dev_ctx, const lsm6dso32_fs_xl_t value) {
    // Set the accelerometer to High-Performance-Mode and to its highest data rate
    lsm6dso32_xl_data_rate_set(dev_ctx, LSM6DSO32_XL_ODR_6667Hz_HIGH_PERF);
    // Set the scale of the accelerometer to the given scale
    return lsm6dso32_xl_full_scale_set(dev_ctx, value);
}


void set_fifo_mode_stream(const stmdev_ctx_t *dev_ctx) {
    // Set FIFO mode to stream (Continuous)
    lsm6dso32_fifo_mode_set(dev_ctx, LSM6DSO32_STREAM_MODE);
}


void set_fifo_mode_bypass(const stmdev_ctx_t *dev_ctx) {
    // FIFO in Bypass (Reset)
    lsm6dso32_fifo_mode_set(dev_ctx, LSM6DSO32_BYPASS_MODE);
}
