#ifndef LSM6DSO32_H
#define LSM6DSO32_H

#include "driver/spi_master.h"
#include "lsm6dso32_reg.h"


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
void platform_init(void);

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
spi_device_handle_t get_spi_handle(void);

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
void platform_fifo_setup(const stmdev_ctx_t *dev_ctx);

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
int32_t configure_xl(const stmdev_ctx_t *dev_ctx, lsm6dso32_fs_xl_t value);

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
esp_err_t platform_fifo_bulk_read(const stmdev_ctx_t *dev_ctx, uint8_t *buffer, uint16_t num_samples);

/**
 * @brief Configures the FIFO mode of the sensor to stream mode.
 *
 * This function sets the FIFO mode of the sensor to stream mode, ensuring
 * that the oldest data in the FIFO is overwritten by newly acquired data
 * when the FIFO is full. This mode is useful for continuous data collection
 * use cases.
 *
 * @param dev_ctx The context for the sensor device interface, which contains
 *                the information required for communication with the device.
 */
void set_fifo_mode_stream(const stmdev_ctx_t *dev_ctx);

/**
 * @brief Sets the FIFO mode to bypass mode for the LSM6DSO32 sensor.
 *
 * This function configures the FIFO of the LSM6DSO32 sensor to operate
 * in bypass mode. In this mode, data is not stored in the FIFO buffer,
 * and only the most recent data is available for readout. It uses the
 * provided device context to perform the operation.
 *
 * @param dev_ctx The device context used to communicate with the sensor.
 *                It contains the interface and configuration settings
 *                required for communication.
 */
void set_fifo_mode_bypass(const stmdev_ctx_t *dev_ctx);

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
int32_t platform_write(void *handle, uint8_t reg, const uint8_t *bufPtr, uint16_t len);

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
int32_t platform_read(void *handle, uint8_t reg, uint8_t *bufPtr, uint16_t len);

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
void platform_delay(uint32_t ms);

#endif
