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

spi_device_handle_t get_spi_handle(void) {
    return spi_handle;
}

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


void platform_delay(const uint32_t ms) {
    vTaskDelay(pdMS_TO_TICKS(ms));
}


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


void platform_fifo_setup(const stmdev_ctx_t *dev_ctx) {
    // FIFO in Bypass (Reset)
    lsm6dso32_fifo_mode_set(dev_ctx, LSM6DSO32_BYPASS_MODE);

    // Set watermark (ex. 120 Samples, because dividable by 3 for X, Y, Z)
    lsm6dso32_fifo_watermark_set(dev_ctx, FIFO_WATERMARK);
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


esp_err_t platform_fifo_bulk_read(const stmdev_ctx_t *dev_ctx, uint8_t *buffer, const uint16_t num_samples) {
    // Per Sample = 3 (X,Y,Z) * 2 Bytes (16 Bit) + 1 TAG-Byte
    const uint16_t total_bytes = num_samples * 7;

    return dev_ctx->read_reg(dev_ctx->handle, LSM6DSO32_FIFO_DATA_OUT_TAG, buffer, total_bytes);
}


int32_t configure_xl(const stmdev_ctx_t *dev_ctx, const lsm6dso32_fs_xl_t value) {
    // Set the accelerometer to High-Performance-Mode and to its highest data rate
    lsm6dso32_xl_data_rate_set(dev_ctx, LSM6DSO32_XL_ODR_6667Hz_HIGH_PERF);
    // Set the scale of the accelerometer to the given scale
    return lsm6dso32_xl_full_scale_set(dev_ctx, value);
}


void set_fifo_mode_stream(const stmdev_ctx_t *dev_ctx) {
    lsm6dso32_fifo_mode_set(dev_ctx, LSM6DSO32_STREAM_MODE);
}


void set_fifo_mode_bypass(const stmdev_ctx_t *dev_ctx) {
    lsm6dso32_fifo_mode_set(dev_ctx, LSM6DSO32_BYPASS_MODE);
}
