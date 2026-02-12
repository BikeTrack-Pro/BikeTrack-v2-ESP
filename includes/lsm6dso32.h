#ifndef LSM6DSO32_H
#define LSM6DSO32_H

#include "driver/spi_master.h"
#include "lsm6dso32_reg.h"


void platform_init(void);

spi_device_handle_t get_spi_handle(void);

void platform_fifo_setup(const stmdev_ctx_t *dev_ctx);

int32_t configure_xl_scale(const stmdev_ctx_t *dev_ctx, lsm6dso32_fs_xl_t value);

esp_err_t platform_fifo_bulk_read(const stmdev_ctx_t *dev_ctx, uint8_t *buffer, uint16_t num_samples);


// Functions needed for LSM6DSO32's platform independent driver (pid)
int32_t platform_write(void *handle, uint8_t reg, const uint8_t *bufPtr, uint16_t len);

int32_t platform_read(void *handle, uint8_t reg, uint8_t *bufPtr, uint16_t len);

void platform_delay(uint32_t ms);

#endif
