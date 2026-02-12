#ifndef BMP5_INTERFACE_H
#define BMP5_INTERFACE_H

#include <driver/i2c_master.h>
#include <bmp5_defs.h>


/**
 * @brief I2C read function for Bosch BMP5 API.
 *
 * This function performs an I2C read operation to retrieve data from the device
 * at the specified register address. It uses the specified I2C interface pointer
 * to communicate with the device.
 *
 * @param reg_addr The register address to read data from.
 * @param reg_data Pointer to a buffer where the read data will be stored.
 * @param len Number of bytes to read from the register.
 * @param intf_ptr Pointer to the I2C interface for communication.
 *
 * @return BMP5_OK on success, or an error code such as BMP5_E_NULL_PTR if inputs are invalid,
 * or BMP5_E_COM_FAIL if communication fails.
 */
BMP5_INTF_RET_TYPE bmp5_i2c_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr);

/**
 * @brief I2C write function for Bosch BMP5 API.
 *
 * This function performs an I2C write operation to send data to the device
 * at the specified register address. It uses the provided I2C interface pointer
 * for communication.
 *
 * @param reg_addr The register address to write data to.
 * @param reg_data Pointer to a buffer containing the data to be written.
 * @param len Number of bytes to write to the register.
 * @param intf_ptr Pointer to the I2C interface for communication.
 *
 * @return BMP5_OK on success, or an error code such as BMP5_E_NULL_PTR if inputs are invalid,
 * or BMP5_E_COM_FAIL if communication fails.
 */
BMP5_INTF_RET_TYPE bmp5_i2c_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len, void *intf_ptr);

/**
 * @brief Delay function for Bosch BMP5 API in microseconds.
 *
 * This function introduces a delay for the specified period in microseconds.
 * It utilizes different mechanisms to achieve the delay, depending on the delay
 * duration. For durations of 1000 microseconds or more, it uses task delays.
 * For shorter durations, it employs a ROM-based delay function.
 *
 * @param period The delay duration in microseconds.
 * @param intf_ptr Pointer to the interface-specific data, which may be used for
 * implementing the delay if required by the interface.
 */
void bmp5_delay_us(uint32_t period, void *intf_ptr);

#endif
