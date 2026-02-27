#ifndef I2C_MANAGER_H
#define I2C_MANAGER_H

#include <driver/i2c_master.h>
#include <esp_err.h>


#define I2C_MASTER_SDA              CONFIG_BIKETRACK_I2C_SDA
#define I2C_MASTER_SCL              CONFIG_BIKETRACK_I2C_SCL
#define I2C_MASTER_CLK_SRC          I2C_CLK_SRC_DEFAULT
#define I2C_MASTER_NUM              I2C_NUM_0


// Public function declarations
esp_err_t init_i2c_bus(void);

esp_err_t deinit_i2c_bus(void);

i2c_master_bus_handle_t get_i2c_bus_handle(void);

#endif
