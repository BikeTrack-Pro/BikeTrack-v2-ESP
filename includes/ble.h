#ifndef BLE_H
#define BLE_H

#include <stdint.h>
#include <stdbool.h>
#include <nimble/ble.h>

#include "common.h"


typedef struct {
    uint16_t *handle;
    QueueHandle_t queue;
    size_t data_size;
    bool *should_send;

    void (*on_subscribe)(bool);
} ble_notify_t;


// Public functions
void ble_init(void);

bool ble_queue_imu_data(const imu_data_t *data);

bool ble_queue_bmp_data(const bmp_data_t *data);

void ble_set_battery_level(float soc_float);

bool is_ble_connected(void);

#endif
