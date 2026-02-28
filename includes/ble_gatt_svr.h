#ifndef BLE_GATT_SVR_H
#define BLE_GATT_SVR_H

#include <host/ble_uuid.h>


// TODO: Generate truly random UUIDs
// Service UUIDs
static const ble_uuid128_t bikeTrack_svc_uuid = BLE_UUID128_INIT(0x2d, 0x71, 0xa2, 0x59, 0xb4, 0x58, 0xc8, 0x12,
                                                                 0x99, 0x99, 0x43, 0x95, 0x12, 0x2f, 0x46, 0x59);

// Characteristic UUIDs
static const ble_uuid128_t imu_chr_uuid = BLE_UUID128_INIT(0x00, 0x00, 0x00, 0x00, 0x11, 0x11, 0x11, 0x11,
                                                           0x22, 0x22, 0x22, 0x22, 0x33, 0x33, 0x01, 0x01);
static const ble_uuid128_t bmp_chr_uuid = BLE_UUID128_INIT(0x00, 0x00, 0x00, 0x00, 0x11, 0x11, 0x11, 0x11,
                                                           0x22, 0x22, 0x22, 0x22, 0x33, 0x33, 0x01, 0x02);

// Public variables
extern uint16_t battery_handle;
extern uint16_t imu_handle;
extern uint16_t bmp_handle;

extern uint8_t battery_level;

// Public functions
int ble_gatt_svr_init(void);

#endif
