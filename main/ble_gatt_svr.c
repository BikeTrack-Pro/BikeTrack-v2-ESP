#include <host/ble_hs.h>
#include <services/gap/ble_svc_gap.h>
#include <services/ans/ble_svc_ans.h>
#include <services/gatt/ble_svc_gatt.h>

#include "ble_gatt_svr.h"
#include "ble.h"


uint8_t battery_level = 0;

uint16_t battery_handle;
uint16_t imu_handle;
uint16_t bmp_handle;


static int hw_rev_access_cb(uint16_t conn_handle, const uint16_t attr_handle,
                            struct ble_gatt_access_ctxt *ctxt, void *_) {
    if (ctxt->op == BLE_GATT_ACCESS_OP_READ_CHR) {
        const char *hw_rev = CONFIG_BIKETRACK_HW_REVISION;
        return os_mbuf_append(ctxt->om, hw_rev, strlen(hw_rev));
    }
    return BLE_ATT_ERR_UNLIKELY;
}


static int sw_rev_access_cb(uint16_t conn_handle, const uint16_t attr_handle,
                            struct ble_gatt_access_ctxt *ctxt, void *_) {
    if (ctxt->op == BLE_GATT_ACCESS_OP_READ_CHR) {
        const char *sw_rev = CONFIG_BIKETRACK_SW_REVISION;
        return os_mbuf_append(ctxt->om, sw_rev, strlen(sw_rev));
    }
    return BLE_ATT_ERR_UNLIKELY;
}


static int battery_level_access_cb(uint16_t conn_handle, const uint16_t attr_handle,
                                   struct ble_gatt_access_ctxt *ctxt, void *_) {
    if (ctxt->op == BLE_GATT_ACCESS_OP_READ_CHR) {
        return os_mbuf_append(ctxt->om, &battery_level, sizeof(battery_level));
    }
    return BLE_ATT_ERR_UNLIKELY;
}


static int ctrl_chr_cb(uint16_t conn_handle, uint16_t attr_handle,
                       struct ble_gatt_access_ctxt *ctxt, void *_) {
    if (ctxt->op == BLE_GATT_ACCESS_OP_READ_CHR) {
        const uint8_t val = is_recording ? 1 : 0;
        return os_mbuf_append(ctxt->om, &val, sizeof(val));
    }

    if (ctxt->op == BLE_GATT_ACCESS_OP_WRITE_CHR) {
        uint8_t data;

        if (OS_MBUF_PKTLEN(ctxt->om) != 1) return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;

        os_mbuf_copydata(ctxt->om, 0, 1, &data);
        // TODO: ...
        return 0;
    }

    return BLE_ATT_ERR_UNLIKELY;
}


static int imu_access_cb(uint16_t conn_handle, const uint16_t attr_handle,
                         struct ble_gatt_access_ctxt *ctxt, void *_) {
    if (ctxt->op == BLE_GATT_ACCESS_OP_READ_CHR) {
        return 0;
    }
    return BLE_ATT_ERR_UNLIKELY;
}


static int bmp_access_cb(uint16_t conn_handle, const uint16_t attr_handle,
                         struct ble_gatt_access_ctxt *ctxt, void *_) {
    if (ctxt->op == BLE_GATT_ACCESS_OP_READ_CHR) {
        return 0;
    }
    return BLE_ATT_ERR_UNLIKELY;
}


static const struct ble_gatt_svc_def gatt_svr_svcs[] = {
    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = BLE_UUID16_DECLARE(0x180A), // ----------------------------------------- Device Information Service
        .characteristics = (struct ble_gatt_chr_def[]){
            {
                .uuid = BLE_UUID16_DECLARE(0x2A27),
                .access_cb = hw_rev_access_cb,
                .flags = BLE_GATT_CHR_F_READ,
            },
            {
                .uuid = BLE_UUID16_DECLARE(0x2A28),
                .access_cb = sw_rev_access_cb,
                .flags = BLE_GATT_CHR_F_READ,
            },
            {
                0,
            }
        }
    },
    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = BLE_UUID16_DECLARE(0x180F), // ----------------------------------------- Battery Service
        .characteristics = (struct ble_gatt_chr_def[]){
            {
                .uuid = BLE_UUID16_DECLARE(0x2A19),
                .access_cb = battery_level_access_cb,
                .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_NOTIFY,
                .val_handle = &battery_handle,
            },
            {
                0,
            }
        },
    },
    {
        // Custom BikeTrack-Service
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = &bikeTrack_svc_uuid.u, // ---------------------------------------------- Custom BikeTrack Service
        .characteristics = (struct ble_gatt_chr_def[])
        {
            {
                .uuid = &ctrl_chr_uuid.u,
                .access_cb = ctrl_chr_cb,
                .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_WRITE,
            },
            {
                .uuid = &imu_chr_uuid.u,
                .access_cb = imu_access_cb,
                .flags = BLE_GATT_CHR_F_NOTIFY,
                .val_handle = &imu_handle,
            },
            {
                .uuid = &bmp_chr_uuid.u,
                .access_cb = bmp_access_cb,
                .flags = BLE_GATT_CHR_F_NOTIFY,
                .val_handle = &bmp_handle,
            },
            {
                0, // No more characteristics in this service.
            }
        },
    },
    {
        0, // No more services.
    },
};


/**
 * Initializes the GATT server by setting up GAP, GATT, and ANS services,
 * counting the GATT service configuration, and adding the GATT services.
 *
 * @return 0 if the initialization is successful, or a positive error code
 * if an issue occurs during service configuration or addition.
 */
int ble_gatt_svr_init(void) {
    ble_svc_gap_init();
    ble_svc_gatt_init();
    ble_svc_ans_init();

    int rc = ble_gatts_count_cfg(gatt_svr_svcs);
    if (rc != 0) {
        return rc;
    }

    rc = ble_gatts_add_svcs(gatt_svr_svcs);
    if (rc != 0) {
        return rc;
    }

    return 0;
}
