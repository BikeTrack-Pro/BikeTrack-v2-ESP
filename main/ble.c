#include <esp_log.h>
#include <nvs_flash.h>
#include <host/ble_hs.h>
#include <nimble/nimble_port.h>
#include <services/gap/ble_svc_gap.h>
#include <nimble/nimble_port_freertos.h>

#include "ble.h"
#include "imu.h"
#include "barometer.h"
#include "ble_gatt_svr.h"

#define NOTIFY_LIST_COUNT   3
#define STACK_SIZE          CONFIG_BIKETRACK_BLE_TASK_STACK_SIZE
#define PRIORITY            CONFIG_BIKETRACK_BLE_TASK_PRIORITY
#define CORE                CONFIG_BIKETRACK_BLE_TASK_CORE


// Private Forward Declarations
static int ble_gap_event_cb(struct ble_gap_event *event, void *arg);

void imu_sub_cb(bool s);

void bmp_sub_cb(bool s);


// Private variables
static const char *TAG = "[BLE]";

bool battery_is_subscribed = false, imu_is_subscribed = false, bmp_is_subscribed = false;
static uint16_t active_conn_handle = BLE_HS_CONN_HANDLE_NONE;
static ble_notify_t notify_list[NOTIFY_LIST_COUNT];
static uint8_t ble_own_addr;


/**
 * @brief Initializes BLE notification queues and associates them with their respective data types.
 *
 * This function sets up the notify_list array with BLE notification configurations for
 * IMU data, barometer data, and battery status. Each configuration includes a handle,
 * a queue for data storage, the size of the data type, a subscription flag, and
 * an optional subscription callback. The initialized queues are used to manage
 * BLE notification data transmission effectively.
 *
 * The configurations are as follows:
 * - IMU data: Queue size of 10, data type size defined by imu_data_t, and subscription status managed via imu_sub_cb.
 * - Barometer data: Queue size of 2, data type size defined by bmp_data_t, and subscription status managed via bmp_sub_cb.
 * - Battery status: Queue size of 2 with one-byte data, and no subscription callback.
 */
void init_queues() {
    notify_list[0] = (ble_notify_t){
        &imu_handle, xQueueCreate(10, sizeof(imu_data_t)), sizeof(imu_data_t), &imu_is_subscribed, imu_sub_cb
    };
    notify_list[1] = (ble_notify_t){
        &bmp_handle, xQueueCreate(2, sizeof(bmp_data_t)), sizeof(bmp_data_t), &bmp_is_subscribed, bmp_sub_cb
    };
    notify_list[2] = (ble_notify_t){
        &battery_handle, xQueueCreate(2, 1), 1, &battery_is_subscribed, NULL
    };
}


/**
 * @brief Checks if there is an active BLE connection.
 *
 * This function determines whether a Bluetooth Low Energy (BLE) connection
 * is currently established by verifying if the active connection handle is valid.
 *
 * @return true if a BLE connection is active, false otherwise.
 */
bool is_ble_connected(void) {
    return active_conn_handle != BLE_HS_CONN_HANDLE_NONE;
}


/**
 * @brief Enqueues IMU data to the BLE notification queue.
 *
 * This function attempts to send the given IMU data to the specific BLE notification
 * queue in the predefined notify_list. It operates with zero blocking time.
 *
 * @param data Pointer to the IMU data to be enqueued. The data will be sent
 *             using the first BLE notification queue.
 *
 * @return true if the data was successfully enqueued, false otherwise.
 */
bool ble_queue_imu_data(const imu_data_t *data) {
    return xQueueSend(notify_list[0].queue, data, 0) == pdTRUE;
}


/**
 * @brief Enqueues BMP sensor data to the BLE notification queue.
 *
 * This function sends the given BMP sensor data to the specific BLE notification
 * queue for transmission. It operates without blocking to avoid delays in the data flow.
 *
 * @param data Pointer to the BMP sensor data structure to be enqueued. The data
 *             will be sent using the second BLE notification queue in the notify_list.
 *
 * @return true if the data was successfully enqueued, false otherwise.
 */
bool ble_queue_bmp_data(const bmp_data_t *data) {
    return xQueueSend(notify_list[1].queue, data, 0) == pdTRUE;
}


/**
 * @brief Updates the BLE battery level characteristic with the given state of charge (SoC).
 *
 * This function adjusts the provided state of charge (SoC) to ensure it falls
 * within the valid range of 0% to 100%. The adjusted value is then used to
 * update the BLE battery level characteristic. If the battery level value
 * changes, it triggers a characteristic update event and logs the new level.
 *
 * @param soc_float The state of charge as a floating-point value. It is clamped
 *                  to the range of 0.0 to 100.0 before being processed.
 */
void ble_set_battery_level(float soc_float) {
    if (soc_float > 100.0f) soc_float = 100.0f;
    if (soc_float < 0.0f) soc_float = 0.0f;

    const uint8_t new_level = (uint8_t) soc_float;
    if (new_level != battery_level) {
        battery_level = new_level;

        ble_gatts_chr_updated(battery_handle);
        ESP_LOGI(TAG, "Battery Level updated to %d%%", battery_level);
    }
}


/**
 * @brief Callback function for IMU subscription status changes.
 *
 * This function handles the start or stop of IMU data collection based on the
 * subscription status from a BLE client. When a client subscribes to or unsubscribes
 * from IMU notifications, this callback is invoked to adjust the IMU data collection
 * process accordingly.
 *
 * @param s The subscription status indicating whether the client is subscribed
 *          (true) or unsubscribed (false) to IMU data notifications.
 */
void imu_sub_cb(const bool s) {
    if (s) start_imu_data_collecting();
    else stop_imu_data_collecting();
}


/**
 * @brief Callback triggered when the subscription status for barometer data changes.
 *
 * This function manages the state of barometer data collection based on the
 * subscription status. If the barometer is subscribed to, data collection
 * starts. Otherwise, data collection stops.
 *
 * @param s Boolean indicating the subscription status.
 *          True if subscribed, false if unsubscribed.
 */
void bmp_sub_cb(const bool s) {
    if (s) start_barometer_data_collecting();
    else stop_barometer_data_collecting();
}


/**
 * @brief Configures and starts BLE advertising with the specified parameters.
 *
 * This function initializes BLE advertising by setting up advertisement fields and parameters
 * and then begins advertising indefinitely. The advertisement fields include:
 * - Flags indicating general discoverability and BR/EDR unsupported.
 * - The device's name retrieved from the GAP service, marked as complete.
 *
 * The advertisement parameters specify:
 * - The connectability mode set to undirected (BLE_GAP_CONN_MODE_UND).
 * - The discoverability mode set to general (BLE_GAP_DISC_MODE_GEN).
 *
 * Advertising is launched using the `ble_gap_adv_start` API, which begins broadcasting
 * the configured advertisement data. It also associates an event callback to handle advertising
 * events such as connections or disconnections.
 */
static void ble_advertise(void) {
    const struct ble_gap_adv_params adv_params = {
        .conn_mode = BLE_GAP_CONN_MODE_UND, .disc_mode = BLE_GAP_DISC_MODE_GEN
    };
    struct ble_hs_adv_fields fields = {0};

    fields.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;
    fields.name = (uint8_t *) ble_svc_gap_device_name();
    fields.name_len = strlen((char *) fields.name);
    fields.name_is_complete = 1;

    ble_gap_adv_set_fields(&fields);
    ble_gap_adv_start(ble_own_addr, NULL, BLE_HS_FOREVER, &adv_params, ble_gap_event_cb, NULL);
}


/**
 * @brief Synchronizes the BLE host and starts advertising.
 *
 * This function is invoked as the BLE host synchronization callback and is responsible
 * for automatically determining the BLE address type using `ble_hs_id_infer_auto()`
 * and initializing the advertising process by calling `ble_advertise()`.
 *
 * Function responsibilities:
 * - Sets the BLE host's own address type.
 * - Configures and starts BLE advertising with predefined parameters.
 *
 * This synchronization step ensures that the BLE stack is properly initialized and
 * ready to handle connections and advertise its presence to other devices.
 */
static void on_sync(void) {
    ble_hs_id_infer_auto(0, &ble_own_addr);
    ble_advertise();
}


/**
 * @brief Handles various BLE GAP events such as connection, disconnection, and subscription changes.
 *
 * The BLE stack triggers this callback function upon specific GAP events.
 * It processes the events and performs actions such as starting/stopping advertising,
 * managing subscription states, and invoking associated callbacks.
 *
 * The relevant GAP events handled are:
 * - `BLE_GAP_EVENT_CONNECT`: Updates the active connection handle and restarts advertising on connection failure.
 * - `BLE_GAP_EVENT_DISCONNECT`: Clears the active connection handle, resets subscription states,
 *   invokes the associated callbacks, and restarts advertising.
 * - `BLE_GAP_EVENT_SUBSCRIBE`: Updates the subscription status for a notification attribute,
 *   modifies associated states, and invokes any subscribe-callbacks if defined.
 *
 * @param event Pointer to the BLE GAP event structure containing details about the event.
 * @param arg   Optional user-defined argument passed to the callback (unused in this implementation).
 *
 * @return Always returns 0 to indicate the processing of the event is complete.
 */
static int ble_gap_event_cb(struct ble_gap_event *event, void *arg) {
    switch (event->type) {
        case BLE_GAP_EVENT_CONNECT:
            active_conn_handle = (event->connect.status == 0) ? event->connect.conn_handle : BLE_HS_CONN_HANDLE_NONE;
            if (event->connect.status != 0) ble_advertise();
            return 0;

        case BLE_GAP_EVENT_DISCONNECT:
            active_conn_handle = BLE_HS_CONN_HANDLE_NONE;
            battery_is_subscribed = imu_is_subscribed = bmp_is_subscribed = false;
            imu_sub_cb(false);
            bmp_sub_cb(false);
            ble_advertise();
            return 0;

        case BLE_GAP_EVENT_SUBSCRIBE:
            for (int i = 0; i < NOTIFY_LIST_COUNT; i++) {
                if (event->subscribe.attr_handle == *notify_list[i].handle) {
                    const bool sub = event->subscribe.cur_notify;
                    *notify_list[i].should_send = sub;
                    if (notify_list[i].on_subscribe) notify_list[i].on_subscribe(sub);
                    ESP_LOGI(TAG, "Sub changed for idx %d: %d", i, sub);
                }
            }
            return 0;

        default:
            return 0;
    }
}


/**
 * @brief Executes the BLE host task loop and deinitializes FreeRTOS resources upon completion.
 *
 * This function initiates the BLE host task loop using nimble_port_run(), which manages the
 * BLE stack's operations. It will continue running until nimble_port_stop() is called,
 * which terminates the loop. Once the loop ends, it frees allocated FreeRTOS resources
 * by calling nimble_port_freertos_deinit().
 *
 * @param _ Placeholder parameter for compatibility with the FreeRTOS task function signature.
 */
void bleprph_host_task(void *_) {
    nimble_port_run(); // Will return only when nimble_port_stop() is executed
    nimble_port_freertos_deinit();
}


/**
 * @brief Task to handle BLE notifications and transmit pending data from queues.
 *
 * This task continuously monitors notification queues for IMU data, barometer data,
 * and battery status. When data is available and the corresponding subscription is active,
 * it processes and sends the data to the connected BLE client using custom notifications.
 * It ensures that data in the queue is encapsulated into BLE packets and transmitted
 * to the appropriate handles.
 *
 * The task operates indefinitely and pauses briefly between iterations to prevent
 * excessive polling.
 *
 * @param _ Unused parameter. Provided for compatibility with the task creation API.
 */
_Noreturn void ble_tx_disp_task(void *_) {
    uint8_t buffer[20];

    while (true) {
        if (active_conn_handle != BLE_HS_CONN_HANDLE_NONE) {
            for (int i = 0; i < NOTIFY_LIST_COUNT; i++) {
                if (notify_list[i].should_send && *notify_list[i].should_send) {
                    if (xQueueReceive(notify_list[i].queue, buffer, 0) == pdTRUE) {
                        const uint16_t h = *notify_list[i].handle;
                        if (h == 0) continue;

                        struct os_mbuf *om = ble_hs_mbuf_from_flat(buffer, notify_list[i].data_size);
                        if (om) {
                            ble_gatts_notify_custom(active_conn_handle, h, om);
                        }
                    }
                }
            }
        }

        // Wait shortly before rechecking if new data was received
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}


/**
 * @brief Initializes the BLE stack, GATT server, and associated tasks for BLE functionality.
 *
 * This function sets up the BLE stack and services necessary for the BLE communication
 * with the following key initializations and configurations:
 *
 * - Initializes non-volatile storage (NVS) to store BLE stack data.
 * - Initializes notification queues via the `init_queues` function for managing BLE notifications.
 * - Initializes the NimBLE host using `nimble_port_init`.
 * - Configures the sync callback (`on_sync`) to handle BLE address setup and advertising.
 * - Initializes the GATT server via `ble_gatt_svr_init` to define available BLE services and characteristics.
 * - Sets the BLE GAP device name to the selected name for easy identification during discovery.
 * - Starts the NimBLE FreeRTOS host task, which handles BLE stack execution.
 * - Creates and pins a FreeRTOS task (`ble_tx_disp_task`) to the specified core to handle BLE data transmission.
 *
 * This function ensures the BLE stack is properly set up and that tasks for managing BLE communication
 * and notifications are correctly started.
 */
void ble_init(void) {
    esp_log_level_set("NimBLE", ESP_LOG_WARN);

    nvs_flash_init();
    init_queues();
    nimble_port_init();

    ble_hs_cfg.sync_cb = on_sync;

    ble_gatt_svr_init();
    ble_svc_gap_device_name_set("BikeTrack V2");

    nimble_port_freertos_init(bleprph_host_task);
    xTaskCreatePinnedToCore(ble_tx_disp_task, "ble_tx_disp_task", STACK_SIZE, NULL, PRIORITY, NULL, CORE);
}
