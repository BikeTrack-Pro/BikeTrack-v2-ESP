#include "i2c.h"
#include "ble.h"
#include "battery.h"


void app_main(void) {
    init_i2c_bus();

    ble_init();
    start_battery_task();
}
