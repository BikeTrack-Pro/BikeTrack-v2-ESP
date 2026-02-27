#include "i2c.h"
#include "ble.h"


void app_main(void) {
    init_i2c_bus();

    ble_init();
}
