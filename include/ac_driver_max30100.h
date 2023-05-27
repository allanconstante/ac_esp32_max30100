#pragma once

#include <stdint.h>
#include "driver/i2c.h"
#include "esp_log.h"
#include "sdkconfig.h"
#include "ac_driver_max30100_registers.h"
#include "../../ac_driver_controller/include/ac_driver_types.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum
{
    MAX30100_GET_RAW_DATA,
    MAX30100_START_TEMPERTURA_READING,
    MAX30100_IS_TEMPERATURE_READY,
    MAX30100_GET_TEMPERATURE,
    MAX30100_GET_FIFO_WRITE_POINTER,
    MAX30100_GET_OVER_FLOW_COUNTER,
    MAX30100_GET_FIFO_READ_POINTER,
    MAX30100_GET_INTERRUPT_STATUS,
    MAX30100_SET_MODE,
    MAX30100_SET_SAMPLING,
    MAX30100_SET_PULSE_WIDTH,
    MAX30100_SET_LED_CURRENT_RED,
    MAX30100_SET_LED_CURRENT_IR,
    MAX30100_SET_HIGH_RES,
    MAX30100_STANDBY,
    MAX30100_RESET,
    MAX30100_CLEAR,
    MAX30100_END
} ac_max30100_driver_functions_list_t;

ac_driver_t* ac_get_max30100_driver(void);

#ifdef __cplusplus
}
#endif