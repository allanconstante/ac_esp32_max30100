
#ifndef MAX30100
#define MAX30100

#include <stdint.h>
#include <stdbool.h>
#include "driver/i2c.h"
#include "esp_log.h"
#include "sdkconfig.h"
#include "ac_esp32_max30100_registers.h"

#ifdef __cplusplus
extern "C" {
#endif

char initialize_max30100(i2c_port_t port);

char set_mode(int);
char set_sampling(int);
char set_pulse_width(int);
char set_led_current_red(int);
char set_led_current_ir(int);
char set_high_res(bool);

char get_raw_data(uint8_t*);
char get_temperature(void *parameters);
char get_fifo_write_pointer(uint8_t*);
char get_over_flow_counter(void *parameters);
char get_fifo_read_pointer(uint8_t*);
char get_interrupt_status(void *parameters);

char start_temperature_reading(void);
bool is_temperature_ready(void);

char standby(void *parameters);
char reset(void *parameters);
char clear(void);

#ifdef __cplusplus
}
#endif

#endif //MAX30100