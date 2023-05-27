/**
 * @file ac_driver_max30100.c
 * @author Allan Appelt Constante (allan@aconstante.com.br)
 * @brief This driver contains the basic functions for accessing the registers and configuring the MAX30100, integrated
 *        circuit from Analog Devices. This component is intended to read heartbeats and oximetry.
 * @version 0.1
 * @date 2023-04-17
 * 
 * @copyright Copyright (c) 2023
 * 
 */


#include "ac_driver_max30100.h"

#define I2C_SDA 21
#define I2C_SCL 22
#define I2C_FRQ 100000
#define I2C_PORT I2C_NUM_0

#define TAG "Max30100 Driver"

static ac_driver_t max30100_driver;
static ac_driver_function_pointer_t max30100_functions[MAX30100_END];

static char initialize_max30100_driver(void *parameters);

/**
 * @brief Gets the raw data from the FIFO register.
 * 
 * @param parameters Vector pointer that will receivfe the data.
 * @return char 
 */
static char get_raw_data(void *parameters);

/**
 * @brief Starts the temperature reading.
 * 
 * @param parameters Does not receive parameters.
 * @return char 
 * 
 * @see is_temperature_ready(void *parameters) and get_temperature(void *parameters).
 */
static char start_temperature_reading(void *parameters);

/**
 * @brief Returns a non-zero value if the conversion started with start_temperature_reading(void *parameters) is 
 *        complete. Otherwise it returns zero.
 * 
 * @param parameters Integer pointer to receive read operation state.
 * @return char
 * 
 * @see start_temperature_reading(void *parameters) and get_temperature(void *parameters).
 */
static char is_temperature_ready(void *parametrs);

/**
 * @brief It gets the temperature read. This function should be called only after the completion of the reading 
 *        indicated by is_temperature_ready(void *parameters).
 * 
 * @param parameters Pointer of the float variable that will receive the temperature reading.
 * @return char
 * 
 * @see start_temperature_reading(void *parameters) and is_temperature_ready(void *parameters).
 */
static char get_temperature(void *parameters);

static char get_fifo_write_pointer(void *parameters);
static char get_over_flow_counter(void *parameters);
static char get_fifo_read_pointer(void *parameters);
static char get_interrupt_status(void *parameters);
static char set_mode(void *parameters);
static char set_sampling(void *parameters);
static char set_pulse_width(void *parameters);
static char set_led_current_red(void *parameters);
static char set_led_current_ir(void *parameters);
static char set_high_res(void *parameters);
static char standby(void *parameters);
static char reset(void *parameters);

static void initI2C(void);
static void initMax30100(void);
static void readRegister( i2c_port_t i2c_port, uint8_t address, uint8_t* reg, uint8_t size );
static void writeRegister( i2c_port_t i2c_port, uint8_t address, uint8_t val );
static void setMode(int mode);
static void setSampling(int sampling);
static void setPulseWidth(int pulse);
static void setLedCurrentRed(int current_red);
static void setLedCurrentIr(int current_ir);
static void setHighRes(int enable);

static char get_raw_data(void *parameters)
{
    uint8_t *raw = (uint8_t*)parameters;
    readRegister(I2C_PORT, FIFO_DATA_REGISTER, raw, 4);
    return 1;
}

static char start_temperature_reading(void *parameters)
{
    uint8_t data;
    readRegister( I2C_PORT, MODE_CONFIGURATION, &data, 1 );
    writeRegister( I2C_PORT, MODE_CONFIGURATION, (data | READ_TEMPERATURE) ); 
    return 1;
}

static char is_temperature_ready(void *parameters)
{
    uint8_t data;
    int *ready = (int*)parameters;
    readRegister(I2C_PORT, MODE_CONFIGURATION, &data, 1);
    *ready = (data & (1<<3));
    return 1;
}

static char get_temperature(void *parameters)
{
    int8_t temp;
    int8_t temp_fraction;
    float *temperature = (float*)parameters;
    readRegister(I2C_PORT, TEMP_INTEGER, (uint8_t*)&temp, 1);
    readRegister( I2C_PORT, TEMP_FRACTION, (uint8_t*)&temp_fraction, 1);
    *temperature = (float)temp + (float)(temp_fraction * 0.0625);
    return 1;
}

static char get_fifo_write_pointer(void *parameters)
{
    uint8_t *data = (uint8_t*)parameters;
    readRegister( I2C_PORT, FIFO_WRITE_POINTER, data, 1 );
    return 1;
}

static char get_over_flow_counter(void *parameters)
{
    uint8_t *data = (uint8_t*) parameters;
    readRegister( I2C_PORT, OVER_FLOW_COUNTER, data, 1 );
    return 1;
}

static char get_fifo_read_pointer(void *parameters)
{
    uint8_t *data = (uint8_t*) parameters;
    readRegister( I2C_PORT, FIFO_READ_POINTER, data, 1 );
    return 1;
}

static char get_interrupt_status(void *parameters)
{
    uint8_t *data = (uint8_t*) parameters;
    readRegister( I2C_PORT, INTERRUPT_STATUS, data, 1 );
    return 1;
}

static char set_mode(void *parameters)
{
    int *mode = (int*) parameters;
    setMode(*mode);
    return 1;
}

static char set_sampling(void *parameters)
{
    int *sampling = (int*) parameters;
    setSampling(*sampling);
    return 1;
}

static char set_pulse_width(void *parameters)
{
    int *pulse = (int*) parameters;
    setPulseWidth(*pulse);
    return 1;
}

static char set_led_current_red(void *parameters)
{
    int *current_red = (int*) parameters;
    setLedCurrentRed(*current_red);
    return 1;
}

static char set_led_current_ir(void *parameters)
{
    int *current_ir = (int*) parameters;
    setLedCurrentIr(*current_ir);
    return 1;
}

static char set_high_res(void *parameters)
{
    int *enable = (int*) parameters;
    setHighRes(*enable);
    return 1;
}

static char standby(void *parameters)
{
    uint8_t data;
    int *enable = (int*) parameters;
    readRegister( I2C_PORT, MODE_CONFIGURATION, &data, 1);
    if(*enable == 1) writeRegister( I2C_PORT, MODE_CONFIGURATION, data | STANDBY );
    else if(*enable == 0) writeRegister( I2C_PORT, MODE_CONFIGURATION, data & (~STANDBY) );
    return 1;
}

static char reset(void *parameters)
{
    uint8_t data;
    int *enable = (int*) parameters;
    readRegister( I2C_PORT, MODE_CONFIGURATION, &data, 1);
    if(*enable == 1) writeRegister( I2C_PORT, MODE_CONFIGURATION, data | RESET );
    else if(*enable == 0) writeRegister( I2C_PORT, MODE_CONFIGURATION, data & (~RESET) );
    return 1;
}

static char clear(void *parameters)
{
    writeRegister( I2C_PORT, FIFO_WRITE_POINTER, 0x00 ); 
    writeRegister( I2C_PORT, FIFO_READ_POINTER, 0x00 ); 
    writeRegister( I2C_PORT, OVER_FLOW_COUNTER, 0x00 );
    return 1; 
}

static char initialize_max30100_driver(void *parameters)
{
    initI2C();
    initMax30100();
    max30100_driver.driver_id = (int) parameters;
    ESP_LOGI(TAG, "Driver inicializado");
    return 1;
}

ac_driver_t* ac_get_max30100_driver(void)
{
  max30100_driver.driver_initialization = initialize_max30100_driver;
  max30100_functions[MAX30100_GET_RAW_DATA] = get_raw_data;
  max30100_functions[MAX30100_START_TEMPERTURA_READING] = start_temperature_reading;
  max30100_functions[MAX30100_IS_TEMPERATURE_READY] = is_temperature_ready;
  max30100_functions[MAX30100_GET_TEMPERATURE] = get_temperature;
  max30100_functions[MAX30100_GET_FIFO_WRITE_POINTER] = get_fifo_write_pointer;
  max30100_functions[MAX30100_GET_OVER_FLOW_COUNTER] = get_over_flow_counter;
  max30100_functions[MAX30100_GET_FIFO_READ_POINTER] = get_fifo_read_pointer;
  max30100_functions[MAX30100_GET_INTERRUPT_STATUS] = get_interrupt_status;
  max30100_functions[MAX30100_SET_MODE] = set_mode;
  max30100_functions[MAX30100_SET_SAMPLING] = set_sampling;
  max30100_functions[MAX30100_SET_PULSE_WIDTH] = set_pulse_width;
  max30100_functions[MAX30100_SET_LED_CURRENT_RED] = set_led_current_red;
  max30100_functions[MAX30100_SET_LED_CURRENT_IR] = set_led_current_ir;
  max30100_functions[MAX30100_SET_HIGH_RES] = set_high_res;
  max30100_functions[MAX30100_STANDBY] = standby;
  max30100_functions[MAX30100_RESET] = reset;
  max30100_functions[MAX30100_CLEAR] = clear;
  max30100_driver.driver_function = &max30100_functions[0]; //Estudar.
  ESP_LOGI(TAG, "Get driver");
  return &max30100_driver;
}

static void initI2C(void)
{
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_SDA,
        .scl_io_num = I2C_SCL,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_FRQ,
    };
    i2c_param_config(I2C_PORT, &conf);
    i2c_driver_install(I2C_PORT, I2C_MODE_MASTER, 0, 0, 0);
}

static void initMax30100(void)
{
    setMode(SPO2_HR_MODE);
    setSampling(SAMPLING_50HZ);
    setPulseWidth(PULSE_WIDTH_1600US_ADC_16);
    setLedCurrentRed(LED_CURRENT_14_2MA);
    setLedCurrentIr(LED_CURRENT_14_2MA);
    setHighRes(1);
}

static void readRegister( i2c_port_t i2c_port, uint8_t address, uint8_t* reg, uint8_t size )
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (DEVICE_ADDRESS << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, address, true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (DEVICE_ADDRESS << 1) | I2C_MASTER_READ, true);
    if(size > 1) i2c_master_read(cmd, reg, size-1, I2C_MASTER_ACK); //0 is ACK
    i2c_master_read_byte(cmd, reg+size-1, I2C_MASTER_NACK); //1 is NACK
    i2c_master_stop(cmd);
    i2c_master_cmd_begin( i2c_port, cmd, 1000 / portTICK_RATE_MS );
    i2c_cmd_link_delete(cmd);
}

static void writeRegister( i2c_port_t i2c_port, uint8_t address, uint8_t val )
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (DEVICE_ADDRESS << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, address, true); // send register address
    i2c_master_write_byte(cmd, val, true); // send value to write
    i2c_master_stop(cmd);
    i2c_master_cmd_begin( i2c_port, cmd, 1000 / portTICK_RATE_MS );
    i2c_cmd_link_delete(cmd);
}

static void setMode(int mode)
{
    uint8_t data;
    readRegister( I2C_PORT, MODE_CONFIGURATION, &data, 1 );
    writeRegister( I2C_PORT, MODE_CONFIGURATION, (data & 0xF8) | mode );
}

static void setSampling(int sampling)
{
    uint8_t data;
    readRegister( I2C_PORT, SPO2_CONFIGURATION, &data, 1 );
    writeRegister( I2C_PORT, SPO2_CONFIGURATION, (data & 0xE3) | (sampling<<2) );
}

static void setPulseWidth(int pulse)
{
    uint8_t data;
    readRegister( I2C_PORT, SPO2_CONFIGURATION, &data, 1 );
    writeRegister( I2C_PORT, SPO2_CONFIGURATION, (data & 0xFC) | pulse );
}

static void setLedCurrentRed(int current_red)
{
    uint8_t data;
    readRegister( I2C_PORT, LED_CONFIGURATION, &data, 1 );
    writeRegister( I2C_PORT, LED_CONFIGURATION, (data & 0x0F) | (current_red << 4) );
}

static void setLedCurrentIr(int current_ir)
{
    uint8_t data;
    readRegister( I2C_PORT, LED_CONFIGURATION, &data, 1 );
    writeRegister( I2C_PORT, LED_CONFIGURATION, (data & 0xF0) | current_ir );
}

static void setHighRes(int enable)
{
    uint8_t data;
    readRegister( I2C_PORT, SPO2_CONFIGURATION, &data, 1);
    if(enable == 1) writeRegister( I2C_PORT, SPO2_CONFIGURATION, data | ENABLE_SPO2_HI );
    else if(enable == 0) writeRegister( I2C_PORT, SPO2_CONFIGURATION, data & (~ENABLE_SPO2_HI) );
}