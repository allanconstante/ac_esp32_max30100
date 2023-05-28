#include "ac_esp32_max30100.h"

static i2c_port_t i2c_port;

static void readRegister( i2c_port_t, uint8_t, uint8_t*, uint8_t );
static void writeRegister( i2c_port_t, uint8_t, uint8_t );

char initialize_max30100(i2c_port_t port)
{
    i2c_port = port;
    set_mode(SPO2_HR_MODE);
    set_sampling(SAMPLING_50HZ);
    set_pulse_width(PULSE_WIDTH_1600US_ADC_16);
    set_led_current_red(LED_CURRENT_14_2MA);
    set_led_current_ir(LED_CURRENT_14_2MA);
    set_high_res(true);
    return 1;
}

char set_mode(int mode)
{
    uint8_t data;
    readRegister( i2c_port, MODE_CONFIGURATION, &data, 1 );
    writeRegister( i2c_port, MODE_CONFIGURATION, (data & 0xF8) | mode );
    return 1;
}

char set_sampling(int sampling)
{
    uint8_t data;
    readRegister( i2c_port, SPO2_CONFIGURATION, &data, 1 );
    writeRegister( i2c_port, SPO2_CONFIGURATION, (data & 0xE3) | (sampling<<2) );
    return 1;
}

char set_pulse_width(int pulse)
{
    uint8_t data;
    readRegister( i2c_port, SPO2_CONFIGURATION, &data, 1 );
    writeRegister( i2c_port, SPO2_CONFIGURATION, (data & 0xFC) | pulse );
    return 1;
}

char set_led_current_red(int current_red)
{
    uint8_t data;
    readRegister( i2c_port, LED_CONFIGURATION, &data, 1 );
    writeRegister( i2c_port, LED_CONFIGURATION, (data & 0x0F) | (current_red << 4) );
    return 1;
}

char set_led_current_ir(int current_ir)
{
    uint8_t data;
    readRegister( i2c_port, LED_CONFIGURATION, &data, 1 );
    writeRegister( i2c_port, LED_CONFIGURATION, (data & 0xF0) | current_ir );
    return 1;
}

char set_high_res(bool enable)
{
    uint8_t data;
    readRegister( i2c_port, SPO2_CONFIGURATION, &data, 1);
    if(enable) writeRegister( i2c_port, SPO2_CONFIGURATION, data | ENABLE_SPO2_HI );
    else if(enable) writeRegister( i2c_port, SPO2_CONFIGURATION, data & (~ENABLE_SPO2_HI) );
    return 1;
}

char get_raw_data(uint8_t *data_register)
{
    readRegister(i2c_port, FIFO_DATA_REGISTER, data_register, 4);
    return 1;
}

char get_temperature(void *parameters)
{
    int8_t temp;
    int8_t temp_fraction;
    float *temperature = (float*)parameters;
    readRegister(i2c_port, TEMP_INTEGER, (uint8_t*)&temp, 1);
    readRegister( i2c_port, TEMP_FRACTION, (uint8_t*)&temp_fraction, 1);
    *temperature = (float)temp + (float)(temp_fraction * 0.0625);
    return 1;
}

char get_fifo_write_pointer(uint8_t *fifo_write_register)
{
    readRegister( i2c_port, FIFO_WRITE_POINTER, fifo_write_register, 1 );
    return 1;
}

char get_over_flow_counter(void *parameters)
{
    uint8_t *data = (uint8_t*) parameters;
    readRegister( i2c_port, OVER_FLOW_COUNTER, data, 1 );
    return 1;
}

char get_fifo_read_pointer(uint8_t *fifo_read_register)
{
    readRegister( i2c_port, FIFO_READ_POINTER, fifo_read_register, 1 );
    return 1;
}

char get_interrupt_status(void *parameters)
{
    uint8_t *data = (uint8_t*) parameters;
    readRegister( i2c_port, INTERRUPT_STATUS, data, 1 );
    return 1;
}

char start_temperature_reading(void)
{
    uint8_t data;
    readRegister( i2c_port, MODE_CONFIGURATION, &data, 1 );
    writeRegister( i2c_port, MODE_CONFIGURATION, (data | READ_TEMPERATURE) ); 
    return 1;
}

bool is_temperature_ready(void)
{
    uint8_t data;
    readRegister(i2c_port, MODE_CONFIGURATION, &data, 1);
    if ( (data & (1<<3)) ) return true;
    else return false;
}

char standby(void *parameters)
{
    uint8_t data;
    int *enable = (int*) parameters;
    readRegister( i2c_port, MODE_CONFIGURATION, &data, 1);
    if(*enable == 1) writeRegister( i2c_port, MODE_CONFIGURATION, data | STANDBY );
    else if(*enable == 0) writeRegister( i2c_port, MODE_CONFIGURATION, data & (~STANDBY) );
    return 1;
}

char reset(void *parameters)
{
    uint8_t data;
    int *enable = (int*) parameters;
    readRegister( i2c_port, MODE_CONFIGURATION, &data, 1);
    if(*enable == 1) writeRegister( i2c_port, MODE_CONFIGURATION, data | RESET );
    else if(*enable == 0) writeRegister( i2c_port, MODE_CONFIGURATION, data & (~RESET) );
    return 1;
}

char clear(void)
{
    writeRegister( i2c_port, FIFO_WRITE_POINTER, 0x00 ); 
    writeRegister( i2c_port, FIFO_READ_POINTER, 0x00 ); 
    writeRegister( i2c_port, OVER_FLOW_COUNTER, 0x00 );
    return 1; 
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