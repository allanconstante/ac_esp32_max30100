
#include "ac_esp32_max30100.h"

static max30100_t device;

static esp_err_t readRegister( i2c_port_t, uint8_t, uint8_t*, uint8_t );
static esp_err_t writeRegister( i2c_port_t, uint8_t, uint8_t );

max30100_t get_max30100_t(void)
{
    return device;
}

char initialize_max30100(i2c_port_t port)
{
    device.i2c_port = port;
    device.mode = SPO2_HR_MODE;
    device.sampling = SAMPLING_50HZ;
    device.pulse_width = PULSE_WIDTH_1600US_ADC_16;
    device.led_current_red = LED_CURRENT_14_2MA;
    device.led_current_ir = LED_CURRENT_24MA;
    device.high_resolution = true;

    set_mode(device.mode);
    set_sampling(device.sampling);
    set_pulse_width(device.pulse_width);
    set_led_current_red(device.led_current_red);
    set_led_current_ir(device.led_current_ir);
    set_high_res(device.high_resolution);
    
    return 1;
}

char set_mode(int mode)
{
    uint8_t data;
    readRegister( device.i2c_port, MODE_CONFIGURATION, &data, 1 );
    writeRegister( device.i2c_port, MODE_CONFIGURATION, (data & 0xF8) | mode );
    return 1;
}

char set_sampling(int sampling)
{
    uint8_t data;
    readRegister( device.i2c_port, SPO2_CONFIGURATION, &data, 1 );
    writeRegister( device.i2c_port, SPO2_CONFIGURATION, (data & 0xE3) | (sampling<<2) );
    return 1;
}

char set_pulse_width(int pulse)
{
    uint8_t data;
    readRegister( device.i2c_port, SPO2_CONFIGURATION, &data, 1 );
    writeRegister( device.i2c_port, SPO2_CONFIGURATION, (data & 0xFC) | pulse );
    return 1;
}

char set_led_current_red(int current_red)
{
    uint8_t data;
    readRegister( device.i2c_port, LED_CONFIGURATION, &data, 1 );
    writeRegister( device.i2c_port, LED_CONFIGURATION, (data & 0x0F) | (current_red << 4) );
    return 1;
}

char set_led_current_ir(int current_ir)
{
    uint8_t data;
    readRegister( device.i2c_port, LED_CONFIGURATION, &data, 1 );
    writeRegister( device.i2c_port, LED_CONFIGURATION, (data & 0xF0) | current_ir );
    return 1;
}

char set_high_res(bool enable)
{
    uint8_t data;
    readRegister( device.i2c_port, SPO2_CONFIGURATION, &data, 1);
    if(enable) writeRegister( device.i2c_port, SPO2_CONFIGURATION, data | ENABLE_SPO2_HI );
    else if(enable) writeRegister( device.i2c_port, SPO2_CONFIGURATION, data & (~ENABLE_SPO2_HI) );
    return 1;
}

char get_raw_data(uint8_t *data_register)
{
    readRegister(device.i2c_port, FIFO_DATA_REGISTER, data_register, 4);
    return 1;
}

char get_temperature(void *parameters)
{
    int8_t temp;
    int8_t temp_fraction;
    float *temperature = (float*)parameters;
    readRegister( device.i2c_port, TEMP_INTEGER, (uint8_t*)&temp, 1 );
    readRegister( device.i2c_port, TEMP_FRACTION, (uint8_t*)&temp_fraction, 1);
    *temperature = (float)temp + (float)(temp_fraction * 0.0625);
    return 1;
}

char get_fifo_write_pointer(uint8_t *fifo_write_register)
{
    readRegister( device.i2c_port, FIFO_WRITE_POINTER, fifo_write_register, 1 );
    return 1;
}

char get_over_flow_counter(void *parameters)
{
    uint8_t *data = (uint8_t*) parameters;
    readRegister( device.i2c_port, OVER_FLOW_COUNTER, data, 1 );
    return 1;
}

char get_fifo_read_pointer(uint8_t *fifo_read_register)
{
    readRegister( device.i2c_port, FIFO_READ_POINTER, fifo_read_register, 1 );
    return 1;
}

char get_interrupt_status(void *parameters)
{
    uint8_t *data = (uint8_t*) parameters;
    readRegister( device.i2c_port, INTERRUPT_STATUS, data, 1 );
    return 1;
}

char start_temperature_reading(void)
{
    uint8_t data;
    readRegister( device.i2c_port, MODE_CONFIGURATION, &data, 1 );
    writeRegister( device.i2c_port, MODE_CONFIGURATION, (data | READ_TEMPERATURE) ); 
    return 1;
}

bool is_temperature_ready(void)
{
    uint8_t data;
    readRegister( device.i2c_port, MODE_CONFIGURATION, &data, 1 );
    if ( ( data & ( 1<<3 ) ) ) return true;
    else return false;
}

char standby_max30100(bool enable)
{
    uint8_t data;
    readRegister( device.i2c_port, MODE_CONFIGURATION, &data, 1);
    if(enable) writeRegister( device.i2c_port, MODE_CONFIGURATION, data | STANDBY );
    else writeRegister( device.i2c_port, MODE_CONFIGURATION, data & (~STANDBY) );
    return 1;
}

char reset_max30100(bool enable)
{
    uint8_t data;
    readRegister( device.i2c_port, MODE_CONFIGURATION, &data, 1);
    if(enable) writeRegister( device.i2c_port, MODE_CONFIGURATION, data | RESET );
    else writeRegister( device.i2c_port, MODE_CONFIGURATION, data & (~RESET) );
    return 1;
}

char clear_max30100(void)
{
    writeRegister( device.i2c_port, FIFO_WRITE_POINTER, 0x00 ); 
    writeRegister( device.i2c_port, FIFO_READ_POINTER, 0x00 ); 
    writeRegister( device.i2c_port, OVER_FLOW_COUNTER, 0x00 );
    return 1; 
}

static esp_err_t readRegister( i2c_port_t i2c_port, uint8_t address, uint8_t* reg, uint8_t size )
{
    char state = 0;
    esp_err_t error;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();

    while ( state <  9 ) {
        switch ( state ) {
            case 0:
                error = i2c_master_start( cmd );
            break;

            case 1:
                error = i2c_master_write_byte( cmd, (DEVICE_ADDRESS << 1) | I2C_MASTER_WRITE, true );
            break;

            case 2:
                error = i2c_master_write_byte( cmd, address, true );
            break;

            case 3:
                error = i2c_master_start( cmd );
            break;

            case 4:
                error = i2c_master_write_byte( cmd, (DEVICE_ADDRESS << 1) | I2C_MASTER_READ, true );
            break;

            case 5:
                if( size > 1 ) error = i2c_master_read( cmd, reg, size-1, I2C_MASTER_ACK );
            break;

            case 6:
                error = i2c_master_read_byte( cmd, reg+size-1, I2C_MASTER_NACK );
            break;

            case 7:
                error = i2c_master_stop( cmd );
            break;

            case 8:
                error = i2c_master_cmd_begin( i2c_port, cmd, 1000 / portTICK_RATE_MS );
            break;

            default:
                error = ESP_FAIL;
            break;
        }
        if( error == ESP_OK ) ++state;
        else break;
    }
    i2c_cmd_link_delete( cmd );
    return error;
}

static esp_err_t writeRegister( i2c_port_t i2c_port, uint8_t address, uint8_t val )
{
    char state = 0;
    esp_err_t error;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();

    while ( state < 6 ) {
        switch ( state ) {
            case 0:
                error = i2c_master_start( cmd );
            break;

            case 1:
                error = i2c_master_write_byte( cmd, (DEVICE_ADDRESS << 1) | I2C_MASTER_WRITE, true );
            break;

            case 2:
                error = i2c_master_write_byte( cmd, address, true );
            break;

            case 3:
                error = i2c_master_write_byte( cmd, val, true );
            break;

            case 4:
                error = i2c_master_stop( cmd );
            break;

            case 5:
                error = i2c_master_cmd_begin( i2c_port, cmd, 1000 / portTICK_RATE_MS );
            break;
        
            default:
                error = ESP_FAIL;
            break;
        }
        if( error == ESP_OK ) ++state;
        else break;
    }
    i2c_cmd_link_delete( cmd );
    return error;
}