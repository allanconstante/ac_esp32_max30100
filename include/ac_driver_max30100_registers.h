#pragma once

#ifdef __cplusplus
extern "C" {
#endif

//ENDEREÇOS
#define DEVICE_ADDRESS              0x57
#define INTERRUPT_STATUS            0x00
#define INTERRUPT_ENABLE            0x01
#define FIFO_WRITE_POINTER          0x02
#define OVER_FLOW_COUNTER           0x03
#define FIFO_READ_POINTER           0x04
#define FIFO_DATA_REGISTER          0x05
#define MODE_CONFIGURATION          0x06
#define SPO2_CONFIGURATION          0x07
#define LED_CONFIGURATION           0x09
#define TEMP_INTEGER                0x16
#define TEMP_FRACTION               0x17
#define REVISION_ID                 0xFE
#define PART_ID                     0xFF

#define ENABLE_INTERRUPT_A_FULL     (1<<7)
#define ENABLE_INTERRUPT_TEMP_RDY   (1<<6)
#define ENABLE_INTERRUPT_HR_RDY     (1<<5)
#define ENABLE_INTERRUPT_SPO2_RDY   (1<<4)

#define STANDBY                     (1<<7)
#define RESET                       (1<<6)
#define READ_TEMPERATURE            (1<<3)
#define ENABLE_SPO2_HI              (1<<6)

typedef enum
{
    UNUSED_MODE                 = 0x00,
    HR_MODE                     = 0x02,
    SPO2_HR_MODE                = 0x03
} sensor_mode_t;

typedef enum
{
    SAMPLING_50HZ               = 0x00,
    SAMPLING_100HZ              = 0x01,
    SAMPLING_167HZ              = 0x02,
    SAMPLING_200HZ              = 0x03,
    SAMPLING_400HZ              = 0x04,
    SAMPLING_600HZ              = 0x05,
    SAMPLING_800HZ              = 0x06,
    SAMPLING_1000HZ             = 0x07
} sampling_t;

typedef enum
{
    PULSE_WIDTH_200US_ADC_13    = 0x00,
    PULSE_WIDTH_400US_ADC_14    = 0x01,
    PULSE_WIDTH_800US_ADC_15    = 0x02,
    PULSE_WIDTH_1600US_ADC_16   = 0x03
} pulse_width_t;

typedef enum
{
    LED_CURRENT_0MA             = 0x00,
    LED_CURRENT_4_4MA           = 0x01,
    LED_CURRENT_7_6MA           = 0x02,
    LED_CURRENT_11MA            = 0x03,
    LED_CURRENT_14_2MA          = 0x04,
    LED_CURRENT_17_4MA          = 0x05,
    LED_CURRENT_20_8MA          = 0x06,
    LED_CURRENT_24MA            = 0x07,
    LED_CURRENT_27_1MA          = 0x08,
    LED_CURRENT_30_6MA          = 0x09,
    LED_CURRENT_33_8MA          = 0x0A,
    LED_CURRENT_37MA            = 0x0B,
    LED_CURRENT_40_2MA          = 0x0C,
    LED_CURRENT_43_6MA          = 0x0D,
    LED_CURRENT_46_8MA          = 0x0E,
    LED_CURRENT_50MA            = 0x0F
} led_current_t;

#ifdef __cplusplus
}
#endif