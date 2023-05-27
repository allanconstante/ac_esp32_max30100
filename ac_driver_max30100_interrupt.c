#include "ac_driver_max30100_interrupt.h"

#define PORT  GPIO_NUM_17
#define TAG   "Interrupt Driver Max30100"

static ac_driver_t max30100_driver_interrupt;
static ac_driver_function_pointer_t max30100_functions_interrupt[END_MAX30100_INTERRUPT];
static ac_driver_interrupt_function_pointer_t interrupt_function_max30100;

static char initialize_max30100_driver_interrupt(void *parameters);
static char set_interrupt_function(void *parameters);
static char enable_interrupt(void *parameters);
static char disable_interrupt(void *parameters);

static void initPort(void);
static void IRAM_ATTR interruptFunction(void *parameters);

static char set_interrupt_function(void *parameters)
{
  interrupt_function_max30100 = (ac_driver_interrupt_function_pointer_t) parameters;
  return 1;
}

static char enable_interrupt(void *parameters)
{
  gpio_isr_handler_add(PORT, interruptFunction, (void *) PORT);
  return 1;
}

static char disable_interrupt(void *parameters)
{
  gpio_isr_handler_remove(PORT);
  return 1;
}

static char initialize_max30100_driver_interrupt(void *parameters)
{
    initPort();
    max30100_driver_interrupt.driver_id = (int) parameters;
    ESP_LOGI(TAG, "Driver inicializado");
    return 1;
}

ac_driver_t* ac_get_max30100_driver_interrupt(void)
{
  max30100_driver_interrupt.driver_initialization = initialize_max30100_driver_interrupt;
  max30100_functions_interrupt[MAX30100_SET_INTERRUPT] = set_interrupt_function;
  max30100_functions_interrupt[MAX30100_ENABLE_INTERRUPT] = enable_interrupt;
  max30100_functions_interrupt[MAX30100_DISABLE_INTERRUPT] = disable_interrupt;
  max30100_driver_interrupt.driver_function = &max30100_functions_interrupt[0]; //Estudar.
  ESP_LOGI(TAG, "Get driver");
  return &max30100_driver_interrupt;
}

static void initPort(void)
{
    gpio_pad_select_gpio(PORT);
    gpio_set_direction(PORT, GPIO_MODE_INPUT);
    gpio_pulldown_dis(PORT);
    gpio_pullup_en(PORT);
    gpio_set_intr_type(PORT,GPIO_INTR_NEGEDGE);
    gpio_install_isr_service(0);
    //gpio_set_level(CONFIG_GPIO_NUMBER, 1);
}

static void IRAM_ATTR interruptFunction(void *parameters)
{
  interrupt_function_max30100(parameters);
}