#include "electromagnetic_valve_control.h"

/**
  * @brief      初始化电磁阀
  * @param[in]  电磁阀ID
  * @return     none
  */
void valve_init(valve_parameter* valve, uint8_t id, uint16_t pin)
{
    valve->valve_id = id;
    valve->valve_pin = pin;
}

/**
  * @brief      打开电磁阀(将L298N的PWM设置成一直为1)
  * @return     none
  */
void open_electromagnetic_valve(uint16_t pin)
{
    HAL_GPIO_WritePin(GPIOG, pin, GPIO_PIN_SET);
}

/**
  * @brief      关闭电磁阀
  * @return     none
  */
void close_electromagnetic_valve(uint16_t pin)
{
    HAL_GPIO_WritePin(GPIOG, pin, GPIO_PIN_RESET);
}
