#include "electromagnetic_valve_control.h"

/**
  * @brief      打开电磁阀(将L298N的PWM设置成一直为1)
  * @return     none
  */
void open_electromagnetic_valve(void)
{
    HAL_GPIO_WritePin(GPIOG, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5
    |GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11,
    GPIO_PIN_SET);
}

/**
  * @brief      关闭电磁阀
  * @return     none
  */
void close_electromagnetic_valve(void)
{
    HAL_GPIO_WritePin(GPIOG, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5
    |GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11,
    GPIO_PIN_RESET);
}
