#include "pneumatic_control.h"

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

/**
  * @brief      打开真空发生器
  * @return     none
  */
void open_vacuum_generator(uint8_t id)
{
    if (id == 0x09)
    {
        HAL_GPIO_WritePin(GPIOG, GPIO_PIN_0, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOG, GPIO_PIN_1, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOG, GPIO_PIN_2, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOG, GPIO_PIN_3, GPIO_PIN_SET);
    } else if (id == 0x0a)
    {
        HAL_GPIO_WritePin(GPIOG, GPIO_PIN_4, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOG, GPIO_PIN_5, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOG, GPIO_PIN_6, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOG, GPIO_PIN_7, GPIO_PIN_SET);
    }

}

/**
  * @brief      关闭真空发生器
  * @return     none
  */
void close_vacuum_generator(uint8_t id)
{
    if (id == 0x09)
    {
        HAL_GPIO_WritePin(GPIOG, GPIO_PIN_0, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOG, GPIO_PIN_1, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOG, GPIO_PIN_2, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOG, GPIO_PIN_3, GPIO_PIN_RESET);
    } else if (id == 0x0a)
    {
        HAL_GPIO_WritePin(GPIOG, GPIO_PIN_4, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOG, GPIO_PIN_5, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOG, GPIO_PIN_6, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOG, GPIO_PIN_7, GPIO_PIN_RESET);
    }
}
