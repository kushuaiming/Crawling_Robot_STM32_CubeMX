#include "electromagnetic_valve_control.h"

/**
  * @brief      ��ʼ����ŷ�
  * @param[in]  ��ŷ�ID
  * @return     none
  */
void valve_init(valve_parameter* valve, uint8_t id, uint16_t pin)
{
    valve->valve_id = id;
    valve->valve_pin = pin;
}

/**
  * @brief      �򿪵�ŷ�(��L298N��PWM���ó�һֱΪ1)
  * @return     none
  */
void open_electromagnetic_valve(uint16_t pin)
{
    HAL_GPIO_WritePin(GPIOG, pin, GPIO_PIN_SET);
}

/**
  * @brief      �رյ�ŷ�
  * @return     none
  */
void close_electromagnetic_valve(uint16_t pin)
{
    HAL_GPIO_WritePin(GPIOG, pin, GPIO_PIN_RESET);
}
