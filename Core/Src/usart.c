/**
  ******************************************************************************
  * @file    usart.c
  * @brief   This file provides code for the configuration
  *          of the USART instances.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "usart.h"

/* USER CODE BEGIN 0 */
#include "motor_control.h"
/* USER CODE END 0 */

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USART1 init function */

void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_9B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_EVEN;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}
/* USART2 init function */

void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 9600;
  huart2.Init.WordLength = UART_WORDLENGTH_9B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_EVEN;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

void HAL_UART_MspInit(UART_HandleTypeDef* uartHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(uartHandle->Instance==USART1)
  {
  /* USER CODE BEGIN USART1_MspInit 0 */

  /* USER CODE END USART1_MspInit 0 */
    /* USART1 clock enable */
    __HAL_RCC_USART1_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**USART1 GPIO Configuration
    PA9     ------> USART1_TX
    PA10     ------> USART1_RX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_9;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_10;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* USART1 interrupt Init */
    HAL_NVIC_SetPriority(USART1_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(USART1_IRQn);
  /* USER CODE BEGIN USART1_MspInit 1 */

  /* USER CODE END USART1_MspInit 1 */
  }
  else if(uartHandle->Instance==USART2)
  {
  /* USER CODE BEGIN USART2_MspInit 0 */

  /* USER CODE END USART2_MspInit 0 */
    /* USART2 clock enable */
    __HAL_RCC_USART2_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**USART2 GPIO Configuration
    PA2     ------> USART2_TX
    PA3     ------> USART2_RX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_2;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_3;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* USART2 interrupt Init */
    HAL_NVIC_SetPriority(USART2_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(USART2_IRQn);
  /* USER CODE BEGIN USART2_MspInit 1 */

  /* USER CODE END USART2_MspInit 1 */
  }
}

void HAL_UART_MspDeInit(UART_HandleTypeDef* uartHandle)
{

  if(uartHandle->Instance==USART1)
  {
  /* USER CODE BEGIN USART1_MspDeInit 0 */

  /* USER CODE END USART1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_USART1_CLK_DISABLE();

    /**USART1 GPIO Configuration
    PA9     ------> USART1_TX
    PA10     ------> USART1_RX
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_9|GPIO_PIN_10);

    /* USART1 interrupt Deinit */
    HAL_NVIC_DisableIRQ(USART1_IRQn);
  /* USER CODE BEGIN USART1_MspDeInit 1 */

  /* USER CODE END USART1_MspDeInit 1 */
  }
  else if(uartHandle->Instance==USART2)
  {
  /* USER CODE BEGIN USART2_MspDeInit 0 */

  /* USER CODE END USART2_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_USART2_CLK_DISABLE();

    /**USART2 GPIO Configuration
    PA2     ------> USART2_TX
    PA3     ------> USART2_RX
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_2|GPIO_PIN_3);

    /* USART2 interrupt Deinit */
    HAL_NVIC_DisableIRQ(USART2_IRQn);
  /* USER CODE BEGIN USART2_MspDeInit 1 */

  /* USER CODE END USART2_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */
/**
  * @brief      串口回调函数,被串口的中断函数所调用
  * @param[in]  串口
  * @return     none
  */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    // 判断是由哪个串口触发的中断
    if(huart->Instance == USART1)
    {
        read_buffer[current_buffer_length] = single_buffer;
        current_buffer_length++;

        // 接收到数据马上使用串口1发送出去
        // HAL_UART_Transmit(&huart1, &single_buffer, 1, 100);
        // 重新使能串口1接收中断(否则只能接收一次)
        HAL_UART_Receive_IT(&huart1, &single_buffer, 1);
    }
}

/**
  * @brief      空闲中断处理函数(自己写的),目的是为了数据多发或者发送较快时不会出错
  * @param[in]  串口
  * @return     none
  */
void UART_IDLECallBack(UART_HandleTypeDef *huart)
{
    /*uart1 idle processing function*/
    if(huart == &huart1)
    {
        current_buffer_length = 0;
        protocol_handle();
        HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_3);
        __HAL_UART_CLEAR_IDLEFLAG(&huart1); // 清除空闲中断标志(否则会一直不断进入中断)
    }
}

/**
  * @brief      上下位机通讯协议
  * @return     none
  */
extern pid_parameter motor_pid[2];
extern valve_parameter valve[2];
void protocol_handle(void)
{
    // CRC校验
//    uint8_t low_value = 0x00, high_value = 0x00;
//    CRC16_MODBUS(read_buffer, 6, &low_value, &high_value);
//    if (read_buffer[6] != low_value || read_buffer[7] != high_value)
//        return;
    switch (read_buffer[1])
    {
        case 0x01: // 控制灯改变亮灭状态
        {
//            if (read_buffer[2] == 0x01)
//                HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_2);
//            if (read_buffer[3] == 0x01)
//                HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_3);
            if (read_buffer[4] == 0x01)
                HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_4);
            if (read_buffer[5] == 0x01)
                HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_5);
            //printf("LED complete.");
            break;
        }
        case 0x02: // 电机控制
        {
            int32_t pwm_value = 0;
            memcpy(&pwm_value, read_buffer + 2, sizeof(int32_t));

            //printf("pwm_value: %d, motor id: %d\r\n", pwm_value, read_buffer[0]);
            //motor_drive_instruct(read_buffer[0], pwm_value);
            break;
        }
        case 0x03: // 将当前位置设置为新的零点
        {
            if(read_buffer[0] == motor_pid[0].motor_id)
            {
                pid_parameter_init(&motor_pid[0], motor_pid[0].motor_id, htim2);
                //uint8_t return_data[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
                //HAL_UART_Transmit(&huart1, return_data, sizeof(return_data), 500);
                //printf("pos_set[%02X]: %f, pos_curr[%02X]: %f\r\n", motor_pid[0].motor_id, motor_pid[0].pos_set,
                //                                                    motor_pid[0].motor_id, motor_pid[0].pos_curr);
            }
            else if(read_buffer[0] == motor_pid[1].motor_id)
            {
                pid_parameter_init(&motor_pid[1], motor_pid[1].motor_id, htim3);
                //printf("pos_set[%02X]: %f, pos_curr[%02X]: %f\r\n", motor_pid[1].motor_id, motor_pid[1].pos_set,
                //                                                    motor_pid[1].motor_id, motor_pid[1].pos_curr);
            }
            break;
        }
        case 0x04: // 电机的位置PID控制
        {
            float pos_set = 0;
            memcpy(&pos_set, read_buffer + 2, sizeof(float));
            
            if(read_buffer[0] == motor_pid[0].motor_id)
            {
                motor_pid[0].pos_set = pos_set;
//                printf("pos_set[%02X]: %f, pos_curr[%02X]: %f\r\n", motor_pid[0].motor_id, motor_pid[0].pos_set,
//                                                                    motor_pid[0].motor_id, motor_pid[0].pos_curr);
//                uint8_t return_data[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
//                HAL_UART_Transmit(&huart1, return_data, sizeof(return_data), 500);
            }
            else if(read_buffer[0] == motor_pid[1].motor_id)
            {
                motor_pid[1].pos_set = pos_set;
//                printf("pos_set[%02X]: %f, pos_curr[%02X]: %f\r\n", motor_pid[1].motor_id, motor_pid[1].pos_set,
//                                                                    motor_pid[1].motor_id, motor_pid[1].pos_curr);
//                uint8_t return_data[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
//                HAL_UART_Transmit(&huart1, return_data, sizeof(return_data), 500);
            }
            break;
        }
        case 0x05: // 返回对应ID的当前位置
        {
            if(read_buffer[0] == motor_pid[0].motor_id)
            {
                uint8_t return_data[8] = {0x00, 0x09, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
                return_data[0] = motor_pid[0].motor_id;
                memcpy(return_data + 2, &motor_pid[0].pos_curr, 4);
                HAL_UART_Transmit(&huart1, return_data, sizeof(return_data), 500);
            }
            else if(read_buffer[0] == motor_pid[1].motor_id)
            {
                uint8_t return_data[8] = {0x00, 0x09, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
                return_data[0] = motor_pid[1].motor_id;
                memcpy(return_data + 2, &motor_pid[1].pos_curr, 4);
                HAL_UART_Transmit(&huart1, return_data, sizeof(return_data), 500);
            }
            break;
        }
        case 0x06: // 控制电磁阀和真空发生器
        {
            uint16_t valve_instruct = read_buffer[2];
            valve_instruct = (valve_instruct << 8) | read_buffer[3];
            
            if (valve_instruct & (0x01 << (valve[0].valve_id - 1)))
            {
                if(valve[0].valve_id == 0x09)
                    open_vacuum_generator(valve[0].valve_id);
                else open_electromagnetic_valve(valve[0].valve_pin);
            }
            else
            {
                if(valve[0].valve_id == 0x09)
                    close_vacuum_generator(valve[0].valve_id);
                else close_electromagnetic_valve(valve[0].valve_pin);
            }
            
            if (valve_instruct & (0x01 << (valve[1].valve_id - 1)))
            {
                if(valve[1].valve_id == 0x0a)
                    open_vacuum_generator(valve[1].valve_id);
                else open_electromagnetic_valve(valve[1].valve_pin);
            }
            else
            {
                if(valve[1].valve_id == 0x0a)
                    close_vacuum_generator(valve[1].valve_id);
                else close_electromagnetic_valve(valve[1].valve_pin);
            }
            break;
        }
        default:
            break;
    }
}

/**
  * @brief      CRC循环冗余校验
  * @param[in]  待校验的数组
  * @param[in]  待校验的数组的大小
  * @param[out] CRC码低字节
  * @param[out] CRC码高字节
  * @return     none
  */
void CRC16_MODBUS(uint8_t input[], int size, uint8_t* low_value, uint8_t* high_value)
{
    uint16_t crc = 0xffff;
    for (int n = 0; n < size; n++) {/*此处的6 -- 要校验的位数为6个*/
        crc = input[n] ^ crc;
        for (int i = 0; i < 8; i++) {  /*此处的8 -- 指每一个char类型又8bit，每bit都要处理*/
            if (crc & 0x01) {
                crc = crc >> 1;
                crc = crc ^ 0xa001;
            }
            else {
                crc = crc >> 1;
            }
        }
    }
    *low_value = crc & 0xFF;
    *high_value = (uint8_t)(crc >> 8);
}

// 重定向c库函数printf到串口DEBUG_USART，重定向后可使用printf函数
int fputc(int ch, FILE *f)
{
    /* 发送一个字节数据到串口DEBUG_USART */
    HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, 1000);
    return (ch);
}

// 重定向c库函数scanf到串口DEBUG_USART，重写向后可使用scanf、getchar等函数
int fgetc(FILE *f)
{
    int ch;
    HAL_UART_Receive(&huart1, (uint8_t *)&ch, 1, 1000);	
    return (ch);
}
/* USER CODE END 1 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
