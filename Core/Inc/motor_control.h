#ifndef __MOTOR_CONTROL_H__
#define __MOTOR_CONTROL_H__

#include <stdint.h>
#include <stdio.h>

#include "usart.h"
#include "tim.h"
#include "math.h"

typedef struct {
    uint8_t motor_id;
    int16_t encoder_value;
    
    TIM_HandleTypeDef* htim;
    
    float direction;
    float pos_set;
    float pos_curr;

    float err_curr;
    float err_prev;
    
    float integrate;
    
    float Kp;
    float Ki;
    float Kd;
    
    float output;
    int Ph;                          // 滚珠丝杠的导程
    float encoder_number_per_circle; // 直流电机旋转一周输出3个脉冲(PPR)
    float reduction_ratio;           // 减速比
} pid_parameter;

int32_t encoder_read(pid_parameter* pid);
void motor_drive_instruct(uint8_t motor_id, int16_t pwm_value);
void pid_parameter_init(pid_parameter* pid, uint8_t id, TIM_HandleTypeDef* htim);
float pid_calculate(pid_parameter* pid, float dt);
float pwm_value_piecewise_function(float postion_error);
float sign(float num);
#endif
