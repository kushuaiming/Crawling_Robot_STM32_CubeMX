#include "motor_control.h"

uint8_t data[8];
/**
  * @brief      计算驱动器可执行的指令
  * @param[in]  电机的ID,由驱动器拨码开关控制,从0x01开始.参见《电机驱动器AQMD3610NS》2.1.5
  * @param[in]  PWM的数值,范围从-1000~1000
  * @return     none
  */
void motor_drive_instruct(uint8_t motor_id, int32_t pwm_value)
{
    // 对pwm进行限位
    if (pwm_value > 1000)
        pwm_value = 1000;
    else if (pwm_value < -1000)
        pwm_value = -1000;

    // data[0]电机ID
    data[0] = motor_id;
    // data[1]写单个寄存器
    data[1] = 0x06;
    // data[2]寄存器高字节地址,data[3]寄存器低字节地址
    // 速度控制寄存器地址为0x0040
    data[2] = 0x00;
    data[3] = 0x40;
    // data[4]数据高字节,data[5]数据低字节
    data[4] = (uint8_t)(pwm_value >> 8);
    data[5] = pwm_value & 0xFF;
    // data[6]CRC码低字节,data[7]CRC码高字节
    CRC16_MODBUS(data, 6, &data[6], &data[7]);
    HAL_UART_Transmit(&huart2, data, sizeof(data), 500);
    //printf("Motor successfully controled.");
}

extern pid_parameter motor_pid[2];
/**
  * @brief      pid参数初始化
  * @param[in]  pid控制变量
  * @return     none
  */
void pid_parameter_init(pid_parameter* pid, uint8_t id, TIM_HandleTypeDef htim)
{
    pid->motor_id = id;
    pid->encoder_value = 0;
    pid->htim = htim;

    pid->direction = -1.0;

    pid->pos_set = 0.0f;
    pid->pos_curr = 0.0f;
    pid->err_curr = 0.0f;
    pid->err_prev = 0.0f;

    pid->integrate = 0.0f;

    pid->Kp = 30.0f;
    pid->Ki = 0.1f;
    pid->Kd = 3.7f;

    pid->Ph = 4;
    pid->encoder_number_per_circle = 3.0f;
    pid->reduction_ratio = 1 / 181.0f;
}

/**
  * @brief      读取编码器数值
  * @param[in]  计时器
  * @return     编码器数值
  */
int32_t encoder_read(pid_parameter* pid)
{
    int16_t encoder = (int16_t)(__HAL_TIM_GET_COUNTER(&(pid->htim)));
    __HAL_TIM_SET_COUNTER(&(pid->htim),0);
    return encoder;
}

/**
  * @brief      计算PID
  * @param[in]  自定义的PID控制结构体
  * @param[in]  单位时间
  * @return     none
  */

float pid_calculate(pid_parameter* pid, float dt)
{
    pid->encoder_value += encoder_read(pid);
    // 最后有一个除2的修正,单位为mm
    pid->pos_curr = pid->encoder_value / pid->encoder_number_per_circle * pid->reduction_ratio * pid->Ph / 2;
    pid->err_curr = pid->pos_set - pid->pos_curr;

    // 分段函数计算输出pwm值
    float output = pwm_value_piecewise_function(pid->err_curr);
    output *= pid->direction;
    
    // pid计算输出pwm值
//    pid->integrate += pid->err_curr * dt;
//    float output = pid->Kp * pid->err_curr + pid->Ki * pid->integrate + pid->Kd * (pid->err_curr - pid->err_prev) / dt;
//    pid->err_prev = pid->err_curr;
    return output;
}

const float range1 = 0.02f;
const float range2 = 1.0f;
const float range3 = 5.0f;
const float high_range = 1000.0f;
const float low_range = 200.0f;
/**
  * @brief      分段函数,横坐标为误差值,纵坐标为电机的pwm值
  * @param[in]  位置误差
  * @return     pwmvalue
  */
float pwm_value_piecewise_function(float position_error)
{
    float pwm_value = 0.0f;
    if (fabs(position_error) > range3)
        pwm_value = high_range * sign(position_error);
    else if (fabs(position_error) <= range3 && fabs(position_error) > range2)
    {
        float k = (high_range - low_range) / (range3 - range2);
        float b = high_range - range3 * k;
        pwm_value = (k * fabs(position_error) - b) * sign(position_error);
    }
    else if (fabs(position_error) <= range2 && fabs(position_error) > range1)
        pwm_value = low_range * sign(position_error);
    else if (fabs(position_error) <= range1)
        pwm_value = 0.0f;
    return pwm_value;
}

/**
  * @brief      符号函数
  * @param[in]  输入值
  * @return     0 1 -1
  */
float sign(float num)
{
    if (num > 0)
        return 1.0f;
    else if (num < 0)
        return -1.0f;
    else return 0.0f;
}
