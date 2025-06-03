/**
  ******************************************************************************
  * @file           : pid_speed.c
  * @brief          : PI Speed Control Algorithm Implementation
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "pid_speed.h"
#include <stdio.h>
#include <string.h>

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
// 蓝牙命令定义
#define CMD_SPEED_KP_UP     'u'  // Speed Kp增加
#define CMD_SPEED_KP_DOWN   'j'  // Speed Kp减少
#define CMD_SPEED_KI_UP     'i'  // Speed Ki增加
#define CMD_SPEED_KI_DOWN   'k'  // Speed Ki减少
#define CMD_SPEED_PRINT     'v'  // 打印速度参数

/* Private variables ---------------------------------------------------------*/
static PI_Speed_Params_t pi_params = {
    .kp = PI_SPEED_DEFAULT_KP,
    .ki = PI_SPEED_DEFAULT_KI,
    .target_speed = PI_SPEED_DEFAULT_TARGET,
    .integral_limit = PI_SPEED_INTEGRAL_LIMIT
};

static PI_Speed_State_t pi_state = {
    .integral = 0.0f,
    .prev_error = 0.0f,
    .filtered_speed_error = 0.0f
};
extern UART_HandleTypeDef huart6;
/* Private function prototypes -----------------------------------------------*/
static int16_t Speed_Limit(int16_t speed, int16_t max, int16_t min);

/* Functions -----------------------------------------------------------------*/

/**
  * @brief  初始化PI速度控制器
  * @param  None
  * @retval None
  */
void PI_Speed_Init(void)
{
    // 参数已在静态初始化中设置
    PI_Speed_PrintParams();
}

/**
  * @brief  PI速度控制计算函数
  * @param  current_speed: 当前速度（编码器速度或实际速度）
  * @param  target_speed: 目标速度
  * @retval 速度控制输出 (-100 to 100)
  */
int16_t PI_Speed_Calculate(float current_speed, float target_speed)
{
    // 计算速度误差
    float speed_error = target_speed - current_speed;
    
    // 对速度误差进行一阶低通滤波
    pi_state.filtered_speed_error *= 0.84f;                    // 一阶低通滤波器
    pi_state.filtered_speed_error += speed_error * 0.16f;      // 一阶低通滤波器，减缓速度变化
    
    // 计算比例项（使用滤波后的误差）
    float proportional = pi_params.kp * pi_state.filtered_speed_error;
    
    // 计算积分项（使用滤波后的误差）
    pi_state.integral += pi_state.filtered_speed_error;
    
    // 积分限幅
    if (pi_state.integral > pi_params.integral_limit) {
        pi_state.integral = pi_params.integral_limit;
    } else if (pi_state.integral < -pi_params.integral_limit) {
        pi_state.integral = -pi_params.integral_limit;
    }
    
    float integral = pi_params.ki * pi_state.integral;
    
    // 计算总输出
    float output = proportional + integral;
    
    // 输出限幅
    int16_t speed_output = (int16_t)output;
    speed_output = Speed_Limit(speed_output, 100, -100);
    
    // 更新状态
    pi_state.prev_error = pi_state.filtered_speed_error;
    
    return speed_output;
}

/**
  * @brief  设置PI速度控制参数
  * @param  kp: 比例系数
  * @param  ki: 积分系数
  * @retval None
  */
void PI_Speed_SetParams(float kp, float ki)
{
    pi_params.kp = kp;
    pi_params.ki = ki;
    PI_Speed_PrintParams();
}

/**
  * @brief  获取当前PI速度控制参数
  * @param  kp: 比例系数指针
  * @param  ki: 积分系数指针
  * @retval None
  */
void PI_Speed_GetParams(float *kp, float *ki)
{
    if (kp) *kp = pi_params.kp;
    if (ki) *ki = pi_params.ki;
}

/**
  * @brief  通过串口打印当前PI速度参数
  * @param  None
  * @retval None
  */
void PI_Speed_PrintParams(void)
{
    char buffer[100];
    sprintf(buffer, "Speed_Kp=%.2f Speed_Ki=%.2f Target=%.1f\r\n", 
            pi_params.kp, pi_params.ki, pi_params.target_speed);
    HAL_UART_Transmit(&huart6, (uint8_t*)buffer, strlen(buffer), HAL_MAX_DELAY);
}

/**
  * @brief  重置积分项
  * @param  None
  * @retval None
  */
void PI_Speed_ResetIntegral(void)
{
    pi_state.integral = 0.0f;
    pi_state.filtered_speed_error = 0.0f;  // 同时重置滤波状态
}

/**
  * @brief  调整速度Kp参数
  * @param  increase: 1增加，0减少
  * @retval None
  */
void PI_Speed_AdjustKp(uint8_t increase)
{
    if (increase) {
        pi_params.kp += SPEED_PARAM_ADJUST_STEP;
    } else {
        if (pi_params.kp > SPEED_PARAM_ADJUST_STEP) {
            pi_params.kp -= SPEED_PARAM_ADJUST_STEP;
        }
    }
    PI_Speed_PrintParams();
}

/**
  * @brief  调整速度Ki参数
  * @param  increase: 1增加，0减少
  * @retval None
  */
void PI_Speed_AdjustKi(uint8_t increase)
{
    if (increase) {
        pi_params.ki += SPEED_PARAM_ADJUST_STEP;
    } else {
        if (pi_params.ki > SPEED_PARAM_ADJUST_STEP) {
            pi_params.ki -= SPEED_PARAM_ADJUST_STEP;
        }
    }
    PI_Speed_PrintParams();
}

/**
  * @brief  处理蓝牙命令
  * @param  cmd: 命令字符
  * @retval None
  */
void PI_Speed_ProcessCommand(uint8_t cmd)
{
    switch(cmd) {
        case CMD_SPEED_KP_UP:
            PI_Speed_AdjustKp(1);
            break;
        case CMD_SPEED_KP_DOWN:
            PI_Speed_AdjustKp(0);
            break;
        case CMD_SPEED_KI_UP:
            PI_Speed_AdjustKi(1);
            break;
        case CMD_SPEED_KI_DOWN:
            PI_Speed_AdjustKi(0);
            break;
        case CMD_SPEED_PRINT:
            PI_Speed_PrintParams();
            break;
        default:
            break;
    }
}

/**
  * @brief  获取目标速度
  * @param  None
  * @retval 目标速度
  */
float PI_Speed_GetTargetSpeed(void)
{
    return pi_params.target_speed;
}

/**
  * @brief  设置目标速度
  * @param  target: 目标速度
  * @retval None
  */
void PI_Speed_SetTargetSpeed(float target)
{
    pi_params.target_speed = target;
    PI_Speed_PrintParams();
}

/**
  * @brief  速度限幅函数
  * @param  speed: 速度输入值
  * @param  max: 最大限幅值
  * @param  min: 最小限幅值
  * @retval 限幅后的速度值
  */
static int16_t Speed_Limit(int16_t speed, int16_t max, int16_t min)
{
    if (speed > max) return max;
    if (speed < min) return min;
    return speed;
} 
