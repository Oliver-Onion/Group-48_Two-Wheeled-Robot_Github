/**
  ******************************************************************************
  * @file           : pid_balance.c
  * @brief          : PD Balance Control Algorithm Implementation
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "pid_balance.h"
#include <stdio.h>
#include <string.h>

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define DANGEROUS_ANGLE  40.0f    // 危险倾角阈值

// 蓝牙命令定义
#define CMD_KP_UP    'q'  // Kp增加
#define CMD_KP_DOWN  'a'  // Kp减少
#define CMD_KD_UP    'w'  // Kd增加
#define CMD_KD_DOWN  's'  // Kd减少
#define CMD_PRINT    'p'  // 打印当前参数

extern UART_HandleTypeDef huart6;

/* Private variables ---------------------------------------------------------*/
static PD_Balance_Params_t pd_params = {
    .kp = PD_BALANCE_DEFAULT_KP,
    .kd = PD_BALANCE_DEFAULT_KD,
    .target_angle = PD_BALANCE_DEFAULT_TARGET
};

/* Private function prototypes -----------------------------------------------*/
static int PWM_Limit(int pwm, int max, int min);

/* Functions -----------------------------------------------------------------*/

/**
  * @brief  初始化PD平衡控制器
  * @param  None
  * @retval None
  */
void PD_Balance_Init(void)
{
    // 参数已在静态初始化中设置
    PD_Balance_PrintParams();
}

/**
  * @brief  PD平衡控制计算函数
  * @param  current_angle: 当前角度（度）
  * @param  gyro: 角速度（度/秒）
  * @retval PWM控制值
  */
int PD_Balance_Calculate(float current_angle, float gyro)
{
    float angle_bias = pd_params.target_angle - current_angle;  // 计算角度误差
    float gyro_bias = 0 - gyro;  // 角速度误差，目标角速度为0
    
    // PD控制计算
    int balance = -pd_params.kp/100 * angle_bias - gyro_bias * pd_params.kd/100;
    
    // PWM限幅
    return PWM_Limit(balance, PWM_LIMIT, -PWM_LIMIT);
}

/**
  * @brief  设置PD控制参数
  * @param  kp: 比例系数
  * @param  kd: 微分系数
  * @retval None
  */
void PD_Balance_SetParams(float kp, float kd)
{
    pd_params.kp = kp;
    pd_params.kd = kd;
    PD_Balance_PrintParams();  // 打印新的参数
}

/**
  * @brief  获取当前PD控制参数
  * @param  kp: 比例系数指针
  * @param  kd: 微分系数指针
  * @retval None
  */
void PD_Balance_GetParams(float *kp, float *kd)
{
    if (kp) *kp = pd_params.kp;
    if (kd) *kd = pd_params.kd;
}

/**
  * @brief  通过串口打印当前PD参数
  * @param  None
  * @retval None
  */
void PD_Balance_PrintParams(void)
{
    char buffer[100];
    sprintf(buffer, "Balance_Kp=%.2f Balance_Kd=%.2f\r\n", 
            pd_params.kp, pd_params.kd);
    HAL_UART_Transmit(&huart6, (uint8_t*)buffer, strlen(buffer), HAL_MAX_DELAY);
}

/**
  * @brief  检查是否处于危险倾角
  * @param  angle: 当前角度
  * @retval 1: 危险，0: 安全
  */
uint8_t PD_Balance_IsDangerous(float angle)
{
    if (angle < -DANGEROUS_ANGLE || angle > DANGEROUS_ANGLE) {
        return 1;
    }
    return 0;
}

/**
  * @brief  调整Kp参数
  * @param  increase: 1增加，0减少
  * @retval None
  */
void PD_Balance_AdjustKp(uint8_t increase)
{
    if (increase) {
        pd_params.kp += PARAM_ADJUST_STEP;
    } else {
        if (pd_params.kp > PARAM_ADJUST_STEP) {  // 防止变为负值
            pd_params.kp -= PARAM_ADJUST_STEP;
        }
    }
    PD_Balance_PrintParams();
}

/**
  * @brief  调整Kd参数
  * @param  increase: 1增加，0减少
  * @retval None
  */
void PD_Balance_AdjustKd(uint8_t increase)
{
    if (increase) {
        pd_params.kd += PARAM_ADJUST_STEP;
    } else {
        if (pd_params.kd > PARAM_ADJUST_STEP) {  // 防止变为负值
            pd_params.kd -= PARAM_ADJUST_STEP;
        }
    }
    PD_Balance_PrintParams();
}

/**
  * @brief  处理蓝牙命令
  * @param  cmd: 命令字符
  * @retval None
  */
void PD_Balance_ProcessCommand(uint8_t cmd)
{
    switch(cmd) {
        case CMD_KP_UP:
            PD_Balance_AdjustKp(1);
            break;
        case CMD_KP_DOWN:
            PD_Balance_AdjustKp(0);
            break;
        case CMD_KD_UP:
            PD_Balance_AdjustKd(1);
            break;
        case CMD_KD_DOWN:
            PD_Balance_AdjustKd(0);
            break;
        case CMD_PRINT:
            PD_Balance_PrintParams();
            break;
        default:
            break;
    }
}

/**
  * @brief  PWM限幅函数
  * @param  pwm: PWM输入值
  * @param  max: 最大限幅值
  * @param  min: 最小限幅值
  * @retval 限幅后的PWM值
  */
static int PWM_Limit(int pwm, int max, int min)
{
    if (pwm > max) return max;
    if (pwm < min) return min;
    return pwm;
} 
