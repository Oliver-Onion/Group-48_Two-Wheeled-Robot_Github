/**
  ******************************************************************************
  * @file           : pid_turn.c
  * @brief          : PD Turn Control Algorithm Implementation
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "pid_turn.h"
#include <stdio.h>
#include <string.h>

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
// 蓝牙命令定义
#define CMD_TURN_KP_UP      'r'  // Turn Kp增加
#define CMD_TURN_KP_DOWN    'f'  // Turn Kp减少
#define CMD_TURN_KD_UP      't'  // Turn Kd增加
#define CMD_TURN_KD_DOWN    'g'  // Turn Kd减少
#define CMD_TURN_PRINT      'n'  // 打印转向参数

extern UART_HandleTypeDef huart6;

/* Private variables ---------------------------------------------------------*/
static PD_Turn_Params_t pd_turn_params = {
    .kp = PD_TURN_DEFAULT_KP,
    .kd = PD_TURN_DEFAULT_KD,
    .turn_amplitude = PD_TURN_DEFAULT_AMPLITUDE,
    .target_angle = PD_TURN_DEFAULT_ANGLE
};

static PD_Turn_State_t pd_turn_state = {
    .prev_gyro = 0.0f
};

/* Private function prototypes -----------------------------------------------*/
static int16_t Turn_Limit(int16_t turn, int16_t max, int16_t min);

/* Functions -----------------------------------------------------------------*/

/**
  * @brief  初始化PD转向控制器
  * @param  None
  * @retval None
  */
void PD_Turn_Init(void)
{
    // 参数已在静态初始化中设置
    PD_Turn_PrintParams();
}

/**
  * @brief  PD转向控制计算函数
  * @param  target_angle: 目标角度
  * @param  current_angle: 当前角度 (Yaw角度)
  * @param  current_yaw_rate: 当前偏航角速度 (Yaw_dmp)
  * @retval 转向控制输出 (-100 to 100, 正值左转，负值右转)
  */
int16_t PD_Turn_Calculate(float target_angle, float current_angle)
{
    // 计算角度误差
    float angle_error = target_angle - current_angle;
    
    // 处理角度跨越±180度的情况
    if (angle_error > 180.0f) {
        angle_error -= 360.0f;
    } else if (angle_error < -180.0f) {
        angle_error += 360.0f;
    }
    
    // PD控制：比例项使用角度误差，微分项使用角速度
    // 参考control.c中的算法：turn = Turn_Target*Kp/100 + gyro*Kd/100
    float turn_output = angle_error * pd_turn_params.kp / 100.0f + 
    		current_angle * pd_turn_params.kd / 100.0f;
    
    // 转向输出限幅
    int16_t turn_pwm = (int16_t)turn_output;
    turn_pwm = Turn_Limit(turn_pwm, TURN_PWM_LIMIT, -TURN_PWM_LIMIT);
    
    // 更新状态
    pd_turn_state.prev_gyro = current_angle;
    
    return turn_pwm;
}

/**
  * @brief  设置PD转向控制参数
  * @param  kp: 比例系数
  * @param  kd: 微分系数
  * @retval None
  */
void PD_Turn_SetParams(float kp, float kd)
{
    pd_turn_params.kp = kp;
    pd_turn_params.kd = kd;
    PD_Turn_PrintParams();
}

/**
  * @brief  获取当前PD转向控制参数
  * @param  kp: 比例系数指针
  * @param  kd: 微分系数指针
  * @retval None
  */
void PD_Turn_GetParams(float *kp, float *kd)
{
    if (kp) *kp = pd_turn_params.kp;
    if (kd) *kd = pd_turn_params.kd;
}

/**
  * @brief  通过串口打印当前PD转向参数
  * @param  None
  * @retval None
  */
void PD_Turn_PrintParams(void)
{
    char buffer[100];
    sprintf(buffer, "Turn_Kp=%.1f Turn_Kd=%.1f Amplitude=%.1f\r\n", 
            pd_turn_params.kp, pd_turn_params.kd, pd_turn_params.turn_amplitude);
    HAL_UART_Transmit(&huart6, (uint8_t*)buffer, strlen(buffer), HAL_MAX_DELAY);
}

/**
  * @brief  调整转向Kp参数
  * @param  increase: 1增加，0减少
  * @retval None
  */
void PD_Turn_AdjustKp(uint8_t increase)
{
    if (increase) {
        pd_turn_params.kp += TURN_PARAM_ADJUST_STEP;
    } else {
        if (pd_turn_params.kp > TURN_PARAM_ADJUST_STEP) {
            pd_turn_params.kp -= TURN_PARAM_ADJUST_STEP;
        }
    }
    PD_Turn_PrintParams();
}

/**
  * @brief  调整转向Kd参数
  * @param  increase: 1增加，0减少
  * @retval None
  */
void PD_Turn_AdjustKd(uint8_t increase)
{
    if (increase) {
        pd_turn_params.kd += TURN_PARAM_ADJUST_STEP;
    } else {
        if (pd_turn_params.kd > TURN_PARAM_ADJUST_STEP) {
            pd_turn_params.kd -= TURN_PARAM_ADJUST_STEP;
        }
    }
    PD_Turn_PrintParams();
}

/**
  * @brief  设置目标角度
  * @param  target: 目标角度
  * @retval None
  */
void PD_Turn_SetTargetAngle(float target)
{
    pd_turn_params.target_angle = target;
}

/**
  * @brief  获取目标角度
  * @param  None
  * @retval 目标角度
  */
float PD_Turn_GetTargetAngle(void)
{
    return pd_turn_params.target_angle;
}

/**
  * @brief  设置转向幅度
  * @param  amplitude: 转向幅度
  * @retval None
  */
void PD_Turn_SetAmplitude(float amplitude)
{
    pd_turn_params.turn_amplitude = amplitude;
    PD_Turn_PrintParams();
}

/**
  * @brief  获取转向幅度
  * @param  None
  * @retval 转向幅度
  */
float PD_Turn_GetAmplitude(void)
{
    return pd_turn_params.turn_amplitude;
}

/**
  * @brief  转向输出限幅函数
  * @param  turn: 转向输入值
  * @param  max: 最大限幅值
  * @param  min: 最小限幅值
  * @retval 限幅后的转向值
  */
static int16_t Turn_Limit(int16_t turn, int16_t max, int16_t min)
{
    if (turn > max) return max;
    if (turn < min) return min;
    return turn;
}

/**
  * @brief  处理蓝牙命令
  * @param  cmd: 命令字符
  * @retval None
  */
void PD_Turn_ProcessCommand(uint8_t cmd)
{
    switch(cmd) {
        case CMD_TURN_KP_UP:
            PD_Turn_AdjustKp(1);
            break;
        case CMD_TURN_KP_DOWN:
            PD_Turn_AdjustKp(0);
            break;
        case CMD_TURN_KD_UP:
            PD_Turn_AdjustKd(1);
            break;
        case CMD_TURN_KD_DOWN:
            PD_Turn_AdjustKd(0);
            break;
        case CMD_TURN_PRINT:
            PD_Turn_PrintParams();
            break;
        default:
            break;
    }
} 
