/**
  ******************************************************************************
  * @file           : pid_balance.h
  * @brief          : Header for PD Balance Control Algorithm
  ******************************************************************************
  * @attention
  *
  * PD平衡控制算法头文件
  * 
  * 角度约定：
  * - 往前倒时Pitch为正值
  * - 往后倒时Pitch为负值
  * - 目标角度为0度（直立状态）
  * 
  * 控制输出约定：
  * - 正值：向前的控制力
  * - 负值：向后的控制力
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __PID_BALANCE_H
#define __PID_BALANCE_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Exported types ------------------------------------------------------------*/
typedef struct {
    float kp;              // 比例系数
    float kd;              // 微分系数
    float target_angle;    // 目标角度（度）
} PD_Balance_Params_t;

/* Exported constants --------------------------------------------------------*/
#define PD_BALANCE_DEFAULT_KP      350.0f    // 默认比例系数
#define PD_BALANCE_DEFAULT_KD      350.0f    // 默认微分系数
#define PD_BALANCE_DEFAULT_TARGET  0.0f      // 默认目标角度
#define PWM_LIMIT                  6900      // PWM限幅值
#define PARAM_ADJUST_STEP         100.0f     // 参数调整步长

/* Exported functions prototypes ---------------------------------------------*/
void PD_Balance_Init(void);
int PD_Balance_Calculate(float current_angle, float gyro);
void PD_Balance_SetParams(float kp, float kd);
void PD_Balance_GetParams(float *kp, float *kd);
void PD_Balance_PrintParams(void);
uint8_t PD_Balance_IsDangerous(float angle);
void PD_Balance_AdjustKp(uint8_t increase);  // 1: 增加, 0: 减少
void PD_Balance_AdjustKd(uint8_t increase);  // 1: 增加, 0: 减少
void PD_Balance_ProcessCommand(uint8_t cmd);  // 处理蓝牙命令

#ifdef __cplusplus
}
#endif

#endif /* __PID_BALANCE_H */ 