/**
  ******************************************************************************
  * @file           : pid_speed.h
  * @brief          : Header for PI Speed Control Algorithm
  ******************************************************************************
  */

#ifndef __PID_SPEED_H
#define __PID_SPEED_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Exported types ------------------------------------------------------------*/
typedef struct {
    float kp;              // 比例系数
    float ki;              // 积分系数
    float target_speed;    // 目标速度
    float integral_limit;  // 积分限幅
} PI_Speed_Params_t;

typedef struct {
    float integral;        // 积分累积值
    float prev_error;      // 上一次误差
    float filtered_speed_error;   // 低通滤波后的速度误差
} PI_Speed_State_t;

/* Exported constants --------------------------------------------------------*/
#define PI_SPEED_DEFAULT_KP      1.0f        // 默认比例系数
#define PI_SPEED_DEFAULT_KI      0.1f        // 默认积分系数
#define PI_SPEED_DEFAULT_TARGET  0.0f        // 默认目标速度
#define PI_SPEED_INTEGRAL_LIMIT  50.0f       // 积分限幅
#define SPEED_PARAM_ADJUST_STEP  0.1f        // 速度参数调整步长

/* Exported functions prototypes ---------------------------------------------*/
void PI_Speed_Init(void);
int16_t PI_Speed_Calculate(float current_speed, float target_speed);
void PI_Speed_SetParams(float kp, float ki);
void PI_Speed_GetParams(float *kp, float *ki);
void PI_Speed_PrintParams(void);
void PI_Speed_ResetIntegral(void);
void PI_Speed_AdjustKp(uint8_t increase);     // 1: 增加, 0: 减少
void PI_Speed_AdjustKi(uint8_t increase);     // 1: 增加, 0: 减少
void PI_Speed_ProcessCommand(uint8_t cmd);    // 处理蓝牙命令
float PI_Speed_GetTargetSpeed(void);
void PI_Speed_SetTargetSpeed(float target);

#ifdef __cplusplus
}
#endif

#endif /* __PID_SPEED_H */ 