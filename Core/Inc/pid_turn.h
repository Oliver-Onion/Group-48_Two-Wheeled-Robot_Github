/**
  ******************************************************************************
  * @file           : pid_turn.h
  * @brief          : Header for PD Turn Control Algorithm
  ******************************************************************************
  */

#ifndef __PID_TURN_H
#define __PID_TURN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Exported types ------------------------------------------------------------*/
typedef struct {
    float kp;                // 比例系数
    float kd;                // 微分系数
    float turn_amplitude;    // 转向幅度（保留用于遥控）
    float target_angle;      // 目标角度
} PD_Turn_Params_t;

typedef struct {
    float prev_gyro;         // 上一次陀螺仪数据
} PD_Turn_State_t;

/* Exported constants --------------------------------------------------------*/
#define PD_TURN_DEFAULT_KP       100.0f      // 默认比例系数
#define PD_TURN_DEFAULT_KD       50.0f       // 默认微分系数
#define PD_TURN_DEFAULT_AMPLITUDE 54.0f      // 默认转向幅度
#define PD_TURN_DEFAULT_ANGLE    0.0f        // 默认目标角度
#define TURN_PARAM_ADJUST_STEP   10.0f       // 转向参数调整步长
#define TURN_PWM_LIMIT           100         // 转向PWM限幅

/* Exported functions prototypes ---------------------------------------------*/
void PD_Turn_Init(void);
int16_t PD_Turn_Calculate(float target_angle, float current_angle);
void PD_Turn_SetParams(float kp, float kd);
void PD_Turn_GetParams(float *kp, float *kd);
void PD_Turn_PrintParams(void);
void PD_Turn_AdjustKp(uint8_t increase);      // 1: 增加, 0: 减少
void PD_Turn_AdjustKd(uint8_t increase);      // 1: 增加, 0: 减少
void PD_Turn_ProcessCommand(uint8_t cmd);     // 处理蓝牙命令
void PD_Turn_SetTargetAngle(float target);
float PD_Turn_GetTargetAngle(void);
void PD_Turn_SetAmplitude(float amplitude);
float PD_Turn_GetAmplitude(void);

#ifdef __cplusplus
}
#endif

#endif /* __PID_TURN_H */ 
