/**
  ******************************************************************************
  * @file           : motor_debug.h
  * @brief          : Header for motor_debug.c file.
  *                   This file contains the motor debugging function declarations.
  ******************************************************************************
  */

#ifndef __MOTOR_DEBUG_H
#define __MOTOR_DEBUG_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_hal.h"

/* Exported functions prototypes ---------------------------------------------*/
void Motor_Debug_PWMTest(void);
void Motor_Debug_DirectionTest(void);
void Motor_Debug_SpeedFunctionTest(void);
void Motor_Debug_DiagnoseProblem(void);
void Motor_Debug_ShowPWMValue(int16_t speed);

#ifdef __cplusplus
}
#endif

#endif /* __MOTOR_DEBUG_H */ 