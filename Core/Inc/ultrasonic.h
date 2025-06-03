/**
  ******************************************************************************
  * @file           : ultrasonic.h
  * @brief          : Header for ultrasonic distance measurement
  ******************************************************************************
  */

#ifndef __ULTRASONIC_H
#define __ULTRASONIC_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include <stdint.h>

/* Exported types ------------------------------------------------------------*/

/* Exported constants --------------------------------------------------------*/
#define ULTRASONIC_TRIG_PIN GPIO_PIN_9
#define ULTRASONIC_TRIG_PORT GPIOA

/* Exported macro ------------------------------------------------------------*/

/* Exported functions prototypes ---------------------------------------------*/
void Ultrasonic_Init(TIM_HandleTypeDef *htim);
void Ultrasonic_TriggerMeasurement(void);
uint32_t Ultrasonic_GetDistanceMM(void);
uint8_t Ultrasonic_IsDataReady(void);
void Ultrasonic_SetSamplingInterval(uint16_t interval_ms);
uint16_t Ultrasonic_GetSamplingInterval(void);
void Ultrasonic_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim);

/* Private defines -----------------------------------------------------------*/

#ifdef __cplusplus
}
#endif

#endif /* __ULTRASONIC_H */ 