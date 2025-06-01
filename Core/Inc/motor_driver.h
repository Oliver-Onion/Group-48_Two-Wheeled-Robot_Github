/**
  ******************************************************************************
  * @file           : motor_driver.h
  * @brief          : Header for motor_driver.c file.
  *                   This file contains the motor driver definitions.
  ******************************************************************************
  * @attention
  *
  * Motor driver for two-wheeled robot using AT8236 H-bridge driver
  * Compatible with STM32F401RE microcontroller
  * Updated for dual encoder configuration
  *
  ******************************************************************************
  */

#ifndef __MOTOR_DRIVER_H
#define __MOTOR_DRIVER_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_hal.h"

/* Exported types ------------------------------------------------------------*/
typedef enum {
    MOTOR_LEFT = 0,
    MOTOR_RIGHT = 1
} MotorSelector_t;

typedef enum {
    MOTOR_STOP = 0,
    MOTOR_FORWARD = 1,
    MOTOR_BACKWARD = 2,
    MOTOR_BRAKE = 3
} MotorDirection_t;

typedef struct {
    int16_t left_speed;    // Current left motor speed (-100 to 100)
    int16_t right_speed;   // Current right motor speed (-100 to 100)
} MotorSpeeds_t;

typedef struct {
    int32_t left_encoder;      // Left encoder count
    int32_t right_encoder;     // Right encoder count
    int32_t left_encoder_prev; // Previous left encoder count
    int32_t right_encoder_prev;// Previous right encoder count
    float left_speed_rpm;      // Left wheel speed in RPM
    float right_speed_rpm;     // Right wheel speed in RPM
} EncoderData_t;

/* Exported constants --------------------------------------------------------*/
#define MOTOR_MAX_SPEED        100     // Maximum speed percentage
#define MOTOR_MIN_SPEED        -100    // Minimum speed percentage
#define PWM_MAX_VALUE          9999    // Maximum PWM value
#define PWM_MIN_VALUE          10      // Minimum PWM value for movement (根据测试结果调整)
#define ENCODER_CPR            1024    // Encoder counts per revolution
#define SPEED_CALC_PERIOD_MS   100     // Speed calculation period in ms

/* PWM Channel definitions for TIM4 */
#define MOTOR_LEFT_PWM_CHANNEL  TIM_CHANNEL_3  // TIM4_CH3 (PB8)
#define MOTOR_RIGHT_PWM_CHANNEL TIM_CHANNEL_4  // TIM4_CH4 (PB9)

/* Exported macro ------------------------------------------------------------*/
#define CLAMP(x, min, max) ((x) < (min) ? (min) : ((x) > (max) ? (max) : (x)))

/* Exported functions prototypes ---------------------------------------------*/
// Basic motor control
void Motor_Init(void);
void Motor_SetSpeed(MotorSelector_t motor, int16_t speed);
void Motor_SetSpeeds(int16_t left_speed, int16_t right_speed);
void Motor_Stop(void);
void Motor_Forward(int16_t speed);
void Motor_Backward(int16_t speed);
void Motor_TurnLeft(int16_t speed);
void Motor_TurnRight(int16_t speed);
void Motor_Brake(void);
MotorSpeeds_t Motor_GetSpeeds(void);

// Encoder functions
void Encoder_Init(void);
void Encoder_Update(void);
EncoderData_t Encoder_GetData(void);
void Encoder_Reset(void);
int32_t Encoder_GetLeftCount(void);
int32_t Encoder_GetRightCount(void);
float Encoder_GetLeftSpeed(void);
float Encoder_GetRightSpeed(void);

// Advanced control functions
void Motor_SetSpeedWithFeedback(MotorSelector_t motor, float target_rpm);
void Motor_SetSpeedsWithFeedback(float left_rpm, float right_rpm);

#ifdef __cplusplus
}
#endif

#endif /* __MOTOR_DRIVER_H */ 
