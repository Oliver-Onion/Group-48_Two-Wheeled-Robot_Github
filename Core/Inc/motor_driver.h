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
  * Updated for AT8236 PWM differential control
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
#define ENCODER_CPR            1024    // Encoder counts per revolution (adjust based on your encoder)
#define SPEED_CALC_PERIOD_MS   100     // Speed calculation period in ms

/* AT8236 PWM Configuration */
#define PWM_PERIOD_VALUE       8399    // PWM period value (matches TIM4 Period setting)
#define PWM_CENTER_VALUE       4200    // Center PWM value (50% duty cycle)
#define PWM_MAX_DIFF           4200    // Maximum PWM difference for full speed

/* Motor Channel Mapping for AT8236 */
// Left Motor (Motor A): PWMA1 (TIM4_CH3), PWMA2 (TIM4_CH4) 
// Right Motor (Motor B): PWMB1 (TIM4_CH1), PWMB2 (TIM4_CH2)
#define MOTOR_LEFT_PWM_CH1     TIM_CHANNEL_3  // PWMA1 (PB8)
#define MOTOR_LEFT_PWM_CH2     TIM_CHANNEL_4  // PWMA2 (PB9)
#define MOTOR_RIGHT_PWM_CH1    TIM_CHANNEL_1  // PWMB1 (PB6)
#define MOTOR_RIGHT_PWM_CH2    TIM_CHANNEL_2  // PWMB2 (PB7)

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

// Advanced control functions with PI feedback
void Motor_SetSpeedWithFeedback(MotorSelector_t motor, float target_rpm);
void Motor_SetSpeedsWithFeedback(float left_rpm, float right_rpm);

#ifdef __cplusplus
}
#endif

#endif /* __MOTOR_DRIVER_H */ 
