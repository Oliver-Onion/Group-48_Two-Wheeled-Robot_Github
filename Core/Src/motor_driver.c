/**
  ******************************************************************************
  * @file           : motor_driver.c
  * @brief          : Motor driver implementation for two-wheeled robot
  ******************************************************************************
  * @attention
  *
  * Motor driver for two-wheeled robot using AT8236 driver module
  * Hardware Configuration:
  * - Left Motor (Motor A):  PWMA1 (TIM4_CH3-PB8), PWMA2 (TIM4_CH4-PB9)
  * - Right Motor (Motor B): PWMB1 (TIM4_CH1-PB6), PWMB2 (TIM4_CH2-PB7)
  * - Left Encoder: TIM2 (PA0-Encoder_A1, PA1-Encoder_A2)
  * - Right Encoder: TIM3 (PC6-Encoder_B1, PB5-Encoder_B2)
  * 
  * AT8236 Control Logic:
  * Motor speed and direction controlled by PWM differential:
  * - Forward:  PWM_IN1 > PWM_IN2
  * - Backward: PWM_IN1 < PWM_IN2
  * - Stop:     PWM_IN1 = PWM_IN2
  * - Speed determined by |PWM_IN1 - PWM_IN2|
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "motor_driver.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define PWM_PERIOD_VALUE    8399    // PWM period value (matches TIM4 Period setting)
#define PWM_CENTER_VALUE    4200    // Center PWM value (50% duty cycle)
#define PWM_MAX_DIFF        4200    // Maximum PWM difference for full speed

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
extern TIM_HandleTypeDef htim2;  // Left encoder timer handle
extern TIM_HandleTypeDef htim3;  // Right encoder timer handle
extern TIM_HandleTypeDef htim4;  // PWM timer handle

static MotorSpeeds_t current_speeds = {0, 0};
static EncoderData_t encoder_data = {0};
static uint32_t last_encoder_update_time = 0;

/* Private function prototypes -----------------------------------------------*/
static void Motor_SetPWM(MotorSelector_t motor, int16_t speed);
static uint32_t Motor_SpeedToPWMDiff(int16_t speed);

/* Private user code ---------------------------------------------------------*/

/**
  * @brief  Initialize motor driver
  * @param  None
  * @retval None
  */
void Motor_Init(void)
{
    // Start PWM generation on all TIM4 channels
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);  // PWMB1
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);  // PWMB2
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);  // PWMA1
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);  // PWMA2
    
    // Initialize motors to stop state
    Motor_Stop();
    
    // Initialize current speeds
    current_speeds.left_speed = 0;
    current_speeds.right_speed = 0;
    
    // Initialize encoder functionality
    Encoder_Init();
}

/**
  * @brief  Initialize encoder functionality
  * @param  None
  * @retval None
  */
void Encoder_Init(void)
{
    // Start encoder timers
    HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);  // Left encoder (TIM2)
    HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);  // Right encoder (TIM3)
    
    // Reset encoder data
    Encoder_Reset();
}

/**
  * @brief  Reset encoder data
  * @param  None
  * @retval None
  */
void Encoder_Reset(void)
{
    __HAL_TIM_SET_COUNTER(&htim2, 32768);  // Set to middle value to handle negative counts
    __HAL_TIM_SET_COUNTER(&htim3, 32768);
    
    encoder_data.left_encoder = 0;
    encoder_data.right_encoder = 0;
    encoder_data.left_encoder_prev = 0;
    encoder_data.right_encoder_prev = 0;
    encoder_data.left_speed_rpm = 0.0f;
    encoder_data.right_speed_rpm = 0.0f;
    
    last_encoder_update_time = HAL_GetTick();
}

/**
  * @brief  Update encoder readings and calculate speeds
  * @param  None
  * @retval None
  */
void Encoder_Update(void)
{
    uint32_t current_time = HAL_GetTick();
    
    // Update encoder counts
    int32_t left_raw = (int32_t)__HAL_TIM_GET_COUNTER(&htim2) - 32768;
    int32_t right_raw = (int32_t)__HAL_TIM_GET_COUNTER(&htim3) - 32768;
    
    encoder_data.left_encoder = left_raw;
    encoder_data.right_encoder = right_raw;
    
    // Calculate speed if enough time has passed
    if (current_time - last_encoder_update_time >= SPEED_CALC_PERIOD_MS) {
        float time_diff = (current_time - last_encoder_update_time) / 1000.0f; // Convert to seconds
        
        // Calculate speed in counts per second, then convert to RPM
        int32_t left_diff = encoder_data.left_encoder - encoder_data.left_encoder_prev;
        int32_t right_diff = encoder_data.right_encoder - encoder_data.right_encoder_prev;
        
        // Convert to encoder counts per second, then to RPM
        encoder_data.left_speed_rpm = (left_diff / time_diff) / (float)ENCODER_CPR * 60.0f;
        encoder_data.right_speed_rpm = (right_diff / time_diff) / (float)ENCODER_CPR * 60.0f;
        
        // Update previous values
        encoder_data.left_encoder_prev = encoder_data.left_encoder;
        encoder_data.right_encoder_prev = encoder_data.right_encoder;
        last_encoder_update_time = current_time;
    }
}

/**
  * @brief  Get encoder data
  * @param  None
  * @retval Encoder data structure
  */
EncoderData_t Encoder_GetData(void)
{
    return encoder_data;
}

/**
  * @brief  Get left encoder count
  * @param  None
  * @retval Left encoder count
  */
int32_t Encoder_GetLeftCount(void)
{
    return encoder_data.left_encoder;
}

/**
  * @brief  Get right encoder count
  * @param  None
  * @retval Right encoder count
  */
int32_t Encoder_GetRightCount(void)
{
    return encoder_data.right_encoder;
}

/**
  * @brief  Get left wheel speed in RPM
  * @param  None
  * @retval Left wheel speed in RPM
  */
float Encoder_GetLeftSpeed(void)
{
    return encoder_data.left_speed_rpm;
}

/**
  * @brief  Get right wheel speed in RPM
  * @param  None
  * @retval Right wheel speed in RPM
  */
float Encoder_GetRightSpeed(void)
{
    return encoder_data.right_speed_rpm;
}

/**
  * @brief  Convert speed percentage to PWM value
  * @param  speed: Speed percentage (-100 to 100)
  * @retval PWM value (0 to PWM_PERIOD_VALUE)
  */
static uint32_t Motor_SpeedToPWMDiff(int16_t speed)
{
    // Get absolute value for PWM calculation
    int16_t abs_speed = (speed < 0) ? -speed : speed;
    
    // Clamp speed to valid range
    //abs_speed = CLAMP(abs_speed, 0, MOTOR_MAX_SPEED);
    
    // Convert percentage (0-100) to PWM value (0-PWM_PERIOD_VALUE)
    uint32_t pwm_value = (uint32_t)((abs_speed * PWM_PERIOD_VALUE) / 100);
    
    return pwm_value;
}

/**
  * @brief  Set PWM values for AT8236 control
  * @param  motor: Motor selector (MOTOR_LEFT or MOTOR_RIGHT)
  * @param  speed: Speed percentage (-100 to 100, negative for backward)
  * @retval None
  */
static void Motor_SetPWM(MotorSelector_t motor, int16_t speed)
{
    // Clamp speed to valid range
    //speed = CLAMP(speed, MOTOR_MIN_SPEED, MOTOR_MAX_SPEED);
    
    // Correct direction for right motor if physically reversed
    if (motor == MOTOR_RIGHT) {
        speed = -speed;
    }
    
    uint32_t pwm_value = Motor_SpeedToPWMDiff(speed);
    uint32_t pwm_in1, pwm_in2;
    
    if (speed > 0) {
        // Forward direction: IN1=PWM, IN2=0
        pwm_in1 = pwm_value;
        pwm_in2 = 0;
    } else if (speed < 0) {
        // Backward direction: IN1=0, IN2=PWM
        pwm_in1 = 0;
        pwm_in2 = pwm_value;
    } else {
        // Stop: IN1=0, IN2=0
        pwm_in1 = 0;
        pwm_in2 = 0;
    }
    
    // Set PWM values for specified motor
    if (motor == MOTOR_LEFT) {
        // Left motor: PWMA1 (TIM4_CH3), PWMA2 (TIM4_CH4)
        __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, pwm_in1);
        __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, pwm_in2);
    } else {
        // Right motor: PWMB1 (TIM4_CH1), PWMB2 (TIM4_CH2)
        __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, pwm_in1);
        __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, pwm_in2);
    }
}

/**
  * @brief  Set speed for specific motor
  * @param  motor: Motor selector (MOTOR_LEFT or MOTOR_RIGHT)
  * @param  speed: Speed percentage (-100 to 100, negative for backward)
  * @retval None
  */
void Motor_SetSpeed(MotorSelector_t motor, int16_t speed)
{
    // Update current speeds
    if (motor == MOTOR_LEFT) {
        current_speeds.left_speed = speed;
    } else {
        current_speeds.right_speed = speed;
    }
    
    // Set PWM values
    Motor_SetPWM(motor, speed);
}

/**
  * @brief  Set speeds for both motors
  * @param  left_speed: Left motor speed (-100 to 100)
  * @param  right_speed: Right motor speed (-100 to 100)
  * @retval None
  */
void Motor_SetSpeeds(int16_t left_speed, int16_t right_speed)
{
    Motor_SetSpeed(MOTOR_LEFT, left_speed);
    Motor_SetSpeed(MOTOR_RIGHT, right_speed);
}

/**
  * @brief  Stop both motors
  * @param  None
  * @retval None
  */
void Motor_Stop(void)
{
    Motor_SetSpeeds(0, 0);
}

/**
  * @brief  Move both motors forward
  * @param  speed: Forward speed (0 to 100)
  * @retval None
  */
void Motor_Forward(int16_t speed)
{
    speed = CLAMP(speed, 0, MOTOR_MAX_SPEED);
    Motor_SetSpeeds(speed, speed);
}

/**
  * @brief  Move both motors backward
  * @param  speed: Backward speed (0 to 100)
  * @retval None
  */
void Motor_Backward(int16_t speed)
{
    speed = CLAMP(speed, 0, MOTOR_MAX_SPEED);
    Motor_SetSpeeds(-speed, -speed);
}

/**
  * @brief  Turn left (right motor forward, left motor backward)
  * @param  speed: Turn speed (0 to 100)
  * @retval None
  */
void Motor_TurnLeft(int16_t speed)
{
    speed = CLAMP(speed, 0, MOTOR_MAX_SPEED);
    Motor_SetSpeeds(-speed, speed);
}

/**
  * @brief  Turn right (left motor forward, right motor backward)
  * @param  speed: Turn speed (0 to 100)
  * @retval None
  */
void Motor_TurnRight(int16_t speed)
{
    speed = CLAMP(speed, 0, MOTOR_MAX_SPEED);
    Motor_SetSpeeds(speed, -speed);
}

/**
  * @brief  Brake both motors
  * @param  None
  * @retval None
  */
void Motor_Brake(void)
{
    // For AT8236, braking is achieved by setting all PWM signals to 1
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, PWM_PERIOD_VALUE);  // PWMB1
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, PWM_PERIOD_VALUE);  // PWMB2
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, PWM_PERIOD_VALUE);  // PWMA1
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, PWM_PERIOD_VALUE);  // PWMA2
    
    // Update current speeds
    current_speeds.left_speed = 0;
    current_speeds.right_speed = 0;
}

/**
  * @brief  Get current motor speeds
  * @param  None
  * @retval Current motor speeds structure
  */
MotorSpeeds_t Motor_GetSpeeds(void)
{
    return current_speeds;
}

/**
  * @brief  Set motor speed with encoder feedback (placeholder implementation)
  * @param  motor: Motor selector (MOTOR_LEFT or MOTOR_RIGHT)
  * @param  target_rpm: Target speed in RPM
  * @retval None
  */
void Motor_SetSpeedWithFeedback(MotorSelector_t motor, float target_rpm)
{
    // Simple proportional control
    float kp = 1.0f;
    float current_rpm;
    float error;
    int16_t control_output;
    
    // Update encoder readings
    Encoder_Update();
    
    // Get current speed
    if (motor == MOTOR_LEFT) {
        current_rpm = encoder_data.left_speed_rpm;
    } else {
        current_rpm = encoder_data.right_speed_rpm;
    }
    
    // Calculate error and control output
    error = target_rpm - current_rpm;
    control_output = (int16_t)(kp * error);
    
    // Apply control output
    control_output = CLAMP(control_output, MOTOR_MIN_SPEED, MOTOR_MAX_SPEED);
    Motor_SetSpeed(motor, control_output);
}

/**
  * @brief  Set both motor speeds with encoder feedback
  * @param  left_rpm: Target left motor speed in RPM
  * @param  right_rpm: Target right motor speed in RPM
  * @retval None
  */
void Motor_SetSpeedsWithFeedback(float left_rpm, float right_rpm)
{
    Motor_SetSpeedWithFeedback(MOTOR_LEFT, left_rpm);
    Motor_SetSpeedWithFeedback(MOTOR_RIGHT, right_rpm);
} 
