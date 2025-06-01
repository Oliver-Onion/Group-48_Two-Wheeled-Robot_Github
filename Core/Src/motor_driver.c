/**
  ******************************************************************************
  * @file           : motor_driver.c
  * @brief          : Motor driver implementation for two-wheeled robot
  ******************************************************************************
  * @attention
  *
  * Motor driver for two-wheeled robot using MC520 driver module
  * Hardware Configuration:
  * - Left Motor (Motor A):  AIN1 (PB14), AIN2 (PB15), PWM on TIM4_CH3 (PB8)
  * - Right Motor (Motor B): BIN1 (PB13), BIN2 (PB12), PWM on TIM4_CH4 (PB9)
  * - Left Encoder: TIM2 (PA0-Encoder_A1, PA1-Encoder_A2)
  * - Right Encoder: TIM3 (PC6-Encoder_B1, PB5-Encoder_B2)
  * 
  * MC520 Control Logic:
  * Left Motor:
  *   Forward:  AIN1=1, AIN2=0, PWM=duty
  *   Backward: AIN1=0, AIN2=1, PWM=duty
  * Right Motor:
  *   Forward:  BIN1=0, BIN2=1, PWM=duty
  *   Backward: BIN1=1, BIN2=0, PWM=duty
  * Both Motors:
  *   Brake:    xIN1=1, xIN2=1
  *   Stop:     xIN1=0, xIN2=0
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "motor_driver.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
extern TIM_HandleTypeDef htim2;  // Left encoder timer handle
extern TIM_HandleTypeDef htim3;  // Right encoder timer handle
extern TIM_HandleTypeDef htim4;  // PWM timer handle

static MotorSpeeds_t current_speeds = {0, 0};
static EncoderData_t encoder_data = {0};
static uint32_t last_encoder_update_time = 0;

/* Private function prototypes -----------------------------------------------*/
static void Motor_SetDirection(MotorSelector_t motor, MotorDirection_t direction);
static uint32_t Motor_SpeedToPWM(int16_t speed);

/* Private user code ---------------------------------------------------------*/

/**
  * @brief  Initialize motor driver
  * @param  None
  * @retval None
  */
void Motor_Init(void)
{
    // Start PWM generation on TIM4 channels
    HAL_TIM_PWM_Start(&htim4, MOTOR_LEFT_PWM_CHANNEL);
    HAL_TIM_PWM_Start(&htim4, MOTOR_RIGHT_PWM_CHANNEL);
    
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
        
        // Calculate speed in RPM
        int32_t left_diff = encoder_data.left_encoder - encoder_data.left_encoder_prev;
        int32_t right_diff = encoder_data.right_encoder - encoder_data.right_encoder_prev;
        
        encoder_data.left_speed_rpm = (left_diff / (float)ENCODER_CPR) * (60.0f / time_diff);
        encoder_data.right_speed_rpm = (right_diff / (float)ENCODER_CPR) * (60.0f / time_diff);
        
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
  * @brief  Convert speed percentage to PWM value with exponential mapping
  * @param  speed: Speed percentage (-100 to 100)
  * @retval PWM value (0 to PWM_MAX_VALUE)
  */
static uint32_t Motor_SpeedToPWM(int16_t speed)
{
    // Get absolute value for PWM calculation
    int16_t abs_speed = (speed < 0) ? -speed : speed;
    
    // Clamp speed to valid range
    abs_speed = CLAMP(abs_speed, 0, MOTOR_MAX_SPEED);
    
    // Convert percentage (0-100) to PWM value (0-PWM_MAX_VALUE)
    uint32_t pwm_value = (uint32_t)((abs_speed * PWM_MAX_VALUE) / 100);
    
    // Apply minimum PWM threshold for motor startup
    if (pwm_value > 0 && pwm_value < PWM_MIN_VALUE) {
        pwm_value = PWM_MIN_VALUE;
    }
    
    return pwm_value;
}

/**
  * @brief  Set direction pins for specific motor
  * @param  motor: Motor selector (MOTOR_LEFT or MOTOR_RIGHT)
  * @param  direction: Motor direction
  * @retval None
  */
static void Motor_SetDirection(MotorSelector_t motor, MotorDirection_t direction)
{
    if (motor == MOTOR_LEFT) {
        // Left motor control (AIN1=PB14, AIN2=PB15)
        switch (direction) {
            case MOTOR_FORWARD:
                HAL_GPIO_WritePin(AIN1_GPIO_Port, AIN1_Pin, GPIO_PIN_SET);   // AIN1 = 1
                HAL_GPIO_WritePin(AIN2_GPIO_Port, AIN2_Pin, GPIO_PIN_RESET); // AIN2 = 0
                break;
            case MOTOR_BACKWARD:
                HAL_GPIO_WritePin(AIN1_GPIO_Port, AIN1_Pin, GPIO_PIN_RESET); // AIN1 = 0
                HAL_GPIO_WritePin(AIN2_GPIO_Port, AIN2_Pin, GPIO_PIN_SET);   // AIN2 = 1
                break;
            case MOTOR_BRAKE:
                HAL_GPIO_WritePin(AIN1_GPIO_Port, AIN1_Pin, GPIO_PIN_SET);   // AIN1 = 1
                HAL_GPIO_WritePin(AIN2_GPIO_Port, AIN2_Pin, GPIO_PIN_SET);   // AIN2 = 1
                break;
            case MOTOR_STOP:
            default:
                HAL_GPIO_WritePin(AIN1_GPIO_Port, AIN1_Pin, GPIO_PIN_RESET); // AIN1 = 0
                HAL_GPIO_WritePin(AIN2_GPIO_Port, AIN2_Pin, GPIO_PIN_RESET); // AIN2 = 0
                break;
        }
    } else if (motor == MOTOR_RIGHT) {
        // Right motor control (BIN1=PB13, BIN2=PB12)
        switch (direction) {
            case MOTOR_FORWARD:
                HAL_GPIO_WritePin(BIN1_GPIO_Port, BIN1_Pin, GPIO_PIN_RESET); // BIN1 = 0
                HAL_GPIO_WritePin(BIN2_GPIO_Port, BIN2_Pin, GPIO_PIN_SET);   // BIN2 = 1
                break;
            case MOTOR_BACKWARD:
                HAL_GPIO_WritePin(BIN1_GPIO_Port, BIN1_Pin, GPIO_PIN_SET);   // BIN1 = 1
                HAL_GPIO_WritePin(BIN2_GPIO_Port, BIN2_Pin, GPIO_PIN_RESET); // BIN2 = 0
                break;
            case MOTOR_BRAKE:
                HAL_GPIO_WritePin(BIN1_GPIO_Port, BIN1_Pin, GPIO_PIN_SET);   // BIN1 = 1
                HAL_GPIO_WritePin(BIN2_GPIO_Port, BIN2_Pin, GPIO_PIN_SET);   // BIN2 = 1
                break;
            case MOTOR_STOP:
            default:
                HAL_GPIO_WritePin(BIN1_GPIO_Port, BIN1_Pin, GPIO_PIN_RESET); // BIN1 = 0
                HAL_GPIO_WritePin(BIN2_GPIO_Port, BIN2_Pin, GPIO_PIN_RESET); // BIN2 = 0
                break;
        }
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
    // Clamp speed to valid range
    speed = CLAMP(speed, MOTOR_MIN_SPEED, MOTOR_MAX_SPEED);
    
    // Update current speeds
    if (motor == MOTOR_LEFT) {
        current_speeds.left_speed = speed;
    } else {
        current_speeds.right_speed = speed;
    }
    
    // Determine direction and PWM value
    MotorDirection_t direction;
    uint32_t pwm_value = Motor_SpeedToPWM(speed);
    
    if (speed > 0) {
        direction = MOTOR_FORWARD;
    } else if (speed < 0) {
        direction = MOTOR_BACKWARD;
    } else {
        direction = MOTOR_STOP;
        pwm_value = 0;
    }
    
    // Set direction pins
    Motor_SetDirection(motor, direction);
    
    // Set PWM duty cycle
    if (motor == MOTOR_LEFT) {
        __HAL_TIM_SET_COMPARE(&htim4, MOTOR_LEFT_PWM_CHANNEL, pwm_value);
    } else {
        __HAL_TIM_SET_COMPARE(&htim4, MOTOR_RIGHT_PWM_CHANNEL, pwm_value);
    }
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
    // Set PWM to 0 first
    __HAL_TIM_SET_COMPARE(&htim4, MOTOR_LEFT_PWM_CHANNEL, 0);
    __HAL_TIM_SET_COMPARE(&htim4, MOTOR_RIGHT_PWM_CHANNEL, 0);
    
    // Set direction to brake
    Motor_SetDirection(MOTOR_LEFT, MOTOR_BRAKE);
    Motor_SetDirection(MOTOR_RIGHT, MOTOR_BRAKE);
    
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
  * @brief  Set motor speed with encoder feedback (basic implementation)
  * @param  motor: Motor selector (MOTOR_LEFT or MOTOR_RIGHT)
  * @param  target_rpm: Target speed in RPM
  * @retval None
  */
void Motor_SetSpeedWithFeedback(MotorSelector_t motor, float target_rpm)
{
    // Simple proportional control - can be improved with PID
    float kp = 1.0f;  // Proportional gain (needs tuning)
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
