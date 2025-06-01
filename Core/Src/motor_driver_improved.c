/**
  ******************************************************************************
  * @file           : motor_driver_improved.c
  * @brief          : Improved motor driver with better speed control
  ******************************************************************************
  * 
  * 改进要点：
  * 1. 修复PWM最小值问题
  * 2. 改进速度映射曲线
  * 3. 添加PWM校准功能
  * 4. 解决电机驱动器死区问题
  * 
  ******************************************************************************
  */

#include "motor_driver.h"
#include <stdio.h>
#include <string.h>

extern TIM_HandleTypeDef htim4;
extern UART_HandleTypeDef huart6;

// 校准参数
static uint32_t pwm_dead_zone = 500;      // 死区PWM值（需要校准）
static uint32_t pwm_effective_min = 800;  // 有效最小PWM值
static uint32_t pwm_effective_max = 9999; // 有效最大PWM值

/**
  * @brief  改进的速度到PWM转换函数
  * @param  speed: 速度百分比 (-100 to 100)
  * @retval PWM值
  */
static uint32_t Motor_ImprovedSpeedToPWM(int16_t speed)
{
    // 获取绝对值
    int16_t abs_speed = (speed < 0) ? -speed : speed;
    
    // 限制速度范围
    abs_speed = (abs_speed > 100) ? 100 : abs_speed;
    
    if (abs_speed == 0) {
        return 0;
    }
    
    // 使用线性映射，但考虑死区
    // 将1-100%映射到有效PWM范围
    uint32_t effective_range = pwm_effective_max - pwm_effective_min;
    uint32_t pwm_value = pwm_effective_min + (abs_speed * effective_range) / 100;
    
    // 确保不超过最大值
    if (pwm_value > PWM_MAX_VALUE) {
        pwm_value = PWM_MAX_VALUE;
    }
    
    return pwm_value;
}

/**
  * @brief  非线性速度映射（用于低速改善）
  * @param  speed: 速度百分比 (0 to 100)
  * @retval PWM值
  */
static uint32_t Motor_NonlinearSpeedToPWM(int16_t speed)
{
    int16_t abs_speed = (speed < 0) ? -speed : speed;
    abs_speed = (abs_speed > 100) ? 100 : abs_speed;
    
    if (abs_speed == 0) {
        return 0;
    }
    
    // 使用平方根映射提高低速精度
    float normalized_speed = (float)abs_speed / 100.0f;
    
    // 平方根映射：y = sqrt(x) * effective_range + min
    float sqrt_mapped = sqrtf(normalized_speed);
    
    uint32_t effective_range = pwm_effective_max - pwm_effective_min;
    uint32_t pwm_value = pwm_effective_min + (uint32_t)(sqrt_mapped * effective_range);
    
    return pwm_value;
}

/**
  * @brief  PWM死区校准功能
  * @param  None
  * @retval None
  */
void Motor_CalibrateDeadZone(void)
{
    char buffer[100];
    
    sprintf(buffer, "=== PWM Dead Zone Calibration ===\r\n");
    HAL_UART_Transmit(&huart6, (uint8_t*)buffer, strlen(buffer), HAL_MAX_DELAY);
    
    // 设置电机正转方向
    HAL_GPIO_WritePin(AIN1_GPIO_Port, AIN1_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(AIN2_GPIO_Port, AIN2_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(BIN1_GPIO_Port, BIN1_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(BIN2_GPIO_Port, BIN2_Pin, GPIO_PIN_SET);
    
    // 从低PWM值开始测试
    for (uint32_t pwm = 0; pwm <= 2000; pwm += 50) {
        __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, pwm);
        __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, pwm);
        
        sprintf(buffer, "Testing PWM: %4lu (%.1f%%) - Check if motors start\r\n", 
                pwm, (float)pwm/PWM_MAX_VALUE*100.0f);
        HAL_UART_Transmit(&huart6, (uint8_t*)buffer, strlen(buffer), HAL_MAX_DELAY);
        
        HAL_Delay(2000);
    }
    
    // 停止电机
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, 0);
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, 0);
    HAL_GPIO_WritePin(AIN1_GPIO_Port, AIN1_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(AIN2_GPIO_Port, AIN2_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(BIN1_GPIO_Port, BIN1_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(BIN2_GPIO_Port, BIN2_Pin, GPIO_PIN_RESET);
    
    sprintf(buffer, "Calibration completed. Update dead zone values in code.\r\n");
    HAL_UART_Transmit(&huart6, (uint8_t*)buffer, strlen(buffer), HAL_MAX_DELAY);
}

/**
  * @brief  设置PWM校准参数
  * @param  dead_zone: 死区PWM值
  * @param  min_effective: 最小有效PWM值
  * @param  max_effective: 最大有效PWM值
  * @retval None
  */
void Motor_SetCalibrationParams(uint32_t dead_zone, uint32_t min_effective, uint32_t max_effective)
{
    pwm_dead_zone = dead_zone;
    pwm_effective_min = min_effective;
    pwm_effective_max = max_effective;
    
    char buffer[150];
    sprintf(buffer, "PWM Calibration updated: Dead=%lu, Min=%lu, Max=%lu\r\n", 
            dead_zone, min_effective, max_effective);
    HAL_UART_Transmit(&huart6, (uint8_t*)buffer, strlen(buffer), HAL_MAX_DELAY);
}

/**
  * @brief  改进的电机速度设置（替代原函数）
  * @param  motor: 电机选择器
  * @param  speed: 速度百分比 (-100 to 100)
  * @retval None
  */
void Motor_SetSpeedImproved(MotorSelector_t motor, int16_t speed)
{
    // 限制速度范围
    speed = CLAMP(speed, -100, 100);
    
    // 确定方向和PWM值
    MotorDirection_t direction;
    uint32_t pwm_value;
    
    if (speed > 0) {
        direction = MOTOR_FORWARD;
        pwm_value = Motor_ImprovedSpeedToPWM(speed);
    } else if (speed < 0) {
        direction = MOTOR_BACKWARD;
        pwm_value = Motor_ImprovedSpeedToPWM(-speed);
    } else {
        direction = MOTOR_STOP;
        pwm_value = 0;
    }
    
    // 设置方向引脚
    if (motor == MOTOR_LEFT) {
        switch (direction) {
            case MOTOR_FORWARD:
                HAL_GPIO_WritePin(AIN1_GPIO_Port, AIN1_Pin, GPIO_PIN_SET);
                HAL_GPIO_WritePin(AIN2_GPIO_Port, AIN2_Pin, GPIO_PIN_RESET);
                break;
            case MOTOR_BACKWARD:
                HAL_GPIO_WritePin(AIN1_GPIO_Port, AIN1_Pin, GPIO_PIN_RESET);
                HAL_GPIO_WritePin(AIN2_GPIO_Port, AIN2_Pin, GPIO_PIN_SET);
                break;
            case MOTOR_STOP:
            default:
                HAL_GPIO_WritePin(AIN1_GPIO_Port, AIN1_Pin, GPIO_PIN_RESET);
                HAL_GPIO_WritePin(AIN2_GPIO_Port, AIN2_Pin, GPIO_PIN_RESET);
                break;
        }
        __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, pwm_value);
    } else {
        switch (direction) {
            case MOTOR_FORWARD:
                HAL_GPIO_WritePin(BIN1_GPIO_Port, BIN1_Pin, GPIO_PIN_RESET);
                HAL_GPIO_WritePin(BIN2_GPIO_Port, BIN2_Pin, GPIO_PIN_SET);
                break;
            case MOTOR_BACKWARD:
                HAL_GPIO_WritePin(BIN1_GPIO_Port, BIN1_Pin, GPIO_PIN_SET);
                HAL_GPIO_WritePin(BIN2_GPIO_Port, BIN2_Pin, GPIO_PIN_RESET);
                break;
            case MOTOR_STOP:
            default:
                HAL_GPIO_WritePin(BIN1_GPIO_Port, BIN1_Pin, GPIO_PIN_RESET);
                HAL_GPIO_WritePin(BIN2_GPIO_Port, BIN2_Pin, GPIO_PIN_RESET);
                break;
        }
        __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, pwm_value);
    }
}

/**
  * @brief  改进的前进功能
  * @param  speed: 前进速度 (0 to 100)
  * @retval None
  */
void Motor_ForwardImproved(int16_t speed)
{
    speed = CLAMP(speed, 0, 100);
    Motor_SetSpeedImproved(MOTOR_LEFT, speed);
    Motor_SetSpeedImproved(MOTOR_RIGHT, speed);
}

/**
  * @brief  精细速度测试
  * @param  None
  * @retval None
  */
void Motor_FineSpeedTest(void)
{
    char buffer[100];
    
    sprintf(buffer, "=== Fine Speed Control Test ===\r\n");
    HAL_UART_Transmit(&huart6, (uint8_t*)buffer, strlen(buffer), HAL_MAX_DELAY);
    
    // 测试非常细微的速度变化
    int16_t test_speeds[] = {1, 2, 3, 4, 5, 7, 10, 15, 20, 30, 50, 70, 100};
    uint8_t num_speeds = sizeof(test_speeds) / sizeof(test_speeds[0]);
    
    for (uint8_t i = 0; i < num_speeds; i++) {
        int16_t speed = test_speeds[i];
        uint32_t pwm_orig = (speed * PWM_MAX_VALUE) / 100;
        uint32_t pwm_improved = Motor_ImprovedSpeedToPWM(speed);
        
        sprintf(buffer, "Speed %2d%% -> Original PWM: %4lu, Improved PWM: %4lu\r\n", 
                speed, pwm_orig, pwm_improved);
        HAL_UART_Transmit(&huart6, (uint8_t*)buffer, strlen(buffer), HAL_MAX_DELAY);
        
        // 使用改进的速度控制
        Motor_ForwardImproved(speed);
        HAL_Delay(3000);
        Motor_SetSpeedImproved(MOTOR_LEFT, 0);
        Motor_SetSpeedImproved(MOTOR_RIGHT, 0);
        HAL_Delay(1000);
    }
    
    sprintf(buffer, "Fine speed test completed!\r\n");
    HAL_UART_Transmit(&huart6, (uint8_t*)buffer, strlen(buffer), HAL_MAX_DELAY);
}

/**
  * @brief  PWM频率诊断
  * @param  None
  * @retval None
  */
void Motor_DiagnosePWMFrequency(void)
{
    char buffer[150];
    
    sprintf(buffer, "=== PWM Frequency Diagnosis ===\r\n");
    HAL_UART_Transmit(&huart6, (uint8_t*)buffer, strlen(buffer), HAL_MAX_DELAY);
    
    // 计算实际PWM频率
    uint32_t timer_clock = 84000000; // 84MHz APB1 timer clock
    uint32_t prescaler = htim4.Init.Prescaler + 1;
    uint32_t period = htim4.Init.Period + 1;
    uint32_t pwm_frequency = timer_clock / (prescaler * period);
    
    sprintf(buffer, "Timer Clock: %lu Hz\r\n", timer_clock);
    HAL_UART_Transmit(&huart6, (uint8_t*)buffer, strlen(buffer), HAL_MAX_DELAY);
    
    sprintf(buffer, "Prescaler: %lu, Period: %lu\r\n", prescaler, period);
    HAL_UART_Transmit(&huart6, (uint8_t*)buffer, strlen(buffer), HAL_MAX_DELAY);
    
    sprintf(buffer, "PWM Frequency: %lu Hz\r\n", pwm_frequency);
    HAL_UART_Transmit(&huart6, (uint8_t*)buffer, strlen(buffer), HAL_MAX_DELAY);
    
    if (pwm_frequency < 1000) {
        sprintf(buffer, "WARNING: PWM frequency too low, may cause audible noise\r\n");
    } else if (pwm_frequency > 50000) {
        sprintf(buffer, "WARNING: PWM frequency too high, may reduce efficiency\r\n");
    } else {
        sprintf(buffer, "PWM frequency is within acceptable range\r\n");
    }
    HAL_UART_Transmit(&huart6, (uint8_t*)buffer, strlen(buffer), HAL_MAX_DELAY);
    
    sprintf(buffer, "PWM Resolution: %lu steps\r\n", PWM_MAX_VALUE + 1);
    HAL_UART_Transmit(&huart6, (uint8_t*)buffer, strlen(buffer), HAL_MAX_DELAY);
}

// Math functions (需要包含math.h或提供简单实现)
float sqrtf(float x) {
    if (x < 0) return 0;
    if (x == 0) return 0;
    
    // 简单的牛顿法实现
    float result = x;
    for (int i = 0; i < 10; i++) {
        result = 0.5f * (result + x / result);
    }
    return result;
} 