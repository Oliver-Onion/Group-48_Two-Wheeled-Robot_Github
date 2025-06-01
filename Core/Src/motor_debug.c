/**
  ******************************************************************************
  * @file           : motor_debug.c
  * @brief          : Motor debugging functions for diagnosing speed control issues
  ******************************************************************************
  */

#include "motor_debug.h"
#include "motor_driver.h"
#include <stdio.h>
#include <string.h>

extern UART_HandleTypeDef huart6;
extern TIM_HandleTypeDef htim4;

/**
  * @brief  测试PWM输出功能
  * @param  None
  * @retval None
  */
void Motor_Debug_PWMTest(void)
{
    char buffer[200];
    
    // 发送测试开始信息
    HAL_UART_Transmit(&huart6, (uint8_t*)"=== PWM Output Test ===\r\n", 26, HAL_MAX_DELAY);
    
    // 检查TIM4配置信息
    sprintf(buffer, "TIM4 Config:\r\n");
    HAL_UART_Transmit(&huart6, (uint8_t*)buffer, strlen(buffer), HAL_MAX_DELAY);
    
    sprintf(buffer, "  Prescaler: %lu\r\n", htim4.Init.Prescaler);
    HAL_UART_Transmit(&huart6, (uint8_t*)buffer, strlen(buffer), HAL_MAX_DELAY);
    
    sprintf(buffer, "  Period: %lu\r\n", htim4.Init.Period);
    HAL_UART_Transmit(&huart6, (uint8_t*)buffer, strlen(buffer), HAL_MAX_DELAY);
    
    sprintf(buffer, "  Clock Division: %lu\r\n", htim4.Init.ClockDivision);
    HAL_UART_Transmit(&huart6, (uint8_t*)buffer, strlen(buffer), HAL_MAX_DELAY);
    
    // 计算PWM频率
    // APB1TimFreq = 84MHz (从.ioc文件得知)
    uint32_t timer_freq = 84000000; // 84MHz
    uint32_t pwm_freq = timer_freq / ((htim4.Init.Prescaler + 1) * (htim4.Init.Period + 1));
    sprintf(buffer, "  Calculated PWM Freq: %lu Hz\r\n", pwm_freq);
    HAL_UART_Transmit(&huart6, (uint8_t*)buffer, strlen(buffer), HAL_MAX_DELAY);
    
    // 测试不同的PWM值
    HAL_UART_Transmit(&huart6, (uint8_t*)"Testing PWM values:\r\n", 21, HAL_MAX_DELAY);
    
    uint32_t test_values[] = {0, 100, 500, 1000, 2000, 3000, 5000, 7000, 9000, 9999};
    uint8_t num_tests = sizeof(test_values) / sizeof(test_values[0]);
    
    for (uint8_t i = 0; i < num_tests; i++) {
        uint32_t pwm_value = test_values[i];
        
        // 设置PWM值
        __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, pwm_value);  // Left motor
        __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, pwm_value);  // Right motor
        
        // 计算占空比
        float duty_cycle = (float)pwm_value / 9999.0 * 100.0;
        
        sprintf(buffer, "  PWM: %5lu, Duty: %5.1f%%, Speed: %3d%%\r\n", 
                pwm_value, duty_cycle, (int)(duty_cycle));
        HAL_UART_Transmit(&huart6, (uint8_t*)buffer, strlen(buffer), HAL_MAX_DELAY);
        
        HAL_Delay(2000);  // 持续2秒让用户观察
    }
    
    // 停止PWM
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, 0);
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, 0);
    
    HAL_UART_Transmit(&huart6, (uint8_t*)"PWM Test completed!\r\n", 21, HAL_MAX_DELAY);
}

/**
  * @brief  测试电机方向控制
  * @param  None
  * @retval None
  */
void Motor_Debug_DirectionTest(void)
{
    char buffer[100];
    
    HAL_UART_Transmit(&huart6, (uint8_t*)"=== Direction Pin Test ===\r\n", 29, HAL_MAX_DELAY);
    
    // 测试左电机方向引脚
    HAL_UART_Transmit(&huart6, (uint8_t*)"Left Motor Direction Test:\r\n", 28, HAL_MAX_DELAY);
    
    HAL_UART_Transmit(&huart6, (uint8_t*)"  Forward: AIN1=1, AIN2=0\r\n", 27, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(AIN1_GPIO_Port, AIN1_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(AIN2_GPIO_Port, AIN2_Pin, GPIO_PIN_RESET);
    HAL_Delay(1000);
    
    HAL_UART_Transmit(&huart6, (uint8_t*)"  Backward: AIN1=0, AIN2=1\r\n", 28, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(AIN1_GPIO_Port, AIN1_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(AIN2_GPIO_Port, AIN2_Pin, GPIO_PIN_SET);
    HAL_Delay(1000);
    
    HAL_UART_Transmit(&huart6, (uint8_t*)"  Stop: AIN1=0, AIN2=0\r\n", 24, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(AIN1_GPIO_Port, AIN1_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(AIN2_GPIO_Port, AIN2_Pin, GPIO_PIN_RESET);
    HAL_Delay(1000);
    
    // 测试右电机方向引脚
    HAL_UART_Transmit(&huart6, (uint8_t*)"Right Motor Direction Test:\r\n", 29, HAL_MAX_DELAY);
    
    HAL_UART_Transmit(&huart6, (uint8_t*)"  Forward: BIN1=0, BIN2=1\r\n", 27, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(BIN1_GPIO_Port, BIN1_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(BIN2_GPIO_Port, BIN2_Pin, GPIO_PIN_SET);
    HAL_Delay(1000);
    
    HAL_UART_Transmit(&huart6, (uint8_t*)"  Backward: BIN1=1, BIN2=0\r\n", 28, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(BIN1_GPIO_Port, BIN1_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(BIN2_GPIO_Port, BIN2_Pin, GPIO_PIN_RESET);
    HAL_Delay(1000);
    
    HAL_UART_Transmit(&huart6, (uint8_t*)"  Stop: BIN1=0, BIN2=0\r\n", 24, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(BIN1_GPIO_Port, BIN1_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(BIN2_GPIO_Port, BIN2_Pin, GPIO_PIN_RESET);
    
    HAL_UART_Transmit(&huart6, (uint8_t*)"Direction Test completed!\r\n", 27, HAL_MAX_DELAY);
}

/**
  * @brief  测试速度控制函数
  * @param  None
  * @retval None
  */
void Motor_Debug_SpeedFunctionTest(void)
{
    char buffer[150];
    
    HAL_UART_Transmit(&huart6, (uint8_t*)"=== Speed Function Test ===\r\n", 30, HAL_MAX_DELAY);
    
    // 测试不同速度值
    int16_t test_speeds[] = {0, 1, 5, 10, 20, 30, 50, 70, 90, 100};
    uint8_t num_speeds = sizeof(test_speeds) / sizeof(test_speeds[0]);
    
    for (uint8_t i = 0; i < num_speeds; i++) {
        int16_t speed = test_speeds[i];
        
        sprintf(buffer, "Testing speed: %d%%\r\n", speed);
        HAL_UART_Transmit(&huart6, (uint8_t*)buffer, strlen(buffer), HAL_MAX_DELAY);
        
        // 使用电机控制函数
        Motor_Forward(speed);
        
        // 读取实际PWM值
        uint32_t left_pwm = __HAL_TIM_GET_COMPARE(&htim4, TIM_CHANNEL_3);
        uint32_t right_pwm = __HAL_TIM_GET_COMPARE(&htim4, TIM_CHANNEL_4);
        
        sprintf(buffer, "  Left PWM: %5lu, Right PWM: %5lu\r\n", left_pwm, right_pwm);
        HAL_UART_Transmit(&huart6, (uint8_t*)buffer, strlen(buffer), HAL_MAX_DELAY);
        
        HAL_Delay(1500);
    }
    
    Motor_Stop();
    HAL_UART_Transmit(&huart6, (uint8_t*)"Speed Function Test completed!\r\n", 32, HAL_MAX_DELAY);
}

/**
  * @brief  诊断电机控制问题
  * @param  None
  * @retval None
  */
void Motor_Debug_DiagnoseProblem(void)
{
    char buffer[200];
    
    HAL_UART_Transmit(&huart6, (uint8_t*)"=== Motor Control Diagnosis ===\r\n", 34, HAL_MAX_DELAY);
    
    // 1. 检查PWM配置
    HAL_UART_Transmit(&huart6, (uint8_t*)"1. Checking PWM Configuration...\r\n", 35, HAL_MAX_DELAY);
    
    if (htim4.Init.Prescaler == 0 && htim4.Init.Period == 9999) {
        HAL_UART_Transmit(&huart6, (uint8_t*)"   PWM Config: OK\r\n", 19, HAL_MAX_DELAY);
    } else {
        HAL_UART_Transmit(&huart6, (uint8_t*)"   PWM Config: ISSUE DETECTED\r\n", 31, HAL_MAX_DELAY);
        sprintf(buffer, "   Expected: Prescaler=0, Period=9999\r\n   Actual: Prescaler=%lu, Period=%lu\r\n", 
                htim4.Init.Prescaler, htim4.Init.Period);
        HAL_UART_Transmit(&huart6, (uint8_t*)buffer, strlen(buffer), HAL_MAX_DELAY);
    }
    
    // 2. 检查PWM启动状态
    HAL_UART_Transmit(&huart6, (uint8_t*)"2. Checking PWM Start Status...\r\n", 34, HAL_MAX_DELAY);
    
    // 这里可以通过检查TIM4的寄存器来确认PWM是否启动
    if (__HAL_TIM_IS_TIM_COUNTING_DOWN(&htim4) || __HAL_TIM_GET_COUNTER(&htim4) > 0) {
        HAL_UART_Transmit(&huart6, (uint8_t*)"   PWM Timer: RUNNING\r\n", 23, HAL_MAX_DELAY);
    } else {
        HAL_UART_Transmit(&huart6, (uint8_t*)"   PWM Timer: NOT RUNNING - ISSUE!\r\n", 37, HAL_MAX_DELAY);
    }
    
    // 3. 测试最小PWM值
    HAL_UART_Transmit(&huart6, (uint8_t*)"3. Testing minimum PWM threshold...\r\n", 38, HAL_MAX_DELAY);
    
    HAL_UART_Transmit(&huart6, (uint8_t*)"   Setting very low PWM (50)...\r\n", 33, HAL_MAX_DELAY);
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, 50);
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, 50);
    
    // 设置正转方向
    HAL_GPIO_WritePin(AIN1_GPIO_Port, AIN1_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(AIN2_GPIO_Port, AIN2_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(BIN1_GPIO_Port, BIN1_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(BIN2_GPIO_Port, BIN2_Pin, GPIO_PIN_SET);
    
    HAL_Delay(3000);  // 让用户观察电机是否转动
    
    Motor_Stop();
    HAL_UART_Transmit(&huart6, (uint8_t*)"   Low PWM test completed\r\n", 27, HAL_MAX_DELAY);
    
    // 4. 给出诊断建议
    HAL_UART_Transmit(&huart6, (uint8_t*)"4. Diagnosis Summary:\r\n", 23, HAL_MAX_DELAY);
    HAL_UART_Transmit(&huart6, (uint8_t*)"   Common causes of constant speed:\r\n", 37, HAL_MAX_DELAY);
    HAL_UART_Transmit(&huart6, (uint8_t*)"   a) Motor driver board dead zone\r\n", 36, HAL_MAX_DELAY);
    HAL_UART_Transmit(&huart6, (uint8_t*)"   b) Power supply voltage issues\r\n", 35, HAL_MAX_DELAY);
    HAL_UART_Transmit(&huart6, (uint8_t*)"   c) Motor mechanical issues\r\n", 31, HAL_MAX_DELAY);
    HAL_UART_Transmit(&huart6, (uint8_t*)"   d) PWM not reaching motor driver\r\n", 37, HAL_MAX_DELAY);
    
    HAL_UART_Transmit(&huart6, (uint8_t*)"Diagnosis completed!\r\n", 22, HAL_MAX_DELAY);
}

/**
  * @brief  实时显示PWM值
  * @param  speed 输入速度值
  * @retval None
  */
void Motor_Debug_ShowPWMValue(int16_t speed)
{
    char buffer[100];
    
    // 计算PWM值
    int16_t abs_speed = (speed < 0) ? -speed : speed;
    abs_speed = (abs_speed > 100) ? 100 : abs_speed;
    
    uint32_t pwm_value = (uint32_t)((abs_speed * 9999) / 100);
    if (pwm_value > 0 && pwm_value < 10) {
        pwm_value = 10;  // 最小PWM值
    }
    
    sprintf(buffer, "Speed: %3d%% -> PWM: %4lu (%.1f%%)\r\n", 
            speed, pwm_value, (float)pwm_value/9999.0*100.0);
    HAL_UART_Transmit(&huart6, (uint8_t*)buffer, strlen(buffer), HAL_MAX_DELAY);
} 