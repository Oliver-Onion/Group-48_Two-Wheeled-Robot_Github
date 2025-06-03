/**
  ******************************************************************************
  * @file           : motor_at8236_example.c
  * @brief          : Example usage of AT8236 motor driver
  ******************************************************************************
  * @attention
  *
  * This file demonstrates how to use the motor driver with AT8236 module
  * including basic movement control and speed feedback control
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "motor_driver.h"
#include <stdio.h>

/* Private variables ---------------------------------------------------------*/

/**
  * @brief  Example of basic motor control
  * @param  None
  * @retval None
  */
void Motor_BasicExample(void)
{
    // Initialize motor driver
    Motor_Init();
    
    // Move forward at 50% speed for 2 seconds
    printf("Moving forward at 50%% speed...\n");
    Motor_Forward(50);
    HAL_Delay(2000);
    
    // Stop motors
    printf("Stopping motors...\n");
    Motor_Stop();
    HAL_Delay(1000);
    
    // Move backward at 30% speed for 2 seconds
    printf("Moving backward at 30%% speed...\n");
    Motor_Backward(30);
    HAL_Delay(2000);
    
    // Turn left at 40% speed for 1 second
    printf("Turning left at 40%% speed...\n");
    Motor_TurnLeft(40);
    HAL_Delay(1000);
    
    // Turn right at 40% speed for 1 second
    printf("Turning right at 40%% speed...\n");
    Motor_TurnRight(40);
    HAL_Delay(1000);
    
    // Brake motors
    printf("Braking motors...\n");
    Motor_Brake();
    HAL_Delay(1000);
}

/**
  * @brief  Example of individual motor control
  * @param  None
  * @retval None
  */
void Motor_IndividualControlExample(void)
{
    // Initialize motor driver
    Motor_Init();
    
    // Set different speeds for each motor
    printf("Setting left motor to 60%%, right motor to 40%%...\n");
    Motor_SetSpeeds(60, 40);  // Gentle right curve
    HAL_Delay(2000);
    
    // Reverse the speeds
    printf("Setting left motor to 40%%, right motor to 60%%...\n");
    Motor_SetSpeeds(40, 60);  // Gentle left curve
    HAL_Delay(2000);
    
    // One motor forward, one backward (sharp turn)
    printf("Left motor backward, right motor forward (sharp turn)...\n");
    Motor_SetSpeeds(-50, 50);
    HAL_Delay(1000);
    
    // Stop motors
    Motor_Stop();
}

/**
  * @brief  Example of encoder feedback and speed monitoring
  * @param  None
  * @retval None
  */
void Motor_EncoderExample(void)
{
    EncoderData_t encoder_data;
    MotorSpeeds_t motor_speeds;
    
    // Initialize motor driver
    Motor_Init();
    
    printf("Starting encoder monitoring example...\n");
    
    // Move forward and monitor speed for 5 seconds
    Motor_Forward(70);
    
    for (int i = 0; i < 50; i++) {
        // Update encoder readings
        Encoder_Update();
        
        // Get encoder data
        encoder_data = Encoder_GetData();
        motor_speeds = Motor_GetSpeeds();
        
        // Print data every 100ms
        printf("Time: %d00ms | ", i);
        printf("Left: %ld counts, %.2f RPM | ", 
               encoder_data.left_encoder, encoder_data.left_speed_rpm);
        printf("Right: %ld counts, %.2f RPM | ",
               encoder_data.right_encoder, encoder_data.right_speed_rpm);
        printf("Speeds: L=%d%%, R=%d%%\n",
               motor_speeds.left_speed, motor_speeds.right_speed);
        
        HAL_Delay(100);
    }
    
    Motor_Stop();
}

/**
  * @brief  Example of speed control with feedback
  * @param  None
  * @retval None
  */
void Motor_FeedbackControlExample(void)
{
    EncoderData_t encoder_data;
    float target_left_rpm = 200.0f;   // Target 200 RPM for left motor
    float target_right_rpm = 200.0f;  // Target 200 RPM for right motor
    
    // Initialize motor driver
    Motor_Init();
    
    printf("Starting feedback control example...\n");
    printf("Target speed: %.1f RPM for both motors\n", target_left_rpm);
    
    // Run feedback control for 10 seconds
    for (int i = 0; i < 100; i++) {
        // Set target speeds with feedback control
        Motor_SetSpeedsWithFeedback(target_left_rpm, target_right_rpm);
        
        // Get current speeds for monitoring
        encoder_data = Encoder_GetData();
        
        // Print status every 500ms
        if (i % 5 == 0) {
            printf("Time: %d00ms | ", i);
            printf("Target: %.1f RPM | ", target_left_rpm);
            printf("Actual: L=%.2f RPM, R=%.2f RPM\n",
                   encoder_data.left_speed_rpm, encoder_data.right_speed_rpm);
        }
        
        // Change target speed halfway through
        if (i == 50) {
            target_left_rpm = 150.0f;
            target_right_rpm = 250.0f;
            printf("Changing target speeds: L=%.1f RPM, R=%.1f RPM\n",
                   target_left_rpm, target_right_rpm);
        }
        
        HAL_Delay(100);
    }
    
    Motor_Stop();
}

/**
  * @brief  Main example function demonstrating all features
  * @param  None
  * @retval None
  */
void Motor_CompleteExample(void)
{
    printf("=== AT8236 Motor Driver Example ===\n\n");
    
    // Run basic control example
    printf("1. Basic Motor Control Example\n");
    Motor_BasicExample();
    HAL_Delay(2000);
    
    // Run individual control example
    printf("\n2. Individual Motor Control Example\n");
    Motor_IndividualControlExample();
    HAL_Delay(2000);
    
    // Run encoder monitoring example
    printf("\n3. Encoder Monitoring Example\n");
    Motor_EncoderExample();
    HAL_Delay(2000);
    
    // Run feedback control example
    printf("\n4. Feedback Control Example\n");
    Motor_FeedbackControlExample();
    
    printf("\n=== Example Complete ===\n");
}

/**
  * @brief  Helper function to print current motor status
  * @param  None
  * @retval None
  */
void Motor_PrintStatus(void)
{
    EncoderData_t encoder_data = Encoder_GetData();
    MotorSpeeds_t motor_speeds = Motor_GetSpeeds();
    
    printf("Motor Status:\n");
    printf("  Left Motor:  %d%% speed, %ld counts, %.2f RPM\n",
           motor_speeds.left_speed, encoder_data.left_encoder, encoder_data.left_speed_rpm);
    printf("  Right Motor: %d%% speed, %ld counts, %.2f RPM\n",
           motor_speeds.right_speed, encoder_data.right_encoder, encoder_data.right_speed_rpm);
} 