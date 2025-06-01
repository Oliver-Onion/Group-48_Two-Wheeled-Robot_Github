#include "motor_direction_test.h"
#include "motor_debug.h"
#include <string.h>
#include <stdio.h>

// 外部UART句柄（需要在main.c中定义）
extern UART_HandleTypeDef huart6;  // 根据你的UART配置调整

/**
  * @brief  通过蓝牙发送消息
  * @param  message: 要发送的消息
  * @retval None
  */
static void Bluetooth_SendMessage(const char* message)
{
    HAL_UART_Transmit(&huart6, (uint8_t*)message, strlen(message), HAL_MAX_DELAY);
}

/**
  * @brief  蓝牙电机方向测试
  * @param  None
  * @retval None
  */
void Motor_BluetoothDirectionTest(void)
{
    Bluetooth_SendMessage("Starting direction test...\r\n");
    
    // 1. Forward test
    Bluetooth_SendMessage("Forward test (3 sec)\r\n");
    Motor_Forward(30);
    HAL_Delay(3000);
    Motor_Stop();
    HAL_Delay(1000);
    
    // 2. Backward test
    Bluetooth_SendMessage("Backward test (3 sec)\r\n");
    Motor_Backward(30);
    HAL_Delay(3000);
    Motor_Stop();
    HAL_Delay(1000);
    
    // 3. Left turn test
    Bluetooth_SendMessage("Left turn test (2 sec)\r\n");
    Motor_TurnLeft(30);
    HAL_Delay(2000);
    Motor_Stop();
    HAL_Delay(1000);
    
    // 4. Right turn test
    Bluetooth_SendMessage("Right turn test (2 sec)\r\n");
    Motor_TurnRight(30);
    HAL_Delay(2000);
    Motor_Stop();
    HAL_Delay(1000);
    
    Bluetooth_SendMessage("Direction test completed!\r\n");
}

/**
  * @brief  蓝牙单电机测试
  * @param  None
  * @retval None
  */
void Motor_BluetoothIndividualTest(void)
{
    Bluetooth_SendMessage("Starting individual motor test...\r\n");
    
    // Left motor forward
    Bluetooth_SendMessage("Left motor forward\r\n");
    Motor_SetSpeed(MOTOR_LEFT, 40);
    Motor_SetSpeed(MOTOR_RIGHT, 0);
    HAL_Delay(2000);
    Motor_Stop();
    HAL_Delay(1000);
    
    // Left motor backward
    Bluetooth_SendMessage("Left motor backward\r\n");
    Motor_SetSpeed(MOTOR_LEFT, -40);
    Motor_SetSpeed(MOTOR_RIGHT, 0);
    HAL_Delay(2000);
    Motor_Stop();
    HAL_Delay(1000);
    
    // Right motor forward
    Bluetooth_SendMessage("Right motor forward\r\n");
    Motor_SetSpeed(MOTOR_LEFT, 0);
    Motor_SetSpeed(MOTOR_RIGHT, 40);
    HAL_Delay(2000);
    Motor_Stop();
    HAL_Delay(1000);
    
    // Right motor backward
    Bluetooth_SendMessage("Right motor backward\r\n");
    Motor_SetSpeed(MOTOR_LEFT, 0);
    Motor_SetSpeed(MOTOR_RIGHT, -40);
    HAL_Delay(2000);
    Motor_Stop();
    HAL_Delay(1000);
    
    Bluetooth_SendMessage("Individual motor test completed!\r\n");
}

/**
  * @brief  蓝牙编码器测试
  * @param  None
  * @retval None
  */
void Encoder_BluetoothTest(void)
{
    char buffer[100];
    
    Bluetooth_SendMessage("Starting encoder test...\r\n");
    
    // Reset encoders
    Encoder_Reset();
    Bluetooth_SendMessage("Encoders reset\r\n");
    
    // Forward test
    Bluetooth_SendMessage("Moving forward 5 sec, monitoring encoders...\r\n");
    Motor_Forward(40);
    
    for (int i = 0; i < 10; i++) {  // Output every 0.5 sec
        Encoder_Update();
        int32_t left_count = Encoder_GetLeftCount();
        int32_t right_count = Encoder_GetRightCount();
        
        sprintf(buffer, "Encoders: L=%ld, R=%ld\r\n", left_count, right_count);
        Bluetooth_SendMessage(buffer);
        HAL_Delay(500);
    }
    
    Motor_Stop();
    
    // Final results
    Encoder_Update();
    int32_t final_left = Encoder_GetLeftCount();
    int32_t final_right = Encoder_GetRightCount();
    
    sprintf(buffer, "Final counts: L=%ld, R=%ld\r\n", final_left, final_right);
    Bluetooth_SendMessage(buffer);
    
    if (final_left > 0 && final_right > 0) {
        Bluetooth_SendMessage("Encoder direction correct\r\n");
    } else if (final_left < 0 && final_right < 0) {
        Bluetooth_SendMessage("Encoder direction reversed\r\n");
    } else {
        Bluetooth_SendMessage("Encoder count abnormal\r\n");
    }
}

/**
  * @brief  蓝牙实时编码器监控
  * @param  None
  * @retval None
  */
void Encoder_BluetoothMonitor(void)
{
    char buffer[150];
    
    Encoder_Update();
    EncoderData_t data = Encoder_GetData();
    
    sprintf(buffer, "L:%ld(%.1fRPM) R:%ld(%.1fRPM)\r\n", 
            data.left_encoder, data.left_speed_rpm,
            data.right_encoder, data.right_speed_rpm);
    Bluetooth_SendMessage(buffer);
}

/**
  * @brief  蓝牙电机测试命令处理
  * @param  cmd: 接收到的蓝牙命令
  * @retval None
  */
void Motor_BluetoothCommand(uint8_t cmd)
{
    char buffer[100];
    
    switch (cmd) {
        // Test commands
        case 't':  // Complete direction test
        case 'T':
            Motor_BluetoothDirectionTest();
            break;
            
        case 'i':  // Individual test
        case 'I':
            Motor_BluetoothIndividualTest();
            break;
            
        case 'c':  // Encoder test
        case 'C':
            Encoder_BluetoothTest();
            break;
            
        // Debug commands (NEW)
        case 'p':  // PWM test
        case 'P':
            Motor_Debug_PWMTest();
            break;
            
        case 'g':  // Direction GPIO test
        case 'G':
            Motor_Debug_DirectionTest();
            break;
            
        case 'f':  // Speed Function test
        case 'F':
            Motor_Debug_SpeedFunctionTest();
            break;
            
        case 'z':  // Diagnosis
        case 'Z':
            Motor_Debug_DiagnoseProblem();
            break;
            
        // Basic control
        case 'w':  // Forward
        case 'W':
            Motor_Forward(50);
            Motor_Debug_ShowPWMValue(50);
            Bluetooth_SendMessage("Moving forward (50%)\r\n");
            break;
            
        case 's':  // Backward
        case 'S':
            Motor_Backward(50);
            Motor_Debug_ShowPWMValue(-50);
            Bluetooth_SendMessage("Moving backward (50%)\r\n");
            break;
            
        case 'a':  // Turn left
        case 'A':
            Motor_TurnLeft(50);
            Bluetooth_SendMessage("Turning left (50%)\r\n");
            break;
            
        case 'd':  // Turn right
        case 'D':
            Motor_TurnRight(50);
            Bluetooth_SendMessage("Turning right (50%)\r\n");
            break;
            
        case ' ':  // Stop
            Motor_Stop();
            Bluetooth_SendMessage("Stopped\r\n");
            break;
            
        case 'b':  // Brake
        case 'B':
            Motor_Brake();
            Bluetooth_SendMessage("Braking\r\n");
            break;
            
        // Enhanced speed control with debug info
        case '1':  // Ultra low speed (1%)
            Motor_Forward(1);
            Motor_Debug_ShowPWMValue(1);
            sprintf(buffer, "Ultra low speed forward (1%%)\r\n");
            Bluetooth_SendMessage(buffer);
            break;
            
        case '2':  // Very low speed (5%)
            Motor_Forward(5);
            Motor_Debug_ShowPWMValue(5);
            sprintf(buffer, "Very low speed forward (5%%)\r\n");
            Bluetooth_SendMessage(buffer);
            break;
            
        case '3':  // Low speed (7%)
            Motor_Forward(7);
            Motor_Debug_ShowPWMValue(7);
            sprintf(buffer, "Low speed forward (7%%)\r\n");
            Bluetooth_SendMessage(buffer);
            break;
            
        case '4':  // Medium-low speed (10%)
            Motor_Forward(10);
            Motor_Debug_ShowPWMValue(10);
            sprintf(buffer, "Medium-low speed forward (10%%)\r\n");
            Bluetooth_SendMessage(buffer);
            break;
            
        case '5':  // Medium speed (50%)
            Motor_Forward(50);
            Motor_Debug_ShowPWMValue(50);
            sprintf(buffer, "Medium speed forward (50%%)\r\n");
            Bluetooth_SendMessage(buffer);
            break;
            
        case '6':  // Medium-high speed (65%)
            Motor_Forward(65);
            Motor_Debug_ShowPWMValue(65);
            sprintf(buffer, "Medium-high speed forward (65%%)\r\n");
            Bluetooth_SendMessage(buffer);
            break;
            
        case '7':  // High speed (80%)
            Motor_Forward(80);
            Motor_Debug_ShowPWMValue(80);
            sprintf(buffer, "High speed forward (80%%)\r\n");
            Bluetooth_SendMessage(buffer);
            break;
            
        case '8':  // Very high speed (90%)
            Motor_Forward(90);
            Motor_Debug_ShowPWMValue(90);
            sprintf(buffer, "Very high speed forward (90%%)\r\n");
            Bluetooth_SendMessage(buffer);
            break;
            
        case '9':  // Maximum speed (100%)
            Motor_Forward(100);
            Motor_Debug_ShowPWMValue(100);
            sprintf(buffer, "Maximum speed forward (100%%)\r\n");
            Bluetooth_SendMessage(buffer);
            break;
            
        // Reset encoders
        case 'r':
        case 'R':
            Encoder_Reset();
            Bluetooth_SendMessage("Encoders reset\r\n");
            break;
            
        // Encoder monitor
        case 'e':
        case 'E':
            Encoder_BluetoothMonitor();
            break;
            
        // Help
        case 'h':  // Help
        case 'H':
        case '?':
            Bluetooth_SendMessage("\r\n=== Bluetooth Motor Control ===\r\n");
            Bluetooth_SendMessage("Tests: t-Direction i-Individual c-Encoder\r\n");
            Bluetooth_SendMessage("Debug: p-PWM Test g-GPIO Test f-Function Test z-Diagnose\r\n");
            Bluetooth_SendMessage("Move:  w-Forward s-Back a-Left d-Right\r\n");
            Bluetooth_SendMessage("Other: Space-Stop b-Brake r-Reset e-Encoder\r\n");
            Bluetooth_SendMessage("Speed Control (with PWM info):\r\n");
            Bluetooth_SendMessage("1 - Ultra low (1%)\r\n");
            Bluetooth_SendMessage("2 - Very low (5%)\r\n");
            Bluetooth_SendMessage("3 - Low (7%)\r\n");
            Bluetooth_SendMessage("4 - Medium-low (10%)\r\n");
            Bluetooth_SendMessage("5 - Medium (50%)\r\n");
            Bluetooth_SendMessage("6 - Medium-high (65%)\r\n");
            Bluetooth_SendMessage("7 - High (80%)\r\n");
            Bluetooth_SendMessage("8 - Very high (90%)\r\n");
            Bluetooth_SendMessage("9 - Maximum (100%)\r\n");
            Bluetooth_SendMessage("==========================\r\n");
            break;
            
        default:
            Bluetooth_SendMessage("Unknown command, send h for help\r\n");
            Motor_Stop();  // Safety stop
            break;
    }
} 
