/**
  ******************************************************************************
  * @file           : motor_driver_example.c
  * @brief          : Example usage of motor driver for Two-Wheeled Robot
  ******************************************************************************
  * @attention
  *
  * This file shows how to integrate the motor driver into your main.c file
  * Updated for dual encoder configuration with TIM2, TIM3, and TIM4
  * Copy the relevant sections to your main.c file as needed.
  *
  ******************************************************************************
  */

/*
 * === STEP 1: Add includes to main.c ===
 * Add this line to the USER CODE BEGIN Includes section:
 */
#include "motor_driver.h"

/*
 * === STEP 2: Add motor initialization to main.c ===
 * Add this to the USER CODE BEGIN 2 section (after MX_TIM4_Init()):
 */
void example_motor_initialization(void)
{
    // Initialize motor driver (includes encoder initialization)
    Motor_Init();
    
    // Test motors briefly (optional)
    Motor_Forward(30);      // Move forward at 30% speed
    HAL_Delay(1000);        // Wait 1 second
    Motor_Stop();           // Stop motors
    HAL_Delay(500);         // Wait 0.5 seconds
}

/*
 * === STEP 3: Add encoder monitoring (NEW FEATURE) ===
 * Call this function periodically to update encoder readings
 */
void example_encoder_monitoring(void)
{
    // Update encoder readings
    Encoder_Update();
    
    // Get encoder data
    EncoderData_t encoder_data = Encoder_GetData();
    
    // Send encoder information via UART
    char buffer[200];
    sprintf(buffer, "Left: %ld counts, %.2f RPM | Right: %ld counts, %.2f RPM\r\n",
            encoder_data.left_encoder, encoder_data.left_speed_rpm,
            encoder_data.right_encoder, encoder_data.right_speed_rpm);
    HAL_UART_Transmit(&huart6, (uint8_t*)buffer, strlen(buffer), HAL_MAX_DELAY);
}

/*
 * === STEP 4: Basic movement examples ===
 * These functions can be called from your main loop or interrupt handlers
 */

// Example 1: Basic movement functions
void example_basic_movements(void)
{
    // Move forward at 50% speed
    Motor_Forward(50);
    HAL_Delay(2000);
    
    // Move backward at 40% speed
    Motor_Backward(40);
    HAL_Delay(2000);
    
    // Turn left at 60% speed
    Motor_TurnLeft(60);
    HAL_Delay(1000);
    
    // Turn right at 60% speed
    Motor_TurnRight(60);
    HAL_Delay(1000);
    
    // Stop motors
    Motor_Stop();
}

// Example 2: Individual motor control
void example_individual_control(void)
{
    // Set left motor to 70% forward, right motor to 30% forward (curve right)
    Motor_SetSpeeds(70, 30);
    HAL_Delay(1500);
    
    // Set left motor to 30% forward, right motor to 70% forward (curve left)
    Motor_SetSpeeds(30, 70);
    HAL_Delay(1500);
    
    // Set one motor forward, one backward (spin in place)
    Motor_SetSpeeds(50, -50);
    HAL_Delay(1000);
    
    Motor_Stop();
}

// Example 3: Speed ramping
void example_speed_ramping(void)
{
    // Gradually increase speed from 0 to 80%
    for (int speed = 0; speed <= 80; speed += 10) {
        Motor_Forward(speed);
        HAL_Delay(200);
    }
    
    // Gradually decrease speed from 80% to 0
    for (int speed = 80; speed >= 0; speed -= 10) {
        Motor_Forward(speed);
        HAL_Delay(200);
    }
    
    Motor_Stop();
}

// Example 4: Emergency brake
void example_emergency_brake(void)
{
    // Move forward at high speed
    Motor_Forward(90);
    HAL_Delay(1000);
    
    // Emergency brake (active braking)
    Motor_Brake();
    HAL_Delay(500);
    
    // Return to normal stop
    Motor_Stop();
}

// Example 5: Using encoder feedback (NEW FEATURE)
void example_encoder_feedback(void)
{
    // Reset encoders
    Encoder_Reset();
    
    // Move forward for a specific distance (e.g., 1000 encoder counts)
    Motor_Forward(50);
    
    while (1) {
        Encoder_Update();
        int32_t left_count = Encoder_GetLeftCount();
        int32_t right_count = Encoder_GetRightCount();
        
        // Check if we've traveled the desired distance
        if (left_count >= 1000 || right_count >= 1000) {
            Motor_Stop();
            break;
        }
        
        HAL_Delay(10);  // Small delay
    }
    
    // Get final encoder data
    EncoderData_t final_data = Encoder_GetData();
    char buffer[150];
    sprintf(buffer, "Final position - Left: %ld, Right: %ld counts\r\n",
            final_data.left_encoder, final_data.right_encoder);
    HAL_UART_Transmit(&huart6, (uint8_t*)buffer, strlen(buffer), HAL_MAX_DELAY);
}

// Example 6: Closed-loop speed control (NEW FEATURE)
void example_closed_loop_control(void)
{
    // Set target speeds in RPM
    float target_left_rpm = 100.0f;
    float target_right_rpm = 100.0f;
    
    // Run closed-loop control for 5 seconds
    uint32_t start_time = HAL_GetTick();
    
    while (HAL_GetTick() - start_time < 5000) {
        Motor_SetSpeedsWithFeedback(target_left_rpm, target_right_rpm);
        
        // Monitor actual speeds
        float left_rpm = Encoder_GetLeftSpeed();
        float right_rpm = Encoder_GetRightSpeed();
        
        char buffer[100];
        sprintf(buffer, "Target: %.1f, %.1f | Actual: %.1f, %.1f RPM\r\n",
                target_left_rpm, target_right_rpm, left_rpm, right_rpm);
        HAL_UART_Transmit(&huart6, (uint8_t*)buffer, strlen(buffer), HAL_MAX_DELAY);
        
        HAL_Delay(100);  // Update every 100ms
    }
    
    Motor_Stop();
}

/*
 * === STEP 5: Integration with UART control ===
 * Add this to your UART receive callback function
 */
void example_uart_motor_control(uint8_t received_char)
{
    switch (received_char) {
        case 'w':  // Forward
        case 'W':
            Motor_Forward(50);
            break;
            
        case 's':  // Backward
        case 'S':
            Motor_Backward(50);
            break;
            
        case 'a':  // Turn left
        case 'A':
            Motor_TurnLeft(50);
            break;
            
        case 'd':  // Turn right
        case 'D':
            Motor_TurnRight(50);
            break;
            
        case ' ':  // Stop (spacebar)
            Motor_Stop();
            break;
            
        case 'b':  // Brake
        case 'B':
            Motor_Brake();
            break;
            
        case 'r':  // Reset encoders
        case 'R':
            Encoder_Reset();
            HAL_UART_Transmit(&huart6, (uint8_t*)"Encoders reset\r\n", 16, HAL_MAX_DELAY);
            break;
            
        case 'e':  // Show encoder data
        case 'E':
            example_encoder_monitoring();
            break;
            
        case '1':  // Low speed
            Motor_Forward(25);
            break;
            
        case '2':  // Medium speed
            Motor_Forward(50);
            break;
            
        case '3':  // High speed
            Motor_Forward(75);
            break;
            
        default:
            // Unknown command, stop for safety
            Motor_Stop();
            break;
    }
}

/*
 * === STEP 6: Integration with IMU for self-balancing (advanced) ===
 * This shows how you might integrate motor control with IMU feedback
 */
void example_imu_motor_integration(void)
{
    // This is a simplified example of how you might use IMU data
    // for motor control in a self-balancing application
    
    // Get current IMU angles (assuming these variables exist in your main.c)
    extern imu_t imu_data;
    
    // Simple proportional control based on pitch angle
    float target_angle = 0.0f;  // Target balance angle
    float current_angle = imu_data.pit;  // Current pitch angle
    float error = target_angle - current_angle;
    
    // PID constants (these would need to be tuned for your specific robot)
    float kp = 10.0f;  // Proportional gain
    
    // Calculate motor speed based on error
    int16_t motor_speed = (int16_t)(kp * error);
    
    // Clamp to valid range
    if (motor_speed > 100) motor_speed = 100;
    if (motor_speed < -100) motor_speed = -100;
    
    // Apply speed to both motors (for forward/backward balance)
    Motor_SetSpeeds(motor_speed, motor_speed);
    
    // Update encoder readings for monitoring
    Encoder_Update();
}

/*
 * === STEP 7: Main loop integration examples ===
 */
void example_main_loop_integration(void)
{
    /* In your main while(1) loop, you might add something like: */
    
    // Example: Periodic encoder monitoring
    static uint32_t last_encoder_update = 0;
    if (HAL_GetTick() - last_encoder_update > 500) {  // Every 500ms
        Encoder_Update();
        // Optional: send encoder data or use for control
        last_encoder_update = HAL_GetTick();
    }
    
    // Example: Simple obstacle avoidance using motor control
    /*
    if (obstacle_detected_front()) {
        Motor_Backward(40);
        HAL_Delay(500);
        Motor_TurnRight(60);
        HAL_Delay(800);
    } else {
        Motor_Forward(50);
    }
    */
    
    // Example: Encoder-based distance control
    /*
    static int distance_mode = 0;
    if (distance_mode) {
        Encoder_Update();
        int32_t total_distance = (Encoder_GetLeftCount() + Encoder_GetRightCount()) / 2;
        if (total_distance > 2000) {  // Stop after 2000 counts
            Motor_Stop();
            distance_mode = 0;
        }
    }
    */
}

/*
 * === STEP 8: Add to your main.c file ===
 * 
 * 1. Add #include "motor_driver.h" to USER CODE BEGIN Includes
 * 
 * 2. Add Motor_Init() to USER CODE BEGIN 2 section (after MX_TIM4_Init())
 * 
 * 3. Add Encoder_Update() calls to your main loop for speed calculation
 * 
 * 4. Example integration in main.c:
 */
void example_complete_integration(void)
{
    /* 
     * In main.c USER CODE BEGIN 2 section:
     * Motor_Init();
     * 
     * In main.c while(1) loop:
     * 
     * // Update encoders periodically
     * static uint32_t last_update = 0;
     * if (HAL_GetTick() - last_update > 100) {
     *     Encoder_Update();
     *     last_update = HAL_GetTick();
     * }
     * 
     * // Your motor control logic here
     * // Motor_Forward(50);
     * // example_encoder_monitoring();
     * 
     */
}

/*
 * === Hardware Configuration Summary ===
 * 
 * Updated Pin Connections:
 * 
 * Motors:
 * - Left Motor:  AIN1 (PB14), AIN2 (PB15), PWM (PB8 - TIM4_CH3)
 * - Right Motor: BIN1 (PB13), BIN2 (PB12), PWM (PB9 - TIM4_CH4)
 * 
 * Encoders:
 * - Left Encoder:  Encoder_A1 (PA0 - TIM2_CH1), Encoder_A2 (PA1 - TIM2_CH2)
 * - Right Encoder: Encoder_B1 (PC6 - TIM3_CH1), Encoder_B2 (PB5 - TIM3_CH2)
 * 
 * AT8236 Connections:
 * STM32          AT8236        Function
 * PB14    -->    AIN1         Left motor direction 1
 * PB15    -->    AIN2         Left motor direction 2
 * PB8     -->    PWMA         Left motor speed (TIM4_CH3)
 * PB13    -->    BIN1         Right motor direction 1
 * PB12    -->    BIN2         Right motor direction 2
 * PB9     -->    PWMB         Right motor speed (TIM4_CH4)
 * 
 * PA0     -->    E1A          Left encoder A phase (via AT8236)
 * PA1     -->    E1B          Left encoder B phase (via AT8236)
 * PC6     -->    E2A          Right encoder A phase (via AT8236)
 * PB5     -->    E2B          Right encoder B phase (via AT8236)
 */

/*
 * === Notes for implementation ===
 * 
 * 1. Hardware requirements:
 *    - AT8236 motor driver IC
 *    - Two DC motors with encoders
 *    - Power supply for motors (typically 6-12V)
 *    - Proper wiring according to updated pin definitions
 * 
 * 2. Software features (NEW):
 *    - Dual encoder support for precise position/speed control
 *    - Real-time speed calculation in RPM
 *    - Closed-loop speed control with feedback
 *    - Distance-based movement control
 * 
 * 3. Performance improvements:
 *    - Higher PWM resolution (65535 vs 8399)
 *    - Separate timers for encoders and PWM
 *    - Real-time encoder feedback
 * 
 * 4. Tuning parameters:
 *    - ENCODER_CPR: Adjust for your encoder's counts per revolution
 *    - SPEED_CALC_PERIOD_MS: Adjust speed calculation frequency
 *    - Proportional gain in feedback control functions
 */

/*
 * === 蓝牙测试功能集成示例 ===
 * 将以下代码添加到main.c中以实现蓝牙测试功能
 */

/*
 * 步骤1: 在main.c的USER CODE BEGIN Includes部分添加：
 */
// #include "motor_driver.h"
// extern void Motor_BluetoothCommand(uint8_t cmd);

/*
 * 步骤2: 添加UART接收变量和回调函数
 */
void example_bluetooth_integration(void)
{
    /*
     * 在main.c的USER CODE BEGIN PV部分添加：
     * uint8_t bluetooth_rx_data;
     */
    
    /*
     * 在main.c的USER CODE BEGIN 2部分添加：
     * 
     * // 初始化电机驱动
     * Motor_Init();
     * 
     * // 启动蓝牙接收
     * HAL_UART_Receive_IT(&huart6, &bluetooth_rx_data, 1);
     * 
     * // 发送启动消息
     * const char* welcome = "蓝牙电机控制就绪，发送'h'查看帮助\r\n";
     * HAL_UART_Transmit(&huart6, (uint8_t*)welcome, strlen(welcome), HAL_MAX_DELAY);
     */
    
    /*
     * 在main.c中添加UART接收回调函数：
     * 
     * void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
     * {
     *     if (huart->Instance == USART6) {  // 根据你的UART调整
     *         // 处理蓝牙命令
     *         Motor_BluetoothCommand(bluetooth_rx_data);
     *         
     *         // 重新启动接收
     *         HAL_UART_Receive_IT(&huart6, &bluetooth_rx_data, 1);
     *     }
     * }
     */
    
    /*
     * 在主循环中添加编码器更新：
     * 
     * while (1)
     * {
     *     // 定期更新编码器
     *     static uint32_t last_encoder_update = 0;
     *     if (HAL_GetTick() - last_encoder_update > 100) {
     *         Encoder_Update();
     *         last_encoder_update = HAL_GetTick();
     *     }
     *     
     *     // 其他代码...
     * }
     */
}

/*
 * === 蓝牙命令列表 ===
 * 
 * 发送以下命令测试电机方向：
 * 
 * 't' - 自动方向测试（推荐！）
 *       机器人会自动执行：前进→后退→左转→右转
 *       
 * 'i' - 单电机测试
 *       分别测试左右电机的前进/后退
 *       
 * 'c' - 编码器测试
 *       前进5秒并实时显示编码器计数
 *       
 * 'w'/'s'/'a'/'d' - 手动控制（前/后/左/右）
 * 空格 - 停止
 * 'b' - 制动
 * 'r' - 重置编码器
 * 'e' - 显示编码器数据
 * '1'/'2'/'3' - 慢/中/高速前进
 * 'h' - 显示帮助
 * 
 * === 验证方向是否正确 ===
 * 
 * 1. 发送 't' 进行自动测试
 * 2. 观察机器人运动：
 *    - 前进：机器人向前移动
 *    - 后退：机器人向后移动
 *    - 左转：机器人向左旋转
 *    - 右转：机器人向右旋转
 * 
 * 3. 如果方向不对：
 *    - 查看motor_driver.c中的Motor_SetDirection函数
 *    - 调整MOTOR_LEFT或MOTOR_RIGHT的FORWARD/BACKWARD逻辑
 * 
 * 4. 发送 'c' 测试编码器：
 *    - 前进时编码器计数应该增加
 *    - 如果计数减少，说明编码器A/B相接反了
 */ 