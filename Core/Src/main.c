/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "math.h"
#include "string.h"
#include "stdio.h"
#include "ssd1306.h"
#include "motor_driver.h"
#include "mpu6500.h"
#include "pid_balance.h"  // 添加PD控制器头文件
#include "pid_speed.h"  // 添加PI速度控制器头文件
#include "pid_turn.h"   // 添加PD转向控制器头文件
#include "ultrasonic.h"  // 添加超声波模块头文件
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c2;
I2C_HandleTypeDef hi2c3;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart6;

/* USER CODE BEGIN PV */
volatile uint8_t timer_flag = 0;

uint16_t refresh_interval = 100;  // Screen refresh interval in ms
uint16_t send_interval = 500;     // Bluetooth send interval in ms

volatile uint8_t send_flag = 1;  // 1: 发送数据, 0: 停止发送

uint8_t bluetooth_rx_data;

// 电机控制相关变量
int16_t motor_left_speed;   // 左电机速度百分比 (-100 to 100)
int16_t motor_right_speed;  // 右电机速度百分比 (-100 to 100)

// 平衡控制相关变量
volatile uint8_t balance_start = 0;  // 平衡控制启动标志
float target_angle = 0.0f;          // 目标角度

// 障碍检测相关变量
#define OBSTACLE_THRESHOLD_MM 300    // 障碍检测距离阈值：300mm
volatile uint8_t obstacle_detected = 0;  // 障碍检测标志
volatile uint8_t turning_left = 0;       // 左转标志
float original_target_yaw = 0.0f;        // 原始目标偏航角
float turn_target_yaw = 0.0f;           // 转向目标偏航角（左转30度后的角度）
uint32_t turn_start_time = 0;           // 转向开始时间
uint32_t max_turn_duration = 3000;      // 最大转向持续时间（毫秒）
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM1_Init(void);
static void MX_USART6_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_I2C3_Init(void);
static void MX_I2C2_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
// Send AT command to HC-05
void send_at_command(const char *command) {
    HAL_UART_Transmit(&huart6, (uint8_t*)command, strlen(command), HAL_MAX_DELAY);
}

void send_imu_data(void)
{
    char buffer[100];
    imu_t* imu = MPU6500_GetIMUData();
    sprintf(buffer, "ax: %.2f, ay: %.2f, az: %.2f, roll: %.2f, pitch: %.2f, yaw: %.2f\r\n",
            imu->ax, imu->ay, imu->az,
            imu->rol, imu->pit, imu->yaw);
    HAL_UART_Transmit(&huart6, (uint8_t*)buffer, strlen(buffer), HAL_MAX_DELAY);
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	if (GPIO_Pin == GPIO_PIN_1) { // PB1, the up button
		MPU6500_InitStructures();
	}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART6) {
        if (bluetooth_rx_data == 'b' && !balance_start) {
            // 收到'b'字符且当前未启动平衡控制时，记录当前角度作为目标角度并启动平衡
            target_angle = Pitch_dmp;
            balance_start = 1;
            
            // 发送确认信息
            char buffer[50];
            sprintf(buffer, "Balance start, target: %.2f\r\n", target_angle);
            HAL_UART_Transmit(&huart6, (uint8_t*)buffer, strlen(buffer), HAL_MAX_DELAY);
        } else if (bluetooth_rx_data == 's') {
            // 收到's'字符时停止平衡控制
            balance_start = 0;
            Motor_Stop();
            
            // 发送确认信息
            char *msg = "Balance stopped\r\n";
            HAL_UART_Transmit(&huart6, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
        } else {
            // 检查是否为速度控制命令 (u,j,i,k,v)
            if (bluetooth_rx_data == 'u' || bluetooth_rx_data == 'j' || 
                bluetooth_rx_data == 'i' || bluetooth_rx_data == 'k' ||
                bluetooth_rx_data == 'v') {
                PI_Speed_ProcessCommand(bluetooth_rx_data);
            } else if (bluetooth_rx_data == 'r' || bluetooth_rx_data == 'f' ||
                       bluetooth_rx_data == 't' || bluetooth_rx_data == 'g' ||
                       bluetooth_rx_data == 'n') {
                // 转向控制命令 (r,f,t,g,n)
                PD_Turn_ProcessCommand(bluetooth_rx_data);
            } else {
                // 其他字符作为平衡控制参数调整命令处理
                PD_Balance_ProcessCommand(bluetooth_rx_data);
            }
        }
        
        // 继续接收下一个字符
        HAL_UART_Receive_IT(&huart6, &bluetooth_rx_data, 1);
    }
}

// 超声波输入捕获中断回调函数
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim) {
    // 调用超声波模块的回调函数
    Ultrasonic_TIM_IC_CaptureCallback(htim);
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_SPI1_Init();
  MX_TIM1_Init();
  MX_USART6_UART_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_I2C3_Init();
  MX_I2C2_Init();
  /* USER CODE BEGIN 2 */
  ssd1306_Init(&hi2c3);
  ssd1306_Fill(Black);

  MPU6500_Init();
  MPU6500_InitStructures();
  
  // Initialize DMP for better quaternion calculation
  HAL_Delay(173);
  DMP_Init_SPI();
  
  // 初始化PD控制器
  PD_Balance_Init();
  
  // 初始化PI速度控制器
  PI_Speed_Init();
  
  // 初始化PD转向控制器
  PD_Turn_Init();
  
  // 初始化电机驱动
  Motor_Init();
  
  // 初始化超声波测距模块
  Ultrasonic_Init(&htim4);  // 使用TIM4进行超声波测距
  Ultrasonic_SetSamplingInterval(100);  // 设置采样间隔为100ms
  
  HAL_TIM_Base_Start_IT(&htim1);
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_SET);

  uint8_t dummy;
  HAL_UART_Receive_IT(&huart6, &dummy, 1);

  uint32_t last_send_time = HAL_GetTick();
  uint32_t last_refresh_time = HAL_GetTick();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    uint32_t current_time = HAL_GetTick();
    
    if(timer_flag)
    {
        MPU6500_UpdateData();
        Read_DMP_SPI();

        // 障碍检测逻辑
        uint32_t distance_mm = Ultrasonic_GetDistanceMM();
        
        // 检测到障碍物
        if (distance_mm < OBSTACLE_THRESHOLD_MM && distance_mm > 10) {  // 排除无效读数
            if (!obstacle_detected && !turning_left) {
                // 首次检测到障碍，开始左转
                obstacle_detected = 1;
                turning_left = 1;
                original_target_yaw = PD_Turn_GetTargetAngle();  // 保存原始目标角度
                turn_target_yaw = original_target_yaw + 30.0f;   // 设置左转30度目标
                
                // 处理角度环绕
                if (turn_target_yaw > 180.0f) {
                    turn_target_yaw -= 360.0f;
                }
                
                PD_Turn_SetTargetAngle(turn_target_yaw);         // 设置新的目标角度
                turn_start_time = HAL_GetTick();                 // 记录转向开始时间
            }
        } else {
            obstacle_detected = 0;  // 没有障碍物，清除检测标志
        }
        
        // 转向超时处理
        if (turning_left && (HAL_GetTick() - turn_start_time > max_turn_duration)) {
            turning_left = 0;  // 转向超时，停止转向
            PD_Turn_SetTargetAngle(original_target_yaw);  // 恢复原始目标角度
        }
        
        // 检查转向是否完成（到达目标角度附近）
        if (turning_left) {
            float angle_error = turn_target_yaw - Yaw_dmp;
            // 处理角度跨越±180度的情况
            if (angle_error > 180.0f) {
                angle_error -= 360.0f;
            } else if (angle_error < -180.0f) {
                angle_error += 360.0f;
            }
            
            // 如果角度误差小于5度，认为转向完成
            if (fabs(angle_error) < 5.0f) {
                turning_left = 0;  // 转向完成
                PD_Turn_SetTargetAngle(turn_target_yaw);  // 保持新的角度作为目标
            }
        }

        if (balance_start) {
            // 更新编码器数据
            Encoder_Update();
            
            // 获取编码器数据，右编码器取反以匹配motor_driver的处理
            EncoderData_t encoder_data = Encoder_GetData();
            float left_rpm = encoder_data.left_speed_rpm;
            float right_rpm = -encoder_data.right_speed_rpm;  // 右编码器读数取反
            float current_speed = (left_rpm + right_rpm) / 2.0f;
            
            // 获取目标速度
            float target_speed = PI_Speed_GetTargetSpeed();
            
            // 计算平衡控制速度值 (-100 to 100)
            int16_t balance_speed = PD_Balance_Calculate(Pitch_dmp - target_angle, 0);
            
            // 计算速度控制修正值
            int16_t speed_correction = PI_Speed_Calculate(current_speed, target_speed);
            
            // 计算转向控制输出
            float target_angle = PD_Turn_GetTargetAngle();
            int16_t turn_output = PD_Turn_Calculate(target_angle, Yaw_dmp);
            
            // 合成最终电机速度
            motor_left_speed = balance_speed + speed_correction + turn_output;
            motor_right_speed = balance_speed + speed_correction - turn_output;
            
            // 限制速度范围
            motor_left_speed = CLAMP(motor_left_speed, -100, 100);
            motor_right_speed = CLAMP(motor_right_speed, -100, 100);
            
            // 安全检查
            if (!PD_Balance_IsDangerous(Pitch_dmp - target_angle)) {
                Motor_SetSpeeds(motor_left_speed, motor_right_speed);
            } else {
                Motor_Stop();
                balance_start = 0;  // 危险角度时停止平衡控制
            }
        }

        timer_flag = 0;
    }
    
    // 0.1秒刷新一次屏幕显示
    if (current_time - last_refresh_time >= 100) {
        char display_str1[20];
        char display_str2[20];
        char display_str3[20];
        char display_str4[20];
        float kp, kd;
        
        // 获取当前PD参数
        PD_Balance_GetParams(&kp, &kd);
        
        // 清屏
        ssd1306_Fill(Black);
        
        // 显示角度和参数
        sprintf(display_str1, "Pitch: %.2f", Pitch_dmp);
        sprintf(display_str2, "Speed: %d,%d", motor_left_speed, motor_right_speed);
        //sprintf(display_str3, "Kp: %.0f", kp);
        //sprintf(display_str4, "Kd: %.0f", kd);
        
        ssd1306_SetCursor(0, 0);
        ssd1306_WriteString(display_str1, Font_7x10, White);
        ssd1306_SetCursor(0, 12);
        ssd1306_WriteString(display_str2, Font_7x10, White);
        /*ssd1306_SetCursor(0, 24);
        ssd1306_WriteString(display_str3, Font_7x10, White);
        ssd1306_SetCursor(0, 36);
        ssd1306_WriteString(display_str4, Font_7x10, White);*/
        
        ssd1306_UpdateScreen(&hi2c3);
        
        last_refresh_time = current_time;
    }
    
    // 0.5秒通过蓝牙发送一次数据
    if (current_time - last_send_time >= 500) {
        char buffer[250];
        float balance_kp, balance_kd;
        float speed_kp, speed_ki;
        float turn_kp, turn_kd;
        PD_Balance_GetParams(&balance_kp, &balance_kd);
        PI_Speed_GetParams(&speed_kp, &speed_ki);
        PD_Turn_GetParams(&turn_kp, &turn_kd);
        
        // 获取编码器数据
        EncoderData_t encoder_data = Encoder_GetData();
        float target_speed = PI_Speed_GetTargetSpeed();
        float target_angle = PD_Turn_GetTargetAngle();
        
        // 获取距离和障碍检测状态
        uint32_t distance_mm = Ultrasonic_GetDistanceMM();
        
        sprintf(buffer, "Pitch:%.2f BKp:%.0f BKd:%.0f SKp:%.1f SKi:%.2f TKp:%.0f TKd:%.0f Target:%.1f Angle:%.1f Speed:%d,%d RPM:%.1f,%.1f Dist:%ldmm Obs:%d Turn:%d\r\n", 
                Pitch_dmp, balance_kp, balance_kd, speed_kp, speed_ki, turn_kp, turn_kd, target_speed, target_angle,
                motor_left_speed, motor_right_speed,
                encoder_data.left_speed_rpm, -encoder_data.right_speed_rpm,
                distance_mm, obstacle_detected, turning_left);
        HAL_UART_Transmit(&huart6, (uint8_t*)buffer, strlen(buffer), HAL_MAX_DELAY);
        
        last_send_time = current_time;
    }
    
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 100000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief I2C3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C3_Init(void)
{

  /* USER CODE BEGIN I2C3_Init 0 */

  /* USER CODE END I2C3_Init 0 */

  /* USER CODE BEGIN I2C3_Init 1 */

  /* USER CODE END I2C3_Init 1 */
  hi2c3.Instance = I2C3;
  hi2c3.Init.ClockSpeed = 100000;
  hi2c3.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c3.Init.OwnAddress1 = 0;
  hi2c3.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c3.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c3.Init.OwnAddress2 = 0;
  hi2c3.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c3.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C3_Init 2 */

  /* USER CODE END I2C3_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi1.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_128;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 84-1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 999;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 8399;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 8399;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 8399;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

}

/**
  * @brief USART6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART6_UART_Init(void)
{

  /* USER CODE BEGIN USART6_Init 0 */

  /* USER CODE END USART6_Init 0 */

  /* USER CODE BEGIN USART6_Init 1 */

  /* USER CODE END USART6_Init 1 */
  huart6.Instance = USART6;
  huart6.Init.BaudRate = 9600;
  huart6.Init.WordLength = UART_WORDLENGTH_8B;
  huart6.Init.StopBits = UART_STOPBITS_1;
  huart6.Init.Parity = UART_PARITY_NONE;
  huart6.Init.Mode = UART_MODE_TX_RX;
  huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart6.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart6) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART6_Init 2 */

  /* USER CODE END USART6_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pin : SPI_CS_Pin */
  GPIO_InitStruct.Pin = SPI_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SPI_CS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PB1 */
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PC7 */
  GPIO_InitStruct.Pin = GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PA10 */
  GPIO_InitStruct.Pin = GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if (htim == &htim1)
	{
		timer_flag = 1;
	}
}
int _write(int file, char *ptr, int len)
{
	int DataIdx;

	for (DataIdx = 0; DataIdx < len; DataIdx++)
	{
		ITM_SendChar(*ptr++);
	}
	return len;
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
