# 双轮机器人电机驱动模块 (v2.0 - 双编码器版本)

## 概述

这是一个为Two-Wheeled Robot项目开发的电机驱动模块，基于AT8236 H桥驱动芯片，适用于STM32F401RE微控制器。该模块提供了完整的双电机控制功能，支持前进、后退、转向、制动等操作，**新增双编码器支持**，实现精确的位置和速度控制。

## 版本更新 (v2.0)

✨ **新功能**：
- 🔄 双编码器支持（TIM2 + TIM3）
- 📊 实时速度计算（RPM）
- 🎯 闭环速度控制
- 📏 基于距离的运动控制
- ⚡ 更高PWM分辨率（65535级）

## 文件结构

```
Two-Wheeled Robot/
├── Core/
│   ├── Inc/
│   │   ├── main.h                    # 引脚定义（已更新）
│   │   └── motor_driver.h            # 电机驱动头文件（v2.0）
│   └── Src/
│       ├── main.c                    # 主程序文件
│       └── motor_driver.c            # 电机驱动实现（v2.0）
├── motor_driver_example.c            # 使用示例代码（含编码器）
└── Motor_Driver_README.md            # 本说明文件
```

## 硬件连接（已更新）

### 引脚定义（参考 main.h）

| 功能 | 引脚 | GPIO | 定时器 | 说明 |
|------|------|------|--------|------|
| **电机控制** | | | | |
| 左电机方向1 | AIN1 | PB14 | - | 左电机正转控制 |
| 左电机方向2 | AIN2 | PB15 | - | 左电机反转控制 |
| 右电机方向1 | BIN1 | PB13 | - | 右电机正转控制 |
| 右电机方向2 | BIN2 | PB12 | - | 右电机反转控制 |
| 左电机PWM | PWMA | PB8 | TIM4_CH3 | 左电机速度控制 |
| 右电机PWM | PWMB | PB9 | TIM4_CH4 | 右电机速度控制 |
| **编码器输入** | | | | |
| 左轮编码器A相 | Encoder_A1 | PA0 | TIM2_CH1 | 左轮位置反馈 |
| 左轮编码器B相 | Encoder_A2 | PA1 | TIM2_CH2 | 左轮位置反馈 |
| 右轮编码器A相 | Encoder_B1 | PC6 | TIM3_CH1 | 右轮位置反馈 |
| 右轮编码器B相 | Encoder_B2 | PB5 | TIM3_CH2 | 右轮位置反馈 |

### AT8236连接示意图（已更新）

```
STM32F401RE          AT8236 Driver          Motors & Encoders
                                           
PB14 (AIN1) ------>  AIN1                 
PB15 (AIN2) ------>  AIN2              Left Motor
PB8 (TIM4_CH3) ---->  PWMA                 
                                           
PB13 (BIN1) ------>  BIN1              
PB12 (BIN2) ------>  BIN2              Right Motor
PB9 (TIM4_CH4) ---->  PWMB                 
                                           
PA0 (TIM2_CH1) <----  E1A  <------------ Left Encoder A
PA1 (TIM2_CH2) <----  E1B  <------------ Left Encoder B
PC6 (TIM3_CH1) <----  E2A  <------------ Right Encoder A
PB5 (TIM3_CH2) <----  E2B  <------------ Right Encoder B
                                           
VCC         ------>  VCC (3.3V/5V)     
GND         ------>  GND               
VMOT        ------>  Motor Power (6-12V)
```

## 软件集成

### 步骤1：添加头文件

在 `main.c` 的 `USER CODE BEGIN Includes` 部分添加：

```c
/* USER CODE BEGIN Includes */
#include "motor_driver.h"
/* USER CODE END Includes */
```

### 步骤2：初始化电机驱动

在 `main.c` 的 `USER CODE BEGIN 2` 部分（在 `MX_TIM4_Init()` 之后）添加：

```c
/* USER CODE BEGIN 2 */
// 其他初始化代码...

// 初始化电机驱动（包含编码器初始化）
Motor_Init();

/* USER CODE END 2 */
```

### 步骤3：添加编码器更新

在主循环中添加编码器更新代码：

```c
/* USER CODE BEGIN WHILE */
while (1)
{
    // 定期更新编码器读数
    static uint32_t last_update = 0;
    if (HAL_GetTick() - last_update > 100) {  // 每100ms更新一次
        Encoder_Update();
        last_update = HAL_GetTick();
    }
    
    // 你的电机控制逻辑
    // Motor_Forward(50);
    
    /* USER CODE END WHILE */
    /* USER CODE BEGIN 3 */
}
/* USER CODE END 3 */
```

## API 参考

### 基本控制函数

#### `void Motor_Init(void)`
初始化电机驱动模块，启动PWM输出、编码器读取并设置电机为停止状态。

#### `void Motor_SetSpeed(MotorSelector_t motor, int16_t speed)`
设置单个电机的速度。
- `motor`: `MOTOR_LEFT` 或 `MOTOR_RIGHT`
- `speed`: -100到100，负值表示反向

#### `void Motor_SetSpeeds(int16_t left_speed, int16_t right_speed)`
同时设置两个电机的速度。
- `left_speed`: 左电机速度 (-100到100)
- `right_speed`: 右电机速度 (-100到100)

### 高级控制函数

#### `void Motor_Forward(int16_t speed)` / `Motor_Backward(int16_t speed)`
两个电机同时前进/后退。
- `speed`: 速度 (0到100)

#### `void Motor_TurnLeft(int16_t speed)` / `Motor_TurnRight(int16_t speed)`
原地转向。
- `speed`: 转向速度 (0到100)

#### `void Motor_Stop(void)` / `void Motor_Brake(void)`
停止电机（自由滑行）或主动制动。

### 编码器函数（新增）

#### `void Encoder_Init(void)`
初始化编码器功能（通常由Motor_Init调用）。

#### `void Encoder_Update(void)`
更新编码器读数并计算速度，应定期调用。

#### `void Encoder_Reset(void)`
重置编码器计数值。

#### `int32_t Encoder_GetLeftCount(void)` / `Encoder_GetRightCount(void)`
获取左/右轮编码器计数值。

#### `float Encoder_GetLeftSpeed(void)` / `Encoder_GetRightSpeed(void)`
获取左/右轮速度（RPM）。

#### `EncoderData_t Encoder_GetData(void)`
获取完整的编码器数据结构。

### 闭环控制函数（新增）

#### `void Motor_SetSpeedWithFeedback(MotorSelector_t motor, float target_rpm)`
基于编码器反馈的单电机速度控制。
- `motor`: 电机选择
- `target_rpm`: 目标速度（RPM）

#### `void Motor_SetSpeedsWithFeedback(float left_rpm, float right_rpm)`
基于编码器反馈的双电机速度控制。

## 使用示例

### 示例1：基本运动控制

```c
// 前进50%速度，持续2秒
Motor_Forward(50);
HAL_Delay(2000);

// 右转60%速度，持续1秒
Motor_TurnRight(60);
HAL_Delay(1000);

// 停止
Motor_Stop();
```

### 示例2：编码器监控

```c
// 更新编码器数据
Encoder_Update();

// 获取编码器信息
EncoderData_t data = Encoder_GetData();
printf("Left: %ld counts, %.2f RPM\n", data.left_encoder, data.left_speed_rpm);
printf("Right: %ld counts, %.2f RPM\n", data.right_encoder, data.right_speed_rpm);
```

### 示例3：基于距离的运动

```c
// 重置编码器
Encoder_Reset();

// 前进直到左轮转过1000个编码器计数
Motor_Forward(40);
while (Encoder_GetLeftCount() < 1000) {
    Encoder_Update();
    HAL_Delay(10);
}
Motor_Stop();
```

### 示例4：闭环速度控制

```c
// 设置目标速度为100 RPM
Motor_SetSpeedsWithFeedback(100.0f, 100.0f);

// 监控实际速度
Encoder_Update();
float actual_left = Encoder_GetLeftSpeed();
float actual_right = Encoder_GetRightSpeed();
printf("Target: 100 RPM, Actual: %.1f, %.1f RPM\n", actual_left, actual_right);
```

### 示例5：UART遥控（增强版）

```c
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    static uint8_t cmd;
    HAL_UART_Receive_IT(&huart6, &cmd, 1);
    
    switch (cmd) {
        case 'w': Motor_Forward(50); break;
        case 's': Motor_Backward(50); break;
        case 'a': Motor_TurnLeft(50); break;
        case 'd': Motor_TurnRight(50); break;
        case ' ': Motor_Stop(); break;
        case 'r': Encoder_Reset(); break;        // 重置编码器
        case 'e':                                // 显示编码器数据
            Encoder_Update();
            EncoderData_t data = Encoder_GetData();
            printf("L:%ld(%.1fRPM) R:%ld(%.1fRPM)\n", 
                   data.left_encoder, data.left_speed_rpm,
                   data.right_encoder, data.right_speed_rpm);
            break;
    }
}
```

## 技术参数

- **PWM频率**: 约1kHz (TIM4, 可调整)
- **PWM分辨率**: 65536级 (0-65535)
- **编码器分辨率**: 可配置（ENCODER_CPR宏定义）
- **速度更新频率**: 10Hz (SPEED_CALC_PERIOD_MS = 100ms)
- **速度范围**: -100% 到 +100%
- **响应时间**: < 1ms

## 配置参数

在 `motor_driver.h` 中可以调整以下参数：

```c
#define ENCODER_CPR            1000    // 编码器每转计数（根据实际编码器调整）
#define SPEED_CALC_PERIOD_MS   100     // 速度计算周期（毫秒）
#define PWM_MAX_VALUE          65535   // PWM最大值
```

## 注意事项

### 安全须知

1. **首次测试**：建议使用低速度（20-30%）进行测试
2. **编码器校准**：根据实际编码器调整ENCODER_CPR值
3. **定期更新**：确保在主循环中定期调用Encoder_Update()
4. **闭环调试**：闭环控制的比例增益需要根据实际系统调整

### 性能优化

1. **编码器分辨率**：选择合适的编码器可提高控制精度
2. **速度计算频率**：可根据需要调整SPEED_CALC_PERIOD_MS
3. **PID控制**：可将简单的比例控制升级为完整的PID控制
4. **滤波算法**：可对编码器数据添加低通滤波减少噪声

### 故障排除

| 问题 | 可能原因 | 解决方法 |
|------|----------|----------|
| 电机不转 | PWM未启动/定时器配置错误 | 检查TIM4配置和Motor_Init()调用 |
| 编码器无读数 | 编码器连接错误/定时器未启动 | 检查TIM2/TIM3配置和编码器接线 |
| 速度计算错误 | ENCODER_CPR设置错误 | 根据实际编码器规格调整CPR值 |
| 闭环控制不稳定 | 增益设置不当 | 调整比例增益kp或实现PID控制 |

## 扩展功能建议

基于新的编码器功能，可以开发：

1. **PID速度控制器** - 替换简单的比例控制
2. **里程计功能** - 基于编码器计算机器人位置
3. **路径跟踪** - 结合IMU实现精确路径控制
4. **自适应控制** - 根据负载自动调整控制参数
5. **故障检测** - 监控编码器异常和电机堵转

## 版本历史

- **v2.0**: 新增双编码器支持、闭环控制、实时速度计算
- **v1.0**: 基本电机控制功能，支持STM32F401RE + AT8236

## 联系支持

如有问题或建议，请参考：
1. STM32F401RE数据手册
2. AT8236驱动芯片规格书  
3. STM32 HAL库文档
4. 本项目的示例代码和README文档 