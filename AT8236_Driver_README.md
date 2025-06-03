# AT8236 Motor Driver for Two-Wheeled Robot

## 概述

本驱动程序是专为AT8236双路电机驱动模块设计的，适用于两轮机器人项目。驱动程序基于D157Bdemo项目的控制方式进行改写，实现了PWM差分控制和闭环速度控制。

## 硬件配置

### AT8236模块连接
- **左电机 (Motor A)**:
  - PWMA_IN1: TIM4_CH3 (PB8)
  - PWMA_IN2: TIM4_CH4 (PB9)
  
- **右电机 (Motor B)**:
  - PWMB_IN1: TIM4_CH1 (PB6) 
  - PWMB_IN2: TIM4_CH2 (PB7)

### 编码器连接
- **左编码器**: TIM2 (PA0-Encoder_A1, PA1-Encoder_A2)
- **右编码器**: TIM3 (PC6-Encoder_B1, PB5-Encoder_B2)

## AT8236控制原理

AT8236模块采用PWM差分控制方式：

### 控制逻辑
- **前进**: PWM_IN1 > PWM_IN2
- **后退**: PWM_IN1 < PWM_IN2  
- **停止**: PWM_IN1 = PWM_IN2
- **速度**: 由 |PWM_IN1 - PWM_IN2| 决定

### PWM配置
```c
#define PWM_PERIOD_VALUE    7199    // PWM周期值 (10kHz @ 72MHz)
#define PWM_CENTER_VALUE    3600    // 中心PWM值 (50%占空比)
#define PWM_MAX_DIFF        3600    // 最大PWM差值(全速)
```

## 主要特性

### 基本电机控制
- 前进/后退控制
- 左转/右转控制
- 个别电机速度控制
- 紧急制动

### 编码器反馈
- 实时位置读取
- 速度计算(RPM)
- 编码器计数重置

### 闭环速度控制
- PI控制算法
- 积分饱和限制
- 自动速度调节

## API 接口

### 基本控制函数
```c
void Motor_Init(void);                          // 初始化电机驱动
void Motor_SetSpeed(MotorSelector_t motor, int16_t speed);  // 设置单个电机速度
void Motor_SetSpeeds(int16_t left_speed, int16_t right_speed); // 设置双电机速度
void Motor_Forward(int16_t speed);              // 前进
void Motor_Backward(int16_t speed);             // 后退
void Motor_TurnLeft(int16_t speed);             // 左转
void Motor_TurnRight(int16_t speed);            // 右转
void Motor_Stop(void);                          // 停止
void Motor_Brake(void);                         // 制动
```

### 编码器函数
```c
void Encoder_Init(void);                        // 初始化编码器
void Encoder_Update(void);                      // 更新编码器读数
void Encoder_Reset(void);                       // 重置编码器
int32_t Encoder_GetLeftCount(void);             // 获取左编码器计数
int32_t Encoder_GetRightCount(void);            // 获取右编码器计数
float Encoder_GetLeftSpeed(void);               // 获取左轮速度(RPM)
float Encoder_GetRightSpeed(void);              // 获取右轮速度(RPM)
```

### 闭环控制函数
```c
void Motor_SetSpeedWithFeedback(MotorSelector_t motor, float target_rpm);
void Motor_SetSpeedsWithFeedback(float left_rpm, float right_rpm);
```

## 使用示例

### 基本使用
```c
#include "motor_driver.h"

int main(void) {
    // 系统初始化
    HAL_Init();
    SystemClock_Config();
    
    // 初始化电机驱动
    Motor_Init();
    
    // 前进2秒，速度50%
    Motor_Forward(50);
    HAL_Delay(2000);
    
    // 停止
    Motor_Stop();
    
    while(1) {
        // 主循环
    }
}
```

### 闭环速度控制
```c
void SpeedControlExample(void) {
    Motor_Init();
    
    // 设置目标速度: 左轮200RPM, 右轮200RPM
    while(1) {
        Motor_SetSpeedsWithFeedback(200.0f, 200.0f);
        HAL_Delay(10);  // 10ms控制周期
    }
}
```

### 编码器监控
```c
void EncoderMonitoring(void) {
    EncoderData_t encoder_data;
    
    Motor_Init();
    Motor_Forward(60);
    
    while(1) {
        Encoder_Update();
        encoder_data = Encoder_GetData();
        
        printf("Left: %ld counts, %.2f RPM\n", 
               encoder_data.left_encoder, encoder_data.left_speed_rpm);
        printf("Right: %ld counts, %.2f RPM\n",
               encoder_data.right_encoder, encoder_data.right_speed_rpm);
        
        HAL_Delay(100);
    }
}
```

## 配置参数

### 需要根据实际硬件调整的参数

1. **编码器分辨率**
```c
#define ENCODER_CPR    1024    // 根据实际编码器调整
```

2. **PI控制参数**
```c
// 在motor_driver.c中调整
static PIController_t left_motor_pi = {20.0f, 0.5f, 0.0f, 0.0f, 380000.0f, -380000.0f};
static PIController_t right_motor_pi = {20.0f, 0.5f, 0.0f, 0.0f, 380000.0f, -380000.0f};
```

3. **PWM频率**
```c
#define PWM_PERIOD_VALUE    7199    // 根据系统时钟调整
```

## 注意事项

1. **引脚配置**: 确保在STM32CubeMX中正确配置TIM4的PWM输出引脚
2. **编码器配置**: 确保TIM2和TIM3配置为编码器模式
3. **PI参数调节**: 根据实际电机特性调节PI控制器参数
4. **安全考虑**: 在测试时使用较低的速度值，确认方向正确后再提高速度

## 与MC520的主要区别

| 特性 | MC520 | AT8236 |
|------|--------|---------|
| 控制方式 | 方向引脚 + PWM | 双路PWM差分 |
| 方向控制 | GPIO引脚状态 | PWM占空比差值 |
| 制动方式 | 方向引脚置位 | PWM值相等 |
| 接线复杂度 | 需要方向引脚 | 仅需PWM信号 |

## 故障排除

1. **电机不转**: 检查PWM输出配置和引脚连接
2. **方向错误**: 调整PWM_IN1和PWM_IN2的连接或在代码中交换
3. **编码器无读数**: 检查编码器连接和TIM配置
4. **速度控制不稳定**: 调整PI参数，特别是积分增益

## 文件结构

```
Two-Wheeled Robot/
├── Core/
│   ├── Src/
│   │   └── motor_driver.c          # 主驱动文件
│   └── Inc/
│       └── motor_driver.h          # 头文件
├── motor_at8236_example.c          # 使用示例
└── AT8236_Driver_README.md         # 本说明文档
``` 