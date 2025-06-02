# MPU6500 Library for STM32

本库是为STM32CubeIDE开发的MPU6500陀螺仪库，将MPU6500的功能从main.c中分离出来，形成独立的库文件。

## 文件结构

- `Core/Inc/mpu6500.h` - MPU6500库头文件
- `Core/Src/mpu6500.c` - MPU6500库实现文件

## 数据结构

### mpu_data_t
MPU6500原始数据结构体，包含：
- 加速度原始数据 (ax, ay, az)
- 陀螺仪原始数据 (gx, gy, gz)
- 温度原始数据 (temp)
- 偏移量数据 (ax_offset, ay_offset, az_offset, gx_offset, gy_offset, gz_offset)

### imu_t
IMU处理后的数据结构体，包含：
- 加速度物理量 (ax, ay, az) - g为单位
- 陀螺仪物理量 (gx, gy, gz) - dps为单位
- 温度物理量 (temp) - 摄氏度
- 四元数 (q0, q1, q2, q3)
- 欧拉角 (rol, pit, yaw) - 角度为单位

## 主要函数

### 初始化函数
- `MPU6500_Init(void)` - 初始化MPU6500传感器
- `MPU6500_InitStructures(void)` - 初始化数据结构

### 数据读取函数
- `MPU6500_UpdateData(void)` - 更新MPU6500数据（推荐使用）
- `MPU6500_ReadReg(uint8_t reg, uint8_t *data, uint8_t len)` - 读取寄存器
- `MPU6500_WriteReg(uint8_t reg, uint8_t data)` - 写入寄存器
- `MPU6500_ReadData(void)` - 处理原始数据

### 数据获取函数
- `MPU6500_GetIMUData(void)` - 获取IMU数据指针
- `MPU6500_GetMPUData(void)` - 获取MPU原始数据指针
- `MPU6500_GetBuffer(void)` - 获取数据缓冲区指针

### 校准函数
- `MPU6500_OffsetCall(void)` - 计算偏移量

## 使用示例

```c
#include "mpu6500.h"

int main(void)
{
    // 系统初始化
    HAL_Init();
    SystemClock_Config();
    
    // 外设初始化
    MX_SPI1_Init();  // MPU6500使用SPI1
    
    // MPU6500初始化
    MPU6500_Init();
    MPU6500_InitStructures();
    
    while(1)
    {
        // 更新MPU6500数据
        MPU6500_UpdateData();
        
        // 获取IMU数据
        imu_t* imu = MPU6500_GetIMUData();
        
        // 使用数据
        printf("Roll: %.2f, Pitch: %.2f, Yaw: %.2f\n", 
               imu->rol, imu->pit, imu->yaw);
        
        HAL_Delay(10);
    }
}
```

## 定时器使用示例

```c
volatile uint8_t timer_flag = 0;

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim == &htim1)
    {
        timer_flag = 1;
    }
}

int main(void)
{
    // 初始化代码...
    
    HAL_TIM_Base_Start_IT(&htim1);  // 启动1ms定时器
    
    while(1)
    {
        if(timer_flag)
        {
            MPU6500_UpdateData();
            timer_flag = 0;
        }
        
        // 其他任务...
    }
}
```

## 注意事项

1. 确保SPI1正确配置和初始化
2. 确保SPI_CS引脚正确配置
3. 库依赖于HAL库函数
4. 建议使用1ms定时器定期更新数据
5. 初始化时会自动计算偏移量，需要保持传感器静止

## 硬件连接

- SPI1_SCK -> MPU6500 SCL
- SPI1_MISO -> MPU6500 SDA
- SPI1_MOSI -> MPU6500 SDA
- SPI_CS -> MPU6500 CS (需要在main.h中定义SPI_CS_GPIO_Port和SPI_CS_Pin)

## 移植说明

从原始main.c移植时的主要变化：

1. 包含 `#include "mpu6500.h"`
2. 删除MPU相关的数据结构定义
3. 删除MPU相关的函数实现
4. 使用 `MPU6500_UpdateData()` 替换原来的 `mpu6500_read_reg()` + `read_data()`
5. 使用 `MPU6500_GetIMUData()` 获取数据指针替换直接访问 `imu_data` 