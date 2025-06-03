/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : mpu6500.c
  * @brief          : MPU6500 library implementation
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
#include "mpu6500.h"

/* Private variables ---------------------------------------------------------*/
mpu_data_t mpu_data;		//定义MPU数据句柄
imu_t imu_data = {0};		//IMU数据储存
uint8_t mpu_buffer[14];

// DMP related variables (same as MPU6050)
short gyro_dmp[3], accel_dmp[3], sensors;
float Roll_dmp, Pitch_dmp, Yaw_dmp;
float q0_dmp = 1.0f, q1_dmp = 0.0f, q2_dmp = 0.0f, q3_dmp = 0.0f;

/* External variables --------------------------------------------------------*/
extern SPI_HandleTypeDef hspi1;

/* Private function prototypes -----------------------------------------------*/
/* Private user code ---------------------------------------------------------*/

/**
  * @brief  Test MPU6500 connection by reading WHO_AM_I register
  * @param  None
  * @retval 1 if connected (WHO_AM_I = 0x70), 0 if not connected
  */
uint8_t MPU6500_TestConnection(void)
{
    uint8_t who_am_i;
    MPU6500_ReadReg(MPU6500_RA_WHO_AM_I, &who_am_i, 1);
    return (who_am_i == 0x70) ? 1 : 0;  // MPU6500 WHO_AM_I should be 0x70
}

/**
  * @brief  Initialize DMP using SPI (similar to MPU6050 DMP_Init but for SPI)
  * @param  None
  * @retval None
  * @note   This function initializes the DMP without loading firmware.
  *         For full DMP functionality, you would need to load the DMP firmware.
  */
void DMP_Init_SPI(void)
{
    uint8_t temp;
    
    // Test connection first
    if (!MPU6500_TestConnection()) {
        //return; // MPU6500 not found
    }
    
    // Reset device
    MPU6500_WriteReg(MPU6500_RA_PWR_MGMT_1, 0x80);
    HAL_Delay(100);
    
    // Wake up device and set clock source to PLL with Y Gyro reference
    MPU6500_WriteReg(MPU6500_RA_PWR_MGMT_1, 0x02);
    HAL_Delay(10);
    
    // Set gyroscope full scale range to ±2000°/s
    MPU6500_WriteReg(MPU6500_RA_GYRO_CONFIG, 0x18);
    
    // Set accelerometer full scale range to ±2g
    MPU6500_WriteReg(MPU6500_RA_ACCEL_CONFIG, 0x00);
    
    // Disable all interrupts initially
    MPU6500_WriteReg(MPU6500_RA_INT_ENABLE, 0x00);
    HAL_Delay(5);
    
    // Reset FIFO and DMP
    MPU6500_ReadReg(MPU6500_RA_USER_CTRL, &temp, 1);
    temp |= (BIT_FIFO_RST | BIT_DMP_RST);
    MPU6500_WriteReg(MPU6500_RA_USER_CTRL, temp);
    HAL_Delay(10);
    
    // Clear any existing interrupts
    MPU6500_ReadReg(MPU6500_RA_INT_STATUS, &temp, 1);
    
    // Configure USER_CTRL for DMP without enabling interrupts
    // Enable DMP but disable FIFO for now to prevent overflow
    MPU6500_WriteReg(MPU6500_RA_USER_CTRL, BIT_DMP_EN);
    HAL_Delay(10);
    
    // Clear interrupt status again after DMP enable
    MPU6500_ReadReg(MPU6500_RA_INT_STATUS, &temp, 1);
    
    // Note: For complete DMP functionality, you would need to:
    // 1. Load DMP firmware through SPI memory interface
    // 2. Configure DMP features
    // 3. Set up FIFO
    // This is a basic initialization that prepares the sensor
    
    // Initialize quaternion
    q0_dmp = 1.0f;
    q1_dmp = 0.0f;
    q2_dmp = 0.0f;
    q3_dmp = 0.0f;
}

/**
  * @brief  Read DMP FIFO data using SPI (similar to MPU6050 Read_DMP but for SPI)
  * @param  None
  * @retval None
  * @note   This function reads FIFO data and extracts quaternion if available.
  *         For full functionality, DMP firmware must be loaded first.
  */
void Read_DMP_SPI(void)
{
    uint8_t fifo_count_reg[2];
    uint16_t fifo_count;
    uint8_t int_status;
    uint8_t fifo_buffer[28]; // DMP packet size is typically 28 bytes
    long quat[4];
    
    // Read interrupt status
    MPU6500_ReadReg(MPU6500_RA_INT_STATUS, &int_status, 1);
    
    // Read FIFO count
    MPU6500_ReadReg(MPU6500_RA_FIFO_COUNTH, fifo_count_reg, 2);
    fifo_count = (fifo_count_reg[0] << 8) | fifo_count_reg[1];
    
    // Check for FIFO overflow
    if (int_status & 0x10) {  // FIFO_OFLOW_INT bit
        // FIFO overflow - need to reset FIFO
        uint8_t user_ctrl;
        MPU6500_ReadReg(MPU6500_RA_USER_CTRL, &user_ctrl, 1);
        user_ctrl |= BIT_FIFO_RST;  // Set FIFO reset bit
        MPU6500_WriteReg(MPU6500_RA_USER_CTRL, user_ctrl);
        HAL_Delay(1);
        
        // Clear the overflow bit by reading INT_STATUS again
        MPU6500_ReadReg(MPU6500_RA_INT_STATUS, &int_status, 1);
        
        // Reset DMP variables to fallback mode
        sensors = 0;
        return;
    }
    
    // Check if DMP interrupt is set
    if (int_status != 0x00) {  // DMP_INT bit
        MPU6500_WriteReg(MPU6500_RA_INT_ENABLE, 0x00);
        // Clear DMP interrupt by reading INT_STATUS (already done above)
        // Continue processing DMP data
    }
    
    // Check if we have a complete DMP packet (28 bytes)
    if (fifo_count >= 28) {
        // Read FIFO data
        MPU6500_ReadReg(MPU6500_RA_FIFO_R_W, fifo_buffer, 28);
        
        // Extract quaternion data from FIFO buffer
        // DMP quaternion data typically starts at offset 0 in the packet
        // Format: 32-bit signed integers in big-endian format
        quat[0] = ((long)fifo_buffer[0] << 24) | ((long)fifo_buffer[1] << 16) | 
                  ((long)fifo_buffer[2] << 8) | fifo_buffer[3];
        quat[1] = ((long)fifo_buffer[4] << 24) | ((long)fifo_buffer[5] << 16) | 
                  ((long)fifo_buffer[6] << 8) | fifo_buffer[7];
        quat[2] = ((long)fifo_buffer[8] << 24) | ((long)fifo_buffer[9] << 16) | 
                  ((long)fifo_buffer[10] << 8) | fifo_buffer[11];
        quat[3] = ((long)fifo_buffer[12] << 24) | ((long)fifo_buffer[13] << 16) | 
                  ((long)fifo_buffer[14] << 8) | fifo_buffer[15];
        
        // Convert to float quaternion (same as MPU6050)
        q0_dmp = quat[0] / q30;
        q1_dmp = quat[1] / q30;
        q2_dmp = quat[2] / q30;
        q3_dmp = quat[3] / q30;
        
        // Calculate Euler angles from quaternion (same as MPU6050)
        Roll_dmp = asin(-2 * q1_dmp * q3_dmp + 2 * q0_dmp * q2_dmp) * 57.3f;
        Pitch_dmp = atan2(2 * q2_dmp * q3_dmp + 2 * q0_dmp * q1_dmp, 
                         -2 * q1_dmp * q1_dmp - 2 * q2_dmp * q2_dmp + 1) * 57.3f;
        Yaw_dmp = atan2(2 * (q1_dmp * q2_dmp + q0_dmp * q3_dmp), 
                       q0_dmp * q0_dmp + q1_dmp * q1_dmp - q2_dmp * q2_dmp - q3_dmp * q3_dmp) * 57.3f;
        
        // Extract gyro and accel data from FIFO if available
        // This depends on the DMP configuration and packet format
        // For now, use the regular sensor readings
        gyro_dmp[0] = mpu_data.gx;
        gyro_dmp[1] = mpu_data.gy;
        gyro_dmp[2] = mpu_data.gz;
        
        accel_dmp[0] = mpu_data.ax;
        accel_dmp[1] = mpu_data.ay;
        accel_dmp[2] = mpu_data.az;
        
        sensors = INV_WXYZ_QUAT; // Indicate that quaternion data is available
    }
    else {
        // If no DMP data available, fall back to regular sensor reading
        MPU6500_ReadReg(59, mpu_buffer, sizeof(mpu_buffer));
        MPU6500_ReadData();
        
        // Use the existing quaternion calculation as fallback
        q0_dmp = imu_data.q0;
        q1_dmp = imu_data.q1;
        q2_dmp = imu_data.q2;
        q3_dmp = imu_data.q3;
        
        // Use the calculated Euler angles and ensure proper range
        Roll_dmp = imu_data.rol * 124.13f;
        Pitch_dmp = imu_data.pit * 124.13f;
        Yaw_dmp = imu_data.yaw * 124.13f;
        
        // Ensure angles are in proper range (-180 to 180 degrees)
        while (Roll_dmp > 180.0f) Roll_dmp -= 360.0f;
        while (Roll_dmp < -180.0f) Roll_dmp += 360.0f;
        while (Pitch_dmp > 180.0f) Pitch_dmp -= 360.0f;
        while (Pitch_dmp < -180.0f) Pitch_dmp += 360.0f;
        while (Yaw_dmp > 180.0f) Yaw_dmp -= 360.0f;
        while (Yaw_dmp < -180.0f) Yaw_dmp += 360.0f;
        
        sensors = 0;
    }
    
    // Additional step: If DMP interrupt is still set, force clear it
    if (int_status & 0x02) {
        // Try reading INT_STATUS again to clear persistent interrupt
        uint8_t temp_status;
        MPU6500_ReadReg(MPU6500_RA_INT_STATUS, &temp_status, 1);
    }
}

/**
  * @brief  Initialize MPU6500 sensor
  * @param  None
  * @retval None
  */
void MPU6500_Init(void)
{
	HAL_Delay(100);
	
	// 避免睡眠模式并使用 Gyro 时钟源 / Disable sleep mode and use Gyro clock source
	// 地址：0x6B（PWR_MGMT_1）
	// 二进制值：00000010（CLKSEL=010，选择 X 轴 Gyro 作为时钟源，禁用睡眠模式）
	MPU6500_WriteReg(0x6B, 0x02);
	
	// 设置陀螺仪量程为 ±2000°/s / Set gyroscope range to ±2000°/s
	// 地址：0x1B（GYRO_CONFIG）
	// 二进制值：00011000（FS_SEL=11，表示 ±2000°/s）
	MPU6500_WriteReg(0x1B, 0x18);
	
	// 设置加速度计量程为 ±2g / Set accelerometer range to ±2g
	// 地址：0x1C（ACCEL_CONFIG）
	// 二进制值：00000000（AFS_SEL=00，表示 ±2g）
	MPU6500_WriteReg(0x1C, 0x00);
	
	// 激活 DMP / Activate DMP
	// 地址：0x6A（USER_CTRL）
	// 二进制值：10000000（启用 DMP，禁用其他功能）
	MPU6500_WriteReg(0x6A, 0x80);
	
	/*
	mpu_set_gyro_fsr(3);
	mpu_set_accel_fsr(2);
	*/
	imu_data.q0 = 1.0f; // Initialize quaternions
	imu_data.q1 = 0.0f;
	imu_data.q2 = 0.0f;
	imu_data.q3 = 0.0f;

	MPU6500_OffsetCall();
}

/**
  * @brief  Calculate offset values for MPU6500
  * @param  None
  * @retval None
  */
void MPU6500_OffsetCall(void)
{
	mpu_data.ax_offset = 0;
	mpu_data.ay_offset = 0;
	mpu_data.az_offset = 0;
	mpu_data.gx_offset = 0;
	mpu_data.gy_offset = 0;
	mpu_data.gz_offset = 0;
	int i;
	for (i=0; i<300;i++)
	{
		MPU6500_ReadReg(59, mpu_buffer, sizeof(mpu_buffer));

		mpu_data.ax_offset += mpu_buffer[0] << 8 | mpu_buffer[1];
		mpu_data.ay_offset += mpu_buffer[2] << 8 | mpu_buffer[3];
		mpu_data.az_offset += mpu_buffer[4] << 8 | mpu_buffer[5];

		mpu_data.gx_offset += mpu_buffer[8]  << 8 | mpu_buffer[9];
		mpu_data.gy_offset += mpu_buffer[10] << 8 | mpu_buffer[11];
		mpu_data.gz_offset += mpu_buffer[12] << 8 | mpu_buffer[13];

		HAL_Delay(5);
	}
	mpu_data.ax_offset=mpu_data.ax_offset / 300;
	mpu_data.ay_offset=mpu_data.ay_offset / 300;
	mpu_data.az_offset=mpu_data.az_offset / 300;
	mpu_data.gx_offset=mpu_data.gx_offset / 300;
	mpu_data.gy_offset=mpu_data.gy_offset / 300;
	mpu_data.gz_offset=mpu_data.gz_offset / 300;
}

/**
  * @brief  Write register to MPU6500
  * @param  reg: Register address
  * @param  data: Data to write
  * @retval None
  */
void MPU6500_WriteReg(uint8_t reg, uint8_t data)
{
	HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi1, &reg, 1, 100);
	HAL_SPI_Transmit(&hspi1, &data, 1, 100);
	HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, GPIO_PIN_SET);
}

/**
  * @brief  Read register from MPU6500
  * @param  reg: Register address
  * @param  data: Pointer to data buffer
  * @param  len: Length of data to read
  * @retval None
  */
void MPU6500_ReadReg(uint8_t reg, uint8_t *data, uint8_t len)
{
	uint8_t temp_data = 0x80|reg;
	HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi1, &temp_data , 1, 100);
	HAL_SPI_Receive(&hspi1, data, len, 100);
	HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, GPIO_PIN_SET);
}

/**
  * @brief  Read and process MPU6500 data
  * @param  None
  * @retval None
  */
void MPU6500_ReadData(void)
{
	 // Split data
	mpu_data.ax   = (mpu_buffer[0] << 8 | mpu_buffer[1]);
	mpu_data.ay   = (mpu_buffer[2] << 8 | mpu_buffer[3]);
	mpu_data.az   = (mpu_buffer[4] << 8 | mpu_buffer[5]);
	mpu_data.temp = (mpu_buffer[6] << 8 | mpu_buffer[7]);

	mpu_data.gx = ((mpu_buffer[8]  << 8 | mpu_buffer[9])  - mpu_data.gx_offset);
	mpu_data.gy = ((mpu_buffer[10] << 8 | mpu_buffer[11]) - mpu_data.gy_offset);
	mpu_data.gz = ((mpu_buffer[12] << 8 | mpu_buffer[13]) - mpu_data.gz_offset);

	// Conversion to physical quantities
	imu_data.ax = mpu_data.ax / 16384.0f; // ±2g
	imu_data.ay = mpu_data.ay / 16384.0f;
	imu_data.az = mpu_data.az / 16384.0f;
	imu_data.temp = (mpu_data.temp / 333.87f) + 21.0f;
	imu_data.gx = mpu_data.gx / 16.4f; // ±2000dps (changed from 131.0f for ±250dps)
	imu_data.gy = mpu_data.gy / 16.4f; // ±2000dps (changed from 131.0f for ±250dps)
	imu_data.gz = mpu_data.gz / 16.4f; // ±2000dps (changed from 131.0f for ±250dps)

	// Quaternion Updates
	const float dt = 0.001f;    // Sampling Interval
	const float deg2rad = 0.0174532925f; // degree to radian coefficient

	// Converts angular velocity in rad/s
	float gx_rad = imu_data.gx * deg2rad;
	float gy_rad = imu_data.gy * deg2rad;
	float gz_rad = imu_data.gz * deg2rad;

	// Get the current quaternion
	float q0 = imu_data.q0;
	float q1 = imu_data.q1;
	float q2 = imu_data.q2;
	float q3 = imu_data.q3;

	float dq0 = 0.5f * (-q1*gx_rad - q2*gy_rad - q3*gz_rad) * dt;
	float dq1 = 0.5f * ( q0*gx_rad + q2*gz_rad - q3*gy_rad) * dt;
	float dq2 = 0.5f * ( q0*gy_rad - q1*gz_rad + q3*gx_rad) * dt;
	float dq3 = 0.5f * ( q0*gz_rad + q1*gy_rad - q2*gx_rad) * dt;

	q0 += dq0;
	q1 += dq1;
	q2 += dq2;
	q3 += dq3;

	// Nnormalize quaternions
	float norm = sqrtf(q0*q0 + q1*q1 + q2*q2 + q3*q3);
	norm = (norm == 0.0f) ? 1.0f : norm; // Prevent division by zero
	q0 /= norm;
	q1 /= norm;
	q2 /= norm;
	q3 /= norm;

	imu_data.q0 = q0;
	imu_data.q1 = q1;
	imu_data.q2 = q2;
	imu_data.q3 = q3;

	// Conversion to Euler angles (ZYX order)
	// Roll (X-axis)
	imu_data.rol = atan2f(2.0f*(q0*q1 + q2*q3), 1.0f - 2.0f*(q1*q1 + q2*q2));
	// Pitch (Y-axis)
	imu_data.pit = asinf(2.0f*(q0*q2 - q3*q1));
	// Yaw (Z-axis)
	imu_data.yaw = atan2f(2.0f*(q0*q3 + q1*q2), 1.0f - 2.0f*(q2*q2 + q3*q3));

	// Radian to degree
	const float rad2deg = 57.2957795f;
	imu_data.rol *= rad2deg;
	imu_data.pit *= rad2deg;
	imu_data.yaw *= rad2deg;
}

/**
  * @brief  Initialize MPU and IMU data structures
  * @param  None
  * @retval None
  */
void MPU6500_InitStructures(void)
{
    // 初始化 mpu_data_t 结构体
    mpu_data.ax = 0;
    mpu_data.ay = 0;
    mpu_data.az = 0;

    mpu_data.mx = 0;
    mpu_data.my = 0;
    mpu_data.mz = 0;

    mpu_data.temp = 0;

    mpu_data.gx = 0;
    mpu_data.gy = 0;
    mpu_data.gz = 0;

    // 初始化 imu_t 结构体
    imu_data.ax = 0.0f;
    imu_data.ay = 0.0f;
    imu_data.az = 0.0f;

    imu_data.temp = 0.0f;

    imu_data.gx = 0.0f;
    imu_data.gy = 0.0f;
    imu_data.gz = 0.0f;

    imu_data.vx = 0.0f;
    imu_data.vy = 0.0f;
    imu_data.vz = 0.0f;

    imu_data.q0 = 1.0f; // 初始化四元数为单位四元数
    imu_data.q1 = 0.0f;
    imu_data.q2 = 0.0f;
    imu_data.q3 = 0.0f;

    imu_data.rol = 0.0f;
    imu_data.pit = 0.0f;
    imu_data.yaw = 0.0f;
}

/**
  * @brief  Update MPU6500 data (read and process)
  * @param  None
  * @retval None
  */
void MPU6500_UpdateData(void)
{
    MPU6500_ReadReg(59, mpu_buffer, sizeof(mpu_buffer));
    MPU6500_ReadData();
}

/**
  * @brief  Get IMU data pointer
  * @param  None
  * @retval Pointer to IMU data structure
  */
imu_t* MPU6500_GetIMUData(void)
{
    return &imu_data;
}

/**
  * @brief  Get MPU data pointer
  * @param  None
  * @retval Pointer to MPU data structure
  */
mpu_data_t* MPU6500_GetMPUData(void)
{
    return &mpu_data;
}

/**
  * @brief  Get MPU buffer pointer
  * @param  None
  * @retval Pointer to MPU data buffer
  */
uint8_t* MPU6500_GetBuffer(void)
{
    return mpu_buffer;
} 
