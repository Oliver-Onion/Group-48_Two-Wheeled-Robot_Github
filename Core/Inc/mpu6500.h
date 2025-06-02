/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : mpu6500.h
  * @brief          : MPU6500 library header file
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

#ifndef __MPU6500_H__
#define __MPU6500_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "math.h"

/* Private defines -----------------------------------------------------------*/
// DMP related constants (same as MPU6050)
#define q30                         1073741824.0f
#define DEFAULT_MPU_HZ              200

// MPU6500 DMP registers (same addresses as MPU6050)
#define MPU6500_RA_WHO_AM_I         0x75
#define MPU6500_RA_PWR_MGMT_1       0x6B
#define MPU6500_RA_USER_CTRL        0x6A
#define MPU6500_RA_FIFO_EN          0x23
#define MPU6500_RA_INT_ENABLE       0x38
#define MPU6500_RA_FIFO_COUNTH      0x72
#define MPU6500_RA_FIFO_COUNTL      0x73
#define MPU6500_RA_FIFO_R_W         0x74
#define MPU6500_RA_INT_STATUS       0x3A
#define MPU6500_RA_GYRO_CONFIG      0x1B
#define MPU6500_RA_ACCEL_CONFIG     0x1C

// DMP specific bits
#define BIT_DMP_EN                  0x80
#define BIT_FIFO_EN                 0x40
#define BIT_DMP_RST                 0x08
#define BIT_FIFO_RST                0x04

// DMP packet types
#define INV_WXYZ_QUAT               0x100

/* Private typedef -----------------------------------------------------------*/
//MPU6500初始数据结构体
typedef struct
{
	int16_t ax;
	int16_t ay;
	int16_t az;

	int16_t mx;
	int16_t my;
	int16_t mz;

	int16_t temp;

	int16_t gx;
	int16_t gy;
	int16_t gz;

	int16_t ax_offset;
	int16_t ay_offset;
	int16_t az_offset;

	int16_t gx_offset;
	int16_t gy_offset;
	int16_t gz_offset;
} mpu_data_t;

//惯性测量单元数据结构体
typedef struct
{
	float ax;
	float ay;
	float az;

	float temp;

	float gx;
	float gy;
	float gz;

	float vx;
	float vy;
	float vz;

	float q0; // w
	float q1; // x
	float q2; // y
	float q3; // z

	float rol;
	float pit;
	float yaw;
} imu_t;

/* Exported variables --------------------------------------------------------*/
extern mpu_data_t mpu_data;
extern imu_t imu_data;
extern uint8_t mpu_buffer[14];

// DMP related variables (same as MPU6050)
extern short gyro_dmp[3], accel_dmp[3], sensors;
extern float Roll_dmp, Pitch_dmp, Yaw_dmp;
extern float q0_dmp, q1_dmp, q2_dmp, q3_dmp;

/* Exported functions prototypes ---------------------------------------------*/
void MPU6500_Init(void);
void MPU6500_OffsetCall(void);
void MPU6500_ReadReg(uint8_t reg, uint8_t *data, uint8_t len);
void MPU6500_WriteReg(uint8_t reg, uint8_t data);
void MPU6500_ReadData(void);
void MPU6500_InitStructures(void);
void MPU6500_UpdateData(void);
imu_t* MPU6500_GetIMUData(void);
mpu_data_t* MPU6500_GetMPUData(void);
uint8_t* MPU6500_GetBuffer(void);

// DMP functions (SPI version of MPU6050 DMP functions)
void DMP_Init_SPI(void);
void Read_DMP_SPI(void);
uint8_t MPU6500_TestConnection(void);

#ifdef __cplusplus
}
#endif

#endif /* __MPU6500_H__ */ 