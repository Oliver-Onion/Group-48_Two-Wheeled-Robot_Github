/**
  ******************************************************************************
  * @file           : mpu6500_dmp_usage_example.c
  * @brief          : MPU6500 Real DMP Usage Example
  ******************************************************************************
  * @attention
  *
  * This example shows how to read real DMP quaternion data from MPU6500
  * using SPI interface, similar to MPU6050 DMP implementation.
  *
  ******************************************************************************
  */

#include "mpu6500.h"
#include "usart.h"
#include <stdio.h>

/**
  * @brief  Basic DMP usage example (similar to MPU6050)
  */
void MPU6500_DMP_BasicExample(void)
{
    // Initialize MPU6500 basic functions
    MPU6500_Init();
    
    // Initialize DMP (SPI version)
    DMP_Init_SPI();
    
    printf("MPU6500 DMP Initialized (SPI)\r\n");
    
    // Test connection
    if (MPU6500_TestConnection()) {
        printf("MPU6500 connected successfully (WHO_AM_I = 0x70)\r\n");
    } else {
        printf("MPU6500 connection failed\r\n");
        return;
    }
    
    // Main loop - read DMP data like MPU6050
    while (1) {
        // Read DMP data (similar to Read_DMP() in MPU6050)
        Read_DMP_SPI();
        
        // Check if quaternion data is available
        if (sensors & INV_WXYZ_QUAT) {
            printf("DMP Quaternion: q0=%.3f, q1=%.3f, q2=%.3f, q3=%.3f\r\n", 
                   q0_dmp, q1_dmp, q2_dmp, q3_dmp);
            printf("DMP Euler: Roll=%.1f°, Pitch=%.1f°, Yaw=%.1f°\r\n", 
                   Roll_dmp, Pitch_dmp, Yaw_dmp);
            printf("Gyro: X=%d, Y=%d, Z=%d\r\n", 
                   gyro_dmp[0], gyro_dmp[1], gyro_dmp[2]);
            printf("Accel: X=%d, Y=%d, Z=%d\r\n", 
                   accel_dmp[0], accel_dmp[1], accel_dmp[2]);
            printf("---\r\n");
        } else {
            printf("DMP data not available, using fallback\r\n");
        }
        
        HAL_Delay(50); // 20Hz update rate
    }
}

/**
  * @brief  Compare MPU6500 DMP with regular quaternion calculation
  */
void MPU6500_DMP_CompareExample(void)
{
    // Initialize both regular and DMP functions
    MPU6500_Init();
    MPU6500_InitStructures();
    DMP_Init_SPI();
    
    printf("Comparing DMP vs Regular quaternion calculation\r\n");
    
    while (1) {
        // Update regular sensor data
        MPU6500_UpdateData();
        
        // Update DMP data
        Read_DMP_SPI();
        
        // Get regular IMU data
        imu_t* imu = MPU6500_GetIMUData();
        
        // Compare the results
        printf("Regular: Roll=%.1f°, Pitch=%.1f°, Yaw=%.1f°\r\n", 
               imu->rol, imu->pit, imu->yaw);
        
        if (sensors & INV_WXYZ_QUAT) {
            printf("DMP:     Roll=%.1f°, Pitch=%.1f°, Yaw=%.1f°\r\n", 
                   Roll_dmp, Pitch_dmp, Yaw_dmp);
            printf("Quaternion - Regular: q0=%.3f, DMP: q0=%.3f\r\n", 
                   imu->q0, q0_dmp);
        } else {
            printf("DMP: No data available\r\n");
        }
        
        printf("---\r\n");
        HAL_Delay(100);
    }
}

/**
  * @brief  High-frequency DMP reading example
  */
void MPU6500_DMP_HighFrequencyExample(void)
{
    MPU6500_Init();
    DMP_Init_SPI();
    
    printf("High-frequency DMP reading (100Hz)\r\n");
    
    while (1) {
        Read_DMP_SPI();
        
        if (sensors & INV_WXYZ_QUAT) {
            // Use DMP quaternion for high-frequency control
            // For example: attitude control, drone stabilization, etc.
            
            // Quick output (only essential data)
            printf("Q: %.3f,%.3f,%.3f,%.3f\r\n", 
                   q0_dmp, q1_dmp, q2_dmp, q3_dmp);
        }
        
        HAL_Delay(10); // 100Hz
    }
}

/**
  * @brief  Integration with existing code
  */
void MPU6500_DMP_IntegrationExample(void)
{
    // This shows how to add DMP functionality to existing MPU6500 code
    
    // Your existing initialization
    MPU6500_Init();
    MPU6500_OffsetCall(); // Your existing offset calibration
    
    // Add DMP initialization
    DMP_Init_SPI();
    
    printf("MPU6500 with DMP integration example\r\n");
    
    while (1) {
        // Your existing sensor reading (if needed)
        MPU6500_UpdateData();
        
        // Add DMP reading for better quaternion data
        Read_DMP_SPI();
        
        // Use DMP data if available, otherwise fall back to regular calculation
        float current_roll, current_pitch, current_yaw;
        float current_q0, current_q1, current_q2, current_q3;
        
        if (sensors & INV_WXYZ_QUAT) {
            // Use DMP data (more accurate)
            current_roll = Roll_dmp;
            current_pitch = Pitch_dmp;
            current_yaw = Yaw_dmp;
            current_q0 = q0_dmp;
            current_q1 = q1_dmp;
            current_q2 = q2_dmp;
            current_q3 = q3_dmp;
            printf("Using DMP data\r\n");
        } else {
            // Fall back to regular calculation
            imu_t* imu = MPU6500_GetIMUData();
            current_roll = imu->rol;
            current_pitch = imu->pit;
            current_yaw = imu->yaw;
            current_q0 = imu->q0;
            current_q1 = imu->q1;
            current_q2 = imu->q2;
            current_q3 = imu->q3;
            printf("Using regular calculation\r\n");
        }
        
        // Use the current data for your application
        printf("Current attitude: Roll=%.1f°, Pitch=%.1f°, Yaw=%.1f°\r\n", 
               current_roll, current_pitch, current_yaw);
        
        HAL_Delay(50);
    }
}

/**
  * @brief  Raw FIFO data inspection
  */
void MPU6500_DMP_RawDataExample(void)
{
    uint8_t fifo_count_reg[2];
    uint16_t fifo_count;
    uint8_t fifo_buffer[28];
    
    MPU6500_Init();
    DMP_Init_SPI();
    
    printf("Raw FIFO data inspection\r\n");
    
    while (1) {
        // Check FIFO count
        MPU6500_ReadReg(MPU6500_RA_FIFO_COUNTH, fifo_count_reg, 2);
        fifo_count = (fifo_count_reg[0] << 8) | fifo_count_reg[1];
        
        printf("FIFO count: %d bytes\r\n", fifo_count);
        
        if (fifo_count >= 28) {
            // Read raw FIFO data
            MPU6500_ReadReg(MPU6500_RA_FIFO_R_W, fifo_buffer, 28);
            
            // Display first 16 bytes (quaternion area)
            printf("Raw quaternion data: ");
            for (int i = 0; i < 16; i++) {
                printf("%02X ", fifo_buffer[i]);
            }
            printf("\r\n");
            
            // Also process with our function
            Read_DMP_SPI();
            if (sensors & INV_WXYZ_QUAT) {
                printf("Processed: q0=%.3f, q1=%.3f, q2=%.3f, q3=%.3f\r\n", 
                       q0_dmp, q1_dmp, q2_dmp, q3_dmp);
            }
        }
        
        HAL_Delay(100);
    }
} 