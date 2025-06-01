#ifndef __MOTOR_DIRECTION_TEST_H
#define __MOTOR_DIRECTION_TEST_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "motor_driver.h"

/* Exported functions prototypes ---------------------------------------------*/
void Motor_BluetoothDirectionTest(void);
void Motor_BluetoothIndividualTest(void);
void Encoder_BluetoothTest(void);
void Encoder_BluetoothMonitor(void);
void Motor_BluetoothCommand(uint8_t cmd);

#ifdef __cplusplus
}
#endif

#endif /* __MOTOR_DIRECTION_TEST_H */ 