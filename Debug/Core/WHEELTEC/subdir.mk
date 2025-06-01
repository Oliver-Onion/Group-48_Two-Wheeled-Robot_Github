################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/WHEELTEC/DataScope_DP.c \
../Core/WHEELTEC/KF.c \
../Core/WHEELTEC/MPU6050.c \
../Core/WHEELTEC/ReadDistance.c \
../Core/WHEELTEC/control.c \
../Core/WHEELTEC/filter.c \
../Core/WHEELTEC/inv_mpu.c \
../Core/WHEELTEC/inv_mpu_dmp_motion_driver.c \
../Core/WHEELTEC/show.c 

OBJS += \
./Core/WHEELTEC/DataScope_DP.o \
./Core/WHEELTEC/KF.o \
./Core/WHEELTEC/MPU6050.o \
./Core/WHEELTEC/ReadDistance.o \
./Core/WHEELTEC/control.o \
./Core/WHEELTEC/filter.o \
./Core/WHEELTEC/inv_mpu.o \
./Core/WHEELTEC/inv_mpu_dmp_motion_driver.o \
./Core/WHEELTEC/show.o 

C_DEPS += \
./Core/WHEELTEC/DataScope_DP.d \
./Core/WHEELTEC/KF.d \
./Core/WHEELTEC/MPU6050.d \
./Core/WHEELTEC/ReadDistance.d \
./Core/WHEELTEC/control.d \
./Core/WHEELTEC/filter.d \
./Core/WHEELTEC/inv_mpu.d \
./Core/WHEELTEC/inv_mpu_dmp_motion_driver.d \
./Core/WHEELTEC/show.d 


# Each subdirectory must supply rules for building sources it contributes
Core/WHEELTEC/%.o Core/WHEELTEC/%.su Core/WHEELTEC/%.cyclo: ../Core/WHEELTEC/%.c Core/WHEELTEC/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F401xE -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-WHEELTEC

clean-Core-2f-WHEELTEC:
	-$(RM) ./Core/WHEELTEC/DataScope_DP.cyclo ./Core/WHEELTEC/DataScope_DP.d ./Core/WHEELTEC/DataScope_DP.o ./Core/WHEELTEC/DataScope_DP.su ./Core/WHEELTEC/KF.cyclo ./Core/WHEELTEC/KF.d ./Core/WHEELTEC/KF.o ./Core/WHEELTEC/KF.su ./Core/WHEELTEC/MPU6050.cyclo ./Core/WHEELTEC/MPU6050.d ./Core/WHEELTEC/MPU6050.o ./Core/WHEELTEC/MPU6050.su ./Core/WHEELTEC/ReadDistance.cyclo ./Core/WHEELTEC/ReadDistance.d ./Core/WHEELTEC/ReadDistance.o ./Core/WHEELTEC/ReadDistance.su ./Core/WHEELTEC/control.cyclo ./Core/WHEELTEC/control.d ./Core/WHEELTEC/control.o ./Core/WHEELTEC/control.su ./Core/WHEELTEC/filter.cyclo ./Core/WHEELTEC/filter.d ./Core/WHEELTEC/filter.o ./Core/WHEELTEC/filter.su ./Core/WHEELTEC/inv_mpu.cyclo ./Core/WHEELTEC/inv_mpu.d ./Core/WHEELTEC/inv_mpu.o ./Core/WHEELTEC/inv_mpu.su ./Core/WHEELTEC/inv_mpu_dmp_motion_driver.cyclo ./Core/WHEELTEC/inv_mpu_dmp_motion_driver.d ./Core/WHEELTEC/inv_mpu_dmp_motion_driver.o ./Core/WHEELTEC/inv_mpu_dmp_motion_driver.su ./Core/WHEELTEC/show.cyclo ./Core/WHEELTEC/show.d ./Core/WHEELTEC/show.o ./Core/WHEELTEC/show.su

.PHONY: clean-Core-2f-WHEELTEC

