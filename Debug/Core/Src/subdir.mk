################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/fonts.c \
../Core/Src/main.c \
../Core/Src/motor_driver.c \
../Core/Src/mpu6500.c \
../Core/Src/pid_balance.c \
../Core/Src/pid_speed.c \
../Core/Src/pid_turn.c \
../Core/Src/ssd1306.c \
../Core/Src/stm32f4xx_hal_msp.c \
../Core/Src/stm32f4xx_it.c \
../Core/Src/syscalls.c \
../Core/Src/sysmem.c \
../Core/Src/system_stm32f4xx.c \
../Core/Src/ultrasonic.c 

OBJS += \
./Core/Src/fonts.o \
./Core/Src/main.o \
./Core/Src/motor_driver.o \
./Core/Src/mpu6500.o \
./Core/Src/pid_balance.o \
./Core/Src/pid_speed.o \
./Core/Src/pid_turn.o \
./Core/Src/ssd1306.o \
./Core/Src/stm32f4xx_hal_msp.o \
./Core/Src/stm32f4xx_it.o \
./Core/Src/syscalls.o \
./Core/Src/sysmem.o \
./Core/Src/system_stm32f4xx.o \
./Core/Src/ultrasonic.o 

C_DEPS += \
./Core/Src/fonts.d \
./Core/Src/main.d \
./Core/Src/motor_driver.d \
./Core/Src/mpu6500.d \
./Core/Src/pid_balance.d \
./Core/Src/pid_speed.d \
./Core/Src/pid_turn.d \
./Core/Src/ssd1306.d \
./Core/Src/stm32f4xx_hal_msp.d \
./Core/Src/stm32f4xx_it.d \
./Core/Src/syscalls.d \
./Core/Src/sysmem.d \
./Core/Src/system_stm32f4xx.d \
./Core/Src/ultrasonic.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/%.o Core/Src/%.su Core/Src/%.cyclo: ../Core/Src/%.c Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F401xE -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src

clean-Core-2f-Src:
	-$(RM) ./Core/Src/fonts.cyclo ./Core/Src/fonts.d ./Core/Src/fonts.o ./Core/Src/fonts.su ./Core/Src/main.cyclo ./Core/Src/main.d ./Core/Src/main.o ./Core/Src/main.su ./Core/Src/motor_driver.cyclo ./Core/Src/motor_driver.d ./Core/Src/motor_driver.o ./Core/Src/motor_driver.su ./Core/Src/mpu6500.cyclo ./Core/Src/mpu6500.d ./Core/Src/mpu6500.o ./Core/Src/mpu6500.su ./Core/Src/pid_balance.cyclo ./Core/Src/pid_balance.d ./Core/Src/pid_balance.o ./Core/Src/pid_balance.su ./Core/Src/pid_speed.cyclo ./Core/Src/pid_speed.d ./Core/Src/pid_speed.o ./Core/Src/pid_speed.su ./Core/Src/pid_turn.cyclo ./Core/Src/pid_turn.d ./Core/Src/pid_turn.o ./Core/Src/pid_turn.su ./Core/Src/ssd1306.cyclo ./Core/Src/ssd1306.d ./Core/Src/ssd1306.o ./Core/Src/ssd1306.su ./Core/Src/stm32f4xx_hal_msp.cyclo ./Core/Src/stm32f4xx_hal_msp.d ./Core/Src/stm32f4xx_hal_msp.o ./Core/Src/stm32f4xx_hal_msp.su ./Core/Src/stm32f4xx_it.cyclo ./Core/Src/stm32f4xx_it.d ./Core/Src/stm32f4xx_it.o ./Core/Src/stm32f4xx_it.su ./Core/Src/syscalls.cyclo ./Core/Src/syscalls.d ./Core/Src/syscalls.o ./Core/Src/syscalls.su ./Core/Src/sysmem.cyclo ./Core/Src/sysmem.d ./Core/Src/sysmem.o ./Core/Src/sysmem.su ./Core/Src/system_stm32f4xx.cyclo ./Core/Src/system_stm32f4xx.d ./Core/Src/system_stm32f4xx.o ./Core/Src/system_stm32f4xx.su ./Core/Src/ultrasonic.cyclo ./Core/Src/ultrasonic.d ./Core/Src/ultrasonic.o ./Core/Src/ultrasonic.su

.PHONY: clean-Core-2f-Src

