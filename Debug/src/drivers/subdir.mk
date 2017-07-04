################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../src/drivers/adc.c \
../src/drivers/exti.c \
../src/drivers/i2cdev.c \
../src/drivers/i2croutines.c \
../src/drivers/motors.c \
../src/drivers/mpu6050.c \
../src/drivers/nrf24l01.c \
../src/drivers/nvic.c 

OBJS += \
./src/drivers/adc.o \
./src/drivers/exti.o \
./src/drivers/i2cdev.o \
./src/drivers/i2croutines.o \
./src/drivers/motors.o \
./src/drivers/mpu6050.o \
./src/drivers/nrf24l01.o \
./src/drivers/nvic.o 

C_DEPS += \
./src/drivers/adc.d \
./src/drivers/exti.d \
./src/drivers/i2cdev.d \
./src/drivers/i2croutines.d \
./src/drivers/motors.d \
./src/drivers/mpu6050.d \
./src/drivers/nrf24l01.d \
./src/drivers/nvic.d 


# Each subdirectory must supply rules for building sources it contributes
src/drivers/%.o: ../src/drivers/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross ARM C Compiler'
	arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -mfloat-abi=softfp -mfpu=fpv4-sp-d16 -Og -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -ffreestanding -fno-move-loop-invariants -Wall -Wextra  -g3 -DDEBUG -DUSE_FULL_ASSERT -DTRACE -DOS_USE_TRACE_SEMIHOSTING_DEBUG -DSTM32F4XX -DUSE_HAL_DRIVER -DHSE_VALUE=84000000 -DUSE_STM32F4_DISCOVERY -DSTM32F4XX -DUSE_STDPERIPH_DRIVER -I/home/eric/Documents/Experiments/cpp/eclipse_workspace/elka_stm32f4_makefile/include -I"/home/eric/Documents/Experiments/cpp/eclipse_workspace/elka_stm32f4_makefile/include" -I"/home/eric/Documents/Experiments/cpp/eclipse_workspace/elka_stm32f4_makefile/include/CMSIS" -I"/home/eric/Documents/Experiments/cpp/eclipse_workspace/elka_stm32f4_makefile/include/Device" -I"/home/eric/Documents/Experiments/cpp/eclipse_workspace/elka_stm32f4_makefile/include/Device/STM32F4xx" -I"/home/eric/Documents/Experiments/cpp/eclipse_workspace/elka_stm32f4_makefile/include/drivers" -I"/home/eric/Documents/Experiments/cpp/eclipse_workspace/elka_stm32f4_makefile/include/elka_hal" -I"/home/eric/Documents/Experiments/cpp/eclipse_workspace/elka_stm32f4_makefile/include/FreeRTOS" -I"/home/eric/Documents/Experiments/cpp/eclipse_workspace/elka_stm32f4_makefile/include/FreeRTOS/GCC/ARM_CM4F" -I"/home/eric/Documents/Experiments/cpp/eclipse_workspace/elka_stm32f4_makefile/include/modules" -I"/home/eric/Documents/Experiments/cpp/eclipse_workspace/elka_stm32f4_makefile/include/nvicconf" -I"/home/eric/Documents/Experiments/cpp/eclipse_workspace/elka_stm32f4_makefile/include/STM32F4xx_StdPeriph_Driver" -I"/home/eric/Documents/Experiments/cpp/eclipse_workspace/elka_stm32f4_makefile/include/utils" -I/home/eric/Documents/Experiments/cpp/eclipse_workspace/elka_stm32f4_makefile/system/include -I/home/eric/Documents/Experiments/cpp/eclipse_workspace/elka_stm32f4_makefile/system/include/cmsis -I/home/eric/Documents/Experiments/cpp/eclipse_workspace/elka_stm32f4_makefile/system/include/stm32f4-hal -std=gnu11 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


