################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../src/elka_hal/imu.c 

OBJS += \
./src/elka_hal/imu.o 

C_DEPS += \
./src/elka_hal/imu.d 


# Each subdirectory must supply rules for building sources it contributes
src/elka_hal/%.o: ../src/elka_hal/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross ARM C Compiler'
	arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -mfloat-abi=softfp -mfpu=fpv4-sp-d16 -Og -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -ffreestanding -fno-move-loop-invariants -Wall -Wextra  -g3 -DDEBUG -DUSE_FULL_ASSERT -DTRACE -DOS_USE_TRACE_SEMIHOSTING_DEBUG -DSTM32F4XX -DUSE_HAL_DRIVER -DHSE_VALUE=84000000 -DUSE_STM32F4_DISCOVERY -DSTM32F4XX -DUSE_STDPERIPH_DRIVER -I/home/eric/Documents/Experiments/cpp/eclipse_workspace/elka_stm32f4_makefile/include -I"/home/eric/Documents/Experiments/cpp/eclipse_workspace/elka_stm32f4_makefile/include" -I"/home/eric/Documents/Experiments/cpp/eclipse_workspace/elka_stm32f4_makefile/include/CMSIS" -I"/home/eric/Documents/Experiments/cpp/eclipse_workspace/elka_stm32f4_makefile/include/Device" -I"/home/eric/Documents/Experiments/cpp/eclipse_workspace/elka_stm32f4_makefile/include/Device/STM32F4xx" -I"/home/eric/Documents/Experiments/cpp/eclipse_workspace/elka_stm32f4_makefile/include/drivers" -I"/home/eric/Documents/Experiments/cpp/eclipse_workspace/elka_stm32f4_makefile/include/elka_hal" -I"/home/eric/Documents/Experiments/cpp/eclipse_workspace/elka_stm32f4_makefile/include/FreeRTOS" -I"/home/eric/Documents/Experiments/cpp/eclipse_workspace/elka_stm32f4_makefile/include/FreeRTOS/GCC/ARM_CM4F" -I"/home/eric/Documents/Experiments/cpp/eclipse_workspace/elka_stm32f4_makefile/include/modules" -I"/home/eric/Documents/Experiments/cpp/eclipse_workspace/elka_stm32f4_makefile/include/nvicconf" -I"/home/eric/Documents/Experiments/cpp/eclipse_workspace/elka_stm32f4_makefile/include/STM32F4xx_StdPeriph_Driver" -I"/home/eric/Documents/Experiments/cpp/eclipse_workspace/elka_stm32f4_makefile/include/utils" -I/home/eric/Documents/Experiments/cpp/eclipse_workspace/elka_stm32f4_makefile/system/include -I/home/eric/Documents/Experiments/cpp/eclipse_workspace/elka_stm32f4_makefile/system/include/cmsis -I/home/eric/Documents/Experiments/cpp/eclipse_workspace/elka_stm32f4_makefile/system/include/stm32f4-hal -std=gnu11 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


