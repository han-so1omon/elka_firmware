################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../system/src/stm32f4-hal/stm32f4xx_hal.c \
../system/src/stm32f4-hal/stm32f4xx_hal_cortex.c \
../system/src/stm32f4-hal/stm32f4xx_hal_flash.c \
../system/src/stm32f4-hal/stm32f4xx_hal_gpio.c \
../system/src/stm32f4-hal/stm32f4xx_hal_iwdg.c \
../system/src/stm32f4-hal/stm32f4xx_hal_pwr.c \
../system/src/stm32f4-hal/stm32f4xx_hal_rcc.c 

OBJS += \
./system/src/stm32f4-hal/stm32f4xx_hal.o \
./system/src/stm32f4-hal/stm32f4xx_hal_cortex.o \
./system/src/stm32f4-hal/stm32f4xx_hal_flash.o \
./system/src/stm32f4-hal/stm32f4xx_hal_gpio.o \
./system/src/stm32f4-hal/stm32f4xx_hal_iwdg.o \
./system/src/stm32f4-hal/stm32f4xx_hal_pwr.o \
./system/src/stm32f4-hal/stm32f4xx_hal_rcc.o 

C_DEPS += \
./system/src/stm32f4-hal/stm32f4xx_hal.d \
./system/src/stm32f4-hal/stm32f4xx_hal_cortex.d \
./system/src/stm32f4-hal/stm32f4xx_hal_flash.d \
./system/src/stm32f4-hal/stm32f4xx_hal_gpio.d \
./system/src/stm32f4-hal/stm32f4xx_hal_iwdg.d \
./system/src/stm32f4-hal/stm32f4xx_hal_pwr.d \
./system/src/stm32f4-hal/stm32f4xx_hal_rcc.d 


# Each subdirectory must supply rules for building sources it contributes
system/src/stm32f4-hal/%.o: ../system/src/stm32f4-hal/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross ARM C Compiler'
	arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -mfloat-abi=soft -Og -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -ffreestanding -fno-move-loop-invariants -Wall -Wextra  -g3 -DDEBUG -DUSE_STM32F4_DISCOVERY -DUSE_FULL_ASSERT -DTRACE -DOS_USE_TRACE_SEMIHOSTING_DEBUG -DSTM32F4XX -DUSE_STDPERIPH_DRIVER -DUSE_HAL_DRIVER -DHSE_VALUE=84000000 -I"/home/eric/Documents/Experiments/cpp/eclipse_workspace/elka_stm32f4_makefile/include" -I"/home/eric/Documents/Experiments/cpp/eclipse_workspace/elka_stm32f4_makefile/include/CMSIS" -I"/home/eric/Documents/Experiments/cpp/eclipse_workspace/elka_stm32f4_makefile/include/Device" -I"/home/eric/Documents/Experiments/cpp/eclipse_workspace/elka_stm32f4_makefile/include/drivers" -I"/home/eric/Documents/Experiments/cpp/eclipse_workspace/elka_stm32f4_makefile/include/elka_hal" -I"/home/eric/Documents/Experiments/cpp/eclipse_workspace/elka_stm32f4_makefile/include/FreeRTOS/GCC/ARM_CM4F" -I"/home/eric/Documents/Experiments/cpp/eclipse_workspace/elka_stm32f4_makefile/include/FreeRTOS" -I"/home/eric/Documents/Experiments/cpp/eclipse_workspace/elka_stm32f4_makefile/include/modules" -I"/home/eric/Documents/Experiments/cpp/eclipse_workspace/elka_stm32f4_makefile/include/nvicconf" -I"/home/eric/Documents/Experiments/cpp/eclipse_workspace/elka_stm32f4_makefile/include/STM32F4xx_StdPeriph_Driver" -I"/home/eric/Documents/Experiments/cpp/eclipse_workspace/elka_stm32f4_makefile/include/utils" -I"../system/include/stm32f4-hal" -I"../system/include" -I"../system/include/cmsis" -std=gnu11 -Wno-bad-function-cast -Wno-conversion -Wno-sign-conversion -Wno-unused-parameter -Wno-sign-compare -Wno-missing-prototypes -Wno-missing-declarations -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


