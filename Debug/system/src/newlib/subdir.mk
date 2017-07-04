################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../system/src/newlib/_cxx.cpp 

C_SRCS += \
../system/src/newlib/_exit.c \
../system/src/newlib/_sbrk.c \
../system/src/newlib/_startup.c \
../system/src/newlib/_syscalls.c \
../system/src/newlib/assert.c 

OBJS += \
./system/src/newlib/_cxx.o \
./system/src/newlib/_exit.o \
./system/src/newlib/_sbrk.o \
./system/src/newlib/_startup.o \
./system/src/newlib/_syscalls.o \
./system/src/newlib/assert.o 

C_DEPS += \
./system/src/newlib/_exit.d \
./system/src/newlib/_sbrk.d \
./system/src/newlib/_startup.d \
./system/src/newlib/_syscalls.d \
./system/src/newlib/assert.d 

CPP_DEPS += \
./system/src/newlib/_cxx.d 


# Each subdirectory must supply rules for building sources it contributes
system/src/newlib/%.o: ../system/src/newlib/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: Cross ARM C++ Compiler'
	arm-none-eabi-g++ -mcpu=cortex-m4 -mthumb -mfloat-abi=soft -Og -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -ffreestanding -fno-move-loop-invariants -Wall -Wextra  -g3 -DDEBUG -DUSE_FULL_ASSERT -DTRACE -DOS_USE_TRACE_SEMIHOSTING_DEBUG -DSTM32F407xx -DUSE_HAL_DRIVER -DHSE_VALUE=84000000 -I"../include" -I"../system/include" -I"../system/include/cmsis" -I"../system/include/stm32f4-hal" -std=gnu++11 -fabi-version=0 -fno-exceptions -fno-rtti -fno-use-cxa-atexit -fno-threadsafe-statics -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

system/src/newlib/%.o: ../system/src/newlib/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross ARM C Compiler'
	arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -mfloat-abi=soft -Og -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -ffreestanding -fno-move-loop-invariants -Wall -Wextra  -g3 -DDEBUG -DUSE_STM32F4_DISCOVERY -DUSE_FULL_ASSERT -DTRACE -DOS_USE_TRACE_SEMIHOSTING_DEBUG -DSTM32F4XX -DUSE_STDPERIPH_DRIVER -DUSE_HAL_DRIVER -DHSE_VALUE=84000000 -I"/home/eric/Documents/Experiments/cpp/eclipse_workspace/elka_stm32f4_makefile/include" -I"/home/eric/Documents/Experiments/cpp/eclipse_workspace/elka_stm32f4_makefile/include/CMSIS" -I"/home/eric/Documents/Experiments/cpp/eclipse_workspace/elka_stm32f4_makefile/include/Device" -I"/home/eric/Documents/Experiments/cpp/eclipse_workspace/elka_stm32f4_makefile/include/drivers" -I"/home/eric/Documents/Experiments/cpp/eclipse_workspace/elka_stm32f4_makefile/include/elka_hal" -I"/home/eric/Documents/Experiments/cpp/eclipse_workspace/elka_stm32f4_makefile/include/FreeRTOS/GCC/ARM_CM4F" -I"/home/eric/Documents/Experiments/cpp/eclipse_workspace/elka_stm32f4_makefile/include/FreeRTOS" -I"/home/eric/Documents/Experiments/cpp/eclipse_workspace/elka_stm32f4_makefile/include/modules" -I"/home/eric/Documents/Experiments/cpp/eclipse_workspace/elka_stm32f4_makefile/include/nvicconf" -I"/home/eric/Documents/Experiments/cpp/eclipse_workspace/elka_stm32f4_makefile/include/STM32F4xx_StdPeriph_Driver" -I"/home/eric/Documents/Experiments/cpp/eclipse_workspace/elka_stm32f4_makefile/include/utils" -I"../system/include/stm32f4-hal" -I"../system/include" -I"../system/include/cmsis" -std=gnu11 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

system/src/newlib/_startup.o: ../system/src/newlib/_startup.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross ARM C Compiler'
	arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -mfloat-abi=soft -Og -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -ffreestanding -fno-move-loop-invariants -Wall -Wextra  -g3 -DDEBUG -DUSE_FULL_ASSERT -DTRACE -DOS_USE_TRACE_SEMIHOSTING_DEBUG -DSTM32F407xx -DUSE_HAL_DRIVER -DHSE_VALUE=84000000 -DOS_INCLUDE_STARTUP_INIT_MULTIPLE_RAM_SECTIONS -I"/home/eric/Documents/Experiments/cpp/eclipse_workspace/elka_stm32f4_makefile/include" -I"/home/eric/Documents/Experiments/cpp/eclipse_workspace/elka_stm32f4_makefile/include/CMSIS" -I"/home/eric/Documents/Experiments/cpp/eclipse_workspace/elka_stm32f4_makefile/include/Device" -I"/home/eric/Documents/Experiments/cpp/eclipse_workspace/elka_stm32f4_makefile/include/drivers" -I"/home/eric/Documents/Experiments/cpp/eclipse_workspace/elka_stm32f4_makefile/include/elka_hal" -I"/home/eric/Documents/Experiments/cpp/eclipse_workspace/elka_stm32f4_makefile/include/FreeRTOS/GCC/ARM_CM4F" -I"/home/eric/Documents/Experiments/cpp/eclipse_workspace/elka_stm32f4_makefile/include/FreeRTOS" -I"/home/eric/Documents/Experiments/cpp/eclipse_workspace/elka_stm32f4_makefile/include/modules" -I"/home/eric/Documents/Experiments/cpp/eclipse_workspace/elka_stm32f4_makefile/include/nvicconf" -I"/home/eric/Documents/Experiments/cpp/eclipse_workspace/elka_stm32f4_makefile/include/STM32F4xx_StdPeriph_Driver" -I"/home/eric/Documents/Experiments/cpp/eclipse_workspace/elka_stm32f4_makefile/include/utils" -I"../system/include/stm32f4-hal" -I"../system/include" -I"../system/include/cmsis" -std=gnu11 -MMD -MP -MF"$(@:%.o=%.d)" -MT"system/src/newlib/_startup.d" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


