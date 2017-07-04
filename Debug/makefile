################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include src/utils/subdir.mk
-include src/modules/subdir.mk
-include src/elka_hal/subdir.mk
-include src/drivers/subdir.mk
-include src/STM32F4xx_StdPeriph_Driver/subdir.mk
-include src/FreeRTOS/MemMang/subdir.mk
-include src/FreeRTOS/GCC/ARM_CM4F/subdir.mk
-include src/FreeRTOS/subdir.mk
-include src/subdir.mk
-include subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(C++_DEPS)),)
-include $(C++_DEPS)
endif
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
ifneq ($(strip $(ASM_DEPS)),)
-include $(ASM_DEPS)
endif
ifneq ($(strip $(CC_DEPS)),)
-include $(CC_DEPS)
endif
ifneq ($(strip $(CPP_DEPS)),)
-include $(CPP_DEPS)
endif
ifneq ($(strip $(CXX_DEPS)),)
-include $(CXX_DEPS)
endif
ifneq ($(strip $(C_UPPER_DEPS)),)
-include $(C_UPPER_DEPS)
endif
ifneq ($(strip $(S_UPPER_DEPS)),)
-include $(S_UPPER_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 
SECONDARY_FLASH += \
elka_stm32f4_makefile.hex \

SECONDARY_SIZE += \
elka_stm32f4_makefile.siz \


# All Target
all: elka_stm32f4_makefile.elf secondary-outputs

# Tool invocations
elka_stm32f4_makefile.elf: $(OBJS) $(USER_OBJS)
	@echo 'Building target: $@'
	@echo 'Invoking: Cross ARM C++ Linker'
	arm-none-eabi-g++ -mcpu=cortex-m4 -mthumb -mfloat-abi=softfp -mfpu=fpv4-sp-d16 -Og -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -ffreestanding -fno-move-loop-invariants -Wall -Wextra  -g3 -T "/home/eric/Documents/ElkaControlProject/Elka/elka_stm32f4_makefile/ldscripts/mem.ld" -T "/home/eric/Documents/ElkaControlProject/Elka/elka_stm32f4_makefile/ldscripts/libs.ld" -T "/home/eric/Documents/ElkaControlProject/Elka/elka_stm32f4_makefile/ldscripts/sections.ld" -nostartfiles -Xlinker --gc-sections -Wl,-Map,"elka_stm32f4_makefile.map" --specs=nano.specs -o "elka_stm32f4_makefile.elf" $(OBJS) $(USER_OBJS) $(LIBS)
	@echo 'Finished building target: $@'
	@echo ' '

elka_stm32f4_makefile.hex: elka_stm32f4_makefile.elf
	@echo 'Invoking: Cross ARM GNU Create Flash Image'
	arm-none-eabi-objcopy -O ihex "elka_stm32f4_makefile.elf"  "elka_stm32f4_makefile.hex"
	@echo 'Finished building: $@'
	@echo ' '

elka_stm32f4_makefile.siz: elka_stm32f4_makefile.elf
	@echo 'Invoking: Cross ARM GNU Print Size'
	arm-none-eabi-size --format=berkeley "elka_stm32f4_makefile.elf"
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) $(SECONDARY_SIZE)$(C++_DEPS)$(OBJS)$(C_DEPS)$(ASM_DEPS)$(CC_DEPS)$(SECONDARY_FLASH)$(CPP_DEPS)$(CXX_DEPS)$(C_UPPER_DEPS)$(S_UPPER_DEPS) elka_stm32f4_makefile.elf
	-@echo ' '

secondary-outputs: $(SECONDARY_FLASH) $(SECONDARY_SIZE)

.PHONY: all clean dependents
.SECONDARY:

-include ../makefile.targets
