################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include src/subdir.mk
-include Utilities/STM32F0-Discovery/subdir.mk
-include Libraries/STMTouch_Driver/src/subdir.mk
-include Libraries/STM32F0xx_StdPeriph_Driver/src/subdir.mk
-include subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 

# All Target
all: STM32F0_Discovery_IO_Toggle.elf

# Tool invocations
STM32F0_Discovery_IO_Toggle.elf: $(OBJS) $(USER_OBJS)
	@echo 'Building target: $@'
	@echo 'Invoking: GCC C Linker'
	E:\Program Files\gcc-arm-none-eabi\bin\arm-none-eabi-gcc.exe -lm -o STM32F0_Discovery_IO_Toggle.elf -mthumb -mcpu=cortex-m0 -Wl,--start-group -lc -lm -Wl,--end-group -static -Wl,-cref,-u,Reset_Handler -Wl,-Map=STM32F0_Discovery_IO_Toggle.map -Wl,--gc-sections -Wl,--defsym=malloc_getpagesize_P=0x1000 -Xlinker -T..\stm32_flash.ld -o "STM32F0_Discovery_IO_Toggle.elf" $(OBJS) $(USER_OBJS) $(LIBS)
	@echo 'Finished building target: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) $(OBJS)$(C_DEPS)$(EXECUTABLES) STM32F0_Discovery_IO_Toggle.elf
	-@echo ' '

.PHONY: all clean dependents
.SECONDARY:

-include ../makefile.targets
