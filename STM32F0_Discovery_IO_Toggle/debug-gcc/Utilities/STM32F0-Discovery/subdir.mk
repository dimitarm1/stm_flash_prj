################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Utilities/STM32F0-Discovery/stm32f0_discovery.c 

OBJS += \
./Utilities/STM32F0-Discovery/stm32f0_discovery.o 

C_DEPS += \
./Utilities/STM32F0-Discovery/stm32f0_discovery.d 


# Each subdirectory must supply rules for building sources it contributes
Utilities/STM32F0-Discovery/%.o: ../Utilities/STM32F0-Discovery/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross GCC Compiler'
	gcc -I"/home/tate/stm32f4/stm_flash_prj/STM32F0_Discovery_IO_Toggle/Libraries/STM32F0xx_StdPeriph_Driver/inc" -I"/home/tate/stm32f4/stm_flash_prj/STM32F0_Discovery_IO_Toggle/src" -I"/home/tate/stm32f4/stm_flash_prj/STM32F0_Discovery_IO_Toggle/Libraries/CMSIS/Include" -I"/home/tate/stm32f4/stm_flash_prj/STM32F0_Discovery_IO_Toggle/Libraries/STMTouch_Driver/inc" -I"/home/tate/stm32f4/stm_flash_prj/STM32F0_Discovery_IO_Toggle/Libraries/CMSIS/Device/ST/STM32F0xx/Include" -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


