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
	@echo 'Invoking: GCC C Compiler'
	E:\Program Files\gcc-arm-none-eabi\bin\arm-none-eabi-gcc.exe -DSTM32F0XX -DUSE_STM32F0_DISCOVERY -DHSI_VALUE=8000000 -DUSE_STDPERIPH_DRIVER -I../src -I"..\Libraries\CMSIS\Include" -I"..\Libraries\CMSIS\Device\ST\STM32F0xx\Include" -I"..\Libraries\STM32F0xx_StdPeriph_Driver\inc" -I"..\Utilities\STM32F0-Discovery" -O0 -ffunction-sections -fdata-sections -g -Wall -c -fmessage-length=0 -mthumb -mcpu=cortex-m0 -std=gnu90 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


