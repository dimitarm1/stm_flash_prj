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
	/opt/gcc-arm-none-eabi-4_6-2012q1/bin/arm-none-eabi-gcc -DSTM32F0XX -I/opt/gcc-arm-none-eabi-4_6-2012q1/arm-none-eabi/include -I"/home/tate/stm32f4/stm_flash_prj/STM32F0_Discovery_IO_Toggle/Libraries/STM32F0xx_StdPeriph_Driver/inc" -I"/home/tate/stm32f4/stm_flash_prj/STM32F0_Discovery_IO_Toggle/Libraries/STMTouch_Driver/inc" -I"/home/tate/stm32f4/stm_flash_prj/STM32F0_Discovery_IO_Toggle/Libraries/CMSIS/Include" -I"/home/tate/stm32f4/stm_flash_prj/STM32F0_Discovery_IO_Toggle/Libraries/CMSIS/Device/ST/STM32F0xx/Include" -I"/home/tate/stm32f4/stm_flash_prj/STM32F0_Discovery_IO_Toggle/src" -O2 -g -Wall -c -fmessage-length=0  -mcpu=cortex-m0 -mthumb -mno-thumb-interwork -mfpu=vfp -msoft-float -mlittle-endian  -march=armv6-m -ffunction-sections -fdata-sections -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


