################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../src/main.c \
../src/stm32f0xx_it.c \
../src/syscalls.c \
../src/system_stm32f0xx.c \
../src/tiny_printf.c \
../src/tsl_user.c 

OBJS += \
./src/main.o \
./src/stm32f0xx_it.o \
./src/syscalls.o \
./src/system_stm32f0xx.o \
./src/tiny_printf.o \
./src/tsl_user.o 

C_DEPS += \
./src/main.d \
./src/stm32f0xx_it.d \
./src/syscalls.d \
./src/system_stm32f0xx.d \
./src/tiny_printf.d \
./src/tsl_user.d 


# Each subdirectory must supply rules for building sources it contributes
src/%.o: ../src/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross GCC Compiler'
	gcc -I"/home/tate/stm32f4/stm_flash_prj/STM32F0_Discovery_IO_Toggle/Libraries/STM32F0xx_StdPeriph_Driver/inc" -I"/home/tate/stm32f4/stm_flash_prj/STM32F0_Discovery_IO_Toggle/src" -I"/home/tate/stm32f4/stm_flash_prj/STM32F0_Discovery_IO_Toggle/Libraries/CMSIS/Include" -I"/home/tate/stm32f4/stm_flash_prj/STM32F0_Discovery_IO_Toggle/Libraries/STMTouch_Driver/inc" -I"/home/tate/stm32f4/stm_flash_prj/STM32F0_Discovery_IO_Toggle/Libraries/CMSIS/Device/ST/STM32F0xx/Include" -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


