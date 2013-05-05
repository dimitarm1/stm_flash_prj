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
	@echo 'Invoking: GCC C Compiler'
	E:\Program Files\gcc-arm-none-eabi\bin\arm-none-eabi-gcc.exe -DSTM32F0XX -DUSE_STM32F0_DISCOVERY -DHSI_VALUE=8000000 -DUSE_STDPERIPH_DRIVER -I../src -I"..\Libraries\CMSIS\Include" -I"..\Libraries\CMSIS\Device\ST\STM32F0xx\Include" -I"..\Libraries\STM32F0xx_StdPeriph_Driver\inc" -I"..\Utilities\STM32F0-Discovery" -O0 -ffunction-sections -fdata-sections -g -Wall -c -fmessage-length=0 -mthumb -mcpu=cortex-m0 -std=gnu90 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


