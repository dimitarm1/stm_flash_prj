################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Libraries/STMTouch_Driver/src/tsl.c \
../Libraries/STMTouch_Driver/src/tsl_acq.c \
../Libraries/STMTouch_Driver/src/tsl_acq_stm32f0xx.c \
../Libraries/STMTouch_Driver/src/tsl_dxs.c \
../Libraries/STMTouch_Driver/src/tsl_ecs.c \
../Libraries/STMTouch_Driver/src/tsl_filter.c \
../Libraries/STMTouch_Driver/src/tsl_globals.c \
../Libraries/STMTouch_Driver/src/tsl_linrot.c \
../Libraries/STMTouch_Driver/src/tsl_object.c \
../Libraries/STMTouch_Driver/src/tsl_time.c \
../Libraries/STMTouch_Driver/src/tsl_time_stm32f0xx.c \
../Libraries/STMTouch_Driver/src/tsl_touchkey.c 

OBJS += \
./Libraries/STMTouch_Driver/src/tsl.o \
./Libraries/STMTouch_Driver/src/tsl_acq.o \
./Libraries/STMTouch_Driver/src/tsl_acq_stm32f0xx.o \
./Libraries/STMTouch_Driver/src/tsl_dxs.o \
./Libraries/STMTouch_Driver/src/tsl_ecs.o \
./Libraries/STMTouch_Driver/src/tsl_filter.o \
./Libraries/STMTouch_Driver/src/tsl_globals.o \
./Libraries/STMTouch_Driver/src/tsl_linrot.o \
./Libraries/STMTouch_Driver/src/tsl_object.o \
./Libraries/STMTouch_Driver/src/tsl_time.o \
./Libraries/STMTouch_Driver/src/tsl_time_stm32f0xx.o \
./Libraries/STMTouch_Driver/src/tsl_touchkey.o 

C_DEPS += \
./Libraries/STMTouch_Driver/src/tsl.d \
./Libraries/STMTouch_Driver/src/tsl_acq.d \
./Libraries/STMTouch_Driver/src/tsl_acq_stm32f0xx.d \
./Libraries/STMTouch_Driver/src/tsl_dxs.d \
./Libraries/STMTouch_Driver/src/tsl_ecs.d \
./Libraries/STMTouch_Driver/src/tsl_filter.d \
./Libraries/STMTouch_Driver/src/tsl_globals.d \
./Libraries/STMTouch_Driver/src/tsl_linrot.d \
./Libraries/STMTouch_Driver/src/tsl_object.d \
./Libraries/STMTouch_Driver/src/tsl_time.d \
./Libraries/STMTouch_Driver/src/tsl_time_stm32f0xx.d \
./Libraries/STMTouch_Driver/src/tsl_touchkey.d 


# Each subdirectory must supply rules for building sources it contributes
Libraries/STMTouch_Driver/src/%.o: ../Libraries/STMTouch_Driver/src/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross GCC Compiler'
	gcc -I"/home/tate/stm32f4/stm_flash_prj/STM32F0_Discovery_IO_Toggle/Libraries/STM32F0xx_StdPeriph_Driver/inc" -I"/home/tate/stm32f4/stm_flash_prj/STM32F0_Discovery_IO_Toggle/src" -I"/home/tate/stm32f4/stm_flash_prj/STM32F0_Discovery_IO_Toggle/Libraries/CMSIS/Include" -I"/home/tate/stm32f4/stm_flash_prj/STM32F0_Discovery_IO_Toggle/Libraries/STMTouch_Driver/inc" -I"/home/tate/stm32f4/stm_flash_prj/STM32F0_Discovery_IO_Toggle/Libraries/CMSIS/Device/ST/STM32F0xx/Include" -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


