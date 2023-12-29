################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/modules/math_helpers/math_helpers.c 

OBJS += \
./Core/Src/modules/math_helpers/math_helpers.o 

C_DEPS += \
./Core/Src/modules/math_helpers/math_helpers.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/modules/math_helpers/math_helpers.o: ../Core/Src/modules/math_helpers/math_helpers.c Core/Src/modules/math_helpers/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32L412xx -c -I"/Users/jvincent/STM32CubeIDE/workspace_1.6.1/RC_circuit_feedback/Core/Src/modules/adc" -I"/Users/jvincent/STM32CubeIDE/workspace_1.6.1/RC_circuit_feedback/Core/Src/modules/math_helpers" -I"/Users/jvincent/STM32CubeIDE/workspace_1.6.1/RC_circuit_feedback/Core/Src/modules/pwm" -I../Core/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32L4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Core/Src/modules/math_helpers/math_helpers.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

