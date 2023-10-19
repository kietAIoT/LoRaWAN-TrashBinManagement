################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../VL53L0X/VL53L0X.c 

C_DEPS += \
./VL53L0X/VL53L0X.d 

OBJS += \
./VL53L0X/VL53L0X.o 


# Each subdirectory must supply rules for building sources it contributes
VL53L0X/VL53L0X.o: F:/Workspace/21_EmbeddedSystem/BIN_System_LoRaWAN/VL53L0X/VL53L0X.c VL53L0X/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F103xB -c -I../Core/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/STM32F1xx_HAL_Driver/Inc -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -I"F:/Workspace/21_EmbeddedSystem/BIN_System_LoRaWAN/LoRa" -I"F:/Workspace/21_EmbeddedSystem/BIN_System_LoRaWAN/VL53L0X" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-VL53L0X

clean-VL53L0X:
	-$(RM) ./VL53L0X/VL53L0X.cyclo ./VL53L0X/VL53L0X.d ./VL53L0X/VL53L0X.o ./VL53L0X/VL53L0X.su

.PHONY: clean-VL53L0X

