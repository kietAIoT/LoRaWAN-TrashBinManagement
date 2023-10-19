################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include VL53L0X/subdir.mk
-include LoRa/subdir.mk
-include Drivers/STM32F1xx_HAL_Driver/Src/subdir.mk
-include Core/Startup/subdir.mk
-include Core/Src/subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(S_DEPS)),)
-include $(S_DEPS)
endif
ifneq ($(strip $(S_UPPER_DEPS)),)
-include $(S_UPPER_DEPS)
endif
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

-include ../makefile.defs

OPTIONAL_TOOL_DEPS := \
$(wildcard ../makefile.defs) \
$(wildcard ../makefile.init) \
$(wildcard ../makefile.targets) \


BUILD_ARTIFACT_NAME := BIN_System_LoRaWAN
BUILD_ARTIFACT_EXTENSION := elf
BUILD_ARTIFACT_PREFIX :=
BUILD_ARTIFACT := $(BUILD_ARTIFACT_PREFIX)$(BUILD_ARTIFACT_NAME)$(if $(BUILD_ARTIFACT_EXTENSION),.$(BUILD_ARTIFACT_EXTENSION),)

# Add inputs and outputs from these tool invocations to the build variables 
EXECUTABLES += \
BIN_System_LoRaWAN.elf \

MAP_FILES += \
BIN_System_LoRaWAN.map \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
BIN_System_LoRaWAN.list \

OBJCOPY_HEX += \
BIN_System_LoRaWAN.hex \

OBJCOPY_BIN += \
BIN_System_LoRaWAN.bin \

OBJCOPY_VERILOG += \
BIN_System_LoRaWAN.verilog \

OBJCOPY_SREC += \
BIN_System_LoRaWAN.srec \

OBJCOPY_SYMBOLSREC += \
BIN_System_LoRaWAN.symbolsrec \


# All Target
all: main-build

# Main-build Target
main-build: BIN_System_LoRaWAN.elf secondary-outputs

# Tool invocations
BIN_System_LoRaWAN.elf BIN_System_LoRaWAN.map: $(OBJS) $(USER_OBJS) F:\Workspace\21_EmbeddedSystem\BIN_System_LoRaWAN\STM32F103C8TX_FLASH.ld makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-gcc -o "BIN_System_LoRaWAN.elf" @"objects.list" $(USER_OBJS) $(LIBS) -mcpu=cortex-m3 -T"F:\Workspace\21_EmbeddedSystem\BIN_System_LoRaWAN\STM32F103C8TX_FLASH.ld" --specs=nosys.specs -Wl,-Map="BIN_System_LoRaWAN.map" -Wl,--gc-sections -static --specs=nano.specs -mfloat-abi=soft -mthumb -Wl,--start-group -lc -lm -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES) makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

BIN_System_LoRaWAN.list: $(EXECUTABLES) makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "BIN_System_LoRaWAN.list"
	@echo 'Finished building: $@'
	@echo ' '

BIN_System_LoRaWAN.hex: $(EXECUTABLES) makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-objcopy  -O ihex $(EXECUTABLES) "BIN_System_LoRaWAN.hex"
	@echo 'Finished building: $@'
	@echo ' '

BIN_System_LoRaWAN.bin: $(EXECUTABLES) makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-objcopy  -O binary $(EXECUTABLES) "BIN_System_LoRaWAN.bin"
	@echo 'Finished building: $@'
	@echo ' '

BIN_System_LoRaWAN.verilog: $(EXECUTABLES) makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-objcopy  -O verilog $(EXECUTABLES) "BIN_System_LoRaWAN.verilog"
	@echo 'Finished building: $@'
	@echo ' '

BIN_System_LoRaWAN.srec: $(EXECUTABLES) makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-objcopy  -O srec $(EXECUTABLES) "BIN_System_LoRaWAN.srec"
	@echo 'Finished building: $@'
	@echo ' '

BIN_System_LoRaWAN.symbolsrec: $(EXECUTABLES) makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-objcopy  -O symbolsrec $(EXECUTABLES) "BIN_System_LoRaWAN.symbolsrec"
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) BIN_System_LoRaWAN.bin BIN_System_LoRaWAN.elf BIN_System_LoRaWAN.hex BIN_System_LoRaWAN.list BIN_System_LoRaWAN.map BIN_System_LoRaWAN.srec BIN_System_LoRaWAN.symbolsrec BIN_System_LoRaWAN.verilog default.size.stdout
	-@echo ' '

secondary-outputs: $(SIZE_OUTPUT) $(OBJDUMP_LIST) $(OBJCOPY_HEX) $(OBJCOPY_BIN) $(OBJCOPY_VERILOG) $(OBJCOPY_SREC) $(OBJCOPY_SYMBOLSREC)

fail-specified-linker-script-missing:
	@echo 'Error: Cannot find the specified linker script. Check the linker settings in the build configuration.'
	@exit 2

warn-no-linker-script-specified:
	@echo 'Warning: No linker script specified. Check the linker settings in the build configuration.'

.PHONY: all clean dependents main-build fail-specified-linker-script-missing warn-no-linker-script-specified

-include ../makefile.targets
