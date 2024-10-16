################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include Drivers/STM32F4xx_HAL_Driver/Src/subdir.mk
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


BUILD_ARTIFACT_NAME := KIA_Accelerator_Sensor_Emulator
BUILD_ARTIFACT_EXTENSION := elf
BUILD_ARTIFACT_PREFIX :=
BUILD_ARTIFACT := $(BUILD_ARTIFACT_PREFIX)$(BUILD_ARTIFACT_NAME)$(if $(BUILD_ARTIFACT_EXTENSION),.$(BUILD_ARTIFACT_EXTENSION),)

# Add inputs and outputs from these tool invocations to the build variables 
EXECUTABLES += \
KIA_Accelerator_Sensor_Emulator.elf \

MAP_FILES += \
KIA_Accelerator_Sensor_Emulator.map \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
KIA_Accelerator_Sensor_Emulator.list \


# All Target
all: main-build

# Main-build Target
main-build: KIA_Accelerator_Sensor_Emulator.elf secondary-outputs

# Tool invocations
KIA_Accelerator_Sensor_Emulator.elf KIA_Accelerator_Sensor_Emulator.map: $(OBJS) $(USER_OBJS) C:\Users\xuanp\OneDrive\ -\ email.ucr.edu\Study\Cecert\KIA\KIA-drive-by-wire\KIA_Accelerator_Sensor_Emulator\STM32F446RETX_FLASH.ld makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-gcc -o "KIA_Accelerator_Sensor_Emulator.elf" @"objects.list" $(USER_OBJS) $(LIBS) -mcpu=cortex-m4 -T"C:\Users\xuanp\OneDrive - email.ucr.edu\Study\Cecert\KIA\KIA-drive-by-wire\KIA_Accelerator_Sensor_Emulator\STM32F446RETX_FLASH.ld" --specs=nosys.specs -Wl,-Map="KIA_Accelerator_Sensor_Emulator.map" -Wl,--gc-sections -static --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -Wl,--start-group -lc -lm -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES) makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

KIA_Accelerator_Sensor_Emulator.list: $(EXECUTABLES) makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "KIA_Accelerator_Sensor_Emulator.list"
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) KIA_Accelerator_Sensor_Emulator.elf KIA_Accelerator_Sensor_Emulator.list KIA_Accelerator_Sensor_Emulator.map default.size.stdout
	-@echo ' '

secondary-outputs: $(SIZE_OUTPUT) $(OBJDUMP_LIST)

fail-specified-linker-script-missing:
	@echo 'Error: Cannot find the specified linker script. Check the linker settings in the build configuration.'
	@exit 2

warn-no-linker-script-specified:
	@echo 'Warning: No linker script specified. Check the linker settings in the build configuration.'

.PHONY: all clean dependents main-build fail-specified-linker-script-missing warn-no-linker-script-specified

-include ../makefile.targets
