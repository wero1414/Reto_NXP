################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include utilities/subdir.mk
-include trustzone/subdir.mk
-include startup/subdir.mk
-include source/subdir.mk
-include drivers/subdir.mk
-include device/subdir.mk
-include component/uart/subdir.mk
-include component/serial_manager/subdir.mk
-include component/lists/subdir.mk
-include board/subdir.mk
-include subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 

# All Target
all: secure_s.axf

# Tool invocations
secure_s.axf: $(OBJS) $(USER_OBJS)
	@echo 'Building target: $@'
	@echo 'Invoking: MCU Linker'
	arm-none-eabi-gcc -nostdlib -L"C:\Users\eduardoContreras\Desktop\Demo Day\03-Secure Lab\secure_lab\secure_s\libs" -Xlinker --cmse-implib -Xlinker --out-implib=hello_world_s_CMSE_lib.o -Xlinker -Map="secure_s.map" -Xlinker --gc-sections -Xlinker -print-memory-usage -Xlinker --sort-section=alignment -mcpu=cortex-m33 -mfpu=fpv5-sp-d16 -mfloat-abi=hard -mthumb -T lpcxpresso55s69_hello_world_s_Debug.ld -Xlinker --out-implib="secure_s_CMSE_lib.o" -Xlinker --cmse-implib -o "secure_s.axf" $(OBJS) $(USER_OBJS) $(LIBS)
	@echo 'Finished building target: $@'
	@echo ' '
	$(MAKE) --no-print-directory post-build

# Other Targets
clean:
	-$(RM) $(EXECUTABLES)$(OBJS)$(C_DEPS) secure_s.axf
	-@echo ' '

post-build:
	-@echo 'Performing post-build steps'
	-arm-none-eabi-size "secure_s.axf"; # arm-none-eabi-objcopy -v -O binary "secure_s.axf" "secure_s.bin" ; # checksum -p LPC55S69 -d "secure_s.bin";
	-@echo ' '

.PHONY: all clean dependents post-build

-include ../makefile.targets
