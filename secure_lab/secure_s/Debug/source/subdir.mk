################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../source/main_s.c \
../source/secure_gpio.c \
../source/semihost_hardfault.c 

OBJS += \
./source/main_s.o \
./source/secure_gpio.o \
./source/semihost_hardfault.o 

C_DEPS += \
./source/main_s.d \
./source/secure_gpio.d \
./source/semihost_hardfault.d 


# Each subdirectory must supply rules for building sources it contributes
source/%.o: ../source/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU C Compiler'
	arm-none-eabi-gcc -std=gnu99 -DCPU_LPC55S69JBD100_cm33_core0 -D__SEMIHOST_HARDFAULT_DISABLE -DARM_MATH_CM33 -DCPU_LPC55S69JBD100 -DCPU_LPC55S69JBD100_cm33 -DCR_INTEGER_PRINTF -D__MCUXPRESSO -D__USE_CMSIS -DDEBUG -D__MULTICORE_MASTER -D__REDLIB__ -DSDK_DEBUGCONSOLE=1 -I../board -I../source -I../ -I../drivers -I../CMSIS -I../device -I../startup -I../component/serial_manager -I../utilities -I../component/uart -I../component/lists -I../trustzone -O0 -fno-common -g -Wall -c  -ffunction-sections  -fdata-sections  -ffreestanding  -fno-builtin  -mcpu=cortex-m33 -mcpu=cortex-m33 -mfpu=fpv5-sp-d16 -mfloat-abi=hard -mthumb -mcmse -D__REDLIB__ -specs=redlib.specs -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.o)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


