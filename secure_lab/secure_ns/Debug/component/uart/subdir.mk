################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../component/uart/usart_adapter.c 

OBJS += \
./component/uart/usart_adapter.o 

C_DEPS += \
./component/uart/usart_adapter.d 


# Each subdirectory must supply rules for building sources it contributes
component/uart/%.o: ../component/uart/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU C Compiler'
	arm-none-eabi-gcc -std=gnu99 -DCPU_LPC55S69JBD100_cm33_core0 -D__SEMIHOST_HARDFAULT_DISABLE -DARM_MATH_CM33 -DCPU_LPC55S69JBD100 -DCPU_LPC55S69JBD100_cm33 -DCR_INTEGER_PRINTF -D__MCUXPRESSO -D__USE_CMSIS -DDEBUG -D__MULTICORE_MASTER -D__REDLIB__ -DSDK_DEBUGCONSOLE=1 -DFSL_RTOS_BM -DSDK_OS_BAREMETAL -I../drivers -I../component/serial_manager -I../component/uart -I../component/lists -I../CMSIS -I../utilities -I../device -I../board -I../source -I../ -I../startup -I../trustzone -O0 -fno-common -g -Wall -D __SEMIHOST_HARDFAULT_DISABLE  -c  -ffunction-sections  -fdata-sections  -ffreestanding  -fno-builtin  -mcpu=cortex-m33 -mcpu=cortex-m33 -mfpu=fpv5-sp-d16 -mfloat-abi=hard -mthumb -D__REDLIB__ -specs=redlib.specs -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.o)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


