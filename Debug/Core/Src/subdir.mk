################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/APP.c \
../Core/Src/Autonomous_functions.c \
../Core/Src/EMA_Filter.c \
../Core/Src/adc.c \
../Core/Src/autonomous_t26.c \
../Core/Src/ble_handler.c \
../Core/Src/can.c \
../Core/Src/dma.c \
../Core/Src/gpio.c \
../Core/Src/hardware_abstaction.c \
../Core/Src/i2c.c \
../Core/Src/logger.c \
../Core/Src/main.c \
../Core/Src/ring_buffer.c \
../Core/Src/rn4871.c \
../Core/Src/rtc.c \
../Core/Src/state_machine.c \
../Core/Src/stm32f4xx_hal_msp.c \
../Core/Src/stm32f4xx_it.c \
../Core/Src/syscalls.c \
../Core/Src/sysmem.c \
../Core/Src/system_stm32f4xx.c \
../Core/Src/tim.c \
../Core/Src/usart.c 

C_DEPS += \
./Core/Src/APP.d \
./Core/Src/Autonomous_functions.d \
./Core/Src/EMA_Filter.d \
./Core/Src/adc.d \
./Core/Src/autonomous_t26.d \
./Core/Src/ble_handler.d \
./Core/Src/can.d \
./Core/Src/dma.d \
./Core/Src/gpio.d \
./Core/Src/hardware_abstaction.d \
./Core/Src/i2c.d \
./Core/Src/logger.d \
./Core/Src/main.d \
./Core/Src/ring_buffer.d \
./Core/Src/rn4871.d \
./Core/Src/rtc.d \
./Core/Src/state_machine.d \
./Core/Src/stm32f4xx_hal_msp.d \
./Core/Src/stm32f4xx_it.d \
./Core/Src/syscalls.d \
./Core/Src/sysmem.d \
./Core/Src/system_stm32f4xx.d \
./Core/Src/tim.d \
./Core/Src/usart.d 

OBJS += \
./Core/Src/APP.o \
./Core/Src/Autonomous_functions.o \
./Core/Src/EMA_Filter.o \
./Core/Src/adc.o \
./Core/Src/autonomous_t26.o \
./Core/Src/ble_handler.o \
./Core/Src/can.o \
./Core/Src/dma.o \
./Core/Src/gpio.o \
./Core/Src/hardware_abstaction.o \
./Core/Src/i2c.o \
./Core/Src/logger.o \
./Core/Src/main.o \
./Core/Src/ring_buffer.o \
./Core/Src/rn4871.o \
./Core/Src/rtc.o \
./Core/Src/state_machine.o \
./Core/Src/stm32f4xx_hal_msp.o \
./Core/Src/stm32f4xx_it.o \
./Core/Src/syscalls.o \
./Core/Src/sysmem.o \
./Core/Src/system_stm32f4xx.o \
./Core/Src/tim.o \
./Core/Src/usart.o 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/%.o Core/Src/%.su Core/Src/%.cyclo: ../Core/Src/%.c Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F412Rx -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I"C:/Users/jpser/Documents/GitHub/ACU_V3.0/Drivers/24FC08" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src

clean-Core-2f-Src:
	-$(RM) ./Core/Src/APP.cyclo ./Core/Src/APP.d ./Core/Src/APP.o ./Core/Src/APP.su ./Core/Src/Autonomous_functions.cyclo ./Core/Src/Autonomous_functions.d ./Core/Src/Autonomous_functions.o ./Core/Src/Autonomous_functions.su ./Core/Src/EMA_Filter.cyclo ./Core/Src/EMA_Filter.d ./Core/Src/EMA_Filter.o ./Core/Src/EMA_Filter.su ./Core/Src/adc.cyclo ./Core/Src/adc.d ./Core/Src/adc.o ./Core/Src/adc.su ./Core/Src/autonomous_t26.cyclo ./Core/Src/autonomous_t26.d ./Core/Src/autonomous_t26.o ./Core/Src/autonomous_t26.su ./Core/Src/ble_handler.cyclo ./Core/Src/ble_handler.d ./Core/Src/ble_handler.o ./Core/Src/ble_handler.su ./Core/Src/can.cyclo ./Core/Src/can.d ./Core/Src/can.o ./Core/Src/can.su ./Core/Src/dma.cyclo ./Core/Src/dma.d ./Core/Src/dma.o ./Core/Src/dma.su ./Core/Src/gpio.cyclo ./Core/Src/gpio.d ./Core/Src/gpio.o ./Core/Src/gpio.su ./Core/Src/hardware_abstaction.cyclo ./Core/Src/hardware_abstaction.d ./Core/Src/hardware_abstaction.o ./Core/Src/hardware_abstaction.su ./Core/Src/i2c.cyclo ./Core/Src/i2c.d ./Core/Src/i2c.o ./Core/Src/i2c.su ./Core/Src/logger.cyclo ./Core/Src/logger.d ./Core/Src/logger.o ./Core/Src/logger.su ./Core/Src/main.cyclo ./Core/Src/main.d ./Core/Src/main.o ./Core/Src/main.su ./Core/Src/ring_buffer.cyclo ./Core/Src/ring_buffer.d ./Core/Src/ring_buffer.o ./Core/Src/ring_buffer.su ./Core/Src/rn4871.cyclo ./Core/Src/rn4871.d ./Core/Src/rn4871.o ./Core/Src/rn4871.su ./Core/Src/rtc.cyclo ./Core/Src/rtc.d ./Core/Src/rtc.o ./Core/Src/rtc.su ./Core/Src/state_machine.cyclo ./Core/Src/state_machine.d ./Core/Src/state_machine.o ./Core/Src/state_machine.su ./Core/Src/stm32f4xx_hal_msp.cyclo ./Core/Src/stm32f4xx_hal_msp.d ./Core/Src/stm32f4xx_hal_msp.o ./Core/Src/stm32f4xx_hal_msp.su ./Core/Src/stm32f4xx_it.cyclo ./Core/Src/stm32f4xx_it.d ./Core/Src/stm32f4xx_it.o ./Core/Src/stm32f4xx_it.su ./Core/Src/syscalls.cyclo ./Core/Src/syscalls.d ./Core/Src/syscalls.o ./Core/Src/syscalls.su ./Core/Src/sysmem.cyclo ./Core/Src/sysmem.d ./Core/Src/sysmem.o ./Core/Src/sysmem.su ./Core/Src/system_stm32f4xx.cyclo ./Core/Src/system_stm32f4xx.d ./Core/Src/system_stm32f4xx.o ./Core/Src/system_stm32f4xx.su ./Core/Src/tim.cyclo ./Core/Src/tim.d ./Core/Src/tim.o ./Core/Src/tim.su ./Core/Src/usart.cyclo ./Core/Src/usart.d ./Core/Src/usart.o ./Core/Src/usart.su

.PHONY: clean-Core-2f-Src

