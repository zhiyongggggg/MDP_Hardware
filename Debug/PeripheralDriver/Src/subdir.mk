################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (14.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../PeripheralDriver/Src/HCSR04.c \
../PeripheralDriver/Src/UART_Comm.c \
../PeripheralDriver/Src/icm20948.c \
../PeripheralDriver/Src/motor.c \
../PeripheralDriver/Src/oled.c \
../PeripheralDriver/Src/servo.c 

OBJS += \
./PeripheralDriver/Src/HCSR04.o \
./PeripheralDriver/Src/UART_Comm.o \
./PeripheralDriver/Src/icm20948.o \
./PeripheralDriver/Src/motor.o \
./PeripheralDriver/Src/oled.o \
./PeripheralDriver/Src/servo.o 

C_DEPS += \
./PeripheralDriver/Src/HCSR04.d \
./PeripheralDriver/Src/UART_Comm.d \
./PeripheralDriver/Src/icm20948.d \
./PeripheralDriver/Src/motor.d \
./PeripheralDriver/Src/oled.d \
./PeripheralDriver/Src/servo.d 


# Each subdirectory must supply rules for building sources it contributes
PeripheralDriver/Src/%.o PeripheralDriver/Src/%.su PeripheralDriver/Src/%.cyclo: ../PeripheralDriver/Src/%.c PeripheralDriver/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F407xx -c -I../Core/Inc -I"/Users/limzhiyong/Downloads/MDP_task_1 IR no finetune 3/MDP_task_1 IR no finetune/MDP_task_1 2/PeripheralDriver/Inc" -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-PeripheralDriver-2f-Src

clean-PeripheralDriver-2f-Src:
	-$(RM) ./PeripheralDriver/Src/HCSR04.cyclo ./PeripheralDriver/Src/HCSR04.d ./PeripheralDriver/Src/HCSR04.o ./PeripheralDriver/Src/HCSR04.su ./PeripheralDriver/Src/UART_Comm.cyclo ./PeripheralDriver/Src/UART_Comm.d ./PeripheralDriver/Src/UART_Comm.o ./PeripheralDriver/Src/UART_Comm.su ./PeripheralDriver/Src/icm20948.cyclo ./PeripheralDriver/Src/icm20948.d ./PeripheralDriver/Src/icm20948.o ./PeripheralDriver/Src/icm20948.su ./PeripheralDriver/Src/motor.cyclo ./PeripheralDriver/Src/motor.d ./PeripheralDriver/Src/motor.o ./PeripheralDriver/Src/motor.su ./PeripheralDriver/Src/oled.cyclo ./PeripheralDriver/Src/oled.d ./PeripheralDriver/Src/oled.o ./PeripheralDriver/Src/oled.su ./PeripheralDriver/Src/servo.cyclo ./PeripheralDriver/Src/servo.d ./PeripheralDriver/Src/servo.o ./PeripheralDriver/Src/servo.su

.PHONY: clean-PeripheralDriver-2f-Src

