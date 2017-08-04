################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../libmodbus-3.1.4/src/PaxHeaders.46003/modbus-data.c \
../libmodbus-3.1.4/src/PaxHeaders.46003/modbus-rtu.c \
../libmodbus-3.1.4/src/PaxHeaders.46003/modbus-tcp.c \
../libmodbus-3.1.4/src/PaxHeaders.46003/modbus.c 

OBJS += \
./libmodbus-3.1.4/src/PaxHeaders.46003/modbus-data.o \
./libmodbus-3.1.4/src/PaxHeaders.46003/modbus-rtu.o \
./libmodbus-3.1.4/src/PaxHeaders.46003/modbus-tcp.o \
./libmodbus-3.1.4/src/PaxHeaders.46003/modbus.o 

C_DEPS += \
./libmodbus-3.1.4/src/PaxHeaders.46003/modbus-data.d \
./libmodbus-3.1.4/src/PaxHeaders.46003/modbus-rtu.d \
./libmodbus-3.1.4/src/PaxHeaders.46003/modbus-tcp.d \
./libmodbus-3.1.4/src/PaxHeaders.46003/modbus.d 


# Each subdirectory must supply rules for building sources it contributes
libmodbus-3.1.4/src/PaxHeaders.46003/%.o: ../libmodbus-3.1.4/src/PaxHeaders.46003/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross ARM GNU C Compiler'
	arm-linux-gnueabihf-gcc -mcpu=cortex-m3 -mthumb -O2  -g -I"C:\Users\Tom\Desktop\kernel_source\include" -I"C:\etc\arm_gcc\arm-none-eabi\include" -std=gnu11 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


