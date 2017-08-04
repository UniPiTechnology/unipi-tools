################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../armpty.c \
../armspi.c \
../armutil.c \
../bandwidth-client.c \
../fwserial.c \
../fwspi.c \
../nb_modbus.c \
../neuron_tcp_server.c \
../neuronspi.c \
../spicrc.c \
../test_module.c \
../win32_serial.c 

O_SRCS += \
../armutil.o \
../fwserial.o \
../fwspi.o 

OBJS += \
./armpty.o \
./armspi.o \
./armutil.o \
./bandwidth-client.o \
./fwserial.o \
./fwspi.o \
./nb_modbus.o \
./neuron_tcp_server.o \
./neuronspi.o \
./spicrc.o \
./test_module.o \
./win32_serial.o 

C_DEPS += \
./armpty.d \
./armspi.d \
./armutil.d \
./bandwidth-client.d \
./fwserial.d \
./fwspi.d \
./nb_modbus.d \
./neuron_tcp_server.d \
./neuronspi.d \
./spicrc.d \
./test_module.d \
./win32_serial.d 


# Each subdirectory must supply rules for building sources it contributes
%.o: ../%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross ARM GNU C Compiler'
	arm-linux-gnueabihf-gcc -mcpu=cortex-m3 -mthumb -O2  -g -I"C:\Users\Tom\Desktop\kernel_source\include" -I"C:\etc\arm_gcc\arm-none-eabi\include" -std=gnu11 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


