################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../libmodbus-3.1.4/tests/PaxHeaders.46003/bandwidth-client.c \
../libmodbus-3.1.4/tests/PaxHeaders.46003/bandwidth-server-many-up.c \
../libmodbus-3.1.4/tests/PaxHeaders.46003/bandwidth-server-one.c \
../libmodbus-3.1.4/tests/PaxHeaders.46003/random-test-client.c \
../libmodbus-3.1.4/tests/PaxHeaders.46003/random-test-server.c \
../libmodbus-3.1.4/tests/PaxHeaders.46003/unit-test-client.c \
../libmodbus-3.1.4/tests/PaxHeaders.46003/unit-test-server.c \
../libmodbus-3.1.4/tests/PaxHeaders.46003/version.c 

OBJS += \
./libmodbus-3.1.4/tests/PaxHeaders.46003/bandwidth-client.o \
./libmodbus-3.1.4/tests/PaxHeaders.46003/bandwidth-server-many-up.o \
./libmodbus-3.1.4/tests/PaxHeaders.46003/bandwidth-server-one.o \
./libmodbus-3.1.4/tests/PaxHeaders.46003/random-test-client.o \
./libmodbus-3.1.4/tests/PaxHeaders.46003/random-test-server.o \
./libmodbus-3.1.4/tests/PaxHeaders.46003/unit-test-client.o \
./libmodbus-3.1.4/tests/PaxHeaders.46003/unit-test-server.o \
./libmodbus-3.1.4/tests/PaxHeaders.46003/version.o 

C_DEPS += \
./libmodbus-3.1.4/tests/PaxHeaders.46003/bandwidth-client.d \
./libmodbus-3.1.4/tests/PaxHeaders.46003/bandwidth-server-many-up.d \
./libmodbus-3.1.4/tests/PaxHeaders.46003/bandwidth-server-one.d \
./libmodbus-3.1.4/tests/PaxHeaders.46003/random-test-client.d \
./libmodbus-3.1.4/tests/PaxHeaders.46003/random-test-server.d \
./libmodbus-3.1.4/tests/PaxHeaders.46003/unit-test-client.d \
./libmodbus-3.1.4/tests/PaxHeaders.46003/unit-test-server.d \
./libmodbus-3.1.4/tests/PaxHeaders.46003/version.d 


# Each subdirectory must supply rules for building sources it contributes
libmodbus-3.1.4/tests/PaxHeaders.46003/%.o: ../libmodbus-3.1.4/tests/PaxHeaders.46003/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross ARM GNU C Compiler'
	arm-linux-gnueabihf-gcc -mcpu=cortex-m3 -mthumb -O2  -g -I"C:\Users\Tom\Desktop\kernel_source\include" -I"C:\etc\arm_gcc\arm-none-eabi\include" -std=gnu11 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


