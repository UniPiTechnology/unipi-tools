################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../libmodbus-3.1.4/tests/bandwidth-client.c \
../libmodbus-3.1.4/tests/bandwidth-server-many-up.c \
../libmodbus-3.1.4/tests/bandwidth-server-one.c \
../libmodbus-3.1.4/tests/random-test-client.c \
../libmodbus-3.1.4/tests/random-test-server.c \
../libmodbus-3.1.4/tests/unit-test-client.c \
../libmodbus-3.1.4/tests/unit-test-server.c \
../libmodbus-3.1.4/tests/version.c 

O_SRCS += \
../libmodbus-3.1.4/tests/bandwidth-client.o \
../libmodbus-3.1.4/tests/bandwidth-server-many-up.o \
../libmodbus-3.1.4/tests/bandwidth-server-one.o \
../libmodbus-3.1.4/tests/random-test-client.o \
../libmodbus-3.1.4/tests/random-test-server.o \
../libmodbus-3.1.4/tests/unit-test-client.o \
../libmodbus-3.1.4/tests/unit-test-server.o \
../libmodbus-3.1.4/tests/version.o 

OBJS += \
./libmodbus-3.1.4/tests/bandwidth-client.o \
./libmodbus-3.1.4/tests/bandwidth-server-many-up.o \
./libmodbus-3.1.4/tests/bandwidth-server-one.o \
./libmodbus-3.1.4/tests/random-test-client.o \
./libmodbus-3.1.4/tests/random-test-server.o \
./libmodbus-3.1.4/tests/unit-test-client.o \
./libmodbus-3.1.4/tests/unit-test-server.o \
./libmodbus-3.1.4/tests/version.o 

C_DEPS += \
./libmodbus-3.1.4/tests/bandwidth-client.d \
./libmodbus-3.1.4/tests/bandwidth-server-many-up.d \
./libmodbus-3.1.4/tests/bandwidth-server-one.d \
./libmodbus-3.1.4/tests/random-test-client.d \
./libmodbus-3.1.4/tests/random-test-server.d \
./libmodbus-3.1.4/tests/unit-test-client.d \
./libmodbus-3.1.4/tests/unit-test-server.d \
./libmodbus-3.1.4/tests/version.d 


# Each subdirectory must supply rules for building sources it contributes
libmodbus-3.1.4/tests/%.o: ../libmodbus-3.1.4/tests/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross ARM GNU C Compiler'
	arm-linux-gnueabihf-gcc -mcpu=cortex-m3 -mthumb -O2  -g -I"C:\Users\Tom\Desktop\kernel_source\include" -I"C:\etc\arm_gcc\arm-none-eabi\include" -std=gnu11 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


