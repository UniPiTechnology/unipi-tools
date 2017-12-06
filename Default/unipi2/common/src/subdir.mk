################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../unipi2/common/src/adc.c \
../unipi2/common/src/dac3.c \
../unipi2/common/src/daliin.c \
../unipi2/common/src/daliout.c \
../unipi2/common/src/digitalinp.c \
../unipi2/common/src/digitalout.c \
../unipi2/common/src/flash.c \
../unipi2/common/src/i2cled.c \
../unipi2/common/src/main.c \
../unipi2/common/src/mcpdac.c \
../unipi2/common/src/modbus.c \
../unipi2/common/src/modbus_prot.c \
../unipi2/common/src/mspi.c \
../unipi2/common/src/pcado.c \
../unipi2/common/src/sdadc.c \
../unipi2/common/src/spi.c \
../unipi2/common/src/spiflash.c \
../unipi2/common/src/system_stm32f0xx.c \
../unipi2/common/src/system_stm32f37x.c \
../unipi2/common/src/uart.c \
../unipi2/common/src/watchdog.c 

O_SRCS += \
../unipi2/common/src/adc.o \
../unipi2/common/src/digitalinp.o \
../unipi2/common/src/digitalout.o \
../unipi2/common/src/flash.o \
../unipi2/common/src/i2cled.o \
../unipi2/common/src/main.o \
../unipi2/common/src/mcpdac.o \
../unipi2/common/src/modbus.o \
../unipi2/common/src/modbus_prot.o \
../unipi2/common/src/mspi.o \
../unipi2/common/src/pcado.o \
../unipi2/common/src/sdadc.o \
../unipi2/common/src/spi.o \
../unipi2/common/src/spiflash.o \
../unipi2/common/src/system_stm32f0xx.o \
../unipi2/common/src/system_stm32f37x.o \
../unipi2/common/src/uart.o \
../unipi2/common/src/watchdog.o 

OBJS += \
./unipi2/common/src/adc.o \
./unipi2/common/src/dac3.o \
./unipi2/common/src/daliin.o \
./unipi2/common/src/daliout.o \
./unipi2/common/src/digitalinp.o \
./unipi2/common/src/digitalout.o \
./unipi2/common/src/flash.o \
./unipi2/common/src/i2cled.o \
./unipi2/common/src/main.o \
./unipi2/common/src/mcpdac.o \
./unipi2/common/src/modbus.o \
./unipi2/common/src/modbus_prot.o \
./unipi2/common/src/mspi.o \
./unipi2/common/src/pcado.o \
./unipi2/common/src/sdadc.o \
./unipi2/common/src/spi.o \
./unipi2/common/src/spiflash.o \
./unipi2/common/src/system_stm32f0xx.o \
./unipi2/common/src/system_stm32f37x.o \
./unipi2/common/src/uart.o \
./unipi2/common/src/watchdog.o 

C_DEPS += \
./unipi2/common/src/adc.d \
./unipi2/common/src/dac3.d \
./unipi2/common/src/daliin.d \
./unipi2/common/src/daliout.d \
./unipi2/common/src/digitalinp.d \
./unipi2/common/src/digitalout.d \
./unipi2/common/src/flash.d \
./unipi2/common/src/i2cled.d \
./unipi2/common/src/main.d \
./unipi2/common/src/mcpdac.d \
./unipi2/common/src/modbus.d \
./unipi2/common/src/modbus_prot.d \
./unipi2/common/src/mspi.d \
./unipi2/common/src/pcado.d \
./unipi2/common/src/sdadc.d \
./unipi2/common/src/spi.d \
./unipi2/common/src/spiflash.d \
./unipi2/common/src/system_stm32f0xx.d \
./unipi2/common/src/system_stm32f37x.d \
./unipi2/common/src/uart.d \
./unipi2/common/src/watchdog.d 


# Each subdirectory must supply rules for building sources it contributes
unipi2/common/src/%.o: ../unipi2/common/src/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross ARM GNU C Compiler'
	arm-linux-gnueabihf-gcc -mcpu=cortex-m3 -mthumb -O2  -g -I"C:\Users\Tom\Desktop\kernel_source\include" -I"C:\etc\arm_gcc\arm-none-eabi\include" -std=gnu11 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


