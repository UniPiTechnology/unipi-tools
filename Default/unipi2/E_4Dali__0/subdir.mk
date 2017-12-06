################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../unipi2/E_4Dali__0/daliin.c \
../unipi2/E_4Dali__0/daliout.c \
../unipi2/E_4Dali__0/manchester.c 

O_SRCS += \
../unipi2/E_4Dali__0/daliin.o \
../unipi2/E_4Dali__0/daliout.o 

OBJS += \
./unipi2/E_4Dali__0/daliin.o \
./unipi2/E_4Dali__0/daliout.o \
./unipi2/E_4Dali__0/manchester.o 

C_DEPS += \
./unipi2/E_4Dali__0/daliin.d \
./unipi2/E_4Dali__0/daliout.d \
./unipi2/E_4Dali__0/manchester.d 


# Each subdirectory must supply rules for building sources it contributes
unipi2/E_4Dali__0/%.o: ../unipi2/E_4Dali__0/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross ARM GNU C Compiler'
	arm-linux-gnueabihf-gcc -mcpu=cortex-m3 -mthumb -O2  -g -I"C:\Users\Tom\Desktop\kernel_source\include" -I"C:\etc\arm_gcc\arm-none-eabi\include" -std=gnu11 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


