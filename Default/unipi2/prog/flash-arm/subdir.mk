################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../unipi2/prog/flash-arm/s2bin.c 

OBJS += \
./unipi2/prog/flash-arm/s2bin.o 

C_DEPS += \
./unipi2/prog/flash-arm/s2bin.d 


# Each subdirectory must supply rules for building sources it contributes
unipi2/prog/flash-arm/%.o: ../unipi2/prog/flash-arm/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross ARM GNU C Compiler'
	arm-linux-gnueabihf-gcc -mcpu=cortex-m3 -mthumb -O2  -g -I"C:\Users\Tom\Desktop\kernel_source\include" -I"C:\etc\arm_gcc\arm-none-eabi\include" -std=gnu11 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


