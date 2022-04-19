################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../Expansions/Src/GPIOBank.cpp \
../Expansions/Src/PCA9685_PWM_Driver.cpp \
../Expansions/Src/PWMBank.cpp \
../Expansions/Src/TCA9555_GPIO_Driver.cpp 

OBJS += \
./Expansions/Src/GPIOBank.o \
./Expansions/Src/PCA9685_PWM_Driver.o \
./Expansions/Src/PWMBank.o \
./Expansions/Src/TCA9555_GPIO_Driver.o 

CPP_DEPS += \
./Expansions/Src/GPIOBank.d \
./Expansions/Src/PCA9685_PWM_Driver.d \
./Expansions/Src/PWMBank.d \
./Expansions/Src/TCA9555_GPIO_Driver.d 


# Each subdirectory must supply rules for building sources it contributes
Expansions/Src/%.o Expansions/Src/%.su: ../Expansions/Src/%.cpp Expansions/Src/subdir.mk
	arm-none-eabi-g++ "$<" -mcpu=cortex-m7 -std=gnu++14 -g3 -DUSE_HAL_DRIVER -DSTM32H743xx -DDEBUG -c -I../Middlewares/Third_Party/FreeRTOS/Source/include -I"C:/Users/Lizzie/embedded-software/MSARF_Firmware/MSARF_Firmware/Expansions/Inc" -I"C:/Users/Lizzie/embedded-software/MSARF_Firmware/MSARF_Firmware/EthLib/Internet/DHCP" -I../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I"C:/Users/Lizzie/embedded-software/MSARF_Firmware/MSARF_Firmware/Package/Inc" -I"C:/Users/Lizzie/embedded-software/MSARF_Firmware/MSARF_Firmware/EthLib/Ethernet/W5500" -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I"C:/Users/Lizzie/embedded-software/MSARF_Firmware/MSARF_Firmware/EthLib/Internet/DNS" -I"C:/Users/Lizzie/embedded-software/MSARF_Firmware/MSARF_Firmware/LCDLib/Inc" -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I"C:/Users/Lizzie/embedded-software/MSARF_Firmware/MSARF_Firmware/EthLib/loopback" -I"C:/Users/Lizzie/embedded-software/MSARF_Firmware/MSARF_Firmware/EthLib" -I../Drivers/CMSIS/Include -I../Core/Inc -I"C:/Users/Lizzie/embedded-software/MSARF_Firmware/MSARF_Firmware/ClosedLoop" -I"C:/Users/Lizzie/embedded-software/MSARF_Firmware/MSARF_Firmware/EthLib/Internet/XAVIER" -I../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I"C:/Users/Lizzie/embedded-software/MSARF_Firmware/MSARF_Firmware/Sensors/Inc" -I"C:/Users/Lizzie/embedded-software/MSARF_Firmware/MSARF_Firmware/EthLib/Ethernet" -I../Drivers/STM32H7xx_HAL_Driver/Inc -I"C:/Users/Lizzie/embedded-software/MSARF_Firmware/MSARF_Firmware/Actuators/Inc" -O0 -ffunction-sections -fdata-sections -fno-exceptions -fno-rtti -fno-use-cxa-atexit -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Expansions-2f-Src

clean-Expansions-2f-Src:
	-$(RM) ./Expansions/Src/GPIOBank.d ./Expansions/Src/GPIOBank.o ./Expansions/Src/GPIOBank.su ./Expansions/Src/PCA9685_PWM_Driver.d ./Expansions/Src/PCA9685_PWM_Driver.o ./Expansions/Src/PCA9685_PWM_Driver.su ./Expansions/Src/PWMBank.d ./Expansions/Src/PWMBank.o ./Expansions/Src/PWMBank.su ./Expansions/Src/TCA9555_GPIO_Driver.d ./Expansions/Src/TCA9555_GPIO_Driver.o ./Expansions/Src/TCA9555_GPIO_Driver.su

.PHONY: clean-Expansions-2f-Src

