################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../Sensors/Src/AS5048A.cpp \
../Sensors/Src/Encoder.cpp \
../Sensors/Src/Potentiometer.cpp \
../Sensors/Src/Sensor.cpp 

OBJS += \
./Sensors/Src/AS5048A.o \
./Sensors/Src/Encoder.o \
./Sensors/Src/Potentiometer.o \
./Sensors/Src/Sensor.o 

CPP_DEPS += \
./Sensors/Src/AS5048A.d \
./Sensors/Src/Encoder.d \
./Sensors/Src/Potentiometer.d \
./Sensors/Src/Sensor.d 


# Each subdirectory must supply rules for building sources it contributes
Sensors/Src/%.o Sensors/Src/%.su: ../Sensors/Src/%.cpp Sensors/Src/subdir.mk
	arm-none-eabi-g++ "$<" -mcpu=cortex-m7 -std=gnu++14 -g3 -DUSE_HAL_DRIVER -DSTM32H743xx -DDEBUG -c -I../Middlewares/Third_Party/FreeRTOS/Source/include -I"C:/Users/Lizzie/embedded-software/MSARF_Firmware/MSARF_Firmware/Expansions/Inc" -I"C:/Users/Lizzie/embedded-software/MSARF_Firmware/MSARF_Firmware/EthLib/Internet/DHCP" -I../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I"C:/Users/Lizzie/embedded-software/MSARF_Firmware/MSARF_Firmware/Package/Inc" -I"C:/Users/Lizzie/embedded-software/MSARF_Firmware/MSARF_Firmware/EthLib/Ethernet/W5500" -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I"C:/Users/Lizzie/embedded-software/MSARF_Firmware/MSARF_Firmware/EthLib/Internet/DNS" -I"C:/Users/Lizzie/embedded-software/MSARF_Firmware/MSARF_Firmware/LCDLib/Inc" -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I"C:/Users/Lizzie/embedded-software/MSARF_Firmware/MSARF_Firmware/EthLib/loopback" -I"C:/Users/Lizzie/embedded-software/MSARF_Firmware/MSARF_Firmware/EthLib" -I../Drivers/CMSIS/Include -I../Core/Inc -I"C:/Users/Lizzie/embedded-software/MSARF_Firmware/MSARF_Firmware/ClosedLoop" -I"C:/Users/Lizzie/embedded-software/MSARF_Firmware/MSARF_Firmware/EthLib/Internet/XAVIER" -I../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I"C:/Users/Lizzie/embedded-software/MSARF_Firmware/MSARF_Firmware/Sensors/Inc" -I"C:/Users/Lizzie/embedded-software/MSARF_Firmware/MSARF_Firmware/EthLib/Ethernet" -I../Drivers/STM32H7xx_HAL_Driver/Inc -I"C:/Users/Lizzie/embedded-software/MSARF_Firmware/MSARF_Firmware/Actuators/Inc" -O0 -ffunction-sections -fdata-sections -fno-exceptions -fno-rtti -fno-use-cxa-atexit -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Sensors-2f-Src

clean-Sensors-2f-Src:
	-$(RM) ./Sensors/Src/AS5048A.d ./Sensors/Src/AS5048A.o ./Sensors/Src/AS5048A.su ./Sensors/Src/Encoder.d ./Sensors/Src/Encoder.o ./Sensors/Src/Encoder.su ./Sensors/Src/Potentiometer.d ./Sensors/Src/Potentiometer.o ./Sensors/Src/Potentiometer.su ./Sensors/Src/Sensor.d ./Sensors/Src/Sensor.o ./Sensors/Src/Sensor.su

.PHONY: clean-Sensors-2f-Src

