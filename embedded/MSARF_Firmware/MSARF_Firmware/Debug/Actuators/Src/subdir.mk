################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../Actuators/Src/Actuator.cpp \
../Actuators/Src/BrushedDCMotor.cpp \
../Actuators/Src/BrushlessMotor.cpp \
../Actuators/Src/Dynamixelrobotservo.cpp \
../Actuators/Src/PWMServo.cpp \
../Actuators/Src/XYZrobotServo.cpp 

OBJS += \
./Actuators/Src/Actuator.o \
./Actuators/Src/BrushedDCMotor.o \
./Actuators/Src/BrushlessMotor.o \
./Actuators/Src/Dynamixelrobotservo.o \
./Actuators/Src/PWMServo.o \
./Actuators/Src/XYZrobotServo.o 

CPP_DEPS += \
./Actuators/Src/Actuator.d \
./Actuators/Src/BrushedDCMotor.d \
./Actuators/Src/BrushlessMotor.d \
./Actuators/Src/Dynamixelrobotservo.d \
./Actuators/Src/PWMServo.d \
./Actuators/Src/XYZrobotServo.d 


# Each subdirectory must supply rules for building sources it contributes
Actuators/Src/%.o Actuators/Src/%.su: ../Actuators/Src/%.cpp Actuators/Src/subdir.mk
	arm-none-eabi-g++ "$<" -mcpu=cortex-m7 -std=gnu++14 -g3 -DUSE_HAL_DRIVER -DSTM32H743xx -DDEBUG -c -I../Middlewares/Third_Party/FreeRTOS/Source/include -I"C:/Users/Lizzie/embedded-software/MSARF_Firmware/MSARF_Firmware/Expansions/Inc" -I"C:/Users/Lizzie/embedded-software/MSARF_Firmware/MSARF_Firmware/EthLib/Internet/DHCP" -I../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I"C:/Users/Lizzie/embedded-software/MSARF_Firmware/MSARF_Firmware/Package/Inc" -I"C:/Users/Lizzie/embedded-software/MSARF_Firmware/MSARF_Firmware/EthLib/Ethernet/W5500" -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I"C:/Users/Lizzie/embedded-software/MSARF_Firmware/MSARF_Firmware/EthLib/Internet/DNS" -I"C:/Users/Lizzie/embedded-software/MSARF_Firmware/MSARF_Firmware/LCDLib/Inc" -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I"C:/Users/Lizzie/embedded-software/MSARF_Firmware/MSARF_Firmware/EthLib/loopback" -I"C:/Users/Lizzie/embedded-software/MSARF_Firmware/MSARF_Firmware/EthLib" -I../Drivers/CMSIS/Include -I../Core/Inc -I"C:/Users/Lizzie/embedded-software/MSARF_Firmware/MSARF_Firmware/ClosedLoop" -I"C:/Users/Lizzie/embedded-software/MSARF_Firmware/MSARF_Firmware/EthLib/Internet/XAVIER" -I../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I"C:/Users/Lizzie/embedded-software/MSARF_Firmware/MSARF_Firmware/Sensors/Inc" -I"C:/Users/Lizzie/embedded-software/MSARF_Firmware/MSARF_Firmware/EthLib/Ethernet" -I../Drivers/STM32H7xx_HAL_Driver/Inc -I"C:/Users/Lizzie/embedded-software/MSARF_Firmware/MSARF_Firmware/Actuators/Inc" -O0 -ffunction-sections -fdata-sections -fno-exceptions -fno-rtti -fno-use-cxa-atexit -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Actuators-2f-Src

clean-Actuators-2f-Src:
	-$(RM) ./Actuators/Src/Actuator.d ./Actuators/Src/Actuator.o ./Actuators/Src/Actuator.su ./Actuators/Src/BrushedDCMotor.d ./Actuators/Src/BrushedDCMotor.o ./Actuators/Src/BrushedDCMotor.su ./Actuators/Src/BrushlessMotor.d ./Actuators/Src/BrushlessMotor.o ./Actuators/Src/BrushlessMotor.su ./Actuators/Src/Dynamixelrobotservo.d ./Actuators/Src/Dynamixelrobotservo.o ./Actuators/Src/Dynamixelrobotservo.su ./Actuators/Src/PWMServo.d ./Actuators/Src/PWMServo.o ./Actuators/Src/PWMServo.su ./Actuators/Src/XYZrobotServo.d ./Actuators/Src/XYZrobotServo.o ./Actuators/Src/XYZrobotServo.su

.PHONY: clean-Actuators-2f-Src

