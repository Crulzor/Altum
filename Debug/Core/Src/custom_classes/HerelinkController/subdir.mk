################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../Core/Src/custom_classes/HerelinkController/Convertor.cpp \
../Core/Src/custom_classes/HerelinkController/HerelinkController.cpp \
../Core/Src/custom_classes/HerelinkController/MavlinkControl.cpp \
../Core/Src/custom_classes/HerelinkController/SBUS.cpp \
../Core/Src/custom_classes/HerelinkController/altimeter.cpp 

OBJS += \
./Core/Src/custom_classes/HerelinkController/Convertor.o \
./Core/Src/custom_classes/HerelinkController/HerelinkController.o \
./Core/Src/custom_classes/HerelinkController/MavlinkControl.o \
./Core/Src/custom_classes/HerelinkController/SBUS.o \
./Core/Src/custom_classes/HerelinkController/altimeter.o 

CPP_DEPS += \
./Core/Src/custom_classes/HerelinkController/Convertor.d \
./Core/Src/custom_classes/HerelinkController/HerelinkController.d \
./Core/Src/custom_classes/HerelinkController/MavlinkControl.d \
./Core/Src/custom_classes/HerelinkController/SBUS.d \
./Core/Src/custom_classes/HerelinkController/altimeter.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/custom_classes/HerelinkController/%.o Core/Src/custom_classes/HerelinkController/%.su Core/Src/custom_classes/HerelinkController/%.cyclo: ../Core/Src/custom_classes/HerelinkController/%.cpp Core/Src/custom_classes/HerelinkController/subdir.mk
	arm-none-eabi-g++ "$<" -mcpu=cortex-m4 -std=gnu++14 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32G484xx -c -I../Core/Inc -I../Drivers/STM32G4xx_HAL_Driver/Inc -I../Drivers/STM32G4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32G4xx/Include -I../Drivers/CMSIS/Include -I../USB_Device/App -I../USB_Device/Target -I../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -I../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -Os -ffunction-sections -fdata-sections -fno-exceptions -fno-rtti -fno-use-cxa-atexit -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src-2f-custom_classes-2f-HerelinkController

clean-Core-2f-Src-2f-custom_classes-2f-HerelinkController:
	-$(RM) ./Core/Src/custom_classes/HerelinkController/Convertor.cyclo ./Core/Src/custom_classes/HerelinkController/Convertor.d ./Core/Src/custom_classes/HerelinkController/Convertor.o ./Core/Src/custom_classes/HerelinkController/Convertor.su ./Core/Src/custom_classes/HerelinkController/HerelinkController.cyclo ./Core/Src/custom_classes/HerelinkController/HerelinkController.d ./Core/Src/custom_classes/HerelinkController/HerelinkController.o ./Core/Src/custom_classes/HerelinkController/HerelinkController.su ./Core/Src/custom_classes/HerelinkController/MavlinkControl.cyclo ./Core/Src/custom_classes/HerelinkController/MavlinkControl.d ./Core/Src/custom_classes/HerelinkController/MavlinkControl.o ./Core/Src/custom_classes/HerelinkController/MavlinkControl.su ./Core/Src/custom_classes/HerelinkController/SBUS.cyclo ./Core/Src/custom_classes/HerelinkController/SBUS.d ./Core/Src/custom_classes/HerelinkController/SBUS.o ./Core/Src/custom_classes/HerelinkController/SBUS.su ./Core/Src/custom_classes/HerelinkController/altimeter.cyclo ./Core/Src/custom_classes/HerelinkController/altimeter.d ./Core/Src/custom_classes/HerelinkController/altimeter.o ./Core/Src/custom_classes/HerelinkController/altimeter.su

.PHONY: clean-Core-2f-Src-2f-custom_classes-2f-HerelinkController

