################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/custom_classes/handlers.c 

CPP_SRCS += \
../Core/Src/custom_classes/Convertor.cpp \
../Core/Src/custom_classes/Initializer.cpp \
../Core/Src/custom_classes/MavlinkControl.cpp \
../Core/Src/custom_classes/SBUS.cpp \
../Core/Src/custom_classes/components.cpp \
../Core/Src/custom_classes/debugger.cpp 

C_DEPS += \
./Core/Src/custom_classes/handlers.d 

OBJS += \
./Core/Src/custom_classes/Convertor.o \
./Core/Src/custom_classes/Initializer.o \
./Core/Src/custom_classes/MavlinkControl.o \
./Core/Src/custom_classes/SBUS.o \
./Core/Src/custom_classes/components.o \
./Core/Src/custom_classes/debugger.o \
./Core/Src/custom_classes/handlers.o 

CPP_DEPS += \
./Core/Src/custom_classes/Convertor.d \
./Core/Src/custom_classes/Initializer.d \
./Core/Src/custom_classes/MavlinkControl.d \
./Core/Src/custom_classes/SBUS.d \
./Core/Src/custom_classes/components.d \
./Core/Src/custom_classes/debugger.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/custom_classes/%.o Core/Src/custom_classes/%.su Core/Src/custom_classes/%.cyclo: ../Core/Src/custom_classes/%.cpp Core/Src/custom_classes/subdir.mk
	arm-none-eabi-g++ "$<" -mcpu=cortex-m4 -std=gnu++14 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32G484xx -c -I../Core/Inc -I../Drivers/STM32G4xx_HAL_Driver/Inc -I../Drivers/STM32G4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32G4xx/Include -I../Drivers/CMSIS/Include -I../USB_Device/App -I../USB_Device/Target -I../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -I../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -O0 -ffunction-sections -fdata-sections -fno-exceptions -fno-rtti -fno-use-cxa-atexit -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Core/Src/custom_classes/%.o Core/Src/custom_classes/%.su Core/Src/custom_classes/%.cyclo: ../Core/Src/custom_classes/%.c Core/Src/custom_classes/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32G484xx -c -I../Core/Inc -I../Drivers/STM32G4xx_HAL_Driver/Inc -I../Drivers/STM32G4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32G4xx/Include -I../Drivers/CMSIS/Include -I../USB_Device/App -I../USB_Device/Target -I../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -I../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -I"C:/Users/bartc/STM32CubeIDE/workspace_2/Altum/Mavlink_v2" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src-2f-custom_classes

clean-Core-2f-Src-2f-custom_classes:
	-$(RM) ./Core/Src/custom_classes/Convertor.cyclo ./Core/Src/custom_classes/Convertor.d ./Core/Src/custom_classes/Convertor.o ./Core/Src/custom_classes/Convertor.su ./Core/Src/custom_classes/Initializer.cyclo ./Core/Src/custom_classes/Initializer.d ./Core/Src/custom_classes/Initializer.o ./Core/Src/custom_classes/Initializer.su ./Core/Src/custom_classes/MavlinkControl.cyclo ./Core/Src/custom_classes/MavlinkControl.d ./Core/Src/custom_classes/MavlinkControl.o ./Core/Src/custom_classes/MavlinkControl.su ./Core/Src/custom_classes/SBUS.cyclo ./Core/Src/custom_classes/SBUS.d ./Core/Src/custom_classes/SBUS.o ./Core/Src/custom_classes/SBUS.su ./Core/Src/custom_classes/components.cyclo ./Core/Src/custom_classes/components.d ./Core/Src/custom_classes/components.o ./Core/Src/custom_classes/components.su ./Core/Src/custom_classes/debugger.cyclo ./Core/Src/custom_classes/debugger.d ./Core/Src/custom_classes/debugger.o ./Core/Src/custom_classes/debugger.su ./Core/Src/custom_classes/handlers.cyclo ./Core/Src/custom_classes/handlers.d ./Core/Src/custom_classes/handlers.o ./Core/Src/custom_classes/handlers.su

.PHONY: clean-Core-2f-Src-2f-custom_classes

