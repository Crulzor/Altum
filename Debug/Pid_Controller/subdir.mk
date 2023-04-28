################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Pid_Controller/pid_controller.c 

C_DEPS += \
./Pid_Controller/pid_controller.d 

OBJS += \
./Pid_Controller/pid_controller.o 


# Each subdirectory must supply rules for building sources it contributes
Pid_Controller/%.o Pid_Controller/%.su Pid_Controller/%.cyclo: ../Pid_Controller/%.c Pid_Controller/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32G484xx -c -I../Core/Inc -I"C:/Users/bartc/STM32CubeIDE/workspace_2/Altum/Pid_Controller" -I../Drivers/STM32G4xx_HAL_Driver/Inc -I../Drivers/STM32G4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32G4xx/Include -I../Drivers/CMSIS/Include -I../USB_Device/App -I../USB_Device/Target -I../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -I../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -I"C:/Users/bartc/STM32CubeIDE/workspace_2/Altum/Mavlink_v2" -Os -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Pid_Controller

clean-Pid_Controller:
	-$(RM) ./Pid_Controller/pid_controller.cyclo ./Pid_Controller/pid_controller.d ./Pid_Controller/pid_controller.o ./Pid_Controller/pid_controller.su

.PHONY: clean-Pid_Controller

