################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../Inc/ROS/duration.cpp \
../Inc/ROS/time.cpp 

OBJS += \
./Inc/ROS/duration.o \
./Inc/ROS/time.o 

CPP_DEPS += \
./Inc/ROS/duration.d \
./Inc/ROS/time.d 


# Each subdirectory must supply rules for building sources it contributes
Inc/ROS/duration.o: ../Inc/ROS/duration.cpp
	arm-none-eabi-g++ "$<" -mcpu=cortex-m4 -std=gnu++14 -g3 -DUSE_HAL_DRIVER -DSTM32F446xx -DDEBUG -c -I../Inc -I"C:/Users/colson/Desktop/workspace/acsl_interface_board/stm32f4_rosserial/Inc/ROS" -I../Drivers/CMSIS/Include -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -O0 -ffunction-sections -fdata-sections -fno-exceptions -fno-rtti -fno-threadsafe-statics -fno-use-cxa-atexit -Wall -fstack-usage -MMD -MP -MF"Inc/ROS/duration.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Inc/ROS/time.o: ../Inc/ROS/time.cpp
	arm-none-eabi-g++ "$<" -mcpu=cortex-m4 -std=gnu++14 -g3 -DUSE_HAL_DRIVER -DSTM32F446xx -DDEBUG -c -I../Inc -I"C:/Users/colson/Desktop/workspace/acsl_interface_board/stm32f4_rosserial/Inc/ROS" -I../Drivers/CMSIS/Include -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -O0 -ffunction-sections -fdata-sections -fno-exceptions -fno-rtti -fno-threadsafe-statics -fno-use-cxa-atexit -Wall -fstack-usage -MMD -MP -MF"Inc/ROS/time.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

