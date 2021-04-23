################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../Outils/CWrapper_KalmanFilter/Src/CWrapper_KalmanFilter.cpp 

OBJS += \
./Outils/CWrapper_KalmanFilter/Src/CWrapper_KalmanFilter.o 

CPP_DEPS += \
./Outils/CWrapper_KalmanFilter/Src/CWrapper_KalmanFilter.d 


# Each subdirectory must supply rules for building sources it contributes
Outils/CWrapper_KalmanFilter/Src/CWrapper_KalmanFilter.o: ../Outils/CWrapper_KalmanFilter/Src/CWrapper_KalmanFilter.cpp Outils/CWrapper_KalmanFilter/Src/subdir.mk
	arm-none-eabi-g++ "$<" -mcpu=cortex-m4 -std=gnu++14 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F303xC -c -I../Core/Inc -I"D:/GIT_Master/MPU6050_KalmanFilter/F3_MPU6050_KalmanFilter/Outils/CWrapper_KalmanFilter/Inc" -I../Drivers/STM32F3xx_HAL_Driver/Inc -I../Drivers/STM32F3xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F3xx/Include -I../Drivers/CMSIS/Include -I"D:/GIT_Master/MPU6050_KalmanFilter/F3_MPU6050_KalmanFilter/Drivers/MPU6050/Inc" -I"D:/GIT_Master/MPU6050_KalmanFilter/F3_MPU6050_KalmanFilter/Outils/Kalman_Filter/Inc" -I../Middlewares/ST/ARM/DSP/Inc -O0 -ffunction-sections -fdata-sections -fno-exceptions -fno-rtti -fno-use-cxa-atexit -Wall -fstack-usage -MMD -MP -MF"Outils/CWrapper_KalmanFilter/Src/CWrapper_KalmanFilter.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

