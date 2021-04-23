################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/MPU6050/Src/mpu6050.c 

C_DEPS += \
./Drivers/MPU6050/Src/mpu6050.d 

OBJS += \
./Drivers/MPU6050/Src/mpu6050.o 


# Each subdirectory must supply rules for building sources it contributes
Drivers/MPU6050/Src/mpu6050.o: ../Drivers/MPU6050/Src/mpu6050.c Drivers/MPU6050/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F303xC -c -I../Core/Inc -I../Drivers/STM32F3xx_HAL_Driver/Inc -I../Drivers/STM32F3xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F3xx/Include -I../Drivers/CMSIS/Include -I"D:/GIT_Master/MPU6050_KalmanFilter/F3_MPU6050_KalmanFilter/Drivers/MPU6050/Inc" -I"D:/GIT_Master/MPU6050_KalmanFilter/F3_MPU6050_KalmanFilter/Outils/Kalman_Filter/Inc" -I../Middlewares/ST/ARM/DSP/Inc -I"D:/GIT_Master/MPU6050_KalmanFilter/F3_MPU6050_KalmanFilter/Outils/CWrapper_KalmanFilter/Inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Drivers/MPU6050/Src/mpu6050.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

