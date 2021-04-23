/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : mpu6050.h
  * @brief          : Header for mpu6050.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  *Both Accelerometer and Gyroscope are configured to 1 KHz frequency
  *
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MPU6050_H
#define __MPU6050_H

#ifdef __cplusplus
extern "C" {
#endif

/* USER CODE INCLUDE */
#include "stdint.h"
#include "math.h"
#include "main.h"


/* Gyro Acc scale selector -----------------------------------------------*/
#define _Gyro_scale_250
#define _Acc_scale_2g
#define _Calib_Gero 1000 // how many time will the gyro be eterate to get the offset
#define _Acc_Calib_time_ms 5000 //define  how much time to switch between axes to calibrate them
#define _Led_time_Calib_Blink_time_ms 500

/* USER CODE BEGIN REGISTER DEFINITION */

#define SLAVE_ADDRESS_MPU6050 0x68//0x68 // (SLAVE_ADDRESS_MPU6050 << 1) = 0xD0 // 0b11010000
#define MPU6050_WHO_AM_I_REG 0x75

#define PWR_MGMT_1 0x6B			//Power Management
#define SMPLRT_DIV_REG 0x19		//SMPRT_DIV : Sample Rate Divider
#define GYRO_CONFIG 0x1B 		//GYRO_CONFIG
#define ACCEL_CONFIG 0x1C		//ACCEL_CONFIG
#define SIGNAL_PATH_RESET 0x68	//SIGNAL_PATH_RESET : to Set GYRO_RESET = ACCEL_RESET = TEMP_RESET = 1 (register SIGNAL_PATH_RESET)

#define ACCEL_XOUT_H 0x3B		//Accelerometer output data register
#define GYRO_XOUT_H 0x43		//Gyroscope output data register




/* USER CODE END REGISTER DEFINITION */

/* Private constant definition -----------------------------------------------*/

#define	_8MHz_oscillator 0x00	//internal 8MHz oscillator, gyroscope based clock
#define	_SMPLRT_DIV_7 0x07		//Sample Rate = Gyroscope Output Rate / (1 + SMPLRT_DIV)
#define _Reset_ 0x80			//reset MPU
#define _GyroAcc_reset 0x7		//_GyroAcc_reset

#define _Gyro_Scale 0x00		//0x00 250 -- 0x01 500-- 0x02 1000-- 0x03 2000 °/second
#define _Acc_Scale 0x00			//0x00 2g -- 0x01 4g-- 0x02 8g-- 0x03 16g

#define _Wake_up 0x00


/* -----------------------------------------------------------------------*/

/* Gyro scales -----------------------------------------------*/
#ifdef _Gyro_scale_250
#define _Gyro_scale 0x00
#define _Gyro_scale_div 131.f
#endif

#ifdef _Gyro_scale_500
#define _Gyro_scale 0x01
#define _Gyro_scale_div 65.5f
#endif

#ifdef _Gyro_scale_1000
#define _Gyro_scale 0x02
#define _Gyro_scale_div 32.8f
#endif

#ifdef _Gyro_scale_2000
#define _Gyro_scale 0x03
#define _Gyro_scale_div 16.4f
#endif

/* Acc scales -----------------------------------------------*/

#ifdef _Acc_scale_2g
#define _Acc_scale 0x00
#define _Acc_scale_div 16384.f //LSB/g
#endif

#ifdef _Acc_scale_4g
#define _Acc_scale 0x01
#define _Acc_scale_div 8192.f //LSB/g
#endif

#ifdef _Acc_scale_8g
#define _Acc_scale 0x02
#define _Acc_scale_div 4096.f //LSB/g
#endif

#ifdef _Acc_scale_16g
#define _Acc_scale 0x03
#define _Acc_scale_div 2048.f //LSB/g
#endif


/* Private variable ----------------------------------------------------------*/



/* Private function prototypes -----------------------------------------------*/

int MPU6050_Init (void);
void MPU6050_GET_ACC (float* Ax, float* Ay, float* Az); //Get ACC Data
void MPU6050_GET_GYRO (float* Gx, float* Gy, float* Gz); //Get Gyro Data
void MPU6050_ACC_CALIB(void);
void MPU6050_GYRO_CALIB(void);
void _get_acc_angle(float* PITCH, float* ROLL);

float map(float x, float in_min, float in_max, float out_min, float out_max);

#ifdef __cplusplus
}
#endif

#endif /* __MPU6050_H */
