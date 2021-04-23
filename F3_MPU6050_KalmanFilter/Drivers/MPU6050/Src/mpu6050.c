/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : mpu6050.c
  * @brief          : mpu6050 basic driver
  ******************************************************************************
  * @attention
  *
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "mpu6050.h"

float XplusCalib = -0.950927734f;
float XminusCalib = 1.06518555f;
float YplusCalib = -1.0012207f;
float YminusCalib = 0.986572266f;
float ZplusCalib = -1.19116211f;
float ZminusCalib = 0.828125f;

float XGeroCalib = 0.f;
float YGeroCalib = 0.f;
float ZGeroCalib = 0.f;

int MPU6050_Init (void)
{
	uint8_t check, data;

	/* MPU6050 start init sequence */

	HAL_Delay(100);
	data = _Reset_;
	HAL_I2C_Mem_Write(&hi2c1, (SLAVE_ADDRESS_MPU6050 << 1), PWR_MGMT_1, 1, &data, 1, 1000); 			//reset all
	HAL_Delay(100);
	data = _GyroAcc_reset;
	HAL_I2C_Mem_Write(&hi2c1, (SLAVE_ADDRESS_MPU6050 << 1), SIGNAL_PATH_RESET, 1, &data, 1, 1000); 		//_GyroAcc_reset
	HAL_Delay(100);

	/* MPU6050 end init sequence */

	HAL_I2C_Mem_Read(&hi2c1, (SLAVE_ADDRESS_MPU6050 << 1), MPU6050_WHO_AM_I_REG, 1, &check, 1, 1000); 	//check MPU6050_WHO_AM_I_REG
	HAL_Delay(100);

    if (check == 0x68)
    {
        data = _Wake_up;
        HAL_I2C_Mem_Write(&hi2c1, (SLAVE_ADDRESS_MPU6050 << 1), PWR_MGMT_1, 1, &data, 1, 1000); 		//Wakeup MPU
        HAL_Delay(100);

        data = _SMPLRT_DIV_7;
        HAL_I2C_Mem_Write(&hi2c1, (SLAVE_ADDRESS_MPU6050 << 1), SMPLRT_DIV_REG, 1, &data, 1, 1000); 	//choose rate
        HAL_Delay(100);
        data = _Acc_scale;
        HAL_I2C_Mem_Write(&hi2c1, (SLAVE_ADDRESS_MPU6050 << 1), ACCEL_CONFIG, 1, &data, 1, 1000);		//ACC 2g init_config
        HAL_Delay(100);
        data = _Gyro_scale;
        HAL_I2C_Mem_Write(&hi2c1, (SLAVE_ADDRESS_MPU6050 << 1), GYRO_CONFIG, 1, &data, 1, 1000);		//Gero 250dps init_config
        HAL_Delay(100);



        HAL_GPIO_WritePin(LD6_GPIO_Port, LD6_Pin, GPIO_PIN_SET);

        return 0;

    }
    else {
    	HAL_GPIO_WritePin(LD6_GPIO_Port, LD6_Pin, GPIO_PIN_SET);
        HAL_Delay(100);
        HAL_GPIO_WritePin(LD6_GPIO_Port, LD6_Pin, GPIO_PIN_RESET);

        return 1;
    }
}

void MPU6050_GET_ACC (float* Ax, float* Ay, float* Az) //Get ACC Data
{
    uint8_t data[6];
    int16_t Accel_X_RAW, Accel_Y_RAW, Accel_Z_RAW;

    HAL_I2C_Mem_Read(&hi2c1, (SLAVE_ADDRESS_MPU6050 << 1), ACCEL_XOUT_H, 1, &data[0], 6, 1000);
    Accel_X_RAW = (int16_t)((data[0] << 8) | data[1]);
    Accel_Y_RAW = (int16_t)((data[2] << 8) | data[3]);
    Accel_Z_RAW = (int16_t)((data[4] << 8) | data[5]);

    *Ax = map(Accel_X_RAW / _Acc_scale_div, XminusCalib, XplusCalib, -0.98, 0.98);
    *Ay = map(Accel_Y_RAW / _Acc_scale_div, YminusCalib, YplusCalib, -0.98, 0.98);
    *Az = map(Accel_Z_RAW / _Acc_scale_div, ZminusCalib, ZplusCalib, -0.98, 0.98);
}

void MPU6050_GET_GYRO (float* Gx, float* Gy, float* Gz) //Get Gyro Data
{
    uint8_t data[6];
    int16_t GERO_X_RAW, GERO_Y_RAW, GERO_Z_RAW;

    HAL_I2C_Mem_Read(&hi2c1, (SLAVE_ADDRESS_MPU6050 << 1), GYRO_XOUT_H, 1, &data[0], 6, 1000);
    GERO_X_RAW = (int16_t)((data[0] << 8) | data[1]);
    GERO_Y_RAW = (int16_t)((data[2] << 8) | data[3]);
    GERO_Z_RAW = (int16_t)((data[4] << 8) | data[5]);

    *Gx = (GERO_X_RAW / _Gyro_scale_div) - XGeroCalib;
    *Gy = (GERO_Y_RAW / _Gyro_scale_div) - YGeroCalib;
    *Gz = (GERO_Z_RAW / _Gyro_scale_div) - ZGeroCalib;
}

void MPU6050_GET_NO_CAILB_ACC (float* Ax, float* Ay, float* Az) //Get ACC Data
{
    uint8_t data[6];
    int16_t Accel_X_RAW, Accel_Y_RAW, Accel_Z_RAW;

    HAL_I2C_Mem_Read(&hi2c1, (SLAVE_ADDRESS_MPU6050 << 1), ACCEL_XOUT_H, 1, &data[0], 6, 1000);
    Accel_X_RAW = (int16_t)((data[0] << 8) | data[1]);
    Accel_Y_RAW = (int16_t)((data[2] << 8) | data[3]);
    Accel_Z_RAW = (int16_t)((data[4] << 8) | data[5]);

    *Ax = Accel_X_RAW / _Acc_scale_div;
    *Ay = Accel_Y_RAW / _Acc_scale_div;
    *Az = Accel_Z_RAW / _Acc_scale_div;
}

void MPU6050_GET_NO_CAILB_GYRO (float* Gx, float* Gy, float* Gz) //Get Gyro Data
{
    uint8_t data[6];
    int16_t GERO_X_RAW, GERO_Y_RAW, GERO_Z_RAW;

    HAL_I2C_Mem_Read(&hi2c1, (SLAVE_ADDRESS_MPU6050 << 1), GYRO_XOUT_H, 1, &data[0], 6, 1000);
    GERO_X_RAW = (int16_t)((data[0] << 8) | data[1]);
    GERO_Y_RAW = (int16_t)((data[2] << 8) | data[3]);
    GERO_Z_RAW = (int16_t)((data[4] << 8) | data[5]);

    *Gx = GERO_X_RAW / _Gyro_scale_div;
    *Gy = GERO_Y_RAW / _Gyro_scale_div;
    *Gz = GERO_Z_RAW / _Gyro_scale_div;
}

void MPU6050_ACC_CALIB(void)
{
	//X+ - X- ** Y+ - Y- ** Z+ - Z-
	float Ax,Ay,Az;


	//X down-------------------------------------------------------------*/
	HAL_GPIO_WritePin(LD5_GPIO_Port, LD5_Pin, GPIO_PIN_SET);
    HAL_Delay(_Acc_Calib_time_ms);

    MPU6050_GET_NO_CAILB_ACC(&Ax, &Ay, &Az);
	XplusCalib = Ax;

	//X up---------------------------------------------------------------*/
	HAL_GPIO_WritePin(LD5_GPIO_Port, LD5_Pin, GPIO_PIN_RESET);
    HAL_Delay(_Led_time_Calib_Blink_time_ms);
    HAL_GPIO_WritePin(LD5_GPIO_Port, LD5_Pin, GPIO_PIN_SET);
    HAL_Delay(_Acc_Calib_time_ms);

    MPU6050_GET_NO_CAILB_ACC(&Ax, &Ay, &Az);
	XminusCalib = Ax;

	HAL_GPIO_WritePin(LD5_GPIO_Port, LD5_Pin, GPIO_PIN_RESET);
    HAL_Delay(_Led_time_Calib_Blink_time_ms);
	//Y down-------------------------------------------------------------*/
	HAL_GPIO_WritePin(LD5_GPIO_Port, LD5_Pin, GPIO_PIN_SET);
    HAL_Delay(_Acc_Calib_time_ms);

    MPU6050_GET_NO_CAILB_ACC(&Ax, &Ay, &Az);
	YplusCalib = Ay;

	//Y up---------------------------------------------------------------*/
	HAL_GPIO_WritePin(LD5_GPIO_Port, LD5_Pin, GPIO_PIN_RESET);
    HAL_Delay(_Led_time_Calib_Blink_time_ms);
    HAL_GPIO_WritePin(LD5_GPIO_Port, LD5_Pin, GPIO_PIN_SET);
    HAL_Delay(_Acc_Calib_time_ms);

    MPU6050_GET_NO_CAILB_ACC(&Ax, &Ay, &Az);
	YminusCalib = Ay;

	HAL_GPIO_WritePin(LD5_GPIO_Port, LD5_Pin, GPIO_PIN_RESET);
    HAL_Delay(_Led_time_Calib_Blink_time_ms);
	//Z down-------------------------------------------------------------*/
	HAL_GPIO_WritePin(LD5_GPIO_Port, LD5_Pin, GPIO_PIN_SET);
    HAL_Delay(_Acc_Calib_time_ms);

    MPU6050_GET_NO_CAILB_ACC(&Ax, &Ay, &Az);
	ZplusCalib = Az;

	//Z up---------------------------------------------------------------*/
	HAL_GPIO_WritePin(LD5_GPIO_Port, LD5_Pin, GPIO_PIN_RESET);
    HAL_Delay(_Led_time_Calib_Blink_time_ms);
    HAL_GPIO_WritePin(LD5_GPIO_Port, LD5_Pin, GPIO_PIN_SET);
    HAL_Delay(_Acc_Calib_time_ms);

    MPU6050_GET_NO_CAILB_ACC(&Ax, &Ay, &Az);
	ZminusCalib = Az;

	HAL_GPIO_WritePin(LD5_GPIO_Port, LD5_Pin, GPIO_PIN_RESET);
    HAL_Delay(_Led_time_Calib_Blink_time_ms);

}

void MPU6050_GYRO_CALIB(void)
{
	float Gx, Gy, Gz;
	HAL_GPIO_WritePin(LD7_GPIO_Port, LD7_Pin, GPIO_PIN_SET);
	HAL_Delay(_Led_time_Calib_Blink_time_ms);
	for(int i = 1; i<= _Calib_Gero;i++)
	{
		MPU6050_GET_NO_CAILB_GYRO (&Gx, &Gy, &Gz);
		XGeroCalib += Gx;
		YGeroCalib += Gy;
		ZGeroCalib += Gz;
	}

	XGeroCalib /= _Calib_Gero;
	YGeroCalib /= _Calib_Gero;
	ZGeroCalib /= _Calib_Gero;
	HAL_GPIO_WritePin(LD7_GPIO_Port, LD7_Pin, GPIO_PIN_RESET);

}


void _get_acc_angle(float* PITCH, float* ROLL)
{
	float Accx, Accy, Accz;
	float acc_total_vector;

	MPU6050_GET_ACC (&Accx, &Accy, &Accz);

	acc_total_vector = sqrt((Accx*Accx) + (Accy*Accy)+(Accz*Accz));

	*PITCH = asin((float) Accy/acc_total_vector) * 57.2958f; //57,2958 for the conversion from radian to degree
	*ROLL = asin((float) Accx/acc_total_vector) * 57.2958f; //57,2958 for the conversion from radian to degree
}


float map(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}



