/*
 * kalmanFilter.cpp
 *
 *  Created on: Apr 20, 2021
 *      Author: abdim
 */

#include "kalmanFilter.h"
#include "math.h"

KalmanFilter::KalmanFilter(void):
	pkRoll {0, 0.5, 0, 0, 0.01},
	pk1MinusRoll {0, 0.5, 0, 0, 0.01},
	pk1Roll {0, 0.5, 0, 0, 0.01},
	deltaT {0.004},
	xkRoll {0, 0, 0},
	xk1MinusRoll {0, 0, 0},
	xk1Roll {0, 0, 0},
	K {0, 0, 0},
	phi {0, 1, deltaT, 0, 1},
	psi {0, deltaT, 0},
	I {0, 1, 0, 0, 1},
	R {0.01},
	Q {0, 0.0002*0.0002, 0, 0, 0.0001*0.0001},
	H {0, 1, 0}
{}

/* KalmanFilter::getAngle:
 * Angle_Acc : Accelerometer Angle to estimate
 * Gyro_Vel  : Correspondent Gyroscope Velocity
 * dt        : Integration Time Period
 * if we estimate Roll => Angle_Gyro_Correction = Gyro_Yaw_Angle && Angle_Correction = xkPitch[1]: estimated Pitch Angle ::oth init to 1 if not used
 *
 */

float KalmanFilter::getAngle(float Angle_Acc, float Gyro_Vel, float Angle_Gyro_Correction, float Angle_Correction, float dt)
{
	ukRoll = Gyro_Vel;//gyro_y / 131.0f;
	zkRoll = Angle_Acc;
	deltaT = dt;

	//Step 1. Propagate the state and covariance from  k -> k+1
	// pk1Minus = phi x xk + psi x uk
	// pk1Minus = phi x pk x phiT +Q

	xk1MinusRoll[1] = phi[1] * xkRoll[1] + phi[2] * xkRoll[2] + psi[1] * ukRoll;
	xk1MinusRoll[2] = phi[3] * xkRoll[1] + phi[4] * xkRoll[2] + psi[2] * ukRoll;

	pk1MinusRoll[1] = (phi[1] * pkRoll[1] + phi[2] * pkRoll[3]) * phi[1] + (phi[1] * pkRoll[2] + phi[2] * pkRoll[4]) * phi[2] + Q[1];
	pk1MinusRoll[2] = (phi[1] * pkRoll[1] + phi[2] * pkRoll[3]) * phi[3] + (phi[1] * pkRoll[2] + phi[2] * pkRoll[4]) * phi[4] + Q[2];
	pk1MinusRoll[3] = (phi[3] * pkRoll[1] + phi[4] * pkRoll[3]) * phi[1] + (phi[3] * pkRoll[2] + phi[4] * pkRoll[4]) * phi[2] + Q[3];
	pk1MinusRoll[4] = (phi[3] * pkRoll[1] + phi[4] * pkRoll[3]) * phi[3] + (phi[3] * pkRoll[2] + phi[4] * pkRoll[4]) * phi[4] + Q[4];//Q[3]

	//correct Roll for Yaw movement
	xk1MinusRoll[1] = xk1MinusRoll[1] + Angle_Correction * sin(Angle_Gyro_Correction);
	//Angle_Gyro_Correction = gyro_z * 0.000001066f :: 0.000001066f = rad to deg conversion :: = 0.0000166 * (3.142 / 180) note : 0.0000166 = (1/250Hz)/65.5 ->> 250Hz integration time and 65.5 divider of gyro row data

	//Step 2. Calculate Kalman gain
	// S = H * pk1Minus * HT + R
	// KK1 = pk1Minus * HT *inv(S)

	S = (H[1] * pk1MinusRoll[1] + H[2] * pk1MinusRoll[3]) * H[1] + (H[1] * pk1MinusRoll[2] + H[2] * pk1MinusRoll[4]) * H[2] + R;//H[1]

	K[1] = (pk1MinusRoll[1] * H[1] + pk1MinusRoll[2] * H[2]) / S;
	K[2] = (pk1MinusRoll[3] * H[1] + pk1MinusRoll[4] * H[2]) / S;

	//Step 3. Calculate state and covariance updates
	// xk1 = xk1Minus + KK1 * (zk - H * xk1Minus)
	// pk1 = (I - KK1 * H) * pk1Minus

	nuRoll = zkRoll - (H[1] * xk1MinusRoll[1] + H[2] * xk1MinusRoll[2]);

	xk1Roll[1] = xk1MinusRoll[1] + K[1] * nuRoll;
	xk1Roll[2] = xk1MinusRoll[2] + K[2] * nuRoll;

	pk1Roll[1] = (1 - K[1] * H[1]) * pk1MinusRoll[1] + (0 - K[1] * H[2]) * pk1MinusRoll[3];
	pk1Roll[2] = (1 - K[1] * H[1]) * pk1MinusRoll[2] + (0 - K[1] * H[2]) * pk1MinusRoll[4];
	pk1Roll[3] = (0 - K[2] * H[1]) * pk1MinusRoll[1] + (1 - K[2] * H[2]) * pk1MinusRoll[3];
	pk1Roll[4] = (0 - K[2] * H[1]) * pk1MinusRoll[2] + (1 - K[2] * H[2]) * pk1MinusRoll[4];

	xkRoll[1] = xk1Roll[1];
	xkRoll[2] = xk1Roll[2];

	pkRoll[1] = pk1Roll[1];
	pkRoll[2] = pk1Roll[2];
	pkRoll[3] = pk1Roll[3];
	pkRoll[4] = pk1Roll[4];

	return xkRoll[1];
}


void KalmanFilter::set_Angle(float Angle)
{
	xkRoll[1] = Angle;
}

void KalmanFilter::set(int i)
{
	m_i = i;
}

int KalmanFilter::get()
{
	return m_i;
}

