/*
 * CWrapper_KalmanFilter.cpp
 *
 *  Created on: Apr 20, 2021
 *      Author: abdim
 */

#include "kalmanFilter.h"
#include "CWrapper_KalmanFilter.h"

extern "C" {
	KalmanFilter* newKalmanFilter()
	{
		return new KalmanFilter();
	}

	float KalmanFilter_getAngle(KalmanFilter *c, float Angle_Acc, float Gyro_Vel, float Angle_Gyro_Correction, float Angle_Correction, float dt)
	{
		return c->getAngle(Angle_Acc, Gyro_Vel, Angle_Correction, Angle_Gyro_Correction, dt);
	}
	void KalmanFilter_set_Angle(KalmanFilter *c, float Angle)
	{
		c->set_Angle(Angle);
	}
	void KalmanFilter_set(KalmanFilter *c, int i)
	{
		c->set(i);
	}

	int KalmanFilter_get(KalmanFilter *c)
	{
		return c->get();
	}

	void deleteKalmanFilter(KalmanFilter* c)
	{
		delete c;
	}
}


