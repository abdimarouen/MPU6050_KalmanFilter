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


