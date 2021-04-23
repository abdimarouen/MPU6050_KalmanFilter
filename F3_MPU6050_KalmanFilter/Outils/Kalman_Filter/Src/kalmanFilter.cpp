/*
 * kalmanFilter.cpp
 *
 *  Created on: Apr 20, 2021
 *      Author: abdim
 */

#include "kalmanFilter.h"

void KalmanFilter::set(int i)
{
	m_i = i;
}

int KalmanFilter::get()
{
	return m_i;
}

