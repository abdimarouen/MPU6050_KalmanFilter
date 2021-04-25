/*
 * kalmanFilter.h
 *
 *  Created on: Apr 20, 2021
 *      Author: abdim
 */

#ifndef INC_KALMANFILTER_H_
#define INC_KALMANFILTER_H_


class KalmanFilter{
private:
	int m_i;
	float pkRoll[5];
	float pk1MinusRoll[5];
	float pk1Roll[5];
	float deltaT;
	float xkRoll[3];
	float xk1MinusRoll[3];
	float xk1Roll[3];
	float K[3];
	float phi[5];
	float psi[3];
	float I[5];
	float R;
	float Q[5];
	float H[3];
	float S, ukRoll, zkRoll, nuRoll;
public:
	KalmanFilter();
	float getAngle(float Angle_Acc, float Gyro_Vel, float Angle_Correction, float Angle_Gyro_Correction, float dt);
	void set_Angle(float Angle);
	void set(int i);
	int get();

};


#endif /* INC_KALMANFILTER_H_ */
