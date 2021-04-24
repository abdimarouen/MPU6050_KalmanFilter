/*
 * CWrapper_KalmanFilter.h
 *
 *  Created on: Apr 20, 2021
 *      Author: abdim
 */

#ifndef INC_CWRAPPER_KALMANFILTER_H_
#define INC_CWRAPPER_KALMANFILTER_H_

#ifdef __cplusplus
extern "C" {
#endif

typedef struct KalmanFilter KalmanFilter;

KalmanFilter* newKalmanFilter();

float KalmanFilter_getAngle(KalmanFilter *c, float Angle_Acc, float Gyro_Vel, float Angle_Gyro_Correction, float Angle_Correction, float dt);
void KalmanFilter_set_Angle(KalmanFilter *c, float Angle);
void KalmanFilter_set(KalmanFilter *c, int i);
int KalmanFilter_get(KalmanFilter *c);
void deleteKalmanFilter(KalmanFilter* c);


#ifdef __cplusplus
}
#endif

#endif /* INC_CWRAPPER_KALMANFILTER_H_ */
