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

void KalmanFilter_set(KalmanFilter *c, int i);
int KalmanFilter_get(KalmanFilter *c);
void deleteKalmanFilter(KalmanFilter* c);


#ifdef __cplusplus
}
#endif

#endif /* INC_CWRAPPER_KALMANFILTER_H_ */
