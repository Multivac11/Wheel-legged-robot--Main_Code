//
// Created by w1445 on 2025/1/22.
//

#ifndef BALANCE_LEG_V2_OBSERVE_H
#define BALANCE_LEG_V2_OBSERVE_H

#include "observe.h"
#include "stdint.h"
#include "Chassis_L.h"
#include "main.h"
#include "kalman_filter.h"

extern void Observe_task(void);
extern void xvEstimateKF_Init(KalmanFilter_t *EstimateKF);
extern void xvEstimateKF_Update(KalmanFilter_t *EstimateKF ,float acc,float vel);

extern KalmanFilter_t vaEstimateKF;

#endif //BALANCE_LEG_V2_OBSERVE_H
