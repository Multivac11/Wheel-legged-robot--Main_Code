//
// Created by w1445 on 2025/1/14.
//

#ifndef BALANCE_LEG_V2_CHASSIS_L_H
#define BALANCE_LEG_V2_CHASSIS_L_H

#include "Chassis_L.h"
#include "Chassis_R.h"
#include "VMC_calc.h"
#include "pid.h"

void ChassisL_init(chassis_t *chassis,vmc_leg_t *vmc);
void ChassisL_task(void);
void chassisL_feedback_update(chassis_t *chassis,vmc_leg_t *vmc);
void chassisL_control(chassis_t *chassis,vmc_leg_t *vmcl,HI91_T *hi91,double *LQR_K,BasePID_Object LegL_pid);

extern vmc_leg_t left;
extern uint8_t left_flag;

#endif //BALANCE_LEG_V2_CHASSIS_L_H
