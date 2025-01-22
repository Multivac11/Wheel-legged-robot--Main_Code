//
// Created by w1445 on 2025/1/13.
//

#ifndef BALANCE_LEG_V2_ELRS_TASK_H
#define BALANCE_LEG_V2_ELRS_TASK_H

#include "ELRS_task.h"
#include "ELRS_Drive.h"
#include "Chassis_R.h"

void ELRS_Task(ELRS_Data *elrs_data,chassis_t *chassis);
void jump_key(ELRS_Data *Elrs_data,chassis_t *chassis);
void slope_following(float *target,float *set,float acc);

#endif //BALANCE_LEG_V2_ELRS_TASK_H
