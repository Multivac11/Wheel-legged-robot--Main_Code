//
// Created by w1445 on 2025/1/13.
//

#include "ELRS_task.h"
#include "Chassis_R.h"
#include "ELRS_Drive.h"

void ELRS_Task(ELRS_Data *Elrs_data,chassis_t *chassis)
{
    if(Elrs_data->E == 1)
    {
        chassis->start_flag = 1;
    }
    else if(Elrs_data->E == 0)
    {
        chassis->start_flag = 0;
    }
}


