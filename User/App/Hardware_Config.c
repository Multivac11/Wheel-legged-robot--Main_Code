//
// Created by w1445 on 2024/11/25.
//

#include "Hardware_Config.h"
#include "can_bsp.h"
#include "Motor_Dji.h"
#include "ELRS_Drive.h"
#include "CH010_HI91.h"
#include "Control_logic.h"
#include "tim.h"
#include "A1_Motor.h"

Clock task_clk;
void HardwareConfig(void) {
    FDCAN1_Config();
    FDCAN2_Config();
    CH010_HI91_Init();
    ELRS_Init();
    A1_Motor_Init(A1_Motor_left_1);
    A1_Motor_Init(A1_Motor_left_2);
    A1_Motor_Init(A1_Motor_right_1);
    A1_Motor_Init(A1_Motor_right_2);
    HAL_Delay(50);
    HAL_TIM_Base_Start_IT(&htim14);


}