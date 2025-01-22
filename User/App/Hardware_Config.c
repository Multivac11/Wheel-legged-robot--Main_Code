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
#include "Chassis_R.h"
#include "Chassis_L.h"
#include "Facial_expression.h"

Clock task_clk;
void HardwareConfig(void) {

    FDCAN1_Config();
    FDCAN2_Config();
    CH010_HI91_Init();
    ELRS_Init();
    Facial_expression_init();

    ChassisR_init(&chassis_move,&right);
    HAL_Delay(50);
    ChassisL_init(&chassis_move,&left);
    HAL_Delay(50);

    HAL_TIM_Base_Start_IT(&htim14);


}