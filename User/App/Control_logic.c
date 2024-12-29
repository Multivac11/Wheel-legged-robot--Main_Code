//
// Created by w1445 on 2024/11/28.
//

#include "Control_logic.h"
#include "tim.h"
#include "Motor_Dji.h"
#include "fdcan.h"
#include "A1_Motor.h"
#include "CH010_HI91.h"
#include "motor_msg.h"
#include "usart.h"
#include "Check.h"

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if(htim == &htim14)
    {
        task_clk.tim14_clk++;
        Check_FPS();
        Check_Status();
//        set_moto_current(&hfdcan1,0x200,0,0);

//        Modfiy_Speed_Cmd(&A1_Motor[A1_Motor_left_1].motor_send, 1, 2.0f);
//        Modfiy_Speed_Cmd(&A1_Motor[A1_Motor_left_2].motor_send, 1, -0.5f);
//
//        A1_Motor_Send_Cmd(&A1_Motor[A1_Motor_left_1].motor_send, A1_Motor_left_1);
//        A1_Motor_Send_Cmd(&A1_Motor[A1_Motor_left_2].motor_send, A1_Motor_left_2);

    }
}