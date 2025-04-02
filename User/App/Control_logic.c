//
// Created by w1445 on 2024/11/28.
//

#include "Control_logic.h"
#include "CH010_HI91.h"
#include "tim.h"
#include "Motor_Dji.h"
#include "fdcan.h"
#include "A1_Motor.h"
#include "CH010_HI91.h"
#include "motor_msg.h"
#include "usart.h"
#include "Check.h"
#include "ELRS_task.h"
#include "Chassis_R.h"
#include "Chassis_L.h"
#include "Facial_expression.h"
#include "VMC_calc.h"
#include "observe.h"

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if(htim == &htim14)
    {
        task_clk.tim14_clk ++;
        Check_FPS();
        Check_Status();
        Online_check();
        ELRS_Task(&elrs_data,&chassis_move);
        Get_total_yaw(&hi91_data);
        if(task_clk.tim14_clk % 2 == 0)
        {
            ChassisR_task();
            ChassisL_task();

        }

 //       Facial_expression_Control();
        if(task_clk.tim14_clk % 1 == 0)
        {
            Observe_task();
        }


        if(chassis_move.start_flag==1 && task_clk.tim14_clk % 2 == 0)
        {
            Set_moto_current(&hfdcan1,0x200,chassis_move.wheel_motor[1].para.Output);
            Set_moto_current(&hfdcan2,0x200,chassis_move.wheel_motor[0].para.Output);

            Modfiy_Torque_Cmd(&A1_Motor[A1_Motor_left_1].motor_send,A1_Motor_left_1_ID, A1_Motor[A1_Motor_left_1].motor_send.T);
            A1_Motor_Send_Cmd(&A1_Motor[A1_Motor_left_1].motor_send, A1_Motor_left_1);

            Modfiy_Torque_Cmd(&A1_Motor[A1_Motor_left_2].motor_send,A1_Motor_left_2_ID, A1_Motor[A1_Motor_left_2].motor_send.T);
            A1_Motor_Send_Cmd(&A1_Motor[A1_Motor_left_2].motor_send, A1_Motor_left_2);

            Modfiy_Torque_Cmd(&A1_Motor[A1_Motor_right_1].motor_send,A1_Motor_right_1_ID, A1_Motor[A1_Motor_right_1].motor_send.T);
            A1_Motor_Send_Cmd(&A1_Motor[A1_Motor_right_1].motor_send, A1_Motor_right_1);

            Modfiy_Torque_Cmd(&A1_Motor[A1_Motor_right_2].motor_send,A1_Motor_right_2_ID, A1_Motor[A1_Motor_right_2].motor_send.T);
            A1_Motor_Send_Cmd(&A1_Motor[A1_Motor_right_2].motor_send, A1_Motor_right_2);
//            Set_moto_current(&hfdcan1,0x200,0);
//            Set_moto_current(&hfdcan2,0x200,0);
//
//            //阻尼模式
//            Modfiy_Speed_Cmd(&A1_Motor[A1_Motor_left_1].motor_send, A1_Motor_left_1_ID, 0.0f);
//            A1_Motor_Send_Cmd(&A1_Motor[A1_Motor_left_1].motor_send, A1_Motor_left_1);
//
//            Modfiy_Speed_Cmd(&A1_Motor[A1_Motor_left_2].motor_send, A1_Motor_left_2_ID, 0.0f);
//            A1_Motor_Send_Cmd(&A1_Motor[A1_Motor_left_2].motor_send, A1_Motor_left_2);
//
//            Modfiy_Speed_Cmd(&A1_Motor[A1_Motor_right_1].motor_send, A1_Motor_right_1_ID, 0.0f);
//            A1_Motor_Send_Cmd(&A1_Motor[A1_Motor_right_1].motor_send, A1_Motor_right_1);
//
//            Modfiy_Speed_Cmd(&A1_Motor[A1_Motor_right_2].motor_send, A1_Motor_right_2_ID, 0.0f);
//            A1_Motor_Send_Cmd(&A1_Motor[A1_Motor_right_2].motor_send, A1_Motor_right_2);
        }
        else if(chassis_move.start_flag==0 && task_clk.tim14_clk % 2 == 0)
        {
            Set_moto_current(&hfdcan1,0x200,0);
            Set_moto_current(&hfdcan2,0x200,0);

            //阻尼模式
            Modfiy_Speed_Cmd(&A1_Motor[A1_Motor_left_1].motor_send, A1_Motor_left_1_ID, 0.0f);
            A1_Motor_Send_Cmd(&A1_Motor[A1_Motor_left_1].motor_send, A1_Motor_left_1);

            Modfiy_Speed_Cmd(&A1_Motor[A1_Motor_left_2].motor_send, A1_Motor_left_2_ID, 0.0f);
            A1_Motor_Send_Cmd(&A1_Motor[A1_Motor_left_2].motor_send, A1_Motor_left_2);

            Modfiy_Speed_Cmd(&A1_Motor[A1_Motor_right_1].motor_send, A1_Motor_right_1_ID, 0.0f);
            A1_Motor_Send_Cmd(&A1_Motor[A1_Motor_right_1].motor_send, A1_Motor_right_1);

            Modfiy_Speed_Cmd(&A1_Motor[A1_Motor_right_2].motor_send, A1_Motor_right_2_ID, 0.0f);
            A1_Motor_Send_Cmd(&A1_Motor[A1_Motor_right_2].motor_send, A1_Motor_right_2);

        }
    }
}