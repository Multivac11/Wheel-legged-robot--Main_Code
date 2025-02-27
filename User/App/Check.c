//
// Created by w1445 on 2024/12/21.
//

#include "Check.h"
#include "A1_Motor.h"
#include "CH010_HI91.h"
#include "motor_msg.h"
#include "Control_logic.h"
#include "ELRS_Drive.h"

/**
  * @brief  各模块帧率检测
  */
void Check_FPS(void)
{
    if(task_clk.tim14_clk%1000==0)
    {
        hi91_data.FPS = hi91_data.rx_counter;
        elrs_data.FPS = elrs_data.rx_counter;
        A1_Motor[A1_Motor_left_1].motor_recv.FPS = A1_Motor[A1_Motor_left_1].motor_recv.Rx_count;
        A1_Motor[A1_Motor_left_2].motor_recv.FPS = A1_Motor[A1_Motor_left_2].motor_recv.Rx_count;
        A1_Motor[A1_Motor_right_1].motor_recv.FPS = A1_Motor[A1_Motor_right_1].motor_recv.Rx_count;
        A1_Motor[A1_Motor_right_2].motor_recv.FPS = A1_Motor[A1_Motor_right_2].motor_recv.Rx_count;
        hi91_data.rx_counter = 0;
        elrs_data.rx_counter = 0;
        A1_Motor[A1_Motor_left_1].motor_recv.Rx_count = 0;
        A1_Motor[A1_Motor_left_2].motor_recv.Rx_count = 0;
        A1_Motor[A1_Motor_right_1].motor_recv.Rx_count = 0;
        A1_Motor[A1_Motor_right_2].motor_recv.Rx_count = 0;
    }
}

/**
  * @brief  离线检测
  */
void Online_check(void)
{
    elrs_data.Online_counter ++;

    if(elrs_data.Online_counter < 10)
    {
        elrs_data.isOnline = 1;
    }
    else
    {
        elrs_data.isOnline = 0;
        if(elrs_data.isOnline == 0)
        {
            elrs_data.Right_X = 0;
            elrs_data.Right_Y = 0;
            elrs_data.Left_X = 0;
            elrs_data.Left_Y = 0;
            elrs_data.A = 0;
            elrs_data.B = 0;
            elrs_data.C = 0;
            elrs_data.D = 0;
            elrs_data.E = 0;
            elrs_data.F = 0;
            elrs_data.S1 = 0;
            elrs_data.S2 = 0;
        }
        HAL_GPIO_WritePin(GPIOE, GPIO_PIN_11, GPIO_PIN_RESET);
        ELRS_Init();
    }
}

/**
  * @brief  状态指示
  */
void Check_Status(void)
{
    if(task_clk.tim14_clk%500==0)
    {
        HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_9);
    }
}

