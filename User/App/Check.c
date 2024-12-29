//
// Created by w1445 on 2024/12/21.
//

#include "Check.h"
#include "A1_Motor.h"
#include "CH010_HI91.h"
#include "motor_msg.h"
#include "Control_logic.h"

/*各模块帧率检测*/
void Check_FPS(void)
{
    if(task_clk.tim14_clk%1000==0)
    {
        hi91_data.FPS = hi91_data.rx_counter;
        A1_Motor[A1_Motor_left_1].motor_recv.FPS = A1_Motor[A1_Motor_left_1].motor_recv.Rx_count;
        A1_Motor[A1_Motor_left_2].motor_recv.FPS = A1_Motor[A1_Motor_left_2].motor_recv.Rx_count;
        A1_Motor[A1_Motor_right_1].motor_recv.FPS = A1_Motor[A1_Motor_right_1].motor_recv.Rx_count;
        A1_Motor[A1_Motor_right_2].motor_recv.FPS = A1_Motor[A1_Motor_right_2].motor_recv.Rx_count;
        hi91_data.rx_counter = 0;
        A1_Motor[A1_Motor_left_1].motor_recv.Rx_count = 0;
        A1_Motor[A1_Motor_left_2].motor_recv.Rx_count = 0;
        A1_Motor[A1_Motor_right_1].motor_recv.Rx_count = 0;
        A1_Motor[A1_Motor_right_2].motor_recv.Rx_count = 0;
    }
}

/*状态指示*/
void Check_Status(void)
{
    if(task_clk.tim14_clk%500==0)
    {
        HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_9);
    }
}
