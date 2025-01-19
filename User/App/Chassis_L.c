//
// Created by w1445 on 2025/1/14.
//

/**
  *     轮腿俯视图
  *                                   正面
  *     ID:1 usart3  A1_Motor_left_1        ID:2 usart4  A1_Motor_right_1
  *
  *  CAN1                                                               CAN2
  *
  *     ID:0 usart9  A1_Motor_left_2       ID:1 usart7   A1_Motor_right_2
  *
  *                                   背面
  *
  */

#include "Chassis_L.h"
#include "fdcan.h"
#include "VMC_calc.h"
#include "A1_Motor.h"

vmc_leg_t left;
BasePID_Object LegL_pid;//左腿的腿长pd
uint32_t CHASSL_TIME=1;

void ChassisL_init(chassis_t *chassis,vmc_leg_t *vmc)
{
    A1_Motor_Init(A1_Motor_left_1);
    A1_Motor_Init(A1_Motor_left_2);

    VMC_init(vmc);//给杆长赋值

    BasePID_Init(&LegL_pid,350.0f , 0 ,3000.0f, 0);
}

void ChassisL_task(void)
{

}

