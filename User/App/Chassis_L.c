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
#include "Chassis_R.h"
#include "fdcan.h"
#include "VMC_calc.h"
#include "A1_Motor.h"


vmc_leg_t left;
BasePID_Object LegL_pid;//左腿的腿长pd
uint32_t CHASSL_TIME=1;

double LQR_K_L[12]={
        -4.4394,   -0.513  , -1.3493,   -1.3052,    2.0990  ,  0.2698,
        3.1791,   0.3891  ,  1.5877  ,  1.4070  , 13.3789 ,   0.6359};

void ChassisL_init(chassis_t *chassis,vmc_leg_t *vmc)
{
    A1_Motor_Init(A1_Motor_left_1);
    A1_Motor_Init(A1_Motor_left_2);

    VMC_init(vmc);//给杆长赋值

    BasePID_Init(&LegL_pid,350.0f , 0 ,3000.0f, 0);
}

void ChassisL_task(void)
{
    chassisL_feedback_update(&chassis_move,&left);//更新数据
    chassisL_control(&chassis_move,&left,&hi91_data,LQR_K_L,LegL_pid);//控制计算
}

void chassisL_feedback_update(chassis_t *chassis,vmc_leg_t *vmc)
{
    vmc->phi1 = (A1_Motor[A1_Motor_left_2].motor_recv.original_Pos + 195) * (PI/180);
    vmc->phi4 = (A1_Motor[A1_Motor_left_1].motor_recv.original_Pos - 52) * (PI/180);

    chassis->myPithL = 0.0f - hi91_data.pitch * (PI/180);
    chassis->myPithGyroL = 0.0f - hi91_data.gyr[0] * (PI/180);
}

void chassisL_control(chassis_t *chassis,vmc_leg_t *vmcl,HI91_T *hi91,double *LQR_K,BasePID_Object LegL_pid)
{
    VMC_calc_1_left(vmcl,hi91,((float)CHASSL_TIME)*1.0f/1000.0f);//计算theta和d_theta给lqr用，同时也计算左腿长L0,该任务控制周期是1*0.001秒
}

