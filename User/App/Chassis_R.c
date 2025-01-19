//
// Created by w1445 on 2024/11/25.
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

#include "Chassis_R.h"
#include "fdcan.h"
#include "VMC_calc.h"
#include "Chassis_L.h"
#include "A1_Motor.h"

double LQR_K_R[12]={
        -4.4394,   -0.513  , -1.3493,   -1.3052,    2.0990  ,  0.2698,
        3.1791,   0.3891  ,  1.5877  ,  1.4070  , 13.3789 ,   0.6359};

double Poly_Coefficient[12][4]=   //效果不错，机体质量修改theta d_theta x d_x phi d_phi%10 1 100 600 4000 1，R=[90 0;0 4];需测试
        {	{112.845135511629,5.515604457792,-40.918030753879,0.418677700004},
             {44.290985054785,-20.882221333954,-2.778535762860,0.038115599263},
             {-78.990133493834,48.915985251273,-11.172441668133,-0.042482447001},
             {-200.832850036330,126.449034522089,-29.944482494393,-0.152592753436},
             {253.498745408067,-45.460379113788,-21.991592836902,7.750927333252},
             {22.619197049632,-8.725285037129,0.217815640035,0.486935173898},
             {1809.686118752799,-921.703834442367,152.975799233306,2.590199551292},
             {128.010302395005,-77.758254225739,17.815094580415,0.266897307102},
             {158.203345928741,-16.819621080958,-19.209991620949,5.445672337314},
             {387.130536671912,-33.065649075500,-52.059673407760,14.688608543190},
             {2645.381722616551,-1635.924975708334,376.325762023084,-3.121774065205},
             {37.576314134604,-45.652245635121,16.199071209702,-1.172542938133}
        };

vmc_leg_t right;

chassis_t chassis_move;

BasePID_Object Tp_pid;//防劈叉补偿pd
BasePID_Object Turn_pid;//转向pd
BasePID_Object LegR_pid;//右腿的腿长pd
BasePID_Object RollR_Pid;//ROLL轴补偿pid

uint32_t CHASSR_TIME=1;

void ChassisR_init(chassis_t *chassis,vmc_leg_t *vmc)
{
    A1_Motor_Init(A1_Motor_right_1);
    A1_Motor_Init(A1_Motor_right_2);

    VMC_init(vmc);//给杆长赋值

    BasePID_Init(&Tp_pid,6.0f , 0 ,0.5f, 0);
    BasePID_Init(&Turn_pid,2.0f , 0 ,0.5f, 0);
    BasePID_Init(&LegR_pid,350.0f , 0 ,3000.0f, 0);
//	BasePID_Init(&RollR_Pid,0.0f , 0 ,0, 0);
    BasePID_Init(&RollR_Pid,50.0f , 10 ,0, 0);
}

void ChassisR_task(void)
{

}

