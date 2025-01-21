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
#include "CH010_HI91.h"


double LQR_K_R[12]={
        -4.4394,   -0.513  , -1.3493,   -1.3052,    2.0990  ,  0.2698,
        3.1791,   0.3891  ,  1.5877  ,  1.4070  , 13.3789 ,   0.6359};

double Poly_Coefficient[12][4]=   //效果不错，机体质量修改theta d_theta x d_x phi d_phi% 10 1 100 600 4000 1，R=[90 0;0 4];需测试
        {	{-12.754233579154,49.451845973124,-54.121403442394,1.444888977743},
             {13.840279580414,-11.560209227245,-3.769781997109,0.192734316129},
             {-19.049110705715,21.943739099113,-9.202832820038,0.461468782576},
             {-50.450450430310,58.827081665645,-25.440098858530,1.227025757606},
             {26.883639964520,-18.366035264945,-0.032618390796,2.775325808981},
             {4.337046053043,-3.629858164124,0.647586054149,0.318205794621},
             {647.841560581122,-629.401390739836,190.561502624943,3.894888193731},
             {61.796391091738,-70.762990931054,29.283332387461,-0.202373575774},
             {41.417081960371,-22.290117887067,-5.717759106101,5.629587146511},
             {118.540470280731,-67.470677592748,-12.895256038844,15.345722596133},
             {204.624901891231,-242.104689990221,105.578062837527,-7.998132536264},
             {17.776277612236,-23.451003677829,11.579269593512,-1.518344956083}
        };

vmc_leg_t right;

chassis_t chassis_move;

BasePID_Object Tp_pid;//防劈叉补偿pd
BasePID_Object Turn_pid;//转向pd
BasePID_Object LegR_pid;//右腿的腿长pd
BasePID_Object RollR_Pid;//ROLL轴补偿pid

uint32_t CHASSR_TIME = 1;

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
    chassisR_feedback_update(&chassis_move,&right);//更新数据
    chassisR_control(&chassis_move,&right,&hi91_data,LQR_K_R,Tp_pid,Turn_pid,LegR_pid,RollR_Pid);//控制计算
}

void chassisR_feedback_update(chassis_t *chassis,vmc_leg_t *vmc)
{
    vmc->phi1 = (A1_Motor[A1_Motor_right_1].motor_recv.original_Pos + 197) * (PI/180);
    vmc->phi4 = (A1_Motor[A1_Motor_right_2].motor_recv.original_Pos - 65) * (PI/180);

    chassis->myPithR = hi91_data.pitch * (PI/180);
    chassis->myPithGyroR = hi91_data.gyr[0] * (PI/180);

    chassis->total_yaw = hi91_data.yaw * (PI/180);
    chassis->roll = hi91_data.roll * (PI/180);
    chassis->theta_err=0.0f-(vmc->theta+left.theta);
}

void chassisR_control(chassis_t *chassis,vmc_leg_t *vmcr,HI91_T *hi91,double *LQR_K,BasePID_Object Tp_pid,BasePID_Object Turn_pid,BasePID_Object LegR_pid,BasePID_Object RollR_Pid)
{
    VMC_calc_1_right(vmcr,hi91,((float)CHASSR_TIME)*1.0f/1000.0f);//计算theta和d_theta给lqr用，同时也计算右腿长L0,该任务控制周期是1*0.001秒
}
