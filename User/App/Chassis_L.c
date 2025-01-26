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
#include "arm_math.h"

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
    vmc->phi1 = ((A1_Motor[A1_Motor_left_2].motor_recv.original_Pos + 196) * PI/180);
    vmc->phi4 = ((A1_Motor[A1_Motor_left_1].motor_recv.original_Pos - 50) * PI/180);

    chassis->myPithL = (0.0f - hi91_data.pitch * PI/180);
    chassis->myPithGyroL = (0.0f - hi91_data.gyr[0] * PI/180);
}
uint8_t  left_flag;
void chassisL_control(chassis_t *chassis,vmc_leg_t *vmcl,HI91_T *hi91,double *LQR_K,BasePID_Object LegL_pid)
{
    VMC_calc_1_left(vmcl,hi91,((float)CHASSL_TIME)*1.0f/1000.0f);//计算theta和d_theta给lqr用，同时也计算左腿长L0,该任务控制周期是1*0.001秒

    for(int i=0;i<12;i++)
    {
        LQR_K[i]=LQR_K_calc(&Poly_Coefficient[i][0],vmcl->L0 );
    }

    chassis->wheel_motor[1].wheel_T = (float)(LQR_K[0]*(vmcl->theta-0.0f)
                                     +LQR_K[1]*(vmcl->d_theta-0.0f)
                                     +LQR_K[2]*(chassis->x_set-chassis->x_filter)
                                     +LQR_K[3]*(chassis->v_set-chassis->v_filter)
                                     +LQR_K[4]*(chassis->myPithL-0.0f)
                                     +LQR_K[5]*(chassis->myPithGyroL-0.0f));

    //右边髋关节输出力矩
    vmcl->Tp = (float)(LQR_K[6]*(vmcl->theta-0.0f)
              +LQR_K[7]*(vmcl->d_theta-0.0f)
              +LQR_K[8]*(chassis->x_set-chassis->x_filter)
              +LQR_K[9]*(chassis->v_set-chassis->v_filter)
              +LQR_K[10]*(chassis->myPithL-0.0f)
              +LQR_K[11]*(chassis->myPithGyroL-0.0f));

    vmcl->Tp = vmcl->Tp + chassis->leg_tp;//髋关节输出力矩

    vmcl->F0 = Mg/arm_cos_f32(vmcl->theta)+BasePID_PositionControl(&LegL_pid, chassis->leg_set_L , vmcl->L0)+chassis->now_roll_set ;//前馈+pd

    left_flag = ground_detectionL(vmcl);//左腿离地检测

    if(chassis->recover_flag==0)
    {//倒地自起不需要检测是否离地
        if(left_flag==1&&right_flag==1&&vmcl->leg_flag==0)
        {//当两腿同时离地并且遥控器没有在控制腿的伸缩时，才认为离地
            chassis->wheel_motor[1].wheel_T=0.0f;
            vmcl->Tp = (float)(LQR_K[6]*(vmcl->theta-0.0f)+ LQR_K[7]*(vmcl->d_theta-0.0f));

            chassis->x_filter=0.0f;//对位移清零
            chassis->x_set=chassis->x_filter;
            chassis->turn_set=chassis->total_yaw;
            vmcl->Tp=vmcl->Tp+chassis->leg_tp;
        }
        else
        {//没有离地
            vmcl->leg_flag=0;//置为0
        }
    }
    else if(chassis->recover_flag==1)
    {
        vmcl->Tp=0.0f;
    }

    mySaturate_f(&vmcl->F0,-150.0f,150.0f);//限幅

    VMC_calc_2(vmcl);//计算期望的关节输出力矩

    chassis->wheel_motor[1].wheel_T = chassis->wheel_motor[1].wheel_T-chassis->turn_T;	//轮毂电机输出力矩
    chassis->wheel_motor[1].para.Output = (int16_t )(-chassis->wheel_motor[1].wheel_T * 661504/201);
    mySaturate_i(&chassis->wheel_motor[1].para.Output,-16384,+16384);

    A1_Motor[A1_Motor_left_1].motor_send.T = vmcl->torque_set[1];
    A1_Motor[A1_Motor_left_2].motor_send.T = vmcl->torque_set[0];

    if(A1_Motor[A1_Motor_left_1].motor_send.T < -33.0f)
    {
        A1_Motor[A1_Motor_left_1].motor_send.T = -33.0f;
    }
    else if(A1_Motor[A1_Motor_left_1].motor_send.T > 33.0f)
    {
        A1_Motor[A1_Motor_left_1].motor_send.T = 33.0f;
    }

    if(A1_Motor[A1_Motor_left_2].motor_send.T < -33.0f)
    {
        A1_Motor[A1_Motor_left_2].motor_send.T = -33.0f;
    }
    else if(A1_Motor[A1_Motor_left_2].motor_send.T > 33.0f)
    {
        A1_Motor[A1_Motor_left_2].motor_send.T = 33.0f;
    }
//    mySaturate_f(&A1_Motor[A1_Motor_left_1].motor_send.T,-4.0f,4.0f);
//    mySaturate_f(&A1_Motor[A1_Motor_left_2].motor_send.T,-4.0f,4.0f);
}

