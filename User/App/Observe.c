//
// Created by w1445 on 2025/1/22.
//
/**
  *********************************************************************
  * @file      observe_task.c/h
  * @brief     该任务是对机体运动速度估计，用于抑制打滑
  * @note
  * @history
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  *********************************************************************
  */

#include "observe.h"
#include "kalman_filter.h"
#include "CH010_HI91.h"
#include "Chassis_R.h"
#include "VMC_calc.h"
#include "Chassis_L.h"

KalmanFilter_t vaEstimateKF;	   // 卡尔曼滤波器结构体

float vaEstimateKF_F[4] = {1.0f, 0.003f,
                           0.0f, 1.0f};	   // 状态转移矩阵，控制周期为0.001s

float vaEstimateKF_P[4] = {1.0f, 0.0f,
                           0.0f, 1.0f};    // 后验估计协方差初始值

float vaEstimateKF_Q[4] = {0.1f, 0.0f,
                           0.0f, 0.1f};    // Q矩阵初始值

float vaEstimateKF_R[4] = {100.0f, 0.0f,
                           0.0f,  100.0f};

float vaEstimateKF_K[4];

const float vaEstimateKF_H[4] = {1.0f, 0.0f,
                                 0.0f, 1.0f};	// 设置矩阵H为常量

extern chassis_t chassis_move;

extern vmc_leg_t right;
extern vmc_leg_t left;

float vel_acc[2];
uint32_t OBSERVE_TIME = 2;//任务周期是1ms

static float wr,wl=0.0f;
static float vrb,vlb=0.0f;
static float aver_v=0.0f;

void Observe_task(void)
{
    wr= (-chassis_move.wheel_motor[0].para.AngleSpeed-hi91_data.gyr[0]+right.d_alpha)*17/268;//右边驱动轮转子相对大地角速度，这里定义的是顺时针为正
    vrb = wr*0.0603f + right.L0*right.d_theta*arm_cos_f32(right.theta)+right.d_L0*arm_sin_f32(right.theta);//机体b系的速度

    wl= (-chassis_move.wheel_motor[1].para.AngleSpeed+hi91_data.gyr[0]+left.d_alpha)*17/268;//左边驱动轮转子相对大地角速度，这里定义的是顺时针为正
    vlb = wl*0.0603f + left.L0*left.d_theta*arm_cos_f32(left.theta)+left.d_L0*arm_sin_f32(left.theta);//机体b系的速度

    aver_v=(vrb-vlb)/2.0f;//取平均
    xvEstimateKF_Update(&vaEstimateKF,hi91_data.acc[0],aver_v);

    //原地自转的过程中v_filter和x_filter应该都是为0
//    chassis_move.v_filter = aver_v;//得到卡尔曼滤波后的速度
//    chassis_move.x_filter = chassis_move.x_filter+chassis_move.v_filter*((float)OBSERVE_TIME/1000.0f);

    //如果想直接用轮子速度，不做融合的话可以这样
	chassis_move.v_filter = (chassis_move.wheel_motor[0].para.AngleSpeed-chassis_move.wheel_motor[1].para.AngleSpeed)*(0.15f)/2.0f*17/268;//0.075是轮子半径，电机反馈的是角速度，乘半径后得到线速度，数学模型中定义的是轮子顺时针为正，所以要乘个负号
	chassis_move.x_filter = chassis_move.x_filter+chassis_move.v_filter*((float)OBSERVE_TIME/1000.0f);

}

void xvEstimateKF_Init(KalmanFilter_t *EstimateKF)
{
    Kalman_Filter_Init(EstimateKF, 2, 0, 2);	// 状态向量2维 没有控制量 测量向量2维

    memcpy(EstimateKF->F_data, vaEstimateKF_F, sizeof(vaEstimateKF_F));
    memcpy(EstimateKF->P_data, vaEstimateKF_P, sizeof(vaEstimateKF_P));
    memcpy(EstimateKF->Q_data, vaEstimateKF_Q, sizeof(vaEstimateKF_Q));
    memcpy(EstimateKF->R_data, vaEstimateKF_R, sizeof(vaEstimateKF_R));
    memcpy(EstimateKF->H_data, vaEstimateKF_H, sizeof(vaEstimateKF_H));

}

void xvEstimateKF_Update(KalmanFilter_t *EstimateKF ,float acc,float vel)
{
    //卡尔曼滤波器测量值更新
    EstimateKF->MeasuredVector[0] =	vel;//测量速度
    EstimateKF->MeasuredVector[1] = acc;//测量加速度

    //卡尔曼滤波器更新函数
    Kalman_Filter_Update(EstimateKF);

    // 提取估计值
    for (uint8_t i = 0; i < 2; i++)
    {
        vel_acc[i] = EstimateKF->FilteredValue[i];
    }
}



