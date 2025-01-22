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
#include "arm_math.h"


double LQR_K_R[12]={
        -4.4394,   -0.513  , -1.3493,   -1.3052,    2.0990  ,  0.2698,
        3.1791,   0.3891  ,  1.5877  ,  1.4070  , 13.3789 ,   0.6359};

double Poly_Coefficient[12][4]=   //效果不错，机体质量修改theta d_theta x d_x phi d_phi% 10 1 100 600 4000 1，R=[90 0;0 4];需测试
        {	{-141.824157252045,191.647112634727,-110.284665289416,2.602097017937},
             {10.664455150475,-6.791518729134,-6.715863685967,0.341284491490},
             {-28.069323512165,29.696170429149,-11.179964116140,0.546342070993},
             {-79.327888395276,84.400107608781,-32.394660650532,1.554008270861},
             {68.284732823408,-31.229880788662,-13.518154282491,10.025824914629},
             {10.664817460725,-7.355185077739,0.254703586649,0.937401211728},
             {1082.300393193283,-961.774328112779,244.860267304993,11.936140040004},
             {101.806076556086,-105.016819001327,36.691269031518,-0.103809505248},
             {31.067798722415,-8.597698526192,-11.447771548010,6.019847394615},
             {96.042251942091,-32.179263449200,-30.083006761711,17.187236393488},
             {1033.937355730385,-1106.603672091139,424.120200487017,-25.429384761079},
             {85.269752504488,-95.573307454835,39.041738292366,-3.861154312338}
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
    vmc->phi1 = ((A1_Motor[A1_Motor_right_1].motor_recv.original_Pos + 197) * PI/180);
    vmc->phi4 = ((A1_Motor[A1_Motor_right_2].motor_recv.original_Pos - 65) * PI/180);

    chassis->myPithR = (hi91_data.pitch * PI/180);
    chassis->myPithGyroR = (hi91_data.gyr[0] * PI/180);

    chassis->total_yaw = (hi91_data.yaw * PI/180);
    chassis->roll = (hi91_data.roll * PI/180);
    chassis->theta_err=0.0f-(vmc->theta+left.theta);
}

uint8_t right_flag;
void chassisR_control(chassis_t *chassis,vmc_leg_t *vmcr,HI91_T *hi91,double *LQR_K,BasePID_Object Tp_pid,BasePID_Object Turn_pid,BasePID_Object LegR_pid,BasePID_Object RollR_Pid)
{
    VMC_calc_1_right(vmcr,hi91,((float)CHASSR_TIME)*1.0f/1000.0f);//计算theta和d_theta给lqr用，同时也计算右腿长L0,该任务控制周期是1*0.001秒

    for(int i=0;i<12;i++)
    {
        LQR_K[i]=LQR_K_calc(&Poly_Coefficient[i][0],vmcr->L0 );
    }
    //转向
    chassis->turn_T = Turn_pid.Kp * (chassis->turn_set-chassis->total_yaw) - Turn_pid.Kd * hi91->gyr[2] * (PI/180);
    //防劈叉
    chassis->leg_tp = BasePID_PositionControl(&Tp_pid, 0.0f , chassis->theta_err);

    chassis->wheel_motor[0].wheel_T = (float)(LQR_K[0]*(vmcr->theta-0.0f)
                                             +LQR_K[1]*(vmcr->d_theta-0.0f)
                                             +LQR_K[2]*(chassis->x_filter-chassis->x_set)
                                             +LQR_K[3]*(chassis->v_filter-chassis->v_set)
                                             +LQR_K[4]*(chassis->myPithR-0.0f)
                                             +LQR_K[5]*(chassis->myPithGyroR-0.0f));
    //右边髋关节输出力矩
    vmcr->Tp = (float)(LQR_K[6]*(vmcr->theta-0.0f)
                      +LQR_K[7]*(vmcr->d_theta-0.0f)
                      +LQR_K[8]*(chassis->x_filter-chassis->x_set)
                      +LQR_K[9]*(chassis->v_filter-chassis->v_set)
                      +LQR_K[10]*(chassis->myPithR-0.0f)
                      +LQR_K[11]*(chassis->myPithGyroR-0.0f));

    vmcr->Tp = vmcr->Tp+chassis->leg_tp;//髋关节输出力矩

    chassis->now_roll_set = BasePID_PositionControl(&RollR_Pid,chassis->roll_set,chassis->roll);

//    vmcr->F0 = Mg+BasePID_PositionControl(&LegR_pid, chassis->leg_set_R , vmcr->L0);//前馈+pd
    vmcr->F0 = Mg/arm_cos_f32(vmcr->theta) + BasePID_PositionControl(&LegR_pid, chassis->leg_set_R , vmcr->L0)-chassis->now_roll_set;

    right_flag = ground_detectionR(vmcr);//右腿离地检测

    if(chassis->recover_flag==0)		//没倒地
    {//倒地自起不需要检测是否离地
        if(right_flag == 1 && left_flag == 1 && vmcr->leg_flag == 0)
        {//当两腿同时离地并且遥控器没有在控制腿的伸缩时，才认为离地
            chassis->wheel_motor[0].wheel_T=0.0f;
            vmcr->Tp = (float)(LQR_K[6]*(vmcr->theta-0.0f)+ LQR_K[7]*(vmcr->d_theta-0.0f));

            chassis->x_filter = 0.0f;
            chassis->x_set = chassis->x_filter;
            chassis->turn_set = chassis->total_yaw;
            vmcr->Tp = vmcr->Tp+chassis->leg_tp;
        }
        else
        {//没有离地
            vmcr->leg_flag=0;//置为0

        }
    }
    else if(chassis->recover_flag==1)
    {
        vmcr->Tp=0.0f;
    }

    mySaturate_f(&vmcr->F0,-150.0f,150.0f);//限幅
    VMC_calc_2(vmcr);//计算期望的关节输出力矩

    chassis->wheel_motor[0].wheel_T = chassis->wheel_motor[0].wheel_T-chassis->turn_T;	//轮毂电机输出力矩
    chassis->wheel_motor[0].para.Output = (int16_t)(-chassis->wheel_motor[0].wheel_T * 661504/201);

    mySaturate_i(&chassis->wheel_motor[0].para.Output,-16384,+16384);

    A1_Motor[A1_Motor_right_1].motor_send.T = vmcr->torque_set[0];
    A1_Motor[A1_Motor_right_2].motor_send.T = vmcr->torque_set[1];

    if(A1_Motor[A1_Motor_right_1].motor_send.T < -33.0f)
    {
        A1_Motor[A1_Motor_right_1].motor_send.T = -33.0f;
    }
    else if(A1_Motor[A1_Motor_right_1].motor_send.T > 33.0f)
    {
        A1_Motor[A1_Motor_right_1].motor_send.T = 33.0f;
    }

    if(A1_Motor[A1_Motor_right_2].motor_send.T < -33.0f)
    {
        A1_Motor[A1_Motor_right_2].motor_send.T = -33.0f;
    }
    else if(A1_Motor[A1_Motor_right_2].motor_send.T > 33.0f)
    {
        A1_Motor[A1_Motor_right_2].motor_send.T = 33.0f;
    }
//    mySaturate_f(&A1_Motor[A1_Motor_right_1].motor_send.T,-4.0f,+4.0f);
//    mySaturate_f(&A1_Motor[A1_Motor_right_2].motor_send.T,-4.0f,+4.0f);
}

void mySaturate_f(float *in,float min,float max)
{
    if(*in < min)
    {
        *in = min;
    }
    else if(*in > max)
    {
        *in = max;
    }
}

void mySaturate_i(int16_t *in,int32_t min,int32_t max)
{
    if(*in < min)
    {
        *in = min;
    }
    else if(*in > max)
    {
        *in = max;
    }
}
