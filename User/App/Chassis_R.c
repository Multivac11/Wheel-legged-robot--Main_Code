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

//double Poly_Coefficient[12][4]=   //效果不错，机体质量修改theta d_theta x d_x phi d_phi% 1 1 100 600 5000 1，R=[280 0;0 17];需测试
//        {	{-102.617564629657,136.794943045713,-79.532338801389,1.069976249943},
//             {2.342658818196,-0.140129238728,-5.834676044719,0.166724378847},
//             {-10.787203085145,11.072827750737,-4.029526510764,-0.040976590970},
//             {-33.430290055653,34.419411361711,-12.748600382035,-0.151595397397},
//             {28.592953113766,-3.694425818232,-17.754661715563,10.926786445273},
//             {7.802975259528,-5.870943454887,0.519583895773,1.022647407736},
//             {532.636688115534,-498.036864176741,145.616980285400,7.300708426150},
//             {42.227097871253,-43.442091318193,14.772243283723,0.216603243763},
//             {-8.892646970895,15.610994268690,-10.071636525747,2.852537799567},
//             {-26.356369094726,47.183088251483,-30.862587772145,8.879108834894},
//             {639.534053906627,-690.085849146693,273.492877926156,-16.419652578010},
//             {53.569960508030,-61.132672910046,26.337854269367,-2.732545626000}
//        };

//好用
double Poly_Coefficient[12][4]=
        {	{-100.655147996577,135.362790343647,-79.596213601427,1.089002432284},
             {2.665942071491,-0.438340084664,-5.856323434443,0.169408813369},
             {-11.171961500275,11.516106667529,-4.215606831414,-0.032675655711},
             {-34.400714719128,35.573401762736,-13.258532694772,-0.125352584988},
             {31.049143385638,-5.801383358831,-17.313582675353,11.032367816697},
             {8.109425363740,-6.194435819240,0.636785003640,1.021329665182},
             {554.563978661317,-518.966099368473,151.716425012557,7.395173139575},
             {44.487979170543,-45.889184716769,15.721937259169,0.218884432357},
             {-8.294971844442,15.561852250224,-10.432155703694,3.036549354964},
             {-24.201116474481,46.583144168461,-31.729972408617,9.396187073589},
             {659.661121060661,-714.303308554839,284.296082774399,-17.409591688557},
             {54.159430352799,-62.269387519146,27.072877393905,-2.890560575532}
        };


//double Poly_Coefficient[12][4]=   //效果不错，机体质量修改theta d_theta x d_x phi d_phi% 10 1 100 800 5000 1，R=[90 0;0 4];需测试
//        {	{-135.440409031951,158.963729533794,-81.564208590728,-0.783401778264},
//             {-1.857897538500,3.051072941303,-6.395211231806,0.045536003997},
//             {-1.918971093440,1.713146015690,-0.488549170905,-0.575506408758},
//             {-5.836763768863,5.243801518363,-1.680193501038,-1.832420603783},
//             {-102.025028579641,117.480093961421,-53.833497342137,12.577784295532},
//             {-7.845122589870,9.314276030630,-4.443228121596,1.446651240570},
//             {118.756578738586,-118.875214458940,44.363315420976,3.364400588358},
//             {5.122321972815,-4.951786330718,0.891264020515,0.116149415151},
//             {-9.974319394011,10.463086862329,-4.010588180985,0.498002111948},
//             {-31.030299101217,32.560765135688,-12.513924250058,1.517377942164},
//             {174.746027666754,-176.733491822545,65.531066854844,9.711706942433},
//             {18.959511920570,-19.379159833935,7.288696224200,0.726611823472}
//        };


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
    BasePID_Init(&Turn_pid,4.0f , 0 ,0.5f, 0);
    BasePID_Init(&LegR_pid,550.0f , 0 ,3000.0f, 0);
//	BasePID_Init(&RollR_Pid,0.0f , 0 ,0, 0);
    BasePID_Init(&RollR_Pid,100.0f , 0 ,0, 0);
}

void ChassisR_task(void)
{
    chassisR_feedback_update(&chassis_move,&right,&hi91_data);//更新数据
    chassisR_control(&chassis_move,&right,&hi91_data,LQR_K_R,Tp_pid,Turn_pid,LegR_pid,RollR_Pid);//控制计算
}

void chassisR_feedback_update(chassis_t *chassis,vmc_leg_t *vmc,HI91_T *hi91)
{
    vmc->phi1 = ((A1_Motor[A1_Motor_right_1].motor_recv.original_Pos + 197) * PI/180);
    vmc->phi4 = ((A1_Motor[A1_Motor_right_2].motor_recv.original_Pos - 65) * PI/180);

    chassis->myPithR = ((hi91->pitch+1) * PI/180);
    chassis->myPithGyroR = (hi91->gyr[0] * PI/180);

    chassis->total_yaw = (hi91->total_yaw * PI/180);
    chassis->roll = (hi91->roll * PI/180);
    chassis->theta_err = 0.0f - (vmc->theta+left.theta);
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
    vmcr->F0 = (Mg/2)/arm_cos_f32(vmcr->theta) + BasePID_PositionControl(&LegR_pid, chassis->leg_set_R , vmcr->L0)-chassis->now_roll_set;

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

    mySaturate_i(&chassis->wheel_motor[0].para.Output,-16384,+16384); //经过换算目标输出力矩被限制在5N*M

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
