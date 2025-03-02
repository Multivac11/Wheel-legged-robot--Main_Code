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

//double Poly_Coefficient[12][4]=   //效果不错，机体质量修改theta d_theta x d_x phi d_phi% 10 1 100 800 5000 1，R=[90 0;0 4];需测试
//        {	{-79.198404791884,119.749308217421,-85.121844392031,0.929354840340},
//             {7.492002979317,-5.366425816002,-7.048256802725,0.188468941799},
//             {-14.125123288206,15.298538779327,-6.005389625333,-0.122273244037},
//             {-45.146430683461,49.200308623289,-19.836088293121,-0.430383243826},
//             {30.647841527635,-1.755531806660,-22.618852106757,14.663420109274},
//             {8.406012965588,-6.912927865224,1.068066930567,1.264476720931},
//             {720.492691520983,-696.320744182221,215.888207755467,8.273693017661},
//             {68.219042749167,-73.800364543500,28.774398042468,0.309852974441},
//             {-4.861869089936,17.933047213238,-15.598395651397,5.561752396316},
//             {-12.388705173513,54.192609599209,-48.965499303261,18.043835351136},
//             {812.874194814381,-911.481226942225,380.659185344353,-21.753263716563},
//             {53.173674711936,-66.577705878277,32.221266331525,-4.356692506231}
//        };
double Poly_Coefficient[12][4]=   //效果不错，机体质量修改theta d_theta x d_x phi d_phi% 10 1 100 800 5000 1，R=[90 0;0 4];需测试
        {	{-74.205067508278,123.150140638125,-105.702768722740,0.596839843410},
             {12.264165007500,-10.706955612420,-10.485941021707,0.197772421764},
             {-19.689102455602,21.949465479780,-8.974780119592,-0.750795457260},
             {-53.096195732695,59.940605776785,-25.736736074242,-2.134903529251},
             {-0.599690119083,42.106827708130,-48.821017437694,24.774256275824},
             {6.317143032038,-4.897101690886,0.325040283768,2.001255770744},
             {808.320682788940,-804.439046234835,265.038797040722,9.715903279404},
             {84.013047344175,-93.905574721074,40.145476983886,0.673852893756},
             {-15.349629510879,36.619851754991,-28.460203580671,10.131571627014},
             {-36.575363676906,93.968401677886,-75.529484287516,28.124367889588},
             {1012.221135685373,-1147.839955427021,488.467308938423,-9.352527163047},
             {49.333991456348,-66.483580603818,35.019444219974,-5.031056440519}
        };

//Q=diag([1000 1 1 2 20000 1]);
//R=[0.25 0;0 1.5];
//double Poly_Coefficient[12][4]=   //效果不错，机体质量修改theta d_theta x d_x phi d_phi% 10 1 100 800 5000 1，R=[90 0;0 4];需测试
//        {	{-107.261972337761,94.926185330284,-37.601504347092,-65.477835756943},
//             {-2.006215447622,1.098644900028,-8.940428590752,-2.212856708511},
//             {-0.153935612064,-0.466452909758,0.489058543945,-1.977310292240},
//             {105.524568111980,-105.585120063221,38.038165198930,-11.561076635328},
//             {-2949.434974318184,3078.348220301757,-1167.878109566302,76.635889692220},
//             {-70.865248835721,77.585123432704,-32.673789850023,4.491363855389},
//             {-199.932164301102,204.061360582140,-73.199794621235,7.210503253375},
//             {-8.411167509054,9.328301123925,-5.359328918807,0.176281429114},
//             {-0.769366717976,1.441371486059,-0.840036715239,-0.139183834910},
//             {4.190941812099,-2.126848236025,-0.082997314026,-0.925135910028},
//             {91.485781529274,-44.216820716446,-10.231156476084,123.401718775943},
//             {8.298784030978,-8.135488797374,2.959622153310,3.950457737239}
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
    BasePID_Init(&LegR_pid,450.0f , 0 ,3000.0f, 0);
//	BasePID_Init(&RollR_Pid,0.0f , 0 ,0, 0);
    BasePID_Init(&RollR_Pid,0.0f , 0 ,0, 0);
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

    chassis->total_yaw = (hi91->yaw * PI/180);
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
