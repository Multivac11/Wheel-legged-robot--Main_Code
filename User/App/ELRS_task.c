//
// Created by w1445 on 2025/1/13.
//

#include "ELRS_task.h"
#include "Chassis_R.h"
#include "Chassis_L.h"
#include "ELRS_Drive.h"


uint16_t ELRS_TIME = 1;//R9DS任务周期是1ms

void ELRS_Task(ELRS_Data *Elrs_data,chassis_t *chassis)
{
    if(Elrs_data->F == 1 && chassis->start_flag == 0)
    {
        chassis->start_flag = 1;
        if(chassis->recover_flag==0
           &&((hi91_data.pitch < (-8.0f) && hi91_data.pitch > (-11.0f))
              ||(hi91_data.pitch > (8.0f) && hi91_data.pitch <(11.0f))))
        {
            chassis->recover_flag=1;//需要自起
        }
    }
    else if(Elrs_data->F == 0 && chassis->start_flag == 1)
    {
        chassis->start_flag = 0;
        chassis->recover_flag=0;
    }

    if(chassis->start_flag==1)
    {//启动

        chassis->target_v = (float)(Elrs_data->Right_Y)*(0.02f);//往前大于0
        slope_following(&chassis->target_v,&chassis->v_set,0.02f);	//	坡度跟随

        chassis->x_set = (float)(chassis->x_set+(2.0f)*(chassis->v_set)*((float)ELRS_TIME/1000));

//		chassis->v_set = 0;
//		chassis->target_x = chassis->target_x + ((float)(rc_ctrl->rc.ch3-1000))*(-0.0000015f);
//		slope_following(&chassis->target_x,&chassis->x_set,0.001f);

        chassis->turn_set = chassis->turn_set+(Elrs_data->Left_X)*(-0.00002f);//往右大于0

        //腿长变化
        chassis->leg_set = chassis->leg_set+((float)Elrs_data->Right_X)*(0.000002f);
        mySaturate_f(&chassis->leg_set,0.095f,0.398f);//腿长限幅在0.095m到0.398m之间
//		chassis->roll_target = ((float)(rc_ctrl->rc.ch2-127))*(0.0025f);
        chassis->roll_target = 0;
        slope_following(&chassis->roll_target,&chassis->roll_set,0.0075f);

        jump_key(&elrs_data,&chassis_move);

        chassis->leg_set_R = chassis->leg_set;
        chassis->leg_set_L = chassis->leg_set;

//		mySaturate(&chassis->leg_set_R,0.065f,0.18f);//腿长限幅在0.065m到0.18m之间
//		mySaturate(&chassis->leg_set_L,0.065f,0.18f);

        if(fabsf(chassis->last_leg_set-chassis->leg_set)>0.0001f)
        {//遥控器控制腿长在变化
            right.leg_flag=1;	//为1标志着腿长在主动伸缩(不包括自适应伸缩)，根据这个标志可以不进行离地检测，因为当腿长在主动伸缩时，离地检测会误判端为离地了
            left.leg_flag=1;
        }
        chassis->last_leg_set = chassis->leg_set;
        chassis->last_leg_set_R = chassis->leg_set_R;
        chassis->last_leg_set_L = chassis->leg_set_L;
    }
    else if(chassis->start_flag==0)
    {//关闭
        chassis->v_set=0.0f;//清零
        chassis->x_filter = 0;
        chassis->v_filter = 0;
        chassis->x_set=chassis->x_filter;//保存
        chassis->turn_set=chassis->total_yaw;//保存

        chassis->leg_set=0.15f;//原始腿长
        chassis->leg_set_R = chassis->leg_set;
        chassis->leg_set_L = chassis->leg_set;
    }
}

void jump_key(ELRS_Data *Elrs_data,chassis_t *chassis)
{

}

void slope_following(float *target,float *set,float acc)
{
    if(*target > *set)
    {
        *set = *set + acc;
        if(*set >= *target)
            *set = *target;
    }
    else if(*target < *set)
    {
        *set = *set - acc;
        if(*set <= *target)
            *set = *target;
    }

}
