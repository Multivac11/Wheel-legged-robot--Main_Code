//
// Created by w1445 on 2025/1/14.
//

#include "Pid.h"

/**
  * @brief  限幅
  */
float AmplitudeLimit(float input,float amplitude)
{
    if(input<-amplitude)
        return -amplitude;
    else if(input>amplitude)
        return amplitude;
    else return input;
}


/**
  * @brief 单环PID初始化
  */
void BasePID_Init(BasePID_Object* base_pid, float kp, float ki, float kd, float detach )
{
    base_pid->KiPartDetachment = detach;

    base_pid->Kp = kp;
    base_pid->Ki = ki;
    base_pid->Kd = kd;

    base_pid->KpPart = 0;
    base_pid->KiPart = 0;
    base_pid->KdPart = 0;

}


/**
  * @brief 位置式pid
  */
float BasePID_PositionControl(BasePID_Object* base_pid, float target_speed, float feedback_speed)
{
    base_pid->Error = target_speed - feedback_speed;

    base_pid->KpPart = base_pid->Error * base_pid->Kp;
    base_pid->KiPart += base_pid->Error * base_pid->Ki;

    if(base_pid->Error > base_pid->KiPartDetachment)
    {
        base_pid->KiPart = 0;
    }
    else if(base_pid->Error < -(base_pid->KiPartDetachment))
    {
        base_pid->KiPart = 0;
    }

    base_pid->KdPart = (base_pid->Error - base_pid->LastError)*base_pid->KdPart;

    base_pid->Out = base_pid->KpPart + base_pid->KiPart + base_pid->KdPart ;

    base_pid->LastError = base_pid->Error;
    base_pid->LastlastError = base_pid->LastError;


    return base_pid->Out;
}


/**
  * @brief 增量式pid
  */
float BasePID_IncrementalControl(BasePID_Object* base_pid, float target_speed, float feedback_speed)
{
    base_pid->Error = target_speed - feedback_speed;

    base_pid->KpPart = (base_pid->Error - base_pid->LastError) * base_pid->Kp;
    base_pid->KiPart = base_pid->Error * base_pid->Ki;
    base_pid->KdPart = (base_pid->Error - 2.0f*base_pid->LastError + base_pid->LastlastError)*base_pid->KdPart;

    base_pid->Out = base_pid->KpPart + base_pid->KiPart + base_pid->KdPart ;

    base_pid->LastError = base_pid->Error;
    base_pid->LastlastError = base_pid->LastError;

    return base_pid->Out;
}