//
// Created by w1445 on 2024/11/25.
//

#ifndef BALANCE_LEG_V2_CHASSIS_R_H
#define BALANCE_LEG_V2_CHASSIS_R_H

#include "main.h"
#include "Motor_Dji.h"

typedef struct
{
    MotorData para;
}Joint_Motor_t ;

typedef struct
{
    float wheel_T;//轮毂电机的输出扭矩，单位为N
    MotorData para;
}Wheel_Motor_t ;

typedef struct
{
    Joint_Motor_t joint_motor[4];
    Wheel_Motor_t wheel_motor[2];

    float v_set;//期望速度，单位是m/s
    float target_v;
    float x_set;//期望位置，单位是m
    float target_x;
    float turn_set;//期望yaw轴弧度
    float leg_set;//期望腿长，单位是m
    float leg_set_R;
    float leg_set_L;
    float last_leg_set;
    float last_leg_set_R;
    float last_leg_set_L;

    float v_filter;//滤波后的车体速度，单位是m/s
    float x_filter;//滤波后的车体位置，单位是m

    float myPithR;
    float myPithGyroR;
    float myPithL;
    float myPithGyroL;
    float roll;
    float roll_set;
    float roll_target;
    float now_roll_set;
    float total_yaw;
    float theta_err;//两腿夹角误差

    float turn_T;//yaw轴补偿
    float leg_tp;//防劈叉补偿

    uint8_t start_flag;//启动标志

    uint8_t recover_flag;//一种情况下的倒地自起标志

    uint32_t count_key;
    uint8_t jump_flag;
    float jump_leg;
    uint32_t jump_time_r;
    uint32_t jump_time_l;
    uint8_t jump_status_r;
    uint8_t jump_status_l;


} chassis_t;

extern chassis_t chassis_move;

#endif //BALANCE_LEG_V2_CHASSIS_R_H
