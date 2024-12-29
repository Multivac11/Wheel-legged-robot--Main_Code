//
// Created by w1445 on 2024/12/17.
//

#ifndef BALANCE_LEG_V2_A1_MOTOR_H
#define BALANCE_LEG_V2_A1_MOTOR_H

#include "motor_msg.h"
#include "usart.h"

extern unitree_motor_t A1_Motor[4];
#define  A1_Motor_left_1    0
#define  A1_Motor_left_2    1
#define  A1_Motor_right_1   2
#define  A1_Motor_right_2   3
#define  A1_Motor_num       4
#define  A1_Motor_Recv_Len    78
#define  A1_Motor_Send_Len    34

/**
 @brief 对应电机参数修改
 @param send 为MotorA1_send_left或MotorA1_send_right，分别控制左右侧腿部
 @param id   发送接收目标电机的id
 @param pos  为电机旋转圈数，1为一圈
 @param KP   电机刚度系数
 @param KW   电机速度系数
*/
void Modfiy_Pos_Cmd(motor_send_t *send,uint8_t id, float Pos, float KP, float KW);

// 速度模式
void Modfiy_Speed_Cmd(motor_send_t *send,uint8_t id, float Omega);

// 力矩模式
void Modfiy_Torque_Cmd(motor_send_t *send,uint8_t id, float torque);
void A1_Motor_Send_Cmd(motor_send_t *send,uint8_t motor);
void A1_Motor_Recv_Cmd(motor_recv_t *recv,uint8_t motor);
void A1_Motor_Init(uint8_t motor);

#endif //BALANCE_LEG_V2_A1_MOTOR_H
