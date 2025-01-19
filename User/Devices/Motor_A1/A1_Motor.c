//
// Created by w1445 on 2024/12/17.
//

#include "A1_Motor.h"
#include "motor_msg.h"
#include <string.h>
#include <stdio.h>
#include "usart.h"

#define PI 3.1415926535f

uint8_t A1MotorA1_recv_date[A1_Motor_num][A1_Motor_Recv_Len]; // 接收数据缓存区
uint8_t A1MotorA1_send_date[A1_Motor_num][A1_Motor_Send_Len]; // 发送数据
unitree_motor_t A1_Motor[A1_Motor_num];
extern DMA_HandleTypeDef hdma_usart3_rx;
extern DMA_HandleTypeDef hdma_uart4_rx;
extern DMA_HandleTypeDef hdma_uart7_rx;
extern DMA_HandleTypeDef hdma_uart9_rx;

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

void A1_Motor_Init(uint8_t motor)
{
    if(motor == A1_Motor_left_1)
    {
        HAL_UARTEx_ReceiveToIdle_DMA(&huart3, A1MotorA1_recv_date[A1_Motor_left_1], A1_Motor_Recv_Len); // 启用空闲中断接收
        __HAL_DMA_DISABLE_IT(&hdma_usart3_rx, DMA_IT_HT);                                   // 关闭DMA传输过半中断
    }
    if(motor == A1_Motor_left_2)
    {
        HAL_UARTEx_ReceiveToIdle_DMA(&huart9, A1MotorA1_recv_date[A1_Motor_left_2], A1_Motor_Recv_Len); // 启用空闲中断接收
        __HAL_DMA_DISABLE_IT(&hdma_uart9_rx, DMA_IT_HT);                      // 关闭DMA传输过半中断
    }
    if(motor == A1_Motor_right_1)
    {
        HAL_UARTEx_ReceiveToIdle_DMA(&huart4, A1MotorA1_recv_date[A1_Motor_right_1], A1_Motor_Recv_Len); // 启用空闲中断接收
        __HAL_DMA_DISABLE_IT(&hdma_uart4_rx, DMA_IT_HT);                      // 关闭DMA传输过半中断
    }
    if(motor == A1_Motor_right_2)
    {
        HAL_UARTEx_ReceiveToIdle_DMA(&huart7, A1MotorA1_recv_date[A1_Motor_right_2], A1_Motor_Recv_Len); // 启用空闲中断接收
        __HAL_DMA_DISABLE_IT(&hdma_uart7_rx, DMA_IT_HT);                      // 关闭DMA传输过半中断
    }

}

// CRC校验位的代码
uint32_t crc32_core_Ver3(uint32_t *ptr, uint32_t len)
{
    uint32_t bits;
    uint32_t i;
    uint32_t xbit = 0;
    uint32_t data = 0;
    uint32_t CRC32 = 0xFFFFFFFF;
    const uint32_t dwPolynomial = 0x04c11db7;
    for (i = 0; i < len; i++)
    {
        xbit = 1 << 31;
        data = ptr[i];
        for (bits = 0; bits < 32; bits++)
        {
            if (CRC32 & 0x80000000)
            {
                CRC32 <<= 1;
                CRC32 ^= dwPolynomial;
            }
            else
                CRC32 <<= 1;
            if (data & xbit)
                CRC32 ^= dwPolynomial;

            xbit >>= 1;
        }
    }
    return CRC32;
}

// 电机位置修改
void Modfiy_Pos_Cmd(motor_send_t *send,uint8_t id, float Pos, float KP, float KW)
{

    send->hex_len = 34;

    send->mode = 10;
    send->id   = id;

    send->Pos  = 2*PI/360*9.1*Pos;  // 6.2832 = 2 PI // 原先为 6.2832*9.1*2*Pos
    send->W    = 0;
    send->T    = 0.0f;
    send->K_P  = KP;
    send->K_W  = KW;
}

// 电机速度修改
void Modfiy_Speed_Cmd(motor_send_t *send,uint8_t id, float Omega)
{

    send->hex_len = 34;

    send->mode = 10;
    send->id   = id;

    send->Pos  = 0;
    send->W    = Omega * 9.1f;
    send->T    = 0.0f;
    send->K_P  = 0.0f;
    send->K_W  = 3.0f;
}

// 电机力矩修改
void Modfiy_Torque_Cmd(motor_send_t *send,uint8_t id, float torque)
{

    send->hex_len = 34;

    send->mode = 10;
    send->id   = id;

    send->Pos  = 0.0f;
    send->W    = 0.0f;
    if (torque > 10.0f){torque = 0.0f;} // 限幅
    send->T    = torque / 9.1f;
    send->K_P  = 0.0f;
    send->K_W  = 0.0f;
}

// 发送命令
void A1_Motor_Send_Cmd(motor_send_t *send,uint8_t motor)
{

    send->motor_send_data.head.start[0] = 0xFE;
    send->motor_send_data.head.start[1] = 0xEE;
    send->motor_send_data.head.motorID = send->id;

    send->motor_send_data.Mdata.mode = send->mode;
    send->motor_send_data.Mdata.ModifyBit = 0xFF;
    send->motor_send_data.Mdata.ReadBit = 0x00;
    send->motor_send_data.Mdata.reserved = 0x00;
    send->motor_send_data.Mdata.Modify.F = 0;
    send->motor_send_data.Mdata.T = (int16_t )(send->T * 256);
    send->motor_send_data.Mdata.W = (int16_t )(send->W * 128);
    send->motor_send_data.Mdata.Pos = (int)((send->Pos / 6.2832f) * 16384.0f);
    send->motor_send_data.Mdata.K_P = (int16_t )(send->K_P * 2048);
    send->motor_send_data.Mdata.K_W = (int16_t )(send->K_W * 1024);

    send->motor_send_data.Mdata.LowHzMotorCmdIndex = 0;
    send->motor_send_data.Mdata.LowHzMotorCmdByte = 0;
    send->motor_send_data.Mdata.Res[0] = send->Res;

    send->motor_send_data.CRCdata.u32 = crc32_core_Ver3((uint32_t *)&send->motor_send_data, 7);

    memcpy(A1MotorA1_send_date[motor], &send->motor_send_data, A1_Motor_Send_Len);

    if(motor == A1_Motor_left_1)
    {
        HAL_UART_Transmit_DMA(&huart3, A1MotorA1_send_date[motor], A1_Motor_Send_Len);
    }
    if(motor == A1_Motor_left_2)
    {
        HAL_UART_Transmit_DMA(&huart9, A1MotorA1_send_date[motor], A1_Motor_Send_Len);
    }
    if(motor == A1_Motor_right_1)
    {
        HAL_UART_Transmit_DMA(&huart4, A1MotorA1_send_date[motor], A1_Motor_Send_Len);
    }
    if(motor == A1_Motor_right_2)
    {
        HAL_UART_Transmit_DMA(&huart7, A1MotorA1_send_date[motor], A1_Motor_Send_Len);
    }

}

// 接收数据
void A1_Motor_Recv_Cmd(motor_recv_t *recv,uint8_t motor)
{
    recv->motor_recv_data.head.motorID = A1MotorA1_recv_date[motor][2];
    recv->motor_recv_data.Mdata.mode = A1MotorA1_recv_date[motor][4];
    recv->motor_recv_data.Mdata.Temp = (int8_t )(A1MotorA1_recv_date[motor][6]);
    recv->motor_recv_data.Mdata.MError = A1MotorA1_recv_date[motor][7];
    recv->motor_recv_data.Mdata.T = (int16_t )(A1MotorA1_recv_date[motor][13] << 8 | A1MotorA1_recv_date[motor][12]);//反拼
    recv->motor_recv_data.Mdata.W = (int16_t )(A1MotorA1_recv_date[motor][15] << 8 | A1MotorA1_recv_date[motor][14]);//反拼
    recv->motor_recv_data.Mdata.Acc = (int16_t )(A1MotorA1_recv_date[motor][27] << 8 | A1MotorA1_recv_date[motor][26]);
    recv->motor_recv_data.Mdata.Pos = (int32_t )(A1MotorA1_recv_date[motor][33] << 24 | A1MotorA1_recv_date[motor][32] << 16 | A1MotorA1_recv_date[motor][31] << 8 | A1MotorA1_recv_date[motor][30]);

    recv->motor_id = recv->motor_recv_data.head.motorID;
    recv->mode = recv->motor_recv_data.Mdata.mode;
    recv->Temp = recv->motor_recv_data.Mdata.Temp;
    recv->MError = recv->motor_recv_data.Mdata.MError;
    recv->T = ((float)recv->motor_recv_data.Mdata.T / 256.0f)*9.1f;                           //减速后的扭矩
    recv->Pos = ((float)(recv->motor_recv_data.Mdata.Pos / (16384.0f/2/PI)))*(180/PI/9.1f);   //减速后的角度
    recv->W = ((float)recv->motor_recv_data.Mdata.W / 128.0f)/9.1f;                           //减速后的角速度
    recv->Acc = recv->motor_recv_data.Mdata.Acc;
    recv->Rx_count ++;

    memset(A1MotorA1_recv_date[motor], 0, sizeof(A1MotorA1_recv_date[motor]));

    A1_Motor_Init(motor);
}
