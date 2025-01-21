//
// Created by w1445 on 2024/12/20.
//

#ifndef BALANCE_LEG_V2_MOTOR_MSG_H
#define BALANCE_LEG_V2_MOTOR_MSG_H

#include <stdint.h>
#include "motor_msg.h"

#pragma pack(1)

// 发送用单个数据数据结构
typedef union
{
    int32_t L;
    uint8_t u8[4];
    uint16_t u16[2];
    uint32_t u32;
    float F;

} COMData32;

typedef struct
{
    // 定义 数据包头
    uint8_t start[2]; // 包头
    uint8_t motorID;  // 电机ID  0,1,2,3 ...   0xBB 表示向所有电机广播（此时无返回）
    uint8_t reserved;

} COMHead;

#pragma pack()

#pragma pack(1)


typedef struct
{                      // 以 4个字节一组排列 ，不然编译器会凑整
    // 定义 数据
    uint8_t mode;      // 关节模式选择
    uint8_t ModifyBit; // 电机控制参数修改位
    uint8_t ReadBit;   // 电机控制参数发送位
    uint8_t reserved;

    COMData32 Modify; // 电机参数修改 的数据

    //实际给FOC的指令力矩为：
    // K_P*delta_Pos + K_W*delta_W + T
    int16_t T;     // 期望关节的输出力矩（电机本身的力矩）x256, 7 + 8 描述
    int16_t W;     // 期望关节速度 （电机本身的速度） x128,       8 + 7描述
    int32_t Pos; // 期望关节位置 x 16384/6.2832, 14位编码器（主控0点修正，电机关节还是以编码器0点为准）

    int16_t K_P; // 关节刚度系数 x2048  4+11 描述
    int16_t K_W; // 关节速度系数 x1024  5+10 描述

    uint8_t LowHzMotorCmdIndex; // 电机低频率控制命令的索引, 0-7, 分别代表LowHzMotorCmd中的8个字节
    uint8_t LowHzMotorCmdByte;  // 电机低频率控制命令的字节

    COMData32 Res[1]; // 通讯 保留字节  用于实现别的一些通讯内容

} MasterComdV3; // 数据体 加上数据包的包头 和CRC 共34字节


/*发送报文设定*/
typedef struct
{
    // 定义 电机控制命令数据包
    COMHead head;          //数据包头
    MasterComdV3 Mdata;    //数据体
    COMData32 CRCdata;     //CRC校验

} MasterComdDataV3;

//#pragma pack()
//#pragma pack(1)

#pragma pack()

#pragma pack(1)


/*接收数据包定义*/
typedef struct
{ // 以 4个字节一组排列 ，不然编译器会凑整
    // 定义 数据
    uint8_t mode;    // 当前关节模式
    uint8_t ReadBit; // 电机控制参数修改     是否成功位
    int8_t Temp;     // 电机当前平均温度
    uint8_t MError;  // 电机错误 标识

    COMData32 Read; // 读取的当前 电机 的控制数据
    int16_t T;      // 当前实际电机输出力矩       7 + 8 描述

    int16_t W; // 当前实际电机速度（高速）   8 + 7 描述
    float LW;  // 当前实际电机速度（低速）

    int16_t W2; // 当前实际关节速度（高速）   8 + 7 描述
    float LW2;  // 当前实际关节速度（低速）

    int16_t Acc;    // 电机转子加速度       15+0 描述  惯量较小
    int16_t OutAcc; // 输出轴加速度         12+3 描述  惯量较大

    int32_t Pos;  // 当前电机位置（主控0点修正，电机关节还是以编码器0点为准）
    int32_t Pos2; // 关节编码器位置(输出编码器)

    int16_t gyro[3]; // 电机驱动板6轴传感器数据
    int16_t acc[3];

    // 力传感器的数据
    int16_t Fgyro[3]; //
    int16_t Facc[3];
    int16_t Fmag[3];
    uint8_t Ftemp; // 8位表示的温度  7位（-28~100度）  1位0.5度分辨率

    int16_t Force16; // 力传感器高16位数据
    int8_t  Force8;   // 力传感器低8位数据

    uint8_t FError; //  足端传感器错误标识

    int8_t Res[1]; // 通讯 保留字节

} ServoComdV3; // 加上数据包的包头 和CRC 78字节（4+70+4）



//总的电机接收报文//
typedef struct
{
    // 定义 电机控制命令数据包
    COMHead head;         //数据包头

    ServoComdV3 Mdata;    //数据体

    COMData32 CRCdata;    //CRC校验

} ServoComdDataV3; //发送数据



//总的电机发送数据//
typedef struct
{
    // 定义 发送格式化数据
    MasterComdDataV3 motor_send_data;

    int hex_len;
    unsigned short id;   //电机ID，0xBB代表全部电机
    unsigned short mode; // 0:空闲, 5:开环转动, 10:闭环FOC控制
    float T;             //期望关节的输出力矩（电机本身的力矩）（Nm）
    float W;             //期望关节速度（电机本身的速度）(rad/s)
    float Pos;           //期望关节位置（rad）
    float K_P;           //关节刚度系数
    float K_W;           //关节速度系数

    COMData32 Res;                    // 通讯 保留字节  用于实现别的一些通讯内容
    //电机控制数据结构体，详见motor_msg.h
}motor_send_t;   //共34字节



//总的电机接收数据//
typedef struct
{
    // 定义 接收数据

    ServoComdDataV3 motor_recv_data; //电机接收数据结构体，详见motor_msg_A1B1.h

    int hex_len;       //接收的16进制命令数组长度, 78

    //bool correct;   //接收数据是否完整（true完整，false不完整）
    //uint8_t right_Date[MOTOR_RX_LENGTH];

    unsigned char motor_id; // 1Byte 电机ID
    unsigned char mode;     // 1Byte 0:空闲, 5:开环转动, 10:闭环FOC控制
    int Temp;               // 1Byte 温度
    int MError;             // 1Byte 错误码

    float T;                // 2Byte 当前实际电机输出力矩
    float W;                // 2Byte 当前实际电机速度（高速）
    float original_Pos;              // 4Byte 当前电机位置
    float LW;               // 4Byte 当前实际电机速度（低速）
    int Acc;                // 2Byte 电机转子加速度

    float Pos2;             // 添加测试

    float gyro[3]; // 电机驱动板6轴传感器数据
    float acc[3];

    uint16_t  Rx_count;
    uint16_t  FPS;

} motor_recv_t; //共78个字节


typedef struct
{
    motor_send_t motor_send;
    motor_recv_t motor_recv;

}unitree_motor_t;

#pragma pack()

#endif //BALANCE_LEG_V2_MOTOR_MSG_H
