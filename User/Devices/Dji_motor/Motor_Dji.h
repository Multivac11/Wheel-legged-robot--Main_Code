//
// Created by w1445 on 2024/11/25.
//

#ifndef BALANCE_LEG_V2_MOTOR_DJI_H
#define BALANCE_LEG_V2_MOTOR_DJI_H

#include "main.h"
#include "can_bsp.h"

#define K_ECD_TO_ANGLE 0.043945f  		//< 角度转换编码器刻度的系数：360/8192
#define ECD_RANGE_FOR_3508 8191				//< 编码器刻度值为0-8191
#define CURRENT_LIMIT_FOR_3508 16000   //< 控制电流范围为正负16384
#define ECD_RANGE_FOR_6020 8191				//< 编码器刻度值为0-8191
#define CURRENT_LIMIT_FOR_6020 29000   //< 控制电流范围为正负30000
#define ECD_RANGE_FOR_2006 8191				//< 编码器刻度值为0-8191
#define CURRENT_LIMIT_FOR_2006 10000   //< 控制电流范围为正负16384

/**
  * @brief  电机动态数据，电机运行中产生的数据，由CAN中断回调更新
  */
typedef struct
{
    int16_t  Ecd;         	//< 当前编码器返回值
    int16_t  SpeedRPM;			//< 每分钟所转圈数
    int16_t  TorqueCurrent; //< 反馈力矩
    uint8_t  Temperature;	  //< 温度

    int16_t  RawEcd;				//< 原始编码器数据
    int16_t  LastEcd;			  //< 上一时刻编码器返回值
    float    Angle;					//< 解算后的编码器角度
    int16_t  AngleSpeed;	  //< 解算后的编码器角速度
    int32_t  RoundCnt;			//< 累计转动圈数
    int32_t  TotalEcd;			//< 编码器累计增量值
    int32_t  TotalAngle;		//< 累计旋转角度

    int32_t  Target;				//< 电机的期望参数
    int32_t  Output;  			//< 电机输出值，通常为电流和电压
    float CanEcd[20] ;
    float CanAngleSpeed[20] ;
    float LvboAngle;
    int16_t  LvboEcd;
    int16_t  LvboSpeedRPM;

}MotorData;

/**
  * @brief   电机参数，在初始化函数中确定
  */
typedef struct
{
    uint8_t  CanNumber;			 										//< 电机所使用的CAN端口号
    uint16_t CanId;			 												//< 电机ID
    uint8_t  MotorType;			 										//< 电机类型
    uint16_t EcdOffset;	 									  		//< 电机初始零点
    uint16_t EcdFullRange;											//< 编码器量程
    int16_t  CurrentLimit;			 								//< 电调能承受的最大电流
}MotorParam;

/**
  * @brief  电机数据更新回调函数，只在motor.c文件内调用。（大疆电机反馈报文格式相同）
  */
uint8_t Motor3508_update_data(MotorData* motor, uint8_t *rxBuffer);
void set_moto_current(hcan_t* hcan,uint16_t motor_id,int16_t iq1,int16_t iq2);

#endif //BALANCE_LEG_V2_MOTOR_DJI_H
