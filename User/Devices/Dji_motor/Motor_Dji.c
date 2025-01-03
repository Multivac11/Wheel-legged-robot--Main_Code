//
// Created by w1445 on 2024/11/25.
//

#include "Motor_Dji.h"
#include "can_bsp.h"


/**
  * @brief  电机数据更新回调函数，只在motor.c文件内调用。（大疆电机反馈报文格式相同）
  */
uint8_t Motor3508_update_data(MotorData* motor, uint8_t *rxBuffer)
{
    motor->LastEcd       = motor->Ecd;      //< 更新编码器角度前记录上个周期的编码器角度
    motor->RawEcd 		 = rxBuffer[0]<<8|rxBuffer[1];
    motor->SpeedRPM      = rxBuffer[2]<<8|rxBuffer[3];
    motor->TorqueCurrent = rxBuffer[4]<<8|rxBuffer[5];
    motor->Temperature   = rxBuffer[6];
    motor->Ecd           = motor->RawEcd;

    return 0;
}

void set_moto_current(hcan_t* hcan,uint16_t motor_id,int16_t iq1,int16_t iq2){

    uint8_t data[8];

    data[0] = (iq1 >> 8);
    data[1] = iq1;
    data[2] = (iq2 >> 8);
    data[3] = iq2;
    data[4] = 0;
    data[5] = 0;
    data[6] = 0;
    data[7] = 0;

    canx_send_data(hcan, motor_id, data, 8);
}
