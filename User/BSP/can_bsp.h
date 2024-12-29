#ifndef _CAN_BSP_H
#define _CAN_BSP_H


#include "main.h"

typedef FDCAN_HandleTypeDef hcan_t;

/**
	* @brief	CAN接收缓冲区
	*/
typedef struct
{
    uint8_t					Data[64];
}CAN_RxBuffer;

/**
	* @brief	CAN发送缓冲区
	*/
typedef struct
{
    uint8_t					Data[8];
}CAN_TxBuffer;

extern void FDCAN1_Config(void);
extern void FDCAN2_Config(void);
extern uint8_t canx_send_data(FDCAN_HandleTypeDef *hcan, uint16_t id, uint8_t *data, uint32_t len);



#endif

