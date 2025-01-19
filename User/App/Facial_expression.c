//
// Created by w1445 on 2025/1/14.
//

#include "Facial_expression.h"
#include "usart.h"
#include <string.h>

extern DMA_HandleTypeDef hdma_uart8_rx;
uint8_t Face_RxBuff[128];

void Facial_expression_init(void)
{
    HAL_UARTEx_ReceiveToIdle_DMA(&huart8, Face_RxBuff, 128); // 启用空闲中断接收
    __HAL_DMA_DISABLE_IT(&hdma_uart8_rx, DMA_IT_HT);
}

void Facial_expression_Control(void)
{

}

void Facial_expression_Callback(void)
{


    Facial_expression_init();
}