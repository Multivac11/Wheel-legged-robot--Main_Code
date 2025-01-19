//
// Created by w1445 on 2024/11/26.
//

#include "USART_Callback.h"
#include "usart.h"
#include "ELRS_Drive.h"
#include "CH010_HI91.h"
#include "A1_Motor.h"
#include "Facial_expression.h"

/**
  * @brief  串口空闲中断回调函数
  */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if(huart == &huart2)
    {
        HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_10);
        HI91_UARTE_RxCallback(Size);
    }
    if(huart == &huart3)
    {
        HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_12);
        A1_Motor_Recv_Cmd(&A1_Motor[A1_Motor_left_1].motor_recv,A1_Motor_left_1);
    }
    if(huart == &huart9)
    {
        HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_13);
        A1_Motor_Recv_Cmd(&A1_Motor[A1_Motor_left_2].motor_recv,A1_Motor_left_2);
    }
    if(huart == &huart4)
    {
        HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_14);
        A1_Motor_Recv_Cmd(&A1_Motor[A1_Motor_right_1].motor_recv,A1_Motor_right_1);
    }
    if(huart == &huart7)
    {
        HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_15);
        A1_Motor_Recv_Cmd(&A1_Motor[A1_Motor_right_2].motor_recv,A1_Motor_right_2);
    }
    if(huart == &huart8)
    {
        Facial_expression_Callback();
    }
    if (huart == &huart10)
    {
        HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_11);
        ELRS_UARTE_RxCallback(Size);
    }

}
/**
  * @brief  串口接收中断回调函数
  */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{

}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if(huart == &huart2)
    {
        CH010_HI91_Init();
        hi91_data.eorror_check++;
    }
    if(huart == &huart3)
    {
        A1_Motor_Init(A1_Motor_left_1);
    }
    if(huart == &huart9)
    {
        A1_Motor_Init(A1_Motor_left_2);
    }
    if(huart == &huart4)
    {
        A1_Motor_Init(A1_Motor_right_1);
    }
    if(huart == &huart7)
    {
        A1_Motor_Init(A1_Motor_right_2);
    }
    if(huart == &huart8)
    {
        Facial_expression_init();
    }
    if(huart == &huart10)
    {
        ELRS_Init();
    }

}
