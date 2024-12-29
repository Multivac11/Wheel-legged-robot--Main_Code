//
// Created by w1445 on 2024/11/26.
//

#include "ELRS_Drive.h"
#include "usart.h"
#include <string.h>

float float_Map(float input_value, float input_min, float input_max, float output_min, float output_max)
{
    float output_value;
    if (input_value < input_min)
    {
        output_value = output_min;
    }
    else if (input_value > input_max)
    {
        output_value = output_max;
    }
    else
    {
        output_value = output_min + (input_value - input_min) * (output_max - output_min) / (input_max - input_min);
    }
    return output_value;
}
float float_Map_with_median(float input_value, float input_min, float input_max, float median, float output_min, float output_max)
{
    float output_median = (output_max - output_min) / 2 + output_min;
    if (input_min >= input_max || output_min >= output_max || median <= input_min || median >= input_max)
    {
        return output_min;
    }

    if (input_value < median)
    {
        return float_Map(input_value, input_min, median, output_min, output_median);
    }
    else
    {
        return float_Map(input_value, median, input_max, output_median, output_max);
    }
}
extern DMA_HandleTypeDef hdma_usart10_rx;

uint8_t elrs_data_temp[36] = {0};
void ELRS_Init(void)
{
    HAL_UARTEx_ReceiveToIdle_DMA(&huart10, elrs_data_temp, MAX_FRAME_SIZE); // 启用空闲中断接收
    __HAL_DMA_DISABLE_IT(&hdma_usart10_rx, DMA_IT_HT);                      // 关闭DMA传输过半中断
}

ELRS_Data elrs_data;
void ELRS_UARTE_RxCallback(uint16_t Size)
{

    // printf("elrs_data_temp\r\n");
    // (elrs_data_temp[i] == CRSF_ADDRESS_FLIGHT_CONTROLLER) && (elrs_data_temp[i + 1] == FrameLength) && (elrs_data_temp[i + 2] == CRSF_FRAMETYPE_RC_CHANNELS_PACKED);
    if (elrs_data_temp[0] == CRSF_ADDRESS_FLIGHT_CONTROLLER)
    {
        if (elrs_data_temp[2] == CRSF_FRAMETYPE_RC_CHANNELS_PACKED) // 数据帧类型为RC通道数据
        {
            elrs_data.channels[0] = ((uint16_t)elrs_data_temp[3] >> 0 | ((uint16_t)elrs_data_temp[4] << 8)) & 0x07FF;
            elrs_data.channels[1] = ((uint16_t)elrs_data_temp[4] >> 3 | ((uint16_t)elrs_data_temp[5] << 5)) & 0x07FF;
            elrs_data.channels[2] = ((uint16_t)elrs_data_temp[5] >> 6 | ((uint16_t)elrs_data_temp[6] << 2) | ((uint16_t)elrs_data_temp[7] << 10)) & 0x07FF;
            elrs_data.channels[3] = ((uint16_t)elrs_data_temp[7] >> 1 | ((uint16_t)elrs_data_temp[8] << 7)) & 0x07FF;
            elrs_data.channels[4] = ((uint16_t)elrs_data_temp[8] >> 4 | ((uint16_t)elrs_data_temp[9] << 4)) & 0x07FF;
            elrs_data.channels[5] = ((uint16_t)elrs_data_temp[9] >> 7 | ((uint16_t)elrs_data_temp[10] << 1) | ((uint16_t)elrs_data_temp[11] << 9)) & 0x07FF;
            elrs_data.channels[6] = ((uint16_t)elrs_data_temp[11] >> 2 | ((uint16_t)elrs_data_temp[12] << 6)) & 0x07FF;
            elrs_data.channels[7] = ((uint16_t)elrs_data_temp[12] >> 5 | ((uint16_t)elrs_data_temp[13] << 3)) & 0x07FF;
            elrs_data.channels[8] = ((uint16_t)elrs_data_temp[14] >> 0 | ((uint16_t)elrs_data_temp[15] << 8)) & 0x07FF;
            elrs_data.channels[9] = ((uint16_t)elrs_data_temp[15] >> 3 | ((uint16_t)elrs_data_temp[16] << 5)) & 0x07FF;
            elrs_data.channels[10] = ((uint16_t)elrs_data_temp[16] >> 6 | ((uint16_t)elrs_data_temp[17] << 2) | ((uint16_t)elrs_data_temp[18] << 10)) & 0x07FF;
            elrs_data.channels[11] = ((uint16_t)elrs_data_temp[18] >> 1 | ((uint16_t)elrs_data_temp[19] << 7)) & 0x07FF;
            elrs_data.channels[12] = ((uint16_t)elrs_data_temp[19] >> 4 | ((uint16_t)elrs_data_temp[20] << 4)) & 0x07FF;
            elrs_data.channels[13] = ((uint16_t)elrs_data_temp[20] >> 7 | ((uint16_t)elrs_data_temp[21] << 1) | ((uint16_t)elrs_data_temp[22] << 9)) & 0x07FF;
            elrs_data.channels[14] = ((uint16_t)elrs_data_temp[22] >> 2 | ((uint16_t)elrs_data_temp[23] << 6)) & 0x07FF;
            elrs_data.channels[15] = ((uint16_t)elrs_data_temp[23] >> 5 | ((uint16_t)elrs_data_temp[24] << 3)) & 0x07FF;
            elrs_data.Right_X = float_Map_with_median(elrs_data.channels[3], 174, 1808, 992, -100, 100);
            elrs_data.Left_Y = float_Map_with_median(elrs_data.channels[2], 174, 1811, 992, 0, 100);
            elrs_data.Left_X = float_Map_with_median(elrs_data.channels[0], 174, 1811, 992, -100, 100);
            elrs_data.Right_Y = float_Map_with_median(elrs_data.channels[1], 174, 1808, 992, -100, 100);
            elrs_data.S1 = float_Map_with_median(elrs_data.channels[8], 191, 1792, 992, 0, 100);
            elrs_data.S2 = float_Map_with_median(elrs_data.channels[9], 191, 1792, 992, 0, 100);
            elrs_data.A = elrs_data.channels[10] > 1000 ? 1 : 0;
            elrs_data.B = elrs_data.channels[5] == 992 ? 1 : (elrs_data.channels[5] == 1792 ? 2 : 0);
            elrs_data.C = elrs_data.channels[6] == 992 ? 1 : (elrs_data.channels[6] == 1792 ? 2 : 0);
            elrs_data.D = elrs_data.channels[11] > 1000 ? 1 : 0;
            elrs_data.E = elrs_data.channels[4] == 992 ? 1 : (elrs_data.channels[4] == 1792 ? 2 : 0);
            elrs_data.F = elrs_data.channels[7] == 992 ? 1 : (elrs_data.channels[7] == 1792 ? 2 : 0);

        }
        else if (elrs_data_temp[2] == CRSF_FRAMETYPE_LINK_STATISTICS)
        {
            elrs_data.uplink_RSSI_1 = elrs_data_temp[3];
            elrs_data.uplink_RSSI_2 = elrs_data_temp[4];
            elrs_data.uplink_Link_quality = elrs_data_temp[5];
            elrs_data.uplink_SNR = (int8_t )elrs_data_temp[6];
            elrs_data.active_antenna = elrs_data_temp[7];
            elrs_data.rf_Mode = elrs_data_temp[8];
            elrs_data.uplink_TX_Power = elrs_data_temp[9];
            elrs_data.downlink_RSSI = elrs_data_temp[10];
            elrs_data.downlink_Link_quality = elrs_data_temp[11];
            elrs_data.downlink_SNR = (int8_t )elrs_data_temp[12];
            // printf("link_quality=%d,uplink_TX_Power=%d\r\n", elrs_data.uplink_Link_quality, elrs_data.uplink_TX_Power);
        }
        else if (elrs_data_temp[2] == CRSF_FRAMETYPE_HEARTBEAT)
        {
            elrs_data.heartbeat_counter = elrs_data_temp[3];
        }
        else
        {
            // printf("Not support this frame type\r\n");
        }
    }

    memset(elrs_data_temp, 0, sizeof(elrs_data_temp));
    ELRS_Init();
}
