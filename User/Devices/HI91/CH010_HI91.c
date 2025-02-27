//
// Created by w1445 on 2024/11/27.
//

#include "CH010_HI91.h"
#include "usart.h"
#include <string.h>


float hexToFloat(uint32_t hexValue)
{
    // 通过指针转换将 32 位整数视为浮点数
    float result;
    *((uint32_t*)&result) = hexValue;
    return result;
}

HI91_T hi91_data;
static void hipnuc_crc16(uint16_t *inital, const uint8_t *buf, uint32_t len);
extern DMA_HandleTypeDef hdma_usart2_rx;
__attribute__((section("._D1_Area"))) uint8_t hi91_data_temp[82] = {0};
void CH010_HI91_Init(void)
{
    HAL_UARTEx_ReceiveToIdle_DMA(&huart2, hi91_data_temp, 82); // 启用空闲中断接收
    __HAL_DMA_DISABLE_IT(&hdma_usart2_rx, DMA_IT_HT);                      // 关闭DMA传输过半中断
}


void HI91_UARTE_RxCallback(uint16_t Size,HI91_T *hi91_date)
{
    uint16_t payload_len;
    uint16_t crc;
    crc = 0;
    payload_len = hi91_data_temp[2] + (hi91_data_temp[3] << 8);
    hipnuc_crc16(&crc , hi91_data_temp , 4);
    hipnuc_crc16(&crc , hi91_data_temp + 6, payload_len);

    if(crc == ((hi91_data_temp[5]<<8)+hi91_data_temp[4]))
    {
        hi91_date->rx_counter++;
        hi91_date->pps_sync_stamp = hi91_data_temp[7] + (hi91_data_temp[8] << 8);
        hi91_date->pps_sync_stamp = (uint16_t)(hi91_data_temp[7] + (hi91_data_temp[8] << 8));
        hi91_date->temp = (int8_t)(hi91_data_temp[9]);
        hi91_date->air_pressure = (float)(hi91_data_temp[10] + (hi91_data_temp[11] << 8) + (hi91_data_temp[12] << 16) + (hi91_data_temp[13] << 24));
        hi91_date->system_time =(hi91_data_temp[14] + (hi91_data_temp[15] << 8) + (hi91_data_temp[16] << 16) + (hi91_data_temp[17] << 24));
        //pitch/x加速度
        hi91_date->acc[0]  = hexToFloat(hi91_data_temp[18] + (hi91_data_temp[19] << 8) + (hi91_data_temp[20] << 16) + (hi91_data_temp[21] << 24));
        //roll/y加速度
        hi91_date->acc[1]  = hexToFloat(hi91_data_temp[22] + (hi91_data_temp[23] << 8) + (hi91_data_temp[24] << 16) + (hi91_data_temp[25] << 24));
        //yaw/z加速度
        hi91_date->acc[2]  = hexToFloat(hi91_data_temp[26] + (hi91_data_temp[27] << 8) + (hi91_data_temp[28] << 16) + (hi91_data_temp[29] << 24));
        //pitch/x角速度dps
        hi91_date->gyr[0]  = hexToFloat(hi91_data_temp[30] + (hi91_data_temp[31] << 8) + (hi91_data_temp[32] << 16) + (hi91_data_temp[33] << 24));
        //roll/y角速度dps
        hi91_date->gyr[1]  = hexToFloat(hi91_data_temp[34] + (hi91_data_temp[35] << 8) + (hi91_data_temp[36] << 16) + (hi91_data_temp[37] << 24));
        //yaw/z角速度dps
        hi91_date->gyr[2]  = hexToFloat(hi91_data_temp[38] + (hi91_data_temp[39] << 8) + (hi91_data_temp[40] << 16) + (hi91_data_temp[41] << 24));
        hi91_date->mag[0]  = hexToFloat(hi91_data_temp[42] + (hi91_data_temp[43] << 8) + (hi91_data_temp[44] << 16) + (hi91_data_temp[45] << 24));
        hi91_date->mag[1]  = hexToFloat(hi91_data_temp[46] + (hi91_data_temp[47] << 8) + (hi91_data_temp[48] << 16) + (hi91_data_temp[49] << 24));
        hi91_date->mag[2]  = hexToFloat(hi91_data_temp[50] + (hi91_data_temp[51] << 8) + (hi91_data_temp[52] << 16) + (hi91_data_temp[53] << 24));
        hi91_date->roll    = hexToFloat(hi91_data_temp[54] + (hi91_data_temp[55] << 8) + (hi91_data_temp[56] << 16) + (hi91_data_temp[57] << 24));
        hi91_date->pitch   = hexToFloat(hi91_data_temp[58] + (hi91_data_temp[59] << 8) + (hi91_data_temp[60] << 16) + (hi91_data_temp[61] << 24));
        hi91_date->yaw     = hexToFloat(hi91_data_temp[62] + (hi91_data_temp[63] << 8) + (hi91_data_temp[64] << 16) + (hi91_data_temp[65] << 24));
        hi91_date->quat[0] = hexToFloat(hi91_data_temp[66] + (hi91_data_temp[67] << 8) + (hi91_data_temp[68] << 16) + (hi91_data_temp[69] << 24));
        hi91_date->quat[1] = hexToFloat(hi91_data_temp[70] + (hi91_data_temp[71] << 8) + (hi91_data_temp[72] << 16) + (hi91_data_temp[73] << 24));
        hi91_date->quat[2] = hexToFloat(hi91_data_temp[74] + (hi91_data_temp[75] << 8) + (hi91_data_temp[76] << 16) + (hi91_data_temp[77] << 24));
        hi91_date->quat[3] = hexToFloat(hi91_data_temp[78] + (hi91_data_temp[79] << 8) + (hi91_data_temp[80] << 16) + (hi91_data_temp[81] << 24));
        memset(hi91_data_temp, 0, sizeof(hi91_data_temp));
    }
    else
    {
        hi91_date->eorror_crc_count++;
    }

    CH010_HI91_Init();
}

/**
 * @brief    Calculate HiPNUC CRC16
 *
 * @param    inital is initial value
 * @param    buf    is input buffer pointer
 * @param    len    is length of the buffer
 */
static void hipnuc_crc16(uint16_t *inital, const uint8_t *buf, uint32_t len)
{
    uint32_t crc = *inital;
    uint32_t j;
    for (j=0; j < len; ++j)
    {
        uint32_t i;
        uint32_t byte = buf[j];
        crc ^= byte << 8;
        for (i = 0; i < 8; ++i)
        {
            uint32_t temp = crc << 1;
            if (crc & 0x8000)
            {
                temp ^= 0x1021;
            }
            crc = temp;
        }
    }
    *inital = crc;
}
