//
// Created by w1445 on 2024/11/26.
//

#ifndef BALANCE_LEG_V2_ELRS_DRIVE_H
#define BALANCE_LEG_V2_ELRS_DRIVE_H

#include "main.h"
#define MAX_FRAME_SIZE 36 // 空闲中断接收的最大帧长

/*
https://github.com/crsf-wg/crsf/wiki/Packet-Types CRSF协议定义
 */
/*
通道1：右摇杆 x轴
通道2：右摇杆 y轴
通道3：左摇杆 y轴
通道4：左摇杆 x轴
通道5：拨杆F
通道6：拨杆B
通道7：拨杆C
通道8：拨杆F
通道9：左滑块
通道10：右滑块
通道11：左按键A
通道12：右按键B
通道13：
通道14：
通道15：
通道16：
 */
/*
帧格式定义 地址 + 数据长度 + 帧类型 +  数据 + CRC校验码
 */

// 帧类型定义
#define CRSF_FRAMETYPE_GPS 0x02                      // GPS帧类型
#define CRSF_FRAMETYPE_BATTERY_SENSOR 0x08           // 电池传感器帧类型
#define CRSF_FRAMETYPE_LINK_STATISTICS 0x14          // 链路统计帧类型
#define CRSF_FRAMETYPE_OPENTX_SYNC 0x10              // OpenTX同步帧类型
#define CRSF_FRAMETYPE_RADIO_ID 0x3A                 // 无线电ID帧类型
#define CRSF_FRAMETYPE_RC_CHANNELS_PACKED 0x16       // 遥控器通道打包帧类型
#define CRSF_FRAMETYPE_ATTITUDE 0x1E                 // 姿态帧类型
#define CRSF_FRAMETYPE_FLIGHT_MODE 0x21              // 飞行模式帧类型
#define CRSF_FRAMETYPE_DEVICE_PING 0x28              // 设备Ping帧类型
#define CRSF_FRAMETYPE_DEVICE_INFO 0x29              // 设备信息帧类型
#define CRSF_FRAMETYPE_PARAMETER_SETTINGS_ENTRY 0x2B // 参数设置条目帧类型
#define CRSF_FRAMETYPE_PARAMETER_READ 0x2C           // 参数读取帧类型
#define CRSF_FRAMETYPE_PARAMETER_WRITE 0x2D          // 参数写入帧类型
#define CRSF_FRAMETYPE_COMMAND 0x32                  // 命令帧类型
#define CRSF_FRAMETYPE_MSP_REQ 0x7A                  // 使用MSP序列作为命令的响应请求
#define CRSF_FRAMETYPE_MSP_RESP 0x7B                 // 以58字节分块二进制形式回复
#define CRSF_FRAMETYPE_MSP_WRITE 0x7C                // 以8字节分块二进制形式写入（OpenTX出站遥测缓冲区限制）
#define CRSF_FRAMETYPE_HEARTBEAT 0x0B                // （CRSFv3）心跳

// 地址定义
#define CRSF_ADDRESS_BROADCAST 0x00         // 广播地址
#define CRSF_ADDRESS_USB 0x10               // USB地址
#define CRSF_ADDRESS_TBS_CORE_PNP_PRO 0x80  // TBS Core PNP Pro地址
#define CRSF_ADDRESS_RESERVED1 0x8A         // 保留地址1
#define CRSF_ADDRESS_CURRENT_SENSOR 0xC0    // 电流传感器地址
#define CRSF_ADDRESS_GPS 0xC2               // GPS地址
#define CRSF_ADDRESS_TBS_BLACKBOX 0xC4      // TBS黑匣子地址
#define CRSF_ADDRESS_FLIGHT_CONTROLLER 0xC8 // 飞行控制器地址
#define CRSF_ADDRESS_RESERVED2 0xCA         // 保留地址2
#define CRSF_ADDRESS_RACE_TAG 0xCC          // 比赛标签地址
#define CRSF_ADDRESS_RADIO_TRANSMITTER 0xEA // 无线电发射器地址
#define CRSF_ADDRESS_CRSF_RECEIVER 0xEC     // CRSF接收机地址
#define CRSF_ADDRESS_CRSF_TRANSMITTER 0xEE  // CRSF发射机地址

#define CHANNELS_Frame_Length 0x18 // 通道帧长度
#define LINK_Frame_Length 0x0C     // 连接帧长度

typedef struct
{
    // CRSF_FRAMETYPE_RC_CHANNELS_PACKED  遥控器通道打包帧类型
    uint16_t channels[16]; // 存储16个通道的数据
    float Left_X;          // 左摇杆x轴
    float Left_Y;          // 左摇杆y轴
    float Right_X;         // 右摇杆x轴
    float Right_Y;         // 右摇杆y轴
    uint16_t S1;              // 左滑块
    uint16_t S2;              // 右滑块
    uint8_t A;             // 按键A
    uint8_t B;             // 拨杆B
    uint8_t C;             // 拨杆C
    uint8_t D;             // 按键D
    uint8_t E;             // 拨杆E
    uint8_t F;             // 拨杆F

    // CRSF_FRAMETYPE_LINK_STATISTICS  链路统计帧类型
    uint8_t uplink_RSSI_1;         // 上行RSSI 1
    uint8_t uplink_RSSI_2;         // 上行RSSI 2
    uint8_t uplink_Link_quality;   // 上行链路质量
    int8_t uplink_SNR;             // 上行信噪比
    uint8_t active_antenna;        // 当前活跃天线
    uint8_t rf_Mode;               // 射频模式
    uint8_t uplink_TX_Power;       // 上行传输功率  0mW 10mW 25mW 100mW 500mW 1000mW 2000mW 50mW
    uint8_t downlink_RSSI;         // 下行RSSI
    uint8_t downlink_Link_quality; // 下行链路质量
    int8_t downlink_SNR;           // 下行信噪比

    // CRSF_FRAMETYPE_HEARTBEAT 心跳
    uint16_t heartbeat_counter; // 心跳计数器
    uint16_t rx_counter;
    uint16_t FPS;
    uint16_t Online_counter;
    uint8_t  isOnline;

} ELRS_Data;

extern ELRS_Data elrs_data;

void ELRS_Init(void);
void ELRS_UARTE_RxCallback(uint16_t Size);

#endif //BALANCE_LEG_V2_ELRS_DRIVE_H
