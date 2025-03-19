//
// Created by w1445 on 2024/11/27.
//

#ifndef BALANCE_LEG_V2_CH010_HI91_H
#define BALANCE_LEG_V2_CH010_HI91_H

#include "main.h"

typedef struct __attribute__((__packed__))
{
    uint8_t         tag;            /* Data packet tag, if tag = 0x00, means that this packet is null */
    uint16_t        pps_sync_stamp; /* PPS synchronization time in milliseconds */
    int8_t          temp;           /* Temperature */
    float           air_pressure;   /* Pressure */
    uint32_t        system_time;    /* Timestamp */
    float           acc[3];         /* Accelerometer data (x, y, z) */
    float           gyr[3];         /* Gyroscope data (x, y, z) */
    float           mag[3];         /* Magnetometer data (x, y, z) */
    float           roll;           /* Roll angle */
    float           pitch;          /* Pitch angle */
    float           yaw;            /* Yaw angle */
    float           last_yaw;
    float           total_yaw;
    float           quat[4];        /* Quaternion (w, x, y, z) */
    uint8_t         eorror_crc_count;
    uint16_t        eorror_check;
    uint16_t        rx_counter;
    uint16_t        FPS;
} HI91_T;

void CH010_HI91_Init(void);
void Get_total_yaw(HI91_T *hi91_date);
void HI91_UARTE_RxCallback(uint16_t Size,HI91_T *hi91_t);

extern  HI91_T hi91_data;

#endif //BALANCE_LEG_V2_CH010_HI91_H
