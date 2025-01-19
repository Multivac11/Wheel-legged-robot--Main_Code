//
// Created by w1445 on 2025/1/14.
//

#ifndef BALANCE_LEG_V2_FILTER_H
#define BALANCE_LEG_V2_FILTER_H

#include "main.h"

struct LowPassFilter_Info{
    float filter_coefficient;
    float last_output;
    float output;
    float sampling;
};

float LPFilter(float sampling ,struct LowPassFilter_Info *LPF);

extern struct LowPassFilter_Info LPF_pitch_speed;

#endif //BALANCE_LEG_V2_FILTER_H
