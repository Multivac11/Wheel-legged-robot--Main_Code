//
// Created by w1445 on 2025/1/14.
//

#include "Filter.h"

struct LowPassFilter_Info LPF_pitch_speed={
        .filter_coefficient=1.0f,
        .last_output=0,
};


float LPFilter(float sampling ,struct LowPassFilter_Info *LPF){
    //一阶低通滤波器：p(n) = c·q(n) + (1 - c)·p(n - 1)
    (*LPF).sampling =sampling;

    (*LPF).output=(*LPF).filter_coefficient *(*LPF).sampling +(1-(*LPF).filter_coefficient)*(*LPF).last_output;

    (*LPF).last_output =(*LPF).output ;

    return (*LPF).output ;
};


