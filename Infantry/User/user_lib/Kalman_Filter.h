#ifndef __KALMAN_FILTER_H
#define __KALMAN_FILTER_H
#include "main.h"

typedef struct 
{
    fp32 LastP;    //上次估算协方差 初始化值为0.02
    fp32 Now_P;    //当前估算协方差 初始化值为0
    fp32 out;      //卡尔曼滤波器输出 初始化值为0
    fp32 Kg;       //卡尔曼增益 初始化值为0
    fp32 Q;        //过程噪声协方差 初始化值为0.001
    fp32 R;        //观测噪声协方差 初始化值为0.543
}KalmanInfo;        //Kalman Filter parameter

void Kalman_Filter_Init(KalmanInfo *KalmanInfo_Structure);
fp32 Kalman_Filter_Fun(KalmanInfo *info, fp32 new_value);

#endif
