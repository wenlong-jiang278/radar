/********************************************************************************
* @File name: ExtraJudge.h
* @Author: 石海新
* @Version: 1.0
* @Date: 2023.6.7
* @Description: 对滤波后数据额外判断头文件
********************************************************************************/
// #pragma once
#ifndef USER_EXTRAJUDGE_H
#define USER_EXTRAJUDGE_H

#include "List.h"
#include "track_common.h"
#include "utils.h"

#ifdef __cplusplus
extern "C" {
#endif

// 惯性部分参数
#define Inertia_frame 15 //惯性持续帧数
#define Inertia_speed 0.8 //惯性速度降低倍率
#define Inertia_time 0.08 //每一帧惯性持续时间

extern AdjustParams adjust_Params;

void Inertia_Add(struct list* List_stop, int FrameStop);
void Stop_Dect(ARS408RadarObjectInfo* Outdata, int* Outlength, Event_Params* event_params);
void Lane_Judge(ARS408RadarObjectInfo* Outdata, int* Outlength);
void Extra_Judge(ARS408RadarObjectInfo* Outdata, int* Outlength, Event_Params* event_params);

#ifdef __cplusplus
}
#endif

#endif
