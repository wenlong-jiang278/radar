/********************************************************************************
* @File name: trafficEvent.h
* @Author: 石海新
* @Version: 1.0
* @Date: 2023.6.7
* @Description: 事件检测头文件
********************************************************************************/
// #pragma once
#ifndef USER_TRAFFICEVENT_H
#define USER_TRAFFICEVENT_H

#include <time.h>
#include <sys/time.h>

#include "List.h"
#include "track_common.h"
#include "utils.h"
#include "EventDBscan.h"

#ifdef __cplusplus
extern "C" {
#endif

#define PI 3.1415926

extern AdjustParams adjust_Params;

// 计算函数执行时间的函数
double calc_elapsed_time(struct timeval start, struct timeval end);

void TrafficEventRecongition(ARS408RadarObjectInfo* Outdata, int* Outlength, Event_Params* event_params);
void Speed_Dect(ARS408RadarObjectInfo* Outdata, int* Outlength);
void Retrograde_Dect(ARS408RadarObjectInfo* Outdata, int* Outlength);
void LaneChange_Dect(ARS408RadarObjectInfo* Outdata, int* Outlength, Event_Params* event_params);
void StopCar_Dect(ARS408RadarObjectInfo* Outdata, int* Outlength);
void Congest_Dect(ARS408RadarObjectInfo* Outdata, int* Outlength);

#ifdef __cplusplus
}
#endif

#endif
