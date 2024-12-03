/********************************************************************************
* @File name: EventDBscan.h
* @Author: 石海新
* @Version: 1.0
* @Date: 2023.6.8
* @Description: EventDBscan聚类头文件
********************************************************************************/
#ifndef USER_EVENTDBSCAN_H
#define USER_EVENTDBSCAN_H

#include "track_common.h"

#ifdef __cplusplus
extern "C" {
#endif

#define MIN_POINTS          3 // 最小点数阈值

extern AdjustParams adjust_Params;

int Event_Numel(float *neighbors);
void Event_Distance(ARS408RadarObjectInfo *Outdata, float *distance);
void Event_dbscan(ARS408RadarObjectInfo *Outdata, int *Outlength, int *Cluster);
void Event_Regionquery(int i, float *neighbors, float *distance, int *Outlength);
void Event_Expand_Cluster(int *Cluster, int i, float *neighbors, int Cluster_Id, int *visited, float *distance, int *Outlength);

#ifdef __cplusplus
}
#endif

#endif