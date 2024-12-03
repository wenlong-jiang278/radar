/********************************************************************************
 * @File name: TrackDbscan.h
 * @Author: 石海新
 * @Version: 1.0
 * @Date: 2023.4.26
 * @Description: DBSCAN聚类头文件
 ********************************************************************************/

#ifndef USER_TRACKDBSCAN_H
#define USER_TRACKDBSCAN_H

#include "utils.h"
#include "track_common.h"

#ifdef __cplusplus
extern "C" {
#endif

// #pragma once
// 聚类参数
#define DBscanEps_Dist 1
#define DBscanEps_Velo 0.5
#define DBscanPotsNumb 2

//2024.09.18前的版本
// #define TrackEllipse_a 10
// #define TrackEllipse_b 4

//2024.09.18后的版本
#define TrackEllipse_a 10
#define TrackEllipse_b 5

extern AdjustParams adjust_Params;

void Distance_differ(int i, track_float_t *Range_x, track_float_t *Range_y, uint32_t NumberofCdi, track_float_t *TableofDistanceDiffe, track_float_t lamada_r);
void Velocity_differ(track_float_t *Velocity, uint32_t NumberofCdi, track_float_t *TableofVelocityDiffe);
void Regionquery(int i, track_float_t *neighbors, track_float_t *TableofDistanceDiffe, track_float_t *TableofVelocityDiffe, uint32_t NumberofCdi);
int Numel(track_float_t *neighbors);
void Expand_Cluster(int *Cluster, int i, track_float_t *neighbors, uint32_t *NumberofCluster, track_float_t *visited, track_float_t *TableofDistanceDiffe, track_float_t *TableofVelocityDiffe, uint32_t NumberofCdi, track_float_t *Range_x, track_float_t *Range_y, track_float_t lamada_r);
void Track_DBSCAN(int32_t *IDX, track_float_t *Range, track_float_t *Range_x, track_float_t *Range_y, track_float_t *Velocity, track_float_t *Azimuth, track_float_t *Power, uint32_t NumberofCdi, uint32_t *NumberofCluster);
void Secondary_Cluster(int32_t *IDX, track_float_t *Range, track_float_t *Range_x, track_float_t *Range_y, track_float_t *Azimuth, uint32_t NumberofCdi, uint32_t *NumberofCluster);
track_float_t Compare_distance(int32_t *IDX, int32_t IDX1, int32_t IDX2, track_float_t *Range_x, track_float_t *Range_y, uint32_t NumberofCdi, track_float_t *Azimuth);

#ifdef __cplusplus
}
#endif

#endif
