
/********************************************************************************
 * @File name: TrafficFlowDetection.h
 * @Author: 高露
 * @Version: 1.0
 * @Date: 2023.6.14
 * @Description: 车流量检测器功能头文件
 ********************************************************************************/

// #pragma once
#ifndef USER_TRAFFICFLOWDETECTION_H
#define USER_TRAFFICFLOWDETECTION_H

#include "track_common.h"

#ifdef __cplusplus
extern "C" {
#endif

#define LEN 10 // 分车道统计量最大容量
#define TRACK_NUM 150

typedef struct _enableTrafficDetrction
{
	// 最大车道数量
	int MaxLaneNum;

	// 车流量
	int EnabletrafficFlow;
	// 分车道车流量
	int EnablelaneDividedTrafficFlow;

	// 断面平均速度帧计数器
	int sectionAverageSpeedCount;
	// 断面平均速度开始帧号记录
	double sectionAverageSpeedTimestamp;
	// 断面平均速度
	int EnablesectionAverageSpeed;

	// 分车道平均速度帧计数器
	int laneDividedAverageSpeedCount;
	// 分车道平均速度开始帧号记录
	double laneDividedAverageSpeedTimestamp;
	// 分车道平均速度
	int EnablelaneDividedAverageSpeed;

	// 分车道时间占有率帧计数器
	int laneDividedTimeOccupancyCount;
	// 分车道时间占有率开始帧号记录
	double laneDividedTimeOccupancyTimestamp;
	// 分车道时间占有率
	int EnablelaneDividedTimeOccupancy;

	// 分车道车头时距帧计数器
	int laneDividedAverageHeadwayCount;
	// 分车道车头时距开始帧号记录
	double laneDividedAverageHeadwayTimestamp;
	// 分车道车头时距
	int EnablelaneDividedAverageHeadway;

	// 分车道车间时距帧计数器
	int laneDividedAverageHeadwayGapCount;
	// 分车道车间时距开始帧号记录
	double laneDividedAverageHeadwayGapTimestamp;
	// 分车道车间时距
	int EnablelaneDividedAverageHeadwayGap;

} EnableTrafficDetrctionDef;

// 输出统计数据
typedef struct _TrafficFlowDetectionStatistics
{
	// 车流量
	int trafficFlowData;
	// 分车道车流量
	int laneDividedTrafficFlowData[LEN];

	// 按照车型统计车流量
	// 大车车流量
	int bigVehicleFlowData;
	// 中车车流量
	int middleVehicleFlowData;
	// 小车车流量
	int smallVehicleFlowData;

	// 平均速度相关参数
	// 总速度计数器
	float speedCount;
	// 平均速度车流量总数
	int speedvehicleCount;
	// 平均速度
	float sectionAverageSpeedData;

	// 分车道平均速度相关参数
	// 总速度计数器
	float laneDividedspeedCount[LEN];
	// 平均速度车流量总数
	int laneDividedspeedvehicleCount[LEN];
	// 分车道平均速度
	float laneDividedAverageSpeedData[LEN];

	// 分车道时间占有率
	// 分车道时间计数器
	float laneDividedTimeOccupancySum[LEN];
	// 最终分车道时间占有率
	float laneDividedTimeOccupancyData[LEN];

	// 分车道平均车头时距
	float laneDividedAverageHeadwayData[LEN];
	// 分车道平均车间时距
	float laneDividedAverageHeadwayGapData[LEN];

} TrafficFlowDetectionStatisticsDef;

// id memory
typedef struct _IDMemory
{
	// 前一帧id
	int oldIDMemory[LEN * 2];
	// 后一帧id
	int newIDMemory[LEN * 2];
} IDMemoryDef;

// ID存储器
typedef struct _IDMemoryAll
{

	IDMemoryDef trafficFlowidMemoryDef;
	IDMemoryDef laneDividedtrafficFlowidMemoryDef;
	IDMemoryDef speedvehicleidMemoryDef;
	IDMemoryDef laneDividedspeedvehicleidMemoryDef;
	IDMemoryDef laneDividedHeadwayidMemoryDef;
	IDMemoryDef laneDividedTimeOccupancyidMemoryDef;
	IDMemoryDef laneDividedHeadwayGapidMemoryDef;
} IDMemoryAllDef;

// 链表节点
struct LinkNodeV
{
	double timestamp;
	struct LinkNodeV *next;
};

// typedef struct _laneDividedAverageHeadwayList {
//	struct  LinkNodeV* header1;
//	struct  LinkNodeV* header2;
//	struct  LinkNodeV* header3;
//	struct  LinkNodeV* header4;
//
// }laneDividedAverageHeadwayListHeader;

typedef struct _headerAll
{
	// 车头时距头指针
	struct LinkNodeV *headers[LEN];
	// 车间时距头指针
	struct LinkNodeV *headergap[LEN];
} headerAllDef;

extern TrafficFlowDetectionStatisticsDef trafficFlowDetectionStatisticsDef;

extern EnableTrafficDetrctionDef enableTrafficDetrctionDef;

extern IDMemoryAllDef idMemoryAllDef;

extern headerAllDef had;

extern SectionAndCoilRegionDef sectionAndCoilRegionDef;

extern SectionAndCoilRegionDef sections[LEN];

extern AdjustParams adjust_Params;

/* 函数声明 */
// 车流量检测器函数——传入一帧目标数据
void trafficFlowDetection(const ARS408RadarObjectInfo *RadarObjectInfoFrame, int NumberofPointPerFrame);
/* 是否有目标经过断面 */
bool objectDetectionSignal(const ARS408RadarObjectInfo *RadarObjectInfoFrame, int NumberofPointPerFrame);
/* 车流量 */
int trafficFlow(const ARS408RadarObjectInfo *RadarObjectInfoFrame, int NumberofPointPerFrame);
/* 分车道车流量 */
int laneDividedTrafficFlow(const ARS408RadarObjectInfo *RadarObjectInfoFrame, int NumberofPointPerFrame);
/* 计算断面一段时间内总速度和车流量 */
int sectionallSpeed(const ARS408RadarObjectInfo *RadarObjectInfoFrame, int NumberofPointPerFrame);
/* 计算断面平均车速 */
int sectionAverageSpeed(const ARS408RadarObjectInfo *RadarObjectInfoFrame, int NumberofPointPerFrame);
/* 分车道平均车速 总和计算 */
int laneDividedallSpeed(const ARS408RadarObjectInfo *RadarObjectInfoFrame, int NumberofPointPerFrame);
/* 分车道平均车速 */
int laneDividedAverageSpeed(const ARS408RadarObjectInfo *RadarObjectInfoFrame, int NumberofPointPerFrame);
/* 分车道时间占有率 */
int laneDividedTimeOccupancy(const ARS408RadarObjectInfo *RadarObjectInfoFrame, int NumberofPointPerFrame);
/* 最终分车道时间占有率 */
int laneDividedTimeOccupancyFinal(const ARS408RadarObjectInfo *RadarObjectInfoFrame, int NumberofPointPerFrame);
/* 分车道平均车头时距 总和计算 */
int laneDividedAverageHeadwayAllList(const ARS408RadarObjectInfo *RadarObjectInfoFrame, int NumberofPointPerFrame);
/* 分车道平均车头时距 */
int laneDividedAverageHeadway(const ARS408RadarObjectInfo *RadarObjectInfoFrame, int NumberofPointPerFrame);
///* 分车道平均车间时距 总和计算  */
int laneDividedAverageHeadwayGapAllList(const ARS408RadarObjectInfo *RadarObjectInfoFrame, int NumberofPointPerFrame);
/* 分车道平均车间时距 */
int laneDividedAverageHeadwayGap(const ARS408RadarObjectInfo *RadarObjectInfoFrame, int NumberofPointPerFrame);
// 初始化链表
struct LinkNodeV *Init_LinkListV();
// 插入，尾
void InsertLink_LinkListV(struct LinkNodeV *header, double newval);
// 销毁
void Clear_LinkListV(struct LinkNodeV *header);
// 车头时距遍历
float Foreach_LinkListV(struct LinkNodeV *header);
// 车间时距遍历
float HeadwayGap_LinkListV(struct LinkNodeV *header);

#ifdef __cplusplus
}
#endif

#endif
