/********************************************************************************
* @File name: trafficEvent.c
* @Author: 石海新
* @Version: 1.0
* @Date: 2023.6.7
* @Description: 事件检测功能文件
********************************************************************************/
#include "trafficEvent.h"

#include <map>

extern std::map<int, RoadLaneParam> gRoadLaneParam;

// 计算函数执行时间的函数
double calc_elapsed_time(struct timeval start, struct timeval end)
{
    double elapsed;
    long seconds, microseconds;

    seconds = end.tv_sec - start.tv_sec;
    microseconds = end.tv_usec - start.tv_usec;
    elapsed = seconds * 1000 + microseconds / 1000.0;

    return elapsed;
}

/********************************************************
* Function name ：Speed_Dect
* Description   ：实现对车辆的超速事件检测
* Parameter     ：
* @Outdata        滤波生成的目标数据
* @Outlength      目标数据的长度的指针
* @m              雷达的俯仰角
* @ret            雷达的俯仰角(弧度制)
* Return        ：Outdata
**********************************************************/
void Speed_Dect(ARS408RadarObjectInfo *Outdata, int *Outlength)
{
    int i = 0;
    double m = 0;
    double val = PI / 180.0;
    double ret = m * val;    // 俯仰角转弧度

    for (i = 0; i < *Outlength; i++) {
        if (((Outdata[i].Object_VrelLong * Outdata[i].Object_VrelLong + Outdata[i].Object_VrelLat * Outdata[i].Object_VrelLat) * cos(ret)) > (adjust_Params.AdjustParams_MAX_SPEED * adjust_Params.AdjustParams_MAX_SPEED)) {
            Outdata[i].Object_Event = Encode_Event(Outdata[i].Object_Event, Over_Speed);
        }
    }
}

/********************************************************
* Function name ：Retrograde_Dect
* Description   ：实现对车辆的逆行事件检测
* Parameter     ：
* @Outdata        滤波生成的目标数据
* @Outlength      目标数据的长度的指针
* Return        ：Outdata
**********************************************************/
void Retrograde_Dect(ARS408RadarObjectInfo *Outdata, int *Outlength)
{
    int i = 0;

    auto it = gRoadLaneParam.begin();
    int direct = it->second.dire;

    for (i = 0; i < *Outlength; i++) {
        if (direct == 0) {
            if (Outdata[i].Object_VrelLong > 0) {
                Outdata[i].Object_Event = Encode_Event(Outdata[i].Object_Event, Retrograde);
            }
        } else {
            if (Outdata[i].Object_VrelLong < 0) {
                Outdata[i].Object_Event = Encode_Event(Outdata[i].Object_Event, Retrograde);
            }
        }
        
    }
    // for (i = 0; i < *Outlength; i++) {
    //     if (Outdata[i].Object_VrelLong > 0) {
    //         Outdata[i].Object_Event = Encode_Event(Outdata[i].Object_Event, Retrograde);
    //     }
    // }
}

/********************************************************
* Function name ：StopCar_Dect
* Description   ：实现对车辆的停车事件检测
* Parameter     ：
* @Outdata        滤波生成的目标数据
* @Outlength      目标数据的长度的指针
* Return        ：Outdata
**********************************************************/
void StopCar_Dect(ARS408RadarObjectInfo *Outdata, int *Outlength)
{
    int i = 0;

    for (i = 0; i < *Outlength; i++) {
        if ((Outdata[i].Object_VrelLat == 0) && (Outdata[i].Object_VrelLong == 0)) {
            Outdata[i].Object_Event = Encode_Event(Outdata[i].Object_Event, Stop_Car);
        }
    }
}

/********************************************************
* Function name ：LaneChange_Dect
* Description   ：实现对车辆的变道事件检测
* Parameter     ：
* @Outdata          滤波生成的目标数据
* @Outlength        目标数据的长度的指针
* @Lane_Change_List 上一帧的目标数据
* @Lane_List_Length 上一帧目标数据的长度指针
* Return        ：Outdata
* Return        ：Lane_Change_List
**********************************************************/
void LaneChange_Dect(ARS408RadarObjectInfo *Outdata, int *Outlength, Event_Params *event_params)
{
    int i, j;

    // 双重循环对车道突变的车辆进行事件标记
    for (i = 0; i < event_params->Lane_List_Length; i++) {
        for (j = 0; j < *Outlength; j++) {
            // 判断从车辆ID相等且位置相近
            if (event_params->Lane_Change_List[i].Object_ID == Outdata[j].Object_ID) {
                if (event_params->Lane_Change_List[i].Object_Lane != Outdata[j].Object_Lane) {
                    Outdata[j].Object_Event = Encode_Event(Outdata[j].Object_Event, Change_Lane);
                    break;
                }
            }
        }
    }

    // 更新Lane_Change_List数组
    for (i = 0; i < *Outlength; i++) {
        event_params->Lane_Change_List[i] = Outdata[i];
    }

    event_params->Lane_List_Length = *Outlength;
}

/********************************************************
* Function name ：Congest_Dect
* Description   ：实现对车辆的拥堵事件检测
* Parameter     ：
* @Outdata        滤波生成的目标数据
* @Outlength      目标数据的长度的指针
* @Cluster        簇ID数组
* Return        ：Outdata
**********************************************************/
void Congest_Dect(ARS408RadarObjectInfo *Outdata, int *Outlength)
{
    int i, cluster_id[10];
    int Cluster[TRACK_NUM_TRK] = { 0 };

    Event_dbscan(Outdata, Outlength, Cluster);

    for (i = 0; i < *Outlength; i++) {
        if ((Cluster[i] != 0) && (abs(Outdata[i].Object_VrelLong) < adjust_Params.AdjustParams_LIMIT_SPEED)) {
            Outdata[i].Object_Event = Encode_Event(Outdata[i].Object_Event, Congest);
        }
    }
}

/********************************************************
* Function name ：TrafficEventRecongition
* Description   ：实现对车辆的事件检测
* Parameter     ：
* @Outdata        滤波生成的目标数据
* @Outlength      目标数据的长度的指针
* @car_list       需要跟踪的车辆队列链表头节点指针
* @car_stop       停车链表的头节点指针
* Return        ：Outdata
**********************************************************/
void TrafficEventRecongition(ARS408RadarObjectInfo *Outdata, int *Outlength, Event_Params *event_params)
{
    // double elapsed = 0.0;
    // struct timeval starting, finished;

    if (adjust_Params.EnableSpeed_Dect) {
        // gettimeofday(&starting, NULL);
        Speed_Dect(Outdata, Outlength);
        // gettimeofday(&finished, NULL);
        // elapsed = calc_elapsed_time(starting, finished);
        // printf("exec Speed_Dect took: %f ms\n", elapsed);
    }

    if (adjust_Params.EnableRetrograde_Dect) {
        // gettimeofday(&starting, NULL);
        Retrograde_Dect(Outdata, Outlength);
        // gettimeofday(&finished, NULL);
        // elapsed = calc_elapsed_time(starting, finished);
        // printf("exec Retrograde_Dect took: %f ms\n", elapsed);
    }

    // gettimeofday(&starting, NULL);
    StopCar_Dect(Outdata, Outlength);
    // gettimeofday(&finished, NULL);
    // elapsed = calc_elapsed_time(starting, finished);
    // printf("exec StopCar_Dect took: %f ms\n", elapsed);

    if (adjust_Params.EnableLaneChange_Dect) {
        // gettimeofday(&starting, NULL);
        LaneChange_Dect(Outdata, Outlength, event_params);
        // gettimeofday(&finished, NULL);
        // elapsed = calc_elapsed_time(starting, finished);
        // printf("exec LaneChange_Dect took: %f ms\n", elapsed);
    }

    if (adjust_Params.EnableCongest_Dect) {
        // gettimeofday(&starting, NULL);
        Congest_Dect(Outdata, Outlength);
        // gettimeofday(&finished, NULL);
        // elapsed = calc_elapsed_time(starting, finished);
        // printf("exec Congest_Dect took: %f ms\n", elapsed);
    }
}
