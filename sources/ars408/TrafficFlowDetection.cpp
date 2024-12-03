/********************************************************************************
 * @File name: TrafficFlowDetection.c
 * @Author: 高露
 * @Version: 1.0
 * @Date: 2023.6.14
 * @Description: 车流量检测器功能函数
 ********************************************************************************/
#include "jilin.h"
#include "UnitTrack.h"
#include "TrafficFlowDetection.h"

headerAllDef had = {0};
IDMemoryAllDef idMemoryAllDef = {0};
SectionAndCoilRegionDef sectionAndCoilRegionDef = {0};
EnableTrafficDetrctionDef enableTrafficDetrctionDef = {0};
TrafficFlowDetectionStatisticsDef trafficFlowDetectionStatisticsDef = {0};

/********************************************************
* Function name ：trafficDetectionMain
* Description   ：车流量计算总函数
* Parameter     ：
* @enableTrafficDetrctionDef            车流量检测器使能位以及帧计数器
* @trafficFlowDetectionStatisticsDef    结果结构体
* @had                                  车头时距头指针及车间时距头指针
* @idMemoryAllDef                       ID存储器结构体
* @RadarObjectInfoFrame[TRACK_NUM]      车辆目标数据
* @sectionAndCoilRegionDef              断面线圈数据
* @NumberofPointPerFrame                该帧目标数据量
* Return        ：trafficFlowDetectionStatisticsDef
**********************************************************/
void trafficFlowDetection(const ARS408RadarObjectInfo *RadarObjectInfoFrame, int NumberofPointPerFrame)
{
    int i = 0, j, k, kk;
    int lanes = adjust_Params.MaxLaneNum;

    for (i = 0; i < LEN; i++) {
        if (sections[i].section_status != 0) {
            sectionAndCoilRegionDef.section_x = sections[i].section_x;
            // sectionAndCoilRegionDef.detectionCycle = 60;
            sectionAndCoilRegionDef.detectionCycle = sections[i].detectionCycle;

            int count_coil = 0;
            for (j = 0; j < lanes; j++) {
                if (sections[i].CoilPosition[j].laneNum != 0) {
                    sectionAndCoilRegionDef.CoilPosition[j].laneNum = sections[i].CoilPosition[j].laneNum;
                    sectionAndCoilRegionDef.CoilPosition[j].coilX = sections[i].CoilPosition[j].coilY;
                    sectionAndCoilRegionDef.CoilPosition[j].coilY = sections[i].CoilPosition[j].coilX - 3.6;
                    // sectionAndCoilRegionDef.CoilPosition[j].coilY = sections[i].CoilPosition[j].coilX;//测试用
                    sectionAndCoilRegionDef.CoilPosition[j].coilLenth = sections[i].CoilPosition[j].coilLenth;
                    sectionAndCoilRegionDef.CoilPosition[j].coilWidth = sections[i].CoilPosition[j].coilWidth;
                    sectionAndCoilRegionDef.CoilPosition[j].coil_status = sections[i].CoilPosition[j].coil_status;
                    count_coil++;
                } else {
                    sectionAndCoilRegionDef.CoilPosition[j].coil_status = 0;
                }
            }

            // 以断面y坐标进行排序 小--大
            for (k = 1; k < count_coil; k++) {
                for (kk = 0; kk <= count_coil - k; kk++) {
                    if ((sectionAndCoilRegionDef.CoilPosition[kk].coilY > sectionAndCoilRegionDef.CoilPosition[kk + 1].coilY) && (sectionAndCoilRegionDef.CoilPosition[kk + 1].laneNum != 0)) {
                        CoilPositionDef temp = sectionAndCoilRegionDef.CoilPosition[kk];
                        sectionAndCoilRegionDef.CoilPosition[kk] = sectionAndCoilRegionDef.CoilPosition[kk + 1];
                        sectionAndCoilRegionDef.CoilPosition[kk + 1] = temp;
                    }
                }
            }

            break;
        }
    }

    // // 52
    // sectionAndCoilRegionDef.section_x =50;
    // sectionAndCoilRegionDef.detectionCycle = 300.0;
    // sectionAndCoilRegionDef.section_status = 1;
    // // 1
    // sectionAndCoilRegionDef.CoilPosition[0].laneNum = 1;
    // sectionAndCoilRegionDef.CoilPosition[0].coilX = 125;
    // sectionAndCoilRegionDef.CoilPosition[0].coilY = -6;
    // sectionAndCoilRegionDef.CoilPosition[0].coilLenth = 3.6;
    // sectionAndCoilRegionDef.CoilPosition[0].coilWidth = 3.5;
    // sectionAndCoilRegionDef.CoilPosition[0].coil_status = 1;
    // // 2
    // sectionAndCoilRegionDef.CoilPosition[1].laneNum = 2;
    // sectionAndCoilRegionDef.CoilPosition[1].coilX = 125;
    // sectionAndCoilRegionDef.CoilPosition[1].coilY =-2.5;
    // sectionAndCoilRegionDef.CoilPosition[1].coilLenth = 3.6;
    // sectionAndCoilRegionDef.CoilPosition[1].coilWidth = 3.5;
    // sectionAndCoilRegionDef.CoilPosition[1].coil_status = 1;
    // // 3
    // sectionAndCoilRegionDef.CoilPosition[2].laneNum = 3;
    // sectionAndCoilRegionDef.CoilPosition[2].coilX = 125;
    // sectionAndCoilRegionDef.CoilPosition[2].coilY = 1.0;
    // sectionAndCoilRegionDef.CoilPosition[2].coilLenth = 3.6;
    // sectionAndCoilRegionDef.CoilPosition[2].coilWidth = 3.5;
    // sectionAndCoilRegionDef.CoilPosition[2].coil_status = 1;

    // // 4
    // sectionAndCoilRegionDef.CoilPosition[3].laneNum = 4;
    // sectionAndCoilRegionDef.CoilPosition[3].coilX = 125;
    // sectionAndCoilRegionDef.CoilPosition[3].coilY = 4.5;
    // sectionAndCoilRegionDef.CoilPosition[3].coilLenth = 3.6;
    // sectionAndCoilRegionDef.CoilPosition[3].coilWidth = 3.5;
    // sectionAndCoilRegionDef.CoilPosition[3].coil_status = 1;

#if defined(CONFIG_JILIN)
    // 调用目标是否过线圈的函数
    objectDetectionSignal(RadarObjectInfoFrame, NumberofPointPerFrame);
#else
    if (adjust_Params.EnablelaneDividedTrafficFlow) {
        laneDividedTrafficFlow(RadarObjectInfoFrame, NumberofPointPerFrame);
    }

    if (adjust_Params.EnabletrafficFlow) {
        trafficFlowDetectionStatisticsDef.trafficFlowData = 0;
        for (i = 0; i < adjust_Params.MaxLaneNum; i++) {
            trafficFlowDetectionStatisticsDef.trafficFlowData += trafficFlowDetectionStatisticsDef.laneDividedTrafficFlowData[i];
        }

        // trafficFlow(RadarObjectInfoFrame, NumberofPointPerFrame);
    }

    // 调用车型统计车流量的方法
    trafficFlow(RadarObjectInfoFrame, NumberofPointPerFrame);
    trafficFlowDetectionStatisticsDef.smallVehicleFlowData = trafficFlowDetectionStatisticsDef.trafficFlowData - trafficFlowDetectionStatisticsDef.bigVehicleFlowData - trafficFlowDetectionStatisticsDef.middleVehicleFlowData;

    if (adjust_Params.EnablesectionAverageSpeed) {
        if (enableTrafficDetrctionDef.sectionAverageSpeedCount == 0) {
            enableTrafficDetrctionDef.sectionAverageSpeedTimestamp = RadarObjectInfoFrame[0].Object_timestamp;
        }

        enableTrafficDetrctionDef.sectionAverageSpeedCount++;
        sectionallSpeed(RadarObjectInfoFrame, NumberofPointPerFrame);
        if ((RadarObjectInfoFrame[0].Object_timestamp - enableTrafficDetrctionDef.sectionAverageSpeedTimestamp) >= sectionAndCoilRegionDef.detectionCycle) {
            sectionAverageSpeed(RadarObjectInfoFrame, NumberofPointPerFrame);
            // 计算之后对应计数器置零
            trafficFlowDetectionStatisticsDef.speedCount = 0;
            trafficFlowDetectionStatisticsDef.speedvehicleCount = 0;
            enableTrafficDetrctionDef.sectionAverageSpeedCount = 0;
        }
    }

    if (adjust_Params.EnablelaneDividedAverageSpeed) {
        if (enableTrafficDetrctionDef.laneDividedAverageSpeedCount == 0) {
            enableTrafficDetrctionDef.laneDividedAverageSpeedTimestamp = RadarObjectInfoFrame[0].Object_timestamp;
        }

        enableTrafficDetrctionDef.laneDividedAverageSpeedCount++;
        // 分车道平均速度
        laneDividedallSpeed(RadarObjectInfoFrame, NumberofPointPerFrame);
        if ((RadarObjectInfoFrame[0].Object_timestamp - enableTrafficDetrctionDef.laneDividedAverageSpeedTimestamp) >= sectionAndCoilRegionDef.detectionCycle) {
            laneDividedAverageSpeed(RadarObjectInfoFrame, NumberofPointPerFrame);
            enableTrafficDetrctionDef.laneDividedAverageSpeedCount = 0; // 帧计数器清零避免溢出
            for (i = 0; i < adjust_Params.MaxLaneNum; i++) {
                trafficFlowDetectionStatisticsDef.laneDividedspeedCount[i] = 0;
                trafficFlowDetectionStatisticsDef.laneDividedspeedvehicleCount[i] = 0;
            }
        }
    }

    if (adjust_Params.EnablelaneDividedTimeOccupancy) {
        if (enableTrafficDetrctionDef.laneDividedTimeOccupancyCount == 0) {
            enableTrafficDetrctionDef.laneDividedTimeOccupancyTimestamp = RadarObjectInfoFrame[0].Object_timestamp;
        }

        enableTrafficDetrctionDef.laneDividedTimeOccupancyCount++;
        laneDividedTimeOccupancy(RadarObjectInfoFrame, NumberofPointPerFrame);
        if ((RadarObjectInfoFrame[0].Object_timestamp - enableTrafficDetrctionDef.laneDividedTimeOccupancyTimestamp) >= sectionAndCoilRegionDef.detectionCycle) {
            laneDividedTimeOccupancyFinal(RadarObjectInfoFrame, NumberofPointPerFrame);
            enableTrafficDetrctionDef.laneDividedTimeOccupancyCount = 0;
            for (j = 0; j < adjust_Params.MaxLaneNum; j++) {
                trafficFlowDetectionStatisticsDef.laneDividedTimeOccupancySum[j] = 0;
            }
        }
    }

    if (adjust_Params.EnablelaneDividedAverageHeadway) {
        if (enableTrafficDetrctionDef.laneDividedAverageHeadwayCount == 0) {
            enableTrafficDetrctionDef.laneDividedAverageHeadwayTimestamp = RadarObjectInfoFrame[0].Object_timestamp;
        }

        enableTrafficDetrctionDef.laneDividedAverageHeadwayCount++;
        laneDividedAverageHeadwayAllList(RadarObjectInfoFrame, NumberofPointPerFrame);
        if ((RadarObjectInfoFrame[0].Object_timestamp - enableTrafficDetrctionDef.laneDividedAverageHeadwayTimestamp) >= sectionAndCoilRegionDef.detectionCycle) {
            laneDividedAverageHeadway(RadarObjectInfoFrame, NumberofPointPerFrame);
            enableTrafficDetrctionDef.laneDividedAverageHeadwayCount = 0;
            for (i = 0; i < adjust_Params.MaxLaneNum; i++) {
                Clear_LinkListV(had.headers[i]);
            }
        }
    }

    if (adjust_Params.EnablelaneDividedAverageHeadwayGap) {
        if (enableTrafficDetrctionDef.laneDividedAverageHeadwayGapCount == 0) {
            enableTrafficDetrctionDef.laneDividedAverageHeadwayGapTimestamp = RadarObjectInfoFrame[0].Object_timestamp;
        }

        enableTrafficDetrctionDef.laneDividedAverageHeadwayGapCount++;
        laneDividedAverageHeadwayGapAllList(RadarObjectInfoFrame, NumberofPointPerFrame);
        if ((RadarObjectInfoFrame[0].Object_timestamp - enableTrafficDetrctionDef.laneDividedAverageHeadwayGapTimestamp) >= sectionAndCoilRegionDef.detectionCycle) {
            /* 分车道平均车头时距 */
            laneDividedAverageHeadwayGap(RadarObjectInfoFrame, NumberofPointPerFrame);
            enableTrafficDetrctionDef.laneDividedAverageHeadwayGapCount = 0;
            for (i = 0; i < adjust_Params.MaxLaneNum; i++) {
                Clear_LinkListV(had.headergap[i]);
            }
        }
    }
#endif
}

/**************************************************
 * 车辆经过线圈检测函数
 * ********************************************** */
bool objectDetectionSignal(const ARS408RadarObjectInfo *RadarObjectInfoFrame, int NumberofPointPerFrame)
{
    int i, k;
    int maxlanenum = adjust_Params.MaxLaneNum;

    if (maxlanenum <= 0) {
        maxlanenum = 1;
    }

    for (k = 0; k < NumberofPointPerFrame; k++) {
        float objYC = RadarObjectInfoFrame[k].Object_DistLat;
        float objXC = RadarObjectInfoFrame[k].Object_DistLong;

        for (i = 0; i < maxlanenum; i++) {
            /* 线圈位置的最大值 */
            float coilYCM = sectionAndCoilRegionDef.CoilPosition[i].coilY + sectionAndCoilRegionDef.CoilPosition[i].coilWidth;

            /* 目标Y坐标 大于等于 线圈位置的最小值 */
            bool flag1 = objYC >= sectionAndCoilRegionDef.CoilPosition[i].coilY;
            /* 目标Y坐标 小于等于 线圈位置的最大值 */
            bool flag2 = objYC <= coilYCM;

            /* X轴 */
            bool flag3 = objXC <= sectionAndCoilRegionDef.CoilPosition[i].coilX;

            /* TODO: 增加X轴检测范围 */

            if (flag1 && flag2 && flag3) {
                jilin_push_message(0x01);
                return true;
            }
        }
    }

    return true;
}

/********************************************************
* Function name ：trafficFlow
* Description   ：断面车流量计算函数
* Parameter     ：
* @trafficFlowDetectionStatisticsDef    结果结构体
* @idMemoryAllDef                       ID存储器结构体
* @RadarObjectInfoFrame[TRACK_NUM]      车辆目标数据
* @sectionAndCoilRegionDef              断面线圈数据
* @NumberofPointPerFrame                该帧目标数据量
* Return        ：trafficFlowDetectionStatisticsDef
**********************************************************/
int trafficFlow(const ARS408RadarObjectInfo *RadarObjectInfoFrame, int NumberofPointPerFrame)
{
    int i = 0, j, k, n;

    for (k = 0; k < NumberofPointPerFrame; k++) {
        // 判断是否通过断面
        if ((RadarObjectInfoFrame[k].Object_DistLong <= sectionAndCoilRegionDef.section_x) &&
            ((RadarObjectInfoFrame[k].Object_DistLong + RadarObjectInfoFrame[k].Object_Length / 2) >= sectionAndCoilRegionDef.section_x))
        {
            // ID更新
            idMemoryAllDef.trafficFlowidMemoryDef.newIDMemory[i++] = RadarObjectInfoFrame[k].Object_ID;
            // 比较ID
            for (n = 0; n < adjust_Params.MaxLaneNum * 2; n++) {
                if (RadarObjectInfoFrame[k].Object_ID == idMemoryAllDef.trafficFlowidMemoryDef.oldIDMemory[n]) {
                    break;
                } else if (n == (adjust_Params.MaxLaneNum * 2 - 1)) {
                    // trafficFlowDetectionStatisticsDef.trafficFlowData++;

                    if (RadarObjectInfoFrame[k].Object_Class == OBJECT_SmallCar) {          // 小车
                        trafficFlowDetectionStatisticsDef.smallVehicleFlowData++;
                    } else if (RadarObjectInfoFrame[k].Object_Class == OBJECT_MediumCar) {  // 大车
                        trafficFlowDetectionStatisticsDef.middleVehicleFlowData++;
                    } else if (RadarObjectInfoFrame[k].Object_Class == OBJECT_BigCar) {     // 超大车
                        trafficFlowDetectionStatisticsDef.bigVehicleFlowData++;
                    }

                    // // 超大车
                    // if (RadarObjectInfoFrame[k].Object_Class == 1) {
                    //     trafficFlowDetectionStatisticsDef.bigVehicleFlowData++;
                    // } else if (RadarObjectInfoFrame[k].Object_Class == 0) { 
                    //     // 大车
                    //     trafficFlowDetectionStatisticsDef.middleVehicleFlowData++;
                    // } else { 
                    //     // 小车
                    //     trafficFlowDetectionStatisticsDef.smallVehicleFlowData++;
                    // }
                }
            }
        }
    }

    // 更新id数组
    for (j = 0; j < adjust_Params.MaxLaneNum * 2; j++) {
        idMemoryAllDef.trafficFlowidMemoryDef.oldIDMemory[j] = idMemoryAllDef.trafficFlowidMemoryDef.newIDMemory[j];
        idMemoryAllDef.trafficFlowidMemoryDef.newIDMemory[j] = 0;
    }

    return 0;
}

/********************************************************
* Function name ：laneDividedTrafficFlow
* Description   ：分车道车流量计算函数
* Parameter     ：
* @trafficFlowDetectionStatisticsDef    结果结构体
* @idMemoryAllDef                       ID存储器结构体
* @RadarObjectInfoFrame[TRACK_NUM]      车辆目标数据
* @sectionAndCoilRegionDef              断面线圈数据
* @NumberofPointPerFrame                该帧目标数据量
* Return        ：trafficFlowDetectionStatisticsDef
**********************************************************/
int laneDividedTrafficFlow(const ARS408RadarObjectInfo *RadarObjectInfoFrame, int NumberofPointPerFrame)
{
    int i = 0, j, k, n, a;

    for (k = 0; k < NumberofPointPerFrame; k++) {
        if ((RadarObjectInfoFrame[k].Object_DistLong <= sectionAndCoilRegionDef.section_x) &&
            ((RadarObjectInfoFrame[k].Object_DistLong + RadarObjectInfoFrame[k].Object_Length / 2) >= sectionAndCoilRegionDef.section_x))
        {
            idMemoryAllDef.laneDividedtrafficFlowidMemoryDef.newIDMemory[i++] = RadarObjectInfoFrame[k].Object_ID;
            // 比较ID
            for (n = 0; n < adjust_Params.MaxLaneNum * 2; n++) {
                if (RadarObjectInfoFrame[k].Object_ID == idMemoryAllDef.laneDividedtrafficFlowidMemoryDef.oldIDMemory[n]) {
                    break;
                } else if (n == (adjust_Params.MaxLaneNum * 2 - 1)) {
                    for (a = 0; a < adjust_Params.MaxLaneNum; a++) {
                        if ((sectionAndCoilRegionDef.CoilPosition[a].coil_status != 0) &&
                            (RadarObjectInfoFrame[k].Object_DistLat >= sectionAndCoilRegionDef.CoilPosition[a].coilY) &&
                            (RadarObjectInfoFrame[k].Object_DistLat <= (sectionAndCoilRegionDef.CoilPosition[a].coilY + sectionAndCoilRegionDef.CoilPosition[a].coilWidth)))
                        {
                            trafficFlowDetectionStatisticsDef.laneDividedTrafficFlowData[a]++;
                        }
                    }
                }
            }
        }
    }

    // 更新id数组
    for (j = 0; j < adjust_Params.MaxLaneNum * 2; j++) {
        idMemoryAllDef.laneDividedtrafficFlowidMemoryDef.oldIDMemory[j] = idMemoryAllDef.laneDividedtrafficFlowidMemoryDef.newIDMemory[j];
        idMemoryAllDef.laneDividedtrafficFlowidMemoryDef.newIDMemory[j] = 0;
    }

    return 0;
}

/********************************************************
* Function name ：sectionallSpeed
* Description   ：计算断面一段时间内总速度和车流量
* Parameter     ：
* @trafficFlowDetectionStatisticsDef    结果结构体
* @idMemoryAllDef                       ID存储器结构体
* @RadarObjectInfoFrame[TRACK_NUM]      车辆目标数据
* @sectionAndCoilRegionDef              断面线圈数据
* @NumberofPointPerFrame                该帧目标数据量
* Return        ：trafficFlowDetectionStatisticsDef
**********************************************************/
int sectionallSpeed(const ARS408RadarObjectInfo *RadarObjectInfoFrame, int NumberofPointPerFrame)
{
    int i = 0, j, k, n;

    for (k = 0; k < NumberofPointPerFrame; k++) {
        if ((RadarObjectInfoFrame[k].Object_DistLong <= sectionAndCoilRegionDef.section_x) &&
            ((RadarObjectInfoFrame[k].Object_DistLong + RadarObjectInfoFrame[k].Object_Length / 2) >= sectionAndCoilRegionDef.section_x))
        {
            idMemoryAllDef.speedvehicleidMemoryDef.newIDMemory[i++] = RadarObjectInfoFrame[k].Object_ID;
            // 比较ID
            for (n = 0; n < adjust_Params.MaxLaneNum * 2; n++) {
                if (RadarObjectInfoFrame[k].Object_ID == idMemoryAllDef.speedvehicleidMemoryDef.oldIDMemory[n]) {
                    break;
                } else if (n == (adjust_Params.MaxLaneNum * 2 - 1)) {
                    trafficFlowDetectionStatisticsDef.speedvehicleCount++;
                    trafficFlowDetectionStatisticsDef.speedCount = trafficFlowDetectionStatisticsDef.speedCount + sqrt(RadarObjectInfoFrame[k].Object_VrelLat * RadarObjectInfoFrame[k].Object_VrelLat + RadarObjectInfoFrame[k].Object_VrelLong * RadarObjectInfoFrame[k].Object_VrelLong);
                }
            }
        }
    }

    // 更新id数组
    for (j = 0; j < adjust_Params.MaxLaneNum * 2; j++) {
        idMemoryAllDef.speedvehicleidMemoryDef.oldIDMemory[j] = idMemoryAllDef.speedvehicleidMemoryDef.newIDMemory[j];
        idMemoryAllDef.speedvehicleidMemoryDef.newIDMemory[j] = 0;
    }

    return 0;
}

/********************************************************
* Function name ：sectionAverageSpeed
* Description   ：计算断面平均车速
* Parameter     ：
* @trafficFlowDetectionStatisticsDef    结果结构体
* Return        ：trafficFlowDetectionStatisticsDef
**********************************************************/
int sectionAverageSpeed(const ARS408RadarObjectInfo *RadarObjectInfoFrame, int NumberofPointPerFrame)
{
    if (trafficFlowDetectionStatisticsDef.speedvehicleCount != 0) {
        trafficFlowDetectionStatisticsDef.sectionAverageSpeedData = trafficFlowDetectionStatisticsDef.speedCount / trafficFlowDetectionStatisticsDef.speedvehicleCount * 3.6;
    } else {
        // trafficFlowDetectionStatisticsDef.sectionAverageSpeedData = 0;
    }

    return 0;
}

/********************************************************
* Function name ：laneDividedallSpeed
* Description   ：分车道平均车速总和计算
* Parameter     ：
* @trafficFlowDetectionStatisticsDef    结果结构体
* @idMemoryAllDef                       ID存储器结构体
* @RadarObjectInfoFrame[TRACK_NUM]      车辆目标数据
* @sectionAndCoilRegionDef              断面线圈数据
* @NumberofPointPerFrame                该帧目标数据量
* Return        ：trafficFlowDetectionStatisticsDef
**********************************************************/
int laneDividedallSpeed(const ARS408RadarObjectInfo *RadarObjectInfoFrame, int NumberofPointPerFrame)
{
    int i = 0, j, k, n, a;

    for (k = 0; k < NumberofPointPerFrame; k++) {
        if ((RadarObjectInfoFrame[k].Object_DistLong <= sectionAndCoilRegionDef.section_x) &&
            ((RadarObjectInfoFrame[k].Object_DistLong + RadarObjectInfoFrame[k].Object_Length / 2) >= sectionAndCoilRegionDef.section_x))
        {
            idMemoryAllDef.laneDividedspeedvehicleidMemoryDef.newIDMemory[i++] = RadarObjectInfoFrame[k].Object_ID;
            // 比较ID
            for (n = 0; n < adjust_Params.MaxLaneNum * 2; n++) {
                if (RadarObjectInfoFrame[k].Object_ID == idMemoryAllDef.laneDividedspeedvehicleidMemoryDef.oldIDMemory[n]) {
                    break;
                } else if (n == adjust_Params.MaxLaneNum * 2 - 1) {
                    for (a = 0; a < adjust_Params.MaxLaneNum; a++) {
                        // 判断对应线圈是否可用
                        if ((sectionAndCoilRegionDef.CoilPosition[a].coil_status != 0) &&
                            (RadarObjectInfoFrame[k].Object_DistLat >= sectionAndCoilRegionDef.CoilPosition[a].coilY) &&
                            (RadarObjectInfoFrame[k].Object_DistLat <= (sectionAndCoilRegionDef.CoilPosition[a].coilY + sectionAndCoilRegionDef.CoilPosition[a].coilWidth)))
                        {
                            trafficFlowDetectionStatisticsDef.laneDividedspeedvehicleCount[a]++;
                            trafficFlowDetectionStatisticsDef.laneDividedspeedCount[a] = trafficFlowDetectionStatisticsDef.laneDividedspeedCount[a] + sqrt(RadarObjectInfoFrame[k].Object_VrelLat * RadarObjectInfoFrame[k].Object_VrelLat + RadarObjectInfoFrame[k].Object_VrelLong * RadarObjectInfoFrame[k].Object_VrelLong);
                        }
                    }
                }
            }
        }
    }

    // 更新id数组
    for (j = 0; j < adjust_Params.MaxLaneNum * 2; j++) {
        idMemoryAllDef.laneDividedspeedvehicleidMemoryDef.oldIDMemory[j] = idMemoryAllDef.laneDividedspeedvehicleidMemoryDef.newIDMemory[j];
        idMemoryAllDef.laneDividedspeedvehicleidMemoryDef.newIDMemory[j] = 0;
    }

    return 0;
}

/********************************************************
* Function name ：laneDividedAverageSpeed
* Description   ：分车道平均车速计算
* Parameter     ：
* @trafficFlowDetectionStatisticsDef    结果结构体
* Return        ：trafficFlowDetectionStatisticsDef
**********************************************************/
int laneDividedAverageSpeed(const ARS408RadarObjectInfo *RadarObjectInfoFrame, int NumberofPointPerFrame)
{
    int i = 0;

    for (i = 0; i < adjust_Params.MaxLaneNum; i++) {
        if (trafficFlowDetectionStatisticsDef.laneDividedspeedvehicleCount[i] != 0) {
            trafficFlowDetectionStatisticsDef.laneDividedAverageSpeedData[i] = trafficFlowDetectionStatisticsDef.laneDividedspeedCount[i] / trafficFlowDetectionStatisticsDef.laneDividedspeedvehicleCount[i] * 3.6;
        } else {
            // trafficFlowDetectionStatisticsDef.laneDividedAverageSpeedData[i] = 0;
        }
    }

    return 0;
}

/********************************************************
* Function name ：laneDividedTimeOccupancy
* Description   ：分车道时间占有率计算
* Parameter     ：
* @trafficFlowDetectionStatisticsDef    结果结构体
* @idMemoryAllDef                       ID存储器结构体
* @RadarObjectInfoFrame[TRACK_NUM]      车辆目标数据
* @sectionAndCoilRegionDef              断面线圈数据
* @NumberofPointPerFrame                该帧目标数据量
* Return        ：trafficFlowDetectionStatisticsDef
**********************************************************/
int laneDividedTimeOccupancy(const ARS408RadarObjectInfo *RadarObjectInfoFrame, int NumberofPointPerFrame)
{
    int i = 0, j, k, n, a;

    for (k = 0; k < NumberofPointPerFrame; k++) {
        if ((RadarObjectInfoFrame[k].Object_DistLong <= sectionAndCoilRegionDef.section_x) &&
            ((RadarObjectInfoFrame[k].Object_DistLong + RadarObjectInfoFrame[k].Object_Length / 2) >= sectionAndCoilRegionDef.section_x))
        {
            idMemoryAllDef.laneDividedTimeOccupancyidMemoryDef.newIDMemory[i++] = RadarObjectInfoFrame[k].Object_ID;
            // 比较ID
            for (n = 0; n < adjust_Params.MaxLaneNum * 2; n++) {
                if (RadarObjectInfoFrame[k].Object_ID == idMemoryAllDef.laneDividedTimeOccupancyidMemoryDef.oldIDMemory[n]) {
                    break;
                } else if (n == adjust_Params.MaxLaneNum * 2 - 1) {
                    for (a = 0; a < adjust_Params.MaxLaneNum; a++) {
                        if ((sectionAndCoilRegionDef.CoilPosition[a].coil_status != 0) &&
                            (RadarObjectInfoFrame[k].Object_DistLat >= sectionAndCoilRegionDef.CoilPosition[a].coilY) &&
                            (RadarObjectInfoFrame[k].Object_DistLat <= (sectionAndCoilRegionDef.CoilPosition[a].coilY + sectionAndCoilRegionDef.CoilPosition[a].coilWidth)))
                        {
                            trafficFlowDetectionStatisticsDef.laneDividedTimeOccupancySum[a] = trafficFlowDetectionStatisticsDef.laneDividedTimeOccupancySum[a] + 250 / sqrt(RadarObjectInfoFrame[k].Object_VrelLat * RadarObjectInfoFrame[k].Object_VrelLat + RadarObjectInfoFrame[k].Object_VrelLong * RadarObjectInfoFrame[k].Object_VrelLong);
                        }
                    }
                }
            }
        }
    }

    // 更新id数组
    for (j = 0; j < adjust_Params.MaxLaneNum * 2; j++) {
        idMemoryAllDef.laneDividedTimeOccupancyidMemoryDef.oldIDMemory[j] = idMemoryAllDef.laneDividedTimeOccupancyidMemoryDef.newIDMemory[j];
        idMemoryAllDef.laneDividedTimeOccupancyidMemoryDef.newIDMemory[j] = 0;
    }

    return 0;
}

/********************************************************
* Function name ：laneDividedTimeOccupancyFinal
* Description   ：最终分车道时间占有率计算
* Parameter     ：
* @trafficFlowDetectionStatisticsDef    结果结构体
* @sectionAndCoilRegionDef              断面线圈数据
* Return        ：trafficFlowDetectionStatisticsDef
**********************************************************/
int laneDividedTimeOccupancyFinal(const ARS408RadarObjectInfo *RadarObjectInfoFrame, int NumberofPointPerFrame)
{
    int i = 0;

    for (i = 0; i < adjust_Params.MaxLaneNum; i++) {
        if (trafficFlowDetectionStatisticsDef.laneDividedTimeOccupancySum[i] == 0) {
            // trafficFlowDetectionStatisticsDef.laneDividedTimeOccupancyData[i] = 0;
        } else {
            trafficFlowDetectionStatisticsDef.laneDividedTimeOccupancyData[i] = trafficFlowDetectionStatisticsDef.laneDividedTimeOccupancySum[i] / sectionAndCoilRegionDef.detectionCycle * 10;
        }
    }

    return 0;
}

/********************************************************
* Function name ：laneDividedAverageHeadwayAllList
* Description   ：分车道平均车头时距总和计算
* Parameter     ：
* @had                              车头时距头指针及车间时距头指针
* @idMemoryAllDef                   ID存储器结构体
* @RadarObjectInfoFrame[TRACK_NUM]  车辆目标数据
* @sectionAndCoilRegionDef          断面线圈数据
* @NumberofPointPerFrame            该帧目标数据量
* Return        ：trafficFlowDetectionStatisticsDef
**********************************************************/
int laneDividedAverageHeadwayAllList(const ARS408RadarObjectInfo *RadarObjectInfoFrame, int NumberofPointPerFrame)
{
    int i = 0, j, k, n, a;

    for (k = 0; k < NumberofPointPerFrame; k++) {
        if ((RadarObjectInfoFrame[k].Object_DistLong <= sectionAndCoilRegionDef.section_x) &&
            ((RadarObjectInfoFrame[k].Object_DistLong + RadarObjectInfoFrame[k].Object_Length / 2) >= sectionAndCoilRegionDef.section_x))
        {
            idMemoryAllDef.laneDividedHeadwayidMemoryDef.newIDMemory[i++] = RadarObjectInfoFrame[k].Object_ID;
            // 比较ID
            for (n = 0; n < adjust_Params.MaxLaneNum * 2; n++) {
                if (RadarObjectInfoFrame[k].Object_ID == idMemoryAllDef.laneDividedHeadwayidMemoryDef.oldIDMemory[n]) {
                    break;
                } else if (n == adjust_Params.MaxLaneNum * 2 - 1) {
                    for (a = 0; a < adjust_Params.MaxLaneNum; a++) {
                        if ((sectionAndCoilRegionDef.CoilPosition[a].coil_status != 0) &&
                            (RadarObjectInfoFrame[k].Object_DistLat >= sectionAndCoilRegionDef.CoilPosition[a].coilY) &&
                            (RadarObjectInfoFrame[k].Object_DistLat <= (sectionAndCoilRegionDef.CoilPosition[a].coilY + sectionAndCoilRegionDef.CoilPosition[a].coilWidth)))
                        {
                            InsertLink_LinkListV(had.headers[a], RadarObjectInfoFrame[k].Object_timestamp);
                            /*switch (a) {
                                case 0:InsertLink_LinkListV(had.headers.header1, RadarObjectInfoFrame[k].Object_timestamp); break;
                                case 1:InsertLink_LinkListV(had.headers.header2, RadarObjectInfoFrame[k].Object_timestamp); break;
                                case 2:InsertLink_LinkListV(had.headers.header3, RadarObjectInfoFrame[k].Object_timestamp); break;
                                case 3:InsertLink_LinkListV(had.headers.header4, RadarObjectInfoFrame[k].Object_timestamp); break;
                                default: break;
                            }*/
                        }
                    }
                }
            }
        }
    }

    // 更新id数组
    for (j = 0; j < adjust_Params.MaxLaneNum * 2; j++) {
        idMemoryAllDef.laneDividedHeadwayidMemoryDef.oldIDMemory[j] = idMemoryAllDef.laneDividedHeadwayidMemoryDef.newIDMemory[j];
        idMemoryAllDef.laneDividedHeadwayidMemoryDef.newIDMemory[j] = 0;
    }

    return 0;
}

/********************************************************
* Function name ：laneDividedAverageHeadway
* Description   ：分车道平均车头时距计算
* Parameter     ：
* @trafficFlowDetectionStatisticsDef    结果结构体
* @had                                  车头时距头指针及车间时距头指针
* Return        ：trafficFlowDetectionStatisticsDef
**********************************************************/
int laneDividedAverageHeadway(const ARS408RadarObjectInfo *RadarObjectInfoFrame, int NumberofPointPerFrame)
{
    int i = 0;

    for (i = 0; i < adjust_Params.MaxLaneNum; i++) {
        trafficFlowDetectionStatisticsDef.laneDividedAverageHeadwayData[i] = Foreach_LinkListV(had.headers[i]);
    }

    return 0;
}

/********************************************************
* Function name ：laneDividedAverageHeadwayGapAllList
* Description   ：分车道平均车间时距总和计算
* Parameter     ：
* @had                              车头时距头指针及车间时距头指针
* @idMemoryAllDef                   ID存储器结构体
* @RadarObjectInfoFrame[TRACK_NUM]  车辆目标数据
* @sectionAndCoilRegionDef          断面线圈数据
* @NumberofPointPerFrame            该帧目标数据量
* Return        ：trafficFlowDetectionStatisticsDef
**********************************************************/
int laneDividedAverageHeadwayGapAllList(const ARS408RadarObjectInfo *RadarObjectInfoFrame, int NumberofPointPerFrame)
{
    int i = 0, j, k, n, a;

    for (k = 0; k < NumberofPointPerFrame; k++) {
        if ((RadarObjectInfoFrame[k].Object_DistLong <= sectionAndCoilRegionDef.section_x) &&
            ((RadarObjectInfoFrame[k].Object_DistLong + RadarObjectInfoFrame[k].Object_Length / 2) >= sectionAndCoilRegionDef.section_x))
        {
            idMemoryAllDef.laneDividedHeadwayGapidMemoryDef.newIDMemory[i++] = RadarObjectInfoFrame[k].Object_ID;
            // 比较ID
            for (n = 0; n < adjust_Params.MaxLaneNum * 2; n++) {
                if (RadarObjectInfoFrame[k].Object_ID == idMemoryAllDef.laneDividedHeadwayGapidMemoryDef.oldIDMemory[n]) {
                    break;
                } else if (n == adjust_Params.MaxLaneNum * 2 - 1) {
                    for (a = 0; a < adjust_Params.MaxLaneNum; a++) {
                        if ((sectionAndCoilRegionDef.CoilPosition[a].coil_status != 0) &&
                            (RadarObjectInfoFrame[k].Object_DistLat >= sectionAndCoilRegionDef.CoilPosition[a].coilY) &&
                            (RadarObjectInfoFrame[k].Object_DistLat <= (sectionAndCoilRegionDef.CoilPosition[a].coilY + sectionAndCoilRegionDef.CoilPosition[a].coilWidth)))
                        {
                            double GapTime = (RadarObjectInfoFrame[k].Object_Length / sqrt(RadarObjectInfoFrame[k].Object_VrelLat * RadarObjectInfoFrame[k].Object_VrelLat + RadarObjectInfoFrame[k].Object_VrelLong * RadarObjectInfoFrame[k].Object_VrelLong));
                            InsertLink_LinkListV(had.headergap[a], RadarObjectInfoFrame[k].Object_timestamp);
                            InsertLink_LinkListV(had.headergap[a], RadarObjectInfoFrame[k].Object_timestamp + GapTime);
                        }
                    }
                }
            }
        }
    }

    // 更新id数组
    for (j = 0; j < adjust_Params.MaxLaneNum * 2; j++) {
        idMemoryAllDef.laneDividedHeadwayGapidMemoryDef.oldIDMemory[j] = idMemoryAllDef.laneDividedHeadwayGapidMemoryDef.newIDMemory[j];
        idMemoryAllDef.laneDividedHeadwayGapidMemoryDef.newIDMemory[j] = 0;
    }

    return 0;
}

/********************************************************
* Function name ：laneDividedAverageHeadwayGap
* Description   ：分车道平均车间时距计算
* Parameter     ：
* @trafficFlowDetectionStatisticsDef    结果结构体
* @had	车头时距头指针及车间时距头指针
* Return        ：trafficFlowDetectionStatisticsDef
**********************************************************/
int laneDividedAverageHeadwayGap(const ARS408RadarObjectInfo *RadarObjectInfoFrame, int NumberofPointPerFrame)
{
    int i = 0;

    for (i = 0; i < adjust_Params.MaxLaneNum; i++) {
        trafficFlowDetectionStatisticsDef.laneDividedAverageHeadwayGapData[i] = HeadwayGap_LinkListV(had.headergap[i]);
    }

    return 0;
}

// 初始化链表
struct LinkNodeV *Init_LinkListV()
{
    struct LinkNodeV *header = (struct LinkNodeV *)malloc(sizeof(struct LinkNodeV));
    if (header == NULL) {
        exit(EXIT_FAILURE);
    }

    header->timestamp = -1;
    header->next = NULL;
    return header;
}

// 插入，尾
void InsertLink_LinkListV(struct LinkNodeV *header, double newval)
{
    struct LinkNodeV *p_new = (struct LinkNodeV *)malloc(sizeof(struct LinkNodeV));
    if (p_new == NULL) {
        exit(EXIT_FAILURE);
    }

    p_new->timestamp = newval;          /* 新元素data设为num */
    p_new->next = NULL;                 /* 将新元素next置为空 */

    struct LinkNodeV *temp = header;    /* 定义一个临时指针变量指向表头 */
    if (NULL == header) {               /* head为空则赋予p_new为第一个新结点 */
        header = p_new;
        p_new->next = NULL;
    } else {
        /* 最终让temp指向尾结点 */
        while (temp->next != NULL) {
            temp = temp->next;
        }

        temp->next = p_new;             /* 将尾结点temp的next设为p_new,即将p_new设为尾结点 */
    }
}

// 销毁
void Clear_LinkListV(struct LinkNodeV *header)
{
    if (NULL == header) {
        return;
    }

    struct LinkNodeV *pCurrent = header->next;
    while (pCurrent != NULL) {
        struct LinkNodeV *pNext = pCurrent->next;
        free(pCurrent);
        pCurrent = pNext;
    }

    header->next = NULL;
}

// 遍历
float Foreach_LinkListV(struct LinkNodeV *header)
{
    int LinkNodeVCount = 0;
    double timeStampEnd = 0;
    double timeStampBengin = 0;
    float laneOneAverageHeadwayData = 0;

    if (NULL == header) {
        return 0;
    }

    // 辅助指针变量
    struct LinkNodeV *pTeamp = NULL;
    struct LinkNodeV *pCurrent = header->next;
    if (pCurrent != NULL) {
        timeStampBengin = pCurrent->timestamp;
        while (pCurrent != NULL) {
            LinkNodeVCount++;
            pTeamp = pCurrent;
            pCurrent = pCurrent->next;
        }

        timeStampEnd = pTeamp->timestamp;
    }

    if (LinkNodeVCount == 0 || LinkNodeVCount == 1) {
        // laneOneAverageHeadwayData = 0;
    } else {
        laneOneAverageHeadwayData = (timeStampEnd - timeStampBengin) / (LinkNodeVCount - 1);
    }

    return laneOneAverageHeadwayData / 1000;
}

float HeadwayGap_LinkListV(struct LinkNodeV *header)
{
    int LinkNodeVCount = 0;
    double timeStampEnd = 0;
    double timeStampBengin = 0;
    double timeStampGapSum = 0;
    float laneOneAverageHeadwayGapData = 0;

    if (NULL == header) {
        return 0;
    }

    // 辅助指针变量
    struct LinkNodeV *pTeamp = NULL;
    struct LinkNodeV *pCurrent = header->next;

    if (pCurrent != NULL) {
        timeStampBengin = pCurrent->timestamp;
        while (pCurrent != NULL) {
            LinkNodeVCount++;
            if (LinkNodeVCount % 2 == 0) {
                timeStampEnd = pCurrent->timestamp;
                pTeamp = pCurrent->next;
                if (pTeamp == NULL) {
                    break;
                } else {
                    timeStampBengin = pTeamp->timestamp;
                    timeStampGapSum = timeStampGapSum + (timeStampBengin - timeStampEnd);
                }
            }

            pCurrent = pCurrent->next;
        }
    }

    if ((LinkNodeVCount / 2 == 0) || (LinkNodeVCount / 2 == 1)) {
        // laneOneAverageHeadwayGapData = 0;
    } else {
        laneOneAverageHeadwayGapData = timeStampGapSum / (LinkNodeVCount / 2 - 1);
    }

    return laneOneAverageHeadwayGapData / 1000;
}
