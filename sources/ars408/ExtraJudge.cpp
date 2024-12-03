/********************************************************************************
* @File name: ExtraJudge.c
* @Author: 石海新
* @Version: 1.0
* @Date: 2023.6.7
* @Description: 对滤波后数据额外判断功能文件
********************************************************************************/
#include "ExtraJudge.h"

#include <map>
#include <spdlog/spdlog.h>

extern std::map<int, RoadLaneParam> gRoadLaneParam;

/********************************************************
* Function name ：Stop_Dect
* Description   ：实现对车辆的停车判断
* Parameter     ：
* @Outdata        滤波生成的目标数据
* @Outlength      目标数据的长度的指针
* @car_list       需要跟踪的车辆队列链表头节点指针
* @car_stop       停车链表的头节点指针
* @c1             取链表中数据所使用的RadarObjectInfoARS类型指针
* @p1             遍历跟踪链表所使用的list类型指针
* @p_stop1        遍历停车链表所使用的list类型指针
* @car1           用于向跟踪链表添加数据的RadarObjectInfoARS类型指针
* @car2           判断车辆是否启动使用的RadarObjectInfoARS类型指针
* @car3           用于向停车链表添加数据的RadarObjectInfoARS类型指针
* @CarLength      链表长度
* Return        ：Outdata
* Return        : *Outlength
* Return        : 跟踪链表
* Return        ：停车链表
**********************************************************/
void Stop_Dect(ARS408RadarObjectInfo *Outdata, int *Outlength, Event_Params *event_params)
{
    struct list *p1;
    struct list *p_stop1;
    ARS408RadarObjectInfo *c1;

    if ((Outdata == NULL) || (Outlength == NULL) || (event_params == NULL)) {
        return;
    }

    int CarLength = List_Count(event_params->car_head);
    int CarStopLen = List_Count(event_params->car_stop);

    // 遍历链表
    p1 = event_params->car_head;
    while (p1->next != NULL) {
        int flag = 0;  // 判断链表中车辆是否在某一帧数据中的标志位 flag = 1车辆存在 flag = 0车辆停下
        for (int i = 0; i < *Outlength; i++) {
            // 判断这辆车是否在车辆队列中
            if (Outdata[i].Object_ID == p1->next->id) {
                c1 = (ARS408RadarObjectInfo*)p1->next->data;
                if (c1) {
                    if (fabs(c1->Object_DistLat - Outdata[i].Object_DistLat) <= 1 && fabs(c1->Object_DistLong - Outdata[i].Object_DistLong) <= 5) {
                        *c1 = Outdata[i];
                        flag = 1;
                        break;
                    }
                }
            }
        }

        ARS408RadarObjectInfo *stopCar = (ARS408RadarObjectInfo *)(p1->next->data);
        if (flag == 0 && stopCar->Object_DistLong >= 30 && stopCar->Object_DistLong <= 200) {
            ARS408RadarObjectInfo *car3 = (ARS408RadarObjectInfo *)malloc(sizeof(ARS408RadarObjectInfo));
            if (car3 == NULL) {
                spdlog::error("Stop_Dect malloc failed");
                exit(EXIT_FAILURE);
            }

            car3 = (ARS408RadarObjectInfo *)p1->next->data;
            List_Add(event_params->car_stop, car3, car3->Object_ID, 0);
            // 更新停车链表长度
            CarStopLen = List_Count(event_params->car_stop);

            // 删除操作
            struct list *ptemp = p1->next;
            p1->next = p1->next->next;
            free(ptemp);
        } else {
            p1 = p1->next;
        }
    }

    p1 = event_params->car_head;
    while (p1->next != NULL) {
        p1->next->frame++;
        p1 = p1->next;
    }

    p_stop1 = event_params->car_stop;
    while (p_stop1->next != NULL) {
        p_stop1->next->frame++;
        p_stop1 = p_stop1->next;
    }

    for (int i = 0; i < *Outlength; i++) {
        // 判断这辆车是否在车辆队列中
        c1 = (ARS408RadarObjectInfo*)List_GetNode_Id(event_params->car_head, Outdata[i].Object_ID);
        if (!c1) {
            // 判断速度和位置边界
            if (((Outdata[i].Object_VrelLat * Outdata[i].Object_VrelLat + Outdata[i].Object_VrelLong * Outdata[i].Object_VrelLong) < (adjust_Params.AdjustParams_V_Thr * adjust_Params.AdjustParams_V_Thr)) && Outdata[i].Object_DistLong > 0 && Outdata[i].Object_DistLat >= -15 && Outdata[i].Object_DistLat <= 10) {
                ARS408RadarObjectInfo *car1 = (ARS408RadarObjectInfo *)malloc(sizeof(ARS408RadarObjectInfo));
                if (car1 == NULL) {
                    spdlog::error("Stop_Dect malloc failed");
                    exit(EXIT_FAILURE);
                }

                *car1 = Outdata[i];
                List_Add(event_params->car_head, car1, car1->Object_ID, 0);
            }
        }
    }

    p_stop1 = event_params->car_stop;
    while (p_stop1->next != NULL) {
        int flag = 0; // 判断跳出循环的标志位

        c1 = (ARS408RadarObjectInfo *)p_stop1->next->data;

        for (int i = 0; i < *Outlength; i++) {
            if (fabsf(Outdata[i].Object_DistLong - c1->Object_DistLong) <= adjust_Params.AdjustParams_X_Thr && (fabsf(Outdata[i].Object_DistLat - c1->Object_DistLat)) <= adjust_Params.AdjustParams_Y_Thr) {
                struct list *ptemp = p_stop1->next;
                p_stop1->next = p_stop1->next->next;
                free(ptemp);
                flag = 1;
                break;
            }
        }

        if (flag == 0) {
            p_stop1 = p_stop1->next;
        }
    }

    // 如果车辆队列中有了五帧信息就清空
    List_Del_Frame(event_params->car_head, adjust_Params.AdjustParams_Frame_car_list);
    List_Del_Frame(event_params->car_stop, adjust_Params.AdjustParams_Frame_car_stop);

    // 加入停车数据惯性运动补偿
    p_stop1 = event_params->car_stop;
    Inertia_Add(p_stop1, Inertia_frame);

    // 向outdata插入停车数据
    p_stop1 = event_params->car_stop;

    int index = 0;
    while (p_stop1->next != NULL) {
        c1 = (ARS408RadarObjectInfo *)p_stop1->next->data;
        Outdata[*Outlength + index] = *c1;
        Outdata[*Outlength + index].Object_VrelLat = 0;
        Outdata[*Outlength + index].Object_VrelLong = 0;
        Outdata[*Outlength + index].Object_timestamp = Outdata[0].Object_timestamp;

        index++;
        p_stop1 = p_stop1->next;
    }

    *Outlength = *Outlength + index;
}

/********************************************************
* Function name ：Lane_Judge
* Description   ：实现车辆车道的判断
* Parameter     ：
* @Outdata        滤波生成的目标数据
* @Outlength      目标数据的长度的指针
* Return        ：Outdata
**********************************************************/
void Lane_Judge(ARS408RadarObjectInfo *Outdata, int *Outlength)
{
#if 1
    for (int i = 0; i < *Outlength; i++) {
        // if ((Outdata->Object_DistLat >= -5.4) && (Outdata->Object_DistLat < -1.8)) {
        //     Outdata->Object_Lane = 1;
        // } else if ((Outdata->Object_DistLat >= -1.8) && (Outdata->Object_DistLat < 1.8)) {
        //     Outdata->Object_Lane = 2;
        // } else if ((Outdata->Object_DistLat >= 1.8) && (Outdata->Object_DistLat <= 5.4)) {
        //     Outdata->Object_Lane = 3;
        // } else {
        //     Outdata->Object_Lane = 0;
        // }

        // Outdata++;

        bool IsInRoi = 0;
        Point ObjectLoc = {.x = 0, .y = 0};
        ObjectLoc.x = Outdata->Object_DistLong;
        ObjectLoc.y = Outdata->Object_DistLat;
        Outdata->Object_Lane = 0;
        for (auto it = gRoadLaneParam.begin(); it != gRoadLaneParam.end(); it++) {
            RoadLaneParam param = it->second;
            Point ROILoc[4];
            ROILoc[0].x = param.lt_x;
            ROILoc[0].y = param.lt_y;

            ROILoc[1].x = param.rt_x;
            ROILoc[1].y = param.rt_y;

            ROILoc[2].x = param.rb_x;
            ROILoc[2].y = param.rb_y;

            ROILoc[3].x = param.lb_x;
            ROILoc[3].y = param.lb_y;

            IsInRoi = IsInRoi || IsPointInPolygon(ROILoc, 4, ObjectLoc);
            if (IsInRoi) {
                Outdata->Object_Lane = it->first;
                break;
            }
        }
        Outdata++;
    }
#else
    for (int i = 0; i < *Outlength; i++) {
        int lane_number = 0;
        float Object_DistLat = Outdata[i].Object_DistLat;       /* 目标的横向偏移 */

        for (auto laneParam : gRoadLaneParam) {
            if ((Object_DistLat >= laneParam.second.lb_y) && (Object_DistLat <= laneParam.second.rb_y)) {
                lane_number = laneParam.first;                  /* 直接使用车道编号 */
                break;
            }
        }

        Outdata[i].Object_Lane = lane_number;
    }
#endif
}

/********************************************************
* Function name ：Inertia_Add
* Description   ：增加停车惯性补偿
* Parameter     ：
* @List_stop      停车链表头指针
* @FrameStop      惯性补偿持续的帧数
* Return        ：List_stop
**********************************************************/
void Inertia_Add(struct list *List_stop, int FrameStop)
{
    ARS408RadarObjectInfo *car;

    while (List_stop->next != NULL) {
        if (List_stop->next->frame <= FrameStop) {
            car = (ARS408RadarObjectInfo *)List_stop->next->data;
            car->Object_VrelLong = car->Object_VrelLong * Inertia_speed;
            car->Object_VrelLat = car->Object_VrelLat * Inertia_speed;
            car->Object_DistLong = car->Object_DistLong - fabsf(Inertia_time * car->Object_VrelLong);
            car->Object_DistLat = car->Object_DistLat + Inertia_time * car->Object_VrelLat;
        }

        List_stop = List_stop->next;
    }
}

/********************************************************
* Function name ：Extra_Judge
* Description   ：实现对滤波后数据进行额外判断
* Parameter     ：
* @Outdata        滤波生成的目标数据
* @Outlength      目标数据的长度的指针
* @car_list       需要跟踪的车辆队列链表头节点指针
* @car_stop       停车链表的头节点指针
* Return        ：Outdata
**********************************************************/
void Extra_Judge(ARS408RadarObjectInfo *Outdata, int *Outlength, Event_Params *event_params)
{
    if (adjust_Params.EnableStopCar_Dect) {
        Stop_Dect(Outdata, Outlength, event_params);
    }

    if (adjust_Params.EnableLane_Judge) {
        Lane_Judge(Outdata, Outlength);
    }
}
