
#ifndef USER_TRACK_H
#define USER_TRACK_H

#include <queue>

#include "UnitTrack.h"
#include "RadarSysParams.h"
#include "track_common.h"
#include "TrafficFlowDetection.h"

#ifdef __cplusplus
extern "C" {
#endif

// #pragma once
// #ifndef __TRACK
// #define __TRACK

typedef void (*track_if_t)(void *data, void *ctx);
typedef void (*track_frame_interval_t)(void *data, track_float_t delta);
typedef void (*track_update_obj_t)(void *data, void *ctx, uint32_t i);
typedef bool (*track_has_run_t)(void *data);

/* tracker top */
typedef struct {
  track_float_t f_last_time; /* time stamp of last frame */
  track_float_t f_now;       /* time stamp of current frame */
  track_cdi_pkg_t cdi_pkg;   /* raw data read from HW with conversion */

  track_obj_output_t
      output_obj; /* algorithm will update it for object data output */
  track_header_output_t
      output_hdr; /* algorithm will update it for header data output */

  void* trk_data; /* tracking algorithm specific data */

  track_if_t trk_init; /* algorithm init handler */
  track_if_t trk_pre;  /* algorithm pre-run handler */
  track_if_t trk_run;  /* algorithm run handler */
  track_if_t trk_post; /* algorithm post-run handler */
  track_if_t trk_stop; /* algorithm stop handler */

  track_has_run_t trk_has_run; /* whether it is first time to run algorithm */

  track_frame_interval_t
      trk_set_frame_int; /* set frame interval for algorithm to run */

  track_update_obj_t trk_update_obj_info; /* handler to update "output_obj" */
  track_if_t trk_update_header;           /* handler to update "output_hdr" */

  track_if_t trk_Classification; /* Tracking Object Classification */
  track_if_t trk_paramsUpdate;   /* Tracking Object Classification */

  radar_sys_params_t* radar_params; /*Params of Radar Config */

} track_t;

/* track output data */
typedef struct {
  uint32_t* data;
  bool lock;
} trk_t;

#define FRAME_CACHE_MAX 3

typedef struct {
    int             tag;
    track_float_t   fts;
    track_cdi_pkg_t data;
} frame_buff_t;

/* --- 配置跟踪器 -------------------- */
void Install_Ekf_Track(track_t *track);
/*--- DECLARAION ---------------------*/
void track_init(track_t *track);
void track_pre_start(track_t *track);
// bool track_is_ready(track_t* track);
// void track_lock(track_t* track);
void track_Params_update(track_t *track);
void track_stop(track_t *track);
#if TEST_MODE
void track_read(track_t *track, MMWRadarPointCloud *RadarPointCloud, int PointNumber);
#else
void track_read(track_t *track, RadarClusterInfoARS* RadarPointCloud, int PointNumber);
#endif
void track_run(track_t *track);
void track_OutObjectList(track_t *track, ARS408RadarObjectInfo *ObjectList, int *ObjectNum);
void track_TrafficEventDetection();
void track_SortRCS(track_cdi_pkg_t *raw_input);
void track_SortRCSDiffe(track_cdi_pkg_t *raw_input);

void track_coordinate2GPS(CalbParams Calibration_Params, double range_X, double range_Y, long double* tmp);
void Cal_latlon(double lat1, double lon1, double angle, double D_x, double D_y, double *lat2, double *lon2);

/* ---------- 跟踪程序主函数 ------------------ */
#if TEST_MODE

#else
void Track_PointCloud_Process(track_t *track, RadarClusterInfoARS *RadarPointCloud, int PointNumber, ARS408RadarObjectInfo *RadarObjectData, int *ObjectNumber, Event_Params *Event_Params);
#endif

extern track_t track;
#if TEST_MODE
extern FILE* TrackingObjectData_fp;
#endif
extern AdjustParams adjust_Params;
extern int user_flushParams(void* ptr);

extern TrafficFlowDetectionStatisticsDef trafficFlowDetectionStatisticsDef;

extern EnableTrafficDetrctionDef enableTrafficDetrctionDef;

extern IDMemoryAllDef idMemoryAllDef;

extern headerAllDef had;

extern SectionAndCoilRegionDef sectionAndCoilRegionDef;

/*----------输出延时补偿----------------------*/
#define DELAY_FRAME 20

typedef struct ARS408RadarObjectInfoElem_t {
    ARS408RadarObjectInfo              data;
    struct ARS408RadarObjectInfoElem_t *prev;
    struct ARS408RadarObjectInfoElem_t *next;
} ARS408RadarObjectInfoElem;

typedef struct {
    uint32_t                  length;
    ARS408RadarObjectInfoElem *begin;
    ARS408RadarObjectInfoElem *end;
} ConpensationList;

typedef struct Track_ListElem_t {
    ConpensationList        *conpensationList;
    struct Track_ListElem_t *prev;
    struct Track_ListElem_t *next;
    int                     flag;
    uint32_t                Eid;
} Track_ListElem;

typedef struct Time_ListElem_t {
    double timeStamp;
    Time_ListElem_t *prev;
    Time_ListElem_t *next;
}Time_ListElem;

typedef struct {
    int count;
    Time_ListElem* begin;
    Time_ListElem* end;
}Time_List;
typedef struct {
    uint32_t       count;
    Track_ListElem *begin;
    Track_ListElem *end;

    Time_List* timeList;
} Track_ListObj;

void track_listInit(Track_ListObj **pList);
int32_t track_is_ListEmpty(Track_ListObj *list);

int32_t track_listEnQueue(Track_ListObj *list, Track_ListElem *elem);
int32_t track_listDeQueue(Track_ListObj *list, int32_t Elem_id);
Track_ListElem *track_is_Exist(Track_ListObj *list, int32_t Elem_id);
Track_ListElem *track_listGetFirst(Track_ListObj *list);

uint32_t track_listGetCount(Track_ListObj *list);
void track_DelayCompensation(Track_ListObj *list, ARS408RadarObjectInfo *out_RadarObjectData, int *out_ObjectNumber);

void ConpensationListInit(ConpensationList **pc_list);
int32_t ConpensationListIsEmpty(ConpensationList *c_list);
int32_t ConpensationListEnQueue(ConpensationList *c_list, ARS408RadarObjectInfoElem* elem);

ARS408RadarObjectInfo ConpensationListDeQueue(ConpensationList *c_list);

void Time_ListInit(Time_List **ptList);
int32_t Time_ListIsEmpty(Time_List* list);
int32_t Time_ListEnQueue(Time_List* list, Time_ListElem* elem);
void Time_ListDeQueue(Time_List* list);
void Time_ListClear(Time_List* list);
Time_ListElem* Time_ListSearch(Time_List* list);
#ifdef __cplusplus
}
#endif

#endif
