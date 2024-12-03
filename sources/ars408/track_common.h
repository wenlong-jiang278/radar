//#pragma once
#ifndef USER_TRACKCOMMON_H
#define USER_TRACKCOMMON_H


#include <math.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "cluster.h"
#include "List.h"

#ifdef __cplusplus
extern "C" {
#endif

/* 测试运行还是板载运行 */
#define TEST_MODE 0

#define MaxNumberROI 4

/*--- DEFINES ------------------------*/
#ifndef M_PI
// #define M_PI                     3.1415926536
#define M_PI                     3.141592653589793
#endif

#ifndef RAD2ANG
#define RAD2ANG                  57.2957795131
#endif

#ifndef ANG2RAD
#define ANG2RAD                  0.017453292519943
#endif

#define MAX_OUTPUT_OBJS 250


/* 定义事件检测解码参数 */
#define Over_Speed 0x0001  //超速
#define Retrograde 0x0002  //逆行
#define Stop_Car 0x0004    //停车
#define Change_Lane 0x0008 //变道
#define Congest 0x0010     //拥堵

// #define MaxLaneNum 4
#define LEN 10
#define userMaxPloyRoiNum 20

#define TRACK_USE_DOUBLE         1

#ifdef MAX_OUTPUT_OBJS
#define TRACK_NUM_CDI            MAX_OUTPUT_OBJS     /* candidate */
#define TRACK_NUM_TRK            MAX_OUTPUT_OBJS     /* tracker   */
#else
#define TRACK_NUM_CDI            250     /* candidate */
#define TRACK_NUM_TRK            150     /* tracker   */
#endif //MAX_OUTPUT_OBJS 

#if TRACK_USE_DOUBLE
#define TRACK_FABS       fabs
#define TRACK_POW        pow
#define TRACK_SIN        sin
#define TRACK_COS        cos
#define TRACK_SQRT       sqrt
#define TRACK_ATAN       atan
#define TRACK_LOG10      log10
#else
#define TRACK_FABS       fabsf
#define TRACK_POW        powf
#define TRACK_SIN        sinf
#define TRACK_COS        cosf
#define TRACK_SQRT       sqrtf
#define TRACK_ATAN       atanf
#define TRACK_LOG10      log10f
#endif

/*--- TYPEDEF ------------------------*/

/* float type used in track */
#if TRACK_USE_DOUBLE
typedef double track_float_t;
#else
typedef float  track_float_t;
#endif

/* measurement */
typedef struct {
    track_float_t rng;       /* range    */
    track_float_t vel;       /* velocity */
    track_float_t ang;       /* angle    */
#if TRK_CONF_3D
    track_float_t ang_elv;   /* angle of elevation */
    track_float_t sig_elv;   /* power of elevation*/
#endif
    track_float_t sig;       /* power    */
    track_float_t noi;       /* noise    */
} track_measu_t;


/* cluster Info */      // 存储该点聚类后的聚类信息
typedef struct {
    track_float_t NumberOfClusterPoint;   // 聚类簇内点云数量
    track_float_t LengthOfClusterPoint;   // 聚类簇的长即x长
    track_float_t WidthOfClusterPoint;    // 聚类簇的宽即y长
}ClusterState;


/* candidate */
typedef struct {
    uint32_t      index;
    track_measu_t raw_z;
    ClusterState ClusterInfo;
} track_cdi_t;

/* candidate package */
typedef struct {
    uint32_t    raw_number;            /* measurement number before pre-filter */
    uint32_t    cdi_number;            /* measurement number after  pre-filter */
    track_cdi_t cdi[TRACK_NUM_CDI];    /* measurement */
} track_cdi_pkg_t;

typedef struct {
    bool          output;
    uint32_t	  UUID;
    track_float_t DistLong;
    track_float_t DistLat;
    track_float_t VelLong;
    track_float_t VelLat;
    track_float_t ArelLong;
    track_float_t ArelLat;
    track_float_t RCS;
    track_float_t OrienAngle;
    track_float_t Length;
    track_float_t Width;
    uint32_t	  Class;

#if TRK_CONF_3D
    track_float_t ang_elv;
    track_float_t SNR_elv;
#endif
    uint32_t      track_level;
} track_obj_output_t;

typedef struct {
    track_float_t frame_int;
    uint32_t      frame_id;
    uint32_t      track_output_number;
} track_header_output_t;

/*--- ReadRadarPointCloudTypes ------------------------*/
#if TEST_MODE
/* 测试脚本代码用 */
typedef struct
{
    double		timestamp;
    uint32_t	ID;
    float		DistLong;
    float		DistLat;
    float		VrelLong;
    float		VrelLat;
    char		DynProp[30];
    float		RCS;
    char		DistLong_rms[10];
    char		DistLat_rms[10];
    char		VrelLong_rms[10];
    char		VrelLat_rms[10];
    char		Pdh0[10];
    char		AmbigState[10];
    char		InvalidState[10];

}MMWRadarPointCloud;
#endif

/* --- RadarOutputObjectDataTypes------------------- */
typedef struct _ARS408RadarObjectInfo
{
    double Object_timestamp; //时间戳
    uint32_t Object_ID; //目标ID
    float Object_DistLong; //纵(X)坐标
    float Object_DistLat; //横向(Y)坐标
    float Object_DistAlt; //
    float Object_VrelLong; //纵向相对速度(X)
    float Object_VrelLat; //横向相对速度(Y)
    float Object_VrelAlt;
    uint32_t Object_DynProp; //指示物体是运动还是静止
    float Object_RCS; //雷达散射截面积
    uint32_t Obj_DistLong_rms; //纵向距离的标准差
    uint32_t Obj_DistLat_rms; //横向距离的标准差
    uint32_t Obj_DistAlt_rms; //
    uint32_t Obj_VrelLong_rms; //纵向相对速度的标准差
    uint32_t Obj_VrelLat_rms; //横向相对速度的标准差
    uint32_t Obj_VrelAlt_rms; //
    uint32_t Obj_ArelLong_rms; //纵向相对加速度的标准差
    uint32_t Obj_ArelLat_rms; //横向相对加速度的标准差
    uint32_t Obj_ArelAlt_rms; //
    uint32_t Obj_Orientation_rms; //方向角的标准差
    uint32_t Obj_MeasState; //检测状态指示object是否有效并且已在新的测量周期中
    uint32_t Obj_ProbOfExist; //目标存在的可能性
    float Object_ArelLong; //纵向相对加速度
    float Object_ArelLat; //横向相对加速度
    float Object_ArelAlt; //
    float Object_Latitude; //纬度
    float Object_Longitude; //经度
    float Object_Altitude; //海拔
    uint32_t Object_Class; //目标类别
    float Object_OrientationAngel; //目标的方向角
    float Object_Length; //被跟踪对象的长度
    float Object_Width; //被跟踪对象的宽度
    uint32_t Object_CollDetRegionBitfield; //区域的字段
    uint32_t Object_Lane; //车道
    uint32_t Object_Event;  /* 事件检测 */ /* 1.超速 2.逆行 3.变道 4.停车 5.拥堵*/
} ARS408RadarObjectInfo;

typedef struct _ARS408RadarObjectInfo_Wr
{
    double Object_timestamp; //时间戳
    uint32_t Object_ID; //目标ID
    float Object_DistLong; //纵(X)坐标
    float Object_DistLat; //横向(Y)坐标
    float Object_DistAlt; //
    float Object_VrelLong; //纵向相对速度(X)
    float Object_VrelLat; //横向相对速度(Y)
    float Object_VrelAlt;
    uint32_t Object_DynProp; //指示物体是运动还是静止
    float Object_RCS; //雷达散射截面积
    uint32_t Obj_DistLong_rms; //纵向距离的标准差
    uint32_t Obj_DistLat_rms; //横向距离的标准差
    uint32_t Obj_DistAlt_rms; //
    uint32_t Obj_VrelLong_rms; //纵向相对速度的标准差
    uint32_t Obj_VrelLat_rms; //横向相对速度的标准差
    uint32_t Obj_VrelAlt_rms; //
    uint32_t Obj_ArelLong_rms; //纵向相对加速度的标准差
    uint32_t Obj_ArelLat_rms; //横向相对加速度的标准差
    uint32_t Obj_ArelAlt_rms; //
    uint32_t Obj_Orientation_rms; //方向角的标准差
    uint32_t Obj_MeasState; //检测状态指示object是否有效并且已在新的测量周期中
    uint32_t Obj_ProbOfExist; //目标存在的可能性
    float Object_ArelLong; //纵向相对加速度
    float Object_ArelLat; //横向相对加速度
    float Object_ArelAlt; //
    double Object_Latitude; //纬度
    double Object_Longitude; //经度
    double Object_Altitude; //海拔
    uint32_t Object_Class; //目标类别
    float Object_OrientationAngel; //目标的方向角
    float Object_Length; //被跟踪对象的长度
    float Object_Width; //被跟踪对象的宽度
    uint32_t Object_CollDetRegionBitfield; //区域的字段
    uint32_t Object_Lane; //车道
    uint32_t Object_Event;  /* 事件检测 */ /* 1.超速 2.逆行 3.变道 4.停车 5.拥堵*/
} ARS408RadarObjectInfo_Wr;

typedef struct _ROIPosition
{
    char name[25];
    double x1;
    double y1;
    double x2;
    double y2;
    double x3;
    double y3;
    double x4;
    double y4;

}ROIPositionDef;

typedef struct _ROIRegion
{
    int UsingNumbofROI; /* 在使用ROI多少个 */
    ROIPositionDef PositionofROI[MaxNumberROI]; /* 初始化4个ROI位置空间，最多四个 */

}ROIRegionDef;

typedef struct _ADJUST_PARAMS
{
    /* 系统参数 */

    bool EnableTrajectoryManage; /* 以下使能位默认为ture */
    bool EnableExtra_Judge;
    bool EnableSpeed_Dect;
    bool EnableRetrograde_Dect;
    bool EnableStopCar_Dect;
    bool EnableLaneChange_Dect;
    bool EnableCongest_Dect;
    bool EnableLane_Judge;
    bool EnabletrafficFlow;
    bool EnablelaneDividedTrafficFlow;
    bool EnablesectionAverageSpeed;
    bool EnablelaneDividedAverageSpeed;
    bool EnablelaneDividedTimeOccupancy;
    bool EnablelaneDividedAverageHeadway;
    bool EnablelaneDividedAverageHeadwayGap;
    int MaxLaneNum;//最大车辆数

    /* 输入数据滤波器设置选项 */
    double AdjustParams_DisLongMin;  /* 0 -> 500 */   /* 纵向距离最小值 */ /* 10 */
    double AdjustParams_DisLongMax;  /* 0 -> 500 */   /* 纵向距离最大值 */ /* 200 */
    double AdjustParams_DisLatMin;   /* -100 -> 100 */ /* 横向距离最小值 */ /* -50 */
    double AdjustParams_DisLatMax;   /* -100 -> 100 */ /* 横向距离最大值 */ /* -50 */
    double AdjustParams_VrelLongMin; /* -50 -> 50 */  /* 纵向速度最小值 */  /* -50 */
    double AdjustParams_VrelLongMax; /* -50 -> 50 */  /* 纵向速度最大值 */  /* 50 */
    double AdjustParams_VrelLatMin;  /* -50 -> 50 */  /* 横向速度最小值 */  /* - 50 */
    double AdjustParams_VrelLatMax;  /* -50 -> 50 */  /* 横向速度最大值 */  /* 50 */
    double AdjustParams_RCSMin;      /* -50 -> 50 */  /* RCS最小值 */       /* -50 */
    double AdjustParams_RCSMax;      /* -50 -> 50 */  /* RCS最大值 */       /* 50 */
    /* 聚类设置选项 */
    short int AdjustParams_DBSCANMinPts;    /* 1 -> 100 */ /* 聚类最少点数*/ /* 1 */
    double AdjustParams_DBSCANBIGCarEps_a;  /* 0 -> 100 */ /* 大车聚类椭圆短半径*/ /* 1.5 */
    double AdjustParams_DBSCANBIGCarEps_b;  /* 0 -> 100 */ /* 大车聚类椭圆长半径*/ /* 6 */
    double AdjustParams_DBSCANMEDIUMCarEps_a;  /* 0 -> 100 */ /* 中车聚类椭圆短半径*/ /* 1.65 */
    double AdjustParams_DBSCANMEDIUMCarEps_b;  /* 0 -> 100 */ /* 中车聚类椭圆长半径*/ /* 9 */
    double AdjustParams_DBSCANSMALLCarEps_a;   /* 0 -> 100 */ /* 小车聚类椭圆短半径*/ /* 1.8 */
    double AdjustParams_DBSCANSMALLCarEps_b;   /* 0 -> 100 */ /* 小车聚类椭圆短半径*/ /* 12 */
    /* 关联波门大小 */
    double AdjustParams_EKFPairRNGTHR;	/* 0 -> 100 */ /* 距离维关联波门 */ /* 15 */
    double AdjustParams_EKFPairVELTHR;  /* 0 -> 100 */ /* 速度维关联波门 */ /* 25 */
    double AdjustParams_EKFPairANGTHR;  /* 0 -> 100 */ /* 角度维关联波门 */ /* 15 */
    /* 航迹更新帧数阈值 */
    short int AdjustParams_EKFUpdatePreTypeThres; /* 0 -> 100 */ /* 航迹确认 */ /* 3 */
    short int AdjustParams_EKFUpdateValTypeThres; /* 0 -> 100 */ /* 航迹取消 */ /* 5 */
    /* 航迹滤波参数 */
    double AdjustParams_EKFFilterTimeInt;	/* 0 -> 10 */    /* 帧间隔时间 */ /* 0.05 */
    double AdjustParams_EKFFilterACC;		/* 0 -> 100 */   /* 跟踪滤波α*/  /* 0.10 */
    /* 另添加 */
    double AdjustParams_EKFFilterR_RNG;    /* 量测噪声距离方差 */
    double AdjustParams_EKFFilterR_VEL;	   /* 量测噪声速度方差 */
    double AdjustParams_EKFFilterR_ANG;    /* 量测噪声角度方差 */
    /* 车型判断参数 */
    double AdjustParams_SmallCarRCSThres;  /* -50 -> 50 */ /* 小车RCS最大值 */ /* 8 */
    double AdjustParams_MediumCarRCSThres; /* -50 -> 50 */ /* 中车RCS最大值 */ /* 21 */
    double AdjustParams_BigCarRCSThres;    /* -50 -> 50 */ /* 大车RCS最大值 */ /* 50 */
    /* 事件检测参数 */
    double AdjustParams_V_Thr;     /* 跟踪停车时的最小速度 */ /* 5 */
    double AdjustParams_Frame_car_list;    /* 跟车链表的保留最久帧数 */ /* 10 */
    double AdjustParams_Frame_car_stop;    /* 停车链表的保留最久帧数 */ /* 900 */
    double AdjustParams_X_Thr;    /* 重启动X阈值 */ /* 1.5 */
    double AdjustParams_Y_Thr;    /* 重启动Y阈值 */ /* 1.5 */
    double AdjustParams_MAX_SPEED;    /* 超速的速度阈值 */ /* 16.67 */
    double AdjustParams_LIMIT_SPEED;    /* 拥堵车辆速度阈值 */ /* 0.5 */
    double AdjustParams_LIMIT_DIST;    /* 拥堵车辆距离阈值 */ /* 10 */
    /* ROI区域滤波 */
    ROIRegionDef AdjustParams_ROI;


}AdjustParams;

typedef struct _Calibration_Params
{
    /* 标定相关参数 */
    long double matrix_1;
    long double matrix_2;
    long double matrix_3;
    long double matrix_4;
    long double matrix_5;
    long double matrix_6;
    long double p00;
    long double p10;
    long double p01;
    long double p20;
    long double p11;
    long double p02;
    long double origin_longitude;
    long double origin_latitude;
    long double angle;
    long double is_del;
    long double status;
    long double create_by;
    long double create_date;
    long double update_by;
    long double update_date;
}CalbParams;

typedef struct _StaticCalibration_Params
{
    /* 静态标定相关参数 */
    long double matrix_1; //预留
    long double matrix_2; //预留
    long double matrix_3; //预留
    long double matrix_4; //预留
    long double matrix_5; //预留
    long double matrix_6; //预留
    long double p00; //高程系数p00
    long double p10; //高程系数p10
    long double p01; //高程系数p01
    long double p20; //高程系数p20
    long double p11; //高程系数p11
    long double p02; //高程系数p02
    long double origin_longitude; //雷达原点经度
    long double origin_latitude; //雷达原点纬度
    long double angle; //雷达法线正北偏转角
    long double is_del;
    long double status;
    long double create_by;
    long double create_date;
    long double update_by;
    long double update_date;
}StaticCalbParams;

/* 事件检测需要参数结构体 */
typedef struct _Event_Params
{
    struct list* car_stop;
    struct list* car_head;
    ARS408RadarObjectInfo Lane_Change_List[TRACK_NUM_TRK];
    int Lane_List_Length;
} Event_Params;


//线圈的位置
typedef struct _CoilPosition
{
    //线圈对应车道
    int laneNum;
    //线圈x坐标
    float coilX;
    //线圈y坐标
    float coilY;
    //线圈宽度
    float coilWidth;
    //线圈长度
    float coilLenth;
    int coil_status;

}CoilPositionDef;

//断面线圈信息
typedef struct _SectionAndCoilRegion
{
    //断面id
    int section_id;
    //断面的X坐标
    float section_x;
    //线圈配置信息:四条车道
    CoilPositionDef CoilPosition[LEN];
    //检测周期（单位s）
    float detectionCycle;
    //线圈数
    int coilNum;
    int section_status;

}SectionAndCoilRegionDef;

//检测区域
typedef struct userPolyRegion
{
    //区域id
    int id;
    //x坐标
    float P1X;
    //y坐标
    float P1Y;
    //x坐标
    float P2X;
    //y坐标
    float P2Y;
    //x坐标
    float P3X;
    //y坐标
    float P3Y;
    //x坐标
    float P4X;
    //y坐标
    float P4Y;
}UserPolyRegion;

typedef struct {
    int     flipXY;       /* 翻转X和Y轴输出(0不反转，1翻转) */
    float   xOffset;      /* 横向偏移量 */
    float   yOffset;      /* 纵向偏移量 */
    uint8_t xDirect;      /* X轴方向(0右(前 - 如果XY不反转)为正，1左(后 - 如果XY不反转)为正) */
    uint8_t yDirect;      /* Y轴方向(0前(左 - 如果XY不反转)为正，1后(右 - 如果XY不反转)为正) */
} UserOffsetParam;

typedef struct {
    int   dire;           /* 照射方向(0来向，1去向) */
    int   lane;           /* 道路编号(超车道为1车道，依次向慢车道递增) */
    int   turn;           /* 0直道，1向左弯道，2向右弯道 */
    float width;          /* 道路宽度(米) */
    float radius;         /* 曲率 */
    float lt_x;           /* 左上角X - LeftTop */
    float lt_y;           /* 左上角Y */
    float rt_x;           /* 右上角X - RightTop */
    float rt_y;           /* 右上角Y */
    float rb_x;           /* 右下角X - RightBottom */
    float rb_y;           /* 右下角Y */
    float lb_x;           /* 左下角X - LeftBottom */
    float lb_y;           /* 左下角Y */
} RoadLaneParam;

#ifdef __cplusplus
}
#endif

#endif
