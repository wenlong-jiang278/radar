/********************************************************************************
 * @File name:can_tcp.h
 * @Author:李军
 * @Version: 1.0
 * @Date:2023.05.15
 * @Description:与雷达板子的CAN通讯，与JAVA端的TCP通讯
 ********************************************************************************/

#ifndef USER_CAN_TCP_H
#define USER_CAN_TCP_H

#include "List.h"
#include "Track.h"
#include "cluster.h"
#include "ini.h"
#include "sqlite3.h"
#include "epoll_tcp_server.h"

#include <linux/can.h>
#include <pthread.h>

#ifdef __cplusplus
extern "C" {
#endif

#define USER_MAX_CANBUF 8000 // CAN缓存条数

// 接收object列表的状态
#define RADAR_OBJECT_IDLE 0
#define RADAR_OBJECT_GET_GENERAL_INFO 1
#define RADAR_OBJECT_GET_QUALITY_INFO 2
#define RADAR_OBJECT_GET_EXTENDED_INFO 3
#define RADAR_OBJECT_GET_COLLISION_INFO 4

//接收cluster列表的状态
#define RADAR_CLUSTER_IDLE               0
#define RADAR_CLUSTER_GET_GENERAL_INFO   1
#define RADAR_CLUSTER_GET_QUALITY_INFO       2

// 丰海协议数据类型定义
#define FENGHAI_CMD_RADAR_BASIC_CONFIG 0x31  // 雷达基本配置
#define FENGHAI_CMD_RADAR_FILTER_CONFIG 0x32 // 雷达过滤器配�?
#define FENGHAI_CMD_RADAR_REGION_CONFIG 0x33 // 雷达多边形过滤配�?

typedef unsigned char BYTE;
typedef char CHAR;
typedef int INT;
typedef unsigned int UINT;
typedef unsigned short USHORT;
typedef unsigned long ULONG;
typedef unsigned long DWORD;
typedef char T;            // 1
typedef unsigned char UT;  // 1
typedef short S;           // 2
typedef unsigned short US; // 2
typedef int L;             // 4
typedef unsigned int UL;   // 4

typedef struct _CanSend {
  UINT ID;
  BYTE DataLen;
  BYTE SendType;
  BYTE RemoteFlag; // 是否是远程帧
  BYTE ExternFlag; // 是否是扩展帧
  BYTE Data[8];
} CanSend, *PCanSend;

// 时间结构�?
typedef struct _FH_TIME {
  UT year;
  UT month;
  UT day;
  UT hour;
  UT min;
  UT sec;
  US ms;
} FH_TIME, *PFH_TIME;

/*typedef*/ struct FH_TIME_CLUSTER {
  UT year;
  UT month;
  UT day;
  UT hour;
  UT min;
  UT sec;
  US ms;
} __attribute__((packed));

typedef struct _FH_TIME_FORMAT {
  float year;
  float month;
  float day;
  float hour;
  float min;
  float sec;
  float ms;
} FH_TIME_FORMAT, *PFH_TIME_FORMAT;

// 设备结构�?

typedef struct _FH_DEV_ID {
  UT dev[20];
} FH_DEV_ID, *PFH_DEV_ID;

/*typedef*/ struct FH_DEV_ID_CLUSTER {
  UT dev[20];
} __attribute__((packed));

// 目标数据

typedef struct _FH_OBJ_DATA {
  UL second;      // 时间戳（秒）
  UL millisecond; // 时间戳（毫秒�?
  US id;          // 目标编号 循环 ID 分配 取值：0~65535
  UT road; // 目标坐标点位于用户设定的具体车道�? 车道�? 0XFF
  UT object_class;        // 目标类型
  US obj_length;          // 目标长度
  US obj_width;           // 目标宽度
  US orientation_angel;   // 目标偏航�?
  US x;                   // X 坐标
  S y;                    // Y 坐标
  S x_speed;              // X 轴速度
  S y_speed;              // Y 轴速度
  US speed;               // 车�?
  S a_speed;              // 目标运动方向加速度
  S a_x_speed;            // X 轴加速度
  S a_y_speed;            // Y 轴加速度
  float Object_RCS;       // 雷达散射截面
  L longitude;            // 保留，未使用
  L latitude;             // 保留，未使用
  UL event;               // 事件类型
  UL lane;                // 车道
  float Object_Latitude;  // 目标坐标点纬�?
  float Object_Longitude; // 目标坐标点经�?
  float Object_Altitude;  // 目标坐标点高�?
} FH_OBJ_DATA, *PFH_OBJ_DATA;

typedef struct _FH_OBJ_DATA_Wr { // 经纬度、海拔用double表示
  UL second;                     // 时间戳（秒）
  UL millisecond;                // 时间戳（毫秒�?
  US id;   // 目标编号 循环 ID 分配 取值：0~65535
  UT road; // 目标坐标点位于用户设定的具体车道�? 车道�? 0XFF
  UT object_class;         // 目标类型
  US obj_length;           // 目标长度
  US obj_width;            // 目标宽度
  US orientation_angel;    // 目标偏航�?
  US x;                    // X 坐标
  S y;                     // Y 坐标
  S x_speed;               // X 轴速度
  S y_speed;               // Y 轴速度
  US speed;                // 车�?
  S a_speed;               // 目标运动方向加速度
  S a_x_speed;             // X 轴加速度
  S a_y_speed;             // Y 轴加速度
  float Object_RCS;        // 雷达散射截面
  L longitude;             // 保留，未使用
  L latitude;              // 保留，未使用
  UL event;                // 事件类型
  UL lane;                 // 车道
  double Object_Latitude;  // 目标坐标点纬�?
  double Object_Longitude; // 目标坐标点经�?
  double Object_Altitude;  // 目标坐标点高�?
} FH_OBJ_DATA_Wr, *PFH_OBJ_DATA_Wr;

// 帧数据结构体
typedef struct _FH_OBJ {
  UL head;               // 包头 DD FF AA 55
  US length;             // 数据长度
  UT type;               // 数据类型
  FH_DEV_ID dev_id;      // 设备编号
  FH_TIME time;          // 数据发送时间（精确�? ms�?
  FH_OBJ_DATA Data[100]; // 目标数据
  US crc;                // CRC 校验�?
  UL end;                // 包尾 EA EB EC ED
} FH_OBJ, *PFH_OBJ;

// 帧数据结构体
typedef struct _FH_OBJ_Wr { // 经纬度、海拔用double表示
  UL head;                  // 包头 DD FF AA 55
  US length;                // 数据长度
  UT type;                  // 数据类型
  FH_DEV_ID dev_id;         // 设备编号
  FH_TIME time;             // 数据发送时间（精确�? ms�?
  FH_OBJ_DATA_Wr Data[100]; // 目标数据
  US crc;                   // CRC 校验�?
  UL end;                   // 包尾 EA EB EC ED
} FH_OBJ_Wr, *PFH_OBJ_Wr;

// 点云数据透传结构�?

typedef struct _FH_CLUSTER {
  UL head;                         // 包头 DD FF AA 55
  US length;                       // 数据长度
  UT type;                         // 数据类型
  struct FH_DEV_ID_CLUSTER dev_id; // 设备编号
  struct FH_TIME_CLUSTER time;     // 数据发送时间（精确�? ms�?
  RadarClusterInfoARS Data[MAX_CLUSTER_INFO]; // 点云数据内容
  US crc;                                     // CRC 校验�?
  UL end;                                     // 包尾 EA EB EC ED
} __attribute__((packed)) FH_CLUSTER;

// 与上位机通信的丰海协�?
typedef struct {
  UL head;     // 包头 DD FF AA 55
  US length;   // 数据长度
  UT type;     // 数据类型
  char dev_id; // 设备编号
  char data[256];
  US crc; // CRC 校验�?
  UL end; // 包尾 EA EB EC ED
} S_FHCOMMFORM;

// 2.定义CAN信息帧的数据类型结构�?
typedef struct _VCI_CAN_OBJ {
  UINT ID;
  UINT TimeStamp;
  BYTE TimeFlag;
  BYTE SendType;
  BYTE RemoteFlag; // 是否是远程帧
  BYTE ExternFlag; // 是否是扩展帧
  BYTE DataLen;
  BYTE Data[8];
  BYTE Reserved[3];
} VCI_CAN_OBJ, *PVCI_CAN_OBJ;

typedef struct _CanFrame {
  UINT can_id;
  BYTE none1;
  BYTE none2;
  BYTE none3;
  BYTE can_dlc;
  BYTE data[8];
} CanFrame, *PCanFrame;

typedef struct user_CANBuf {
  pthread_rwlock_t mtxCanBuf;
  int wrIdx;
  int rdIdx;
  int cnt;
  struct can_frame canFrmBuf[USER_MAX_CANBUF];
  double canTime[USER_MAX_CANBUF];
} USER_CANBUF;

typedef struct {
    float x;            /* X坐标 */
    float y;            /* Y坐标 */
    float s;            /* 速度 */
    float sx;           /* X轴速度 */
    float sy;           /* Y轴速度 */
    float a;            /* 加速度 */
    float ax;           /* X轴加速度 */
    float ay;           /* Y轴加速度 */
} OffsetAdjustParams;

extern pthread_mutex_t mutex;

extern bool needRecord;
extern bool preNeedRecord;
extern int frameFlag;
extern int recordMaxFlag;
extern const char *fileName;
extern const char *tableName;
extern long pthreadCount;
extern int insertCount;
extern int preMilli;
extern int Canfd;
extern bool respondRadarBasicConfig;
extern char deviceID;
extern int clientfd;
extern radar_sys_params_t sys_params;
extern track_trk_pkg_t trk_pkg;
// extern Event_Params event_params;
extern ARS408RadarObjectInfo RadarObjectData[250];
extern int ObjectNumber;
extern Event_Params event_params;
extern SectionAndCoilRegionDef sections[LEN];
extern int section_num;
extern pthread_mutex_t g_mtxTrafficFlowDetectionStatistics;
extern USER_CANBUF g_canBuf;
extern CalbParams g_CalbPara;
extern StaticCalbParams g_StaticCalbPara;
extern int polyRoi_num;
extern UserPolyRegion polyRoi[];
extern UserOffsetParam gOffsetParam;

void *usr_readCanData(void *arg);
extern void getDevId(uint8_t *devId);
extern void getDevId_cluster(struct FH_DEV_ID_CLUSTER *dev_id);
extern FH_TIME GetLocalTimeWithMs(FH_TIME fh_time);
extern void GetLocalTimeWithMsEx(FH_TIME *fh_time);
extern struct FH_TIME_CLUSTER GetLocalTimeWithMs_cluster(struct FH_TIME_CLUSTER fh_time);
extern void GetLocalTimeWithMsEx_cluster(struct FH_TIME_CLUSTER *fh_time);
extern int getCRC(FH_OBJ *frame);
extern bool isFHOBJ(FH_OBJ frame, FH_OBJ preFrame);
extern void dbOperate(FH_OBJ *frame);
extern void *closeDB(void *frame);
extern void *openOrInsertDB(void *frame);
extern int findMaxRecordFlag(const char *tableName, sqlite3 *dtb);
extern void insertTable(const char *tableName, FH_OBJ frame, int flag, int recordFlag, sqlite3 *dtb);
extern unsigned char *getTimeStr(FH_OBJ *frame, unsigned char timeStr[]);
extern int decodeUpperComputerData(char *receiveData, int receiveNum);
extern int dealRadarConfigData(char dataType, char *dataBuf, int dataLen);
extern int getCRC_cluster(FH_CLUSTER *frame);
extern bool isFHCLUSTER(FH_CLUSTER cluster, FH_CLUSTER preCluster);
extern int user_flushParams(void *ptr);
extern int user_readClusterAndObjectSend2Rearend_func(void);
extern int user_writeClusterAndObjectSend2Rearend_func(void);
extern int user_readDynamicOrStaticCalibrationMode_func(void);
extern int user_writeDynamicOrStaticCalibrationMode_func(void);
extern void *user_hdlCanFrameThread(void *arg);

extern int user_readCalibrationPara(void);
extern int user_readStaticCalibrationPara(void);

extern int user_readRoadLaneParam(void);
extern int user_readOffsetParameters(void);

#ifdef __cplusplus
}
#endif

#endif
