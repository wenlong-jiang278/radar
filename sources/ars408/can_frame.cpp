#include <ctime>
#include <thread>
#include <future>
#include <chrono>
#include <vector>
#include <cstdbool>
#include <spdlog/spdlog.h>

#include "can_frame.h"

#if defined(CONFIG_CANFRAME)
#define MAX_RADAR_OBJECTS                       (100)
#define MAX_RADAR_CLUTSERS                      (250)

/* 接收目标列表的状态 */
#define OBJECT_RADAR_IDLE                       (0)
#define OBJECT_RADAR_GET_GENERAL_INFO           (1)
#define OBJECT_RADAR_GET_QUALITY_INFO           (2)
#define OBJECT_RADAR_GET_EXTENDED_INFO          (3)
#define OBJECT_RADAR_GET_COLLISION_INFO         (4)

/* 接收点云列表的状态 */
#define CLUSTER_RADAR_IDLE                      (0)
#define CLUSTER_RADAR_GET_GENERAL_INFO          (1)
#define CLUSTER_RADAR_GET_QUALITY_INFO          (2)

/* 滤波器索引 */
#define FILTER_NofObj                           (0)
#define FILTER_Distance                         (1)
#define FILTER_Azimuth                          (2)
#define FILTER_VrelOncome                       (3)
#define FILTER_VrelDepart                       (4)
#define FILTER_RCS                              (5)
#define FILTER_Lifetime                         (6)
#define FILTER_Size                             (7)
#define FILTER_ProbExists                       (8)
#define FILTER_Y                                (9)
#define FILTER_X                                (0xA)
#define FILTER_VYRightLeft                      (0xB)
#define FILTER_VXOncome                         (0xC)
#define FILTER_VYLeftRight                      (0xD)
#define FILTER_VXDepart                         (0xE)

/* 雷达CAN输入ID */
typedef enum {
    CAN_I_RADAR_CFG       = 0x200,              /* 雷达传感器配置 */
    CAN_I_FILTER_CFG      = 0x202,              /* 点云和目标过滤配置 */
    CAN_I_POLY_CFG        = 0x205,              /* 多边形过滤器配置 */
    CAN_I_COLLDET_CFG     = 0x400,              /* 碰撞检测配置 */
    CAN_I_COLLDET_RGN_CFG = 0x401,              /* 碰撞检测区域配置 */
    CAN_I_SPEED_INFO      = 0x300,              /* 车辆速度(传感器平台) */
    CAN_I_YAW_RATE_INFO   = 0x301,              /* 车辆偏航角(传感器平台) */
} can_inid_e;

/* 雷达CAN 输出ID */
typedef enum {
    CAN_O_RADAR_STATE       = 0x201,            /* 雷达状态 */
    CAN_O_FILTER_STATE_HEAD = 0x203,            /* 过滤状态头 */
    CAN_O_FILTER_CFG        = 0x204,            /* 过滤配置状态 */
    CAN_O_POLY_STATE        = 0x206,            /* 多边形过滤器状态 */
    CAN_O_COLLDET_STATE     = 0x408,            /* 碰撞检测状态 */
    CAN_O_COLLDET_RGN_STATE = 0x402,            /* 碰撞检测区域状态 */
    CAN_O_CLUSTER_0_STATUS  = 0x600,            /* 点云状态(列表头) */
    CAN_O_CLUSTER_1_GENERAL = 0x701,            /* 点云一般信息 */
    CAN_O_CLUSTER_2_QUALITY = 0x702,            /* 电源质量信息 */
    CAN_O_OBJECT_0_STATUS   = 0x60A,            /* 目标状态(列表头) */
    CAN_O_OBJECT_1_GENERAL  = 0x60B,            /* 目标一般信息 */
    CAN_O_OBJECT_2_QUALITY  = 0x60C,            /* 目标质量信息 */
    CAN_O_OBJECT_3_EXTENDED = 0x60D,            /* 目标扩展信息 */
    CAN_O_OBJECT_4_WARNING  = 0x60E,            /* 目标碰撞检测报警 */
    CAN_O_VERSION_ID        = 0x700,            /* 软件版本标识 */
    CAN_O_COLLDET_RELAY_CTL = 0x8,              /* 控制继电器信息 */
} can_outid_e;

/* 雷达滤波器配置状态结构体 */
typedef struct {
    uint32_t NofClusterFilterCfg;               /* 已配置的点云过滤器的状态消息数量 */
    uint32_t NofObjectFilterCfg;                /* 已配置的目标过滤器的状态消息数量 */
    bool     NofObj_Active;
    bool     Distance_Active;
    bool     Azimuth_Active;
    bool     VrelOncome_Active;
    bool     VrelDepart_Active;
    bool     RCS_Active;
    bool     Lifetime_Active;
    bool     Size_Active;
    bool     ProbExists_Active;
    bool     Y_Active;
    bool     X_Active;
    bool     VYLeftRight_Active;
    bool     VXOncome_Active;
    bool     VYRightLeft_Active;
    bool     VXDepart_Active;
    uint32_t Min_NofObj;
    uint32_t Max_NofObj;
    float    Min_Distance;                      /* 最小径向距离(m) */
    float    Max_Distance;                      /* 最大径向距离(m) */
    float    Min_Azimuth;                       /* 最小方位角(°) */
    float    Max_Azimuth;                       /* 最大方位角(°) */
    float    Min_VrelOncome;                    /* 最小传感器视线中来向的点云或目标径向速度(m/s) */
    float    Max_VrelOncome;                    /* 最大传感器视线中来向的点云或目标径向速度(m/s) */
    float    Min_VrelDepart;                    /* 最小传感器视线中去向的点云或目标径向速度(m/s) */
    float    Max_VrelDepart;                    /* 最大传感器视线中去向的点云或目标径向速度(m/s) */
    float    Min_RCS;                           /* 最小雷达散射截面强度(dBm^2) */
    float    Max_RCS;                           /* 最大雷达散射截面强度(dBm^2) */
    float    Min_Lifetime;                      /* 最小目标存在时间(自首次检测以来，单位：秒) */
    float    Max_Lifetime;                      /* 最大目标存在时间(自首次检测以来，单位：秒) */
    float    Min_Size;                          /* 最小物体尺寸面积(长x宽)，单位：m^2 */
    float    Max_Size;                          /* 最大物体尺寸面积(长x宽)，单位：m^2 */
    uint32_t Min_ProbExists;                    /* 最小存在的概率，即成为真实目标的概率，而不是由多径等引起的传感器假象 */
    uint32_t Max_ProbExists;                    /* 最大存在的概率，即成为真实目标的概率，而不是由多径等引起的传感器假象 */
    float    Min_Y;                             /* 最小横向距离(Y轴)，单位：m */
    float    Max_Y;                             /* 最大横向距离(Y轴)，单位：m */
    float    Min_X;                             /* 最小纵向距离(X轴)，单位：m */
    float    Max_X;                             /* 最大纵向距离(X轴)，单位：m */
    float    Min_VYLeftRight;                   /* 最小左右移动物体的横向速度分量(m/s)(所有左右移动物体) */
    float    Max_VYLeftRight;                   /* 最大左右移动物体的横向速度分量(m/s)(所有左右移动物体) */
    float    Min_VXOncome;                      /* 最小来向的物体的纵向速度分量(m/s)(所有来向的物体都可以) */
    float    Max_VXOncome;                      /* 最大来向的物体的纵向速度分量(m/s)(所有来向的物体都可以) */
    float    Min_VYRightLeft;                   /* 最小左右移动物体的横向速度分量(m/s)(所有左右移动物体) */
    float    Max_VYRightLeft;                   /* 最大左右移动物体的横向速度分量(m/s)(所有左右移动物体) */
    float    Min_VXDepart;                      /* 最小左右移动物体的横向速度分量(m/s))(所有左右移动物体) */
    float    Max_VXDepart;                      /* 最大左右移动物体的横向速度分量(m/s))(所有左右移动物体) */
} radar_filter_status_t;

/* 多边形过滤器状态结构体 */
typedef struct {
    bool     StoredInNvm;               /* 如果为真，表示发送的多边形过滤器配置已经在NVM钟存储 */
    uint32_t NumOfVertices;             /* 接收到的多边形过滤器顶点数量 */
    bool     LastCfgValid;              /* 如果为真，表示发送的多边形过滤器有效 */
    bool     FilterInUse;               /* 如果为真，表示多边形过滤器被使用 */
    bool     Active;                    /* 如果为真，表示多边形过滤器被激活 */
} radar_polygon_status_t;

/* 碰撞检测状态结构体 */
typedef struct {
    bool     Activation;                /* 碰撞检测状态激活 */
    uint32_t NofRegions;                /* 已配置区域的数量 */
    float    MinDetectTime;             /* 配置目标模式触发警告的最小检测时间 */
    uint32_t MeasCounter;               /* 测量循环计数器(自传感器启动以来计数，并在大于65535时重新启动) */
} radar_collision_status_t;

/* 碰撞检测区域状态结构体 */
typedef struct {
    uint32_t WarningLevel;              /* 表示某个对象位于该区域内，或曾经在区内 */
    uint32_t RegionID;                  /* 当前区域的ID编号 */
    float    Point1X;                   /* 第一个纵向矩形区域的坐标X */
    float    Point1Y;                   /* 第一个横向矩形区域的坐标Y */
    float    Point2X;                   /* 第二个纵向矩形区域的坐标X */
    float    Point2Y;                   /* 第二个横向矩形区域的坐标Y */
    uint32_t NofObjects;                /* 当前在区域内检测到的满足碰撞检测条件的目标数量 */
} radar_collision_region_status_t;

/* 雷达点云模式目标结构体 */
typedef struct {
    uint64_t timestamp;                 /* 当前帧的检测时间戳 */
    uint32_t ID;                        /* 点云编号 */
    float    DistLong;                  /* 纵向(X)坐标 */
    float    DistLat;                   /* 横向(Y)坐标 */
    float    VrelLong;                  /* 纵向相对速度(X) */
    float    VrelLat;                   /* 横向相对速度(Y) */
    uint32_t DynProp;                   /* 点云动态属性表示是否移动 */
    float    RCS;                       /* 雷达散射截面积 */
    uint32_t DistLong_rms;              /* 纵向距离的标准差(X) */
    uint32_t DistLat_rms;               /* 横向距离的标准差(Y) */
    uint32_t VrelLong_rms;              /* 纵向速度的标准差 */
    uint32_t VrelLat_rms;               /* 横向速度的标准差 */
    uint32_t Pdh0;                      /* 点云的误报概率 */
    uint32_t AmbigState;                /* 多普勒状态(径向速度)模糊度 */
    uint32_t InvalidState;              /* 点云有效状态 */
} radar_cluster_info_t;

/* 雷达目标模式目标结构体 */
typedef struct {
    uint64_t timestamp;                 /* 当前帧的检测时间戳 */
    uint32_t ID;                        /* 目标编号 */
    float    DistLong;                  /* 纵向(X)坐标 */
    float    DistLat;                   /* 横向(Y)坐标 */
    float    VrelLong;                  /* 纵向相对速度(X) */
    float    VrelLat;                   /* 横向相对速度(Y) */
    uint32_t DynProp;                   /* 点云动态属性表示是否移动 */
    float    RCS;                       /* 雷达散射截面积 */
    uint32_t DistLong_rms;              /* 纵向距离的标准差(X) */
    uint32_t DistLat_rms;               /* 横向距离的标准差(Y) */
    uint32_t VrelLong_rms;              /* 纵向速度的标准差 */
    uint32_t VrelLat_rms;               /* 横向速度的标准差 */
    uint32_t ArelLong_rms;              /* 纵向加速度的标准差 */
    uint32_t ArelLat_rms;               /* 横向加速度的标准差 */
    uint32_t Orientation_rms;           /* 方向角的标准差 */
    uint32_t MeasState;                 /* 检测状态指示目标是否有效并且已在新的测量周期钟由点云确认 */
    uint32_t ProbOfExist;               /* 目标存在的可能性 */
    float    ArelLong;                  /* 纵向相对加速度 */
    float    ArelLat;                   /* 横向相对加速度 */
    uint32_t Class;                     /* 0点，1小汽车，2卡车，3行人，4摩托车，5自行车，6宽广的，7保留 */
    float    OrientationAngel;          /* 目标的方向角 */
    float    Length;                    /* 被跟踪对象的长度 */
    float    Width;                     /* 被跟踪对象的宽度 */
    uint32_t CollDetRegionBitfield;     /* 区域字段，对于于此对象发生冲突的区域，将位设置1 */
} radar_object_info_t;

/* 软件版本结构体 */
typedef struct {
    uint32_t MajorRelease;              /* 软件主版本 */
    uint32_t MinorRelease;              /* 软件次版本 */
    uint32_t PatchLevel;                /* 软件补丁版本 */
    bool     ExtendedRange;             /* 是否支持范围限制扩展 */
    bool     CountryCode;               /* 当前区域是否支持-3dB功率输出 */
} radar_software_version_t;

/* 雷达基本配置状态结构体 */
typedef struct {
    bool     NVMReadStatus;             /* 启动时从非易失性存储器读取配置参数的状态 */
    bool     NVMwriteStatus;            /* 将配置参数存储到非易失性存储器中的状态(最初该值设置为0x0，在配置发送和成功后设置为0x1) */
    bool     Voltage_Error;             /* 如果工作电压低于或高于定义的范围超过5秒钟错误将被激活 */
    bool     Temporary_Error;           /* 在传感器复位后，可能会消失的临时错误 */
    bool     Temperature_Error;         /* 如果温度低于或高于定义的范围，错误将被激活 */
    bool     Interference;              /* 已探测到与另一个雷达传感器的干扰 */
    bool     Persistent_Error;          /* 内部错误，在检测到重置后可能不会消失 */
    uint32_t MaxDistanceCfg;            /* 最大距离的配置 */
    uint32_t SensorID;                  /* 传感器ID 0- 7 */
    uint32_t SortIndex;                 /* 当前目标列表排列索引方式(0不分类，1按范围分类，2按RCS分类) */
    uint32_t RadarPowerCfg;             /* 雷达功率参数的配置(0标准，1:-3dB，2:-6dB，3:-9dB) */
    bool     CtrlRelayCfg;              /* 如果为真，则发送继电器控制消息 */
    uint32_t OutputTypeCfg;             /* 当前数据类型为点云或目标模式(0无，1目标模式，2点云模式) */
    bool     SendQualityCfg;            /* 如果为真，则发送点云或目标的质量信息 */
    bool     SendExtInfoCfg;            /* 如果为真，则发送目标的扩展信息 */
    uint32_t MotionRxState;             /* 显示输入信号速度和横摆角速度的状态 */
    uint32_t RCS_Threshold;             /* 如果为真，则激活传感器的高灵敏度 */
    uint32_t InvalidClusters;           /* 显示为输出先择了哪些无效的点云 */
} radar_config_status_t;

static std::thread can_capture_tid;
static std::thread can_cluster_tid;
static std::thread can_objects_tid;

static radar_config_status_t mRadarConfigStatus;                                /* 雷达达基本配置状态 */
static radar_software_version_t mRadarSWVersion;                                /* 雷达基础的版本信息 */
static radar_filter_status_t mRadarObjectFilter;                                /* 目标过滤结果 */
static radar_filter_status_t mRadarClusterFilter;                               /* 点云过滤结果 */
static radar_polygon_status_t mRadarPolygonStatus;                              /* 多线检测结果 */
static radar_collision_status_t mRadarCollisionStatus;                          /* 碰撞检测结果 */
static radar_collision_region_status_t mRadarCollisionRgnStatus[8];             /* 碰撞区检测结果 */

static uint16_t mNumOfObject = 0;                                               /* 总的目标数量 */
static uint64_t mObjectTimestamp = 0;                                           /* 目标数据帧时间戳 */
static uint16_t mRadarObjectCounter = 0;                                        /* 目标目标接收计数器 */
static uint16_t mRadarObjectRecvState = OBJECT_RADAR_IDLE;                      /* 目标接收状态标志位 */

static uint16_t mNumOfClusters = 0;                                             /* 点云总的目标数量 */
static uint64_t mClustersTimestamp = 0;                                         /* 点云数据帧时间戳 */
static uint16_t mRadarClusterCounter = 0;                                       /* 点云目标接收计数器 */
static uint16_t mRadarClusterRecvState = CLUSTER_RADAR_IDLE;                    /* 点云接收状态标志位 */

std::mutex mCanFrameMutex;                                                      /* CAN数据互斥锁 */
std::condition_variable mCanFrameCond;                                          /* CAN数据条件变量 */
std::deque<can_frame_t> mCanFrameQueue;                                         /* CAN数据队列 */

static std::mutex mCanObjectMutex;                                              /* 目标数据互斥锁 */
static std::condition_variable mCanObjectCond;                                  /* 目标数据条件变量 */
static std::deque<std::vector<radar_object_info_t>> mCanObjectQueue;            /* 目标数据队列 */

static std::mutex mCanClusterMutex;                                             /* 点云数据互斥锁 */
static std::condition_variable mCanClusterCond;                                 /* 点云数据条件变量 */
static std::deque<std::vector<radar_cluster_info_t>> mCanClusterQueue;          /* 点云数据队列 */

static std::vector<radar_object_info_t> rawObjectLists;
static std::vector<radar_cluster_info_t> rawClusterLists;

void __attribute__((weak)) response_upper_computer_config(uint32_t can_id, uint8_t *data, uint8_t len)
{

}

void __attribute__((weak)) ars408_ojects_callback(std::vector<ars408_object_t> objects)
{

}
void __attribute__((weak)) ars408_cluster_callback(std::vector<ars408_cluster_t> cluster)
{

}

std::future<void> ProcessCanFrameAsync(can_frame_t frame)
{
    return std::async(std::launch::async, [frame = std::move(frame)]() {
        canid_t can_id = frame.frame.can_id;

        // if ((can_id == CAN_O_RADAR_STATE) || (can_id == CAN_O_FILTER_CFG) || (can_id == CAN_O_POLY_STATE)) {
            response_upper_computer_config((uint32_t)can_id, (uint8_t *)(frame.frame.data), (uint8_t)(frame.frame.can_dlc));
        // }

        if (can_id == CAN_O_RADAR_STATE) {
            uint32_t temp = 0;

            if ((frame.frame.data[0] & 0x40) == 0x40) {
                mRadarConfigStatus.NVMReadStatus = true;
            } else {
                mRadarConfigStatus.NVMReadStatus = false;
            }

            if ((frame.frame.data[0] & 0x80) == 0x80) {
                mRadarConfigStatus.NVMwriteStatus = true;
            } else {
                mRadarConfigStatus.NVMwriteStatus = false;
            }

            temp = ((uint32_t)frame.frame.data[1]) << 2;
            temp = temp | (((uint32_t)frame.frame.data[2]) >> 6);
            mRadarConfigStatus.MaxDistanceCfg = temp * 2;

            if ((frame.frame.data[2] & 0x20) == 0x20) {
                mRadarConfigStatus.Persistent_Error = true;
            } else {
                mRadarConfigStatus.Persistent_Error = false;
            }

            if ((frame.frame.data[2] & 0x10) == 0x10) {
                mRadarConfigStatus.Interference = true;
            } else {
                mRadarConfigStatus.Interference = false;
            }

            if ((frame.frame.data[2] & 0x08) == 0x08) {
                mRadarConfigStatus.Temperature_Error = true;
            } else {
                mRadarConfigStatus.Temperature_Error = false;
            }

            if ((frame.frame.data[2] & 0x04) == 0x04) {
                mRadarConfigStatus.Temporary_Error = true;
            } else {
                mRadarConfigStatus.Temporary_Error = false;
            }

            if ((frame.frame.data[2] & 0x02) == 0x02) {
                mRadarConfigStatus.Voltage_Error = true;
            } else {
                mRadarConfigStatus.Voltage_Error = false;
            }

            mRadarConfigStatus.SensorID = (uint32_t)frame.frame.data[4] & 0x07;
            mRadarConfigStatus.SortIndex = ((uint32_t)frame.frame.data[4] & 0x70) >> 4;

            temp = (uint32_t)((frame.frame.data[3] & 0x03) << 1);
            temp = temp | ((frame.frame.data[4] & 0x80) >> 7);
            mRadarConfigStatus.RadarPowerCfg = temp;

            if ((frame.frame.data[5] & 0x02) == 0x02) {
                mRadarConfigStatus.CtrlRelayCfg = true;
            } else {
                mRadarConfigStatus.CtrlRelayCfg = false;
            }

            mRadarConfigStatus.OutputTypeCfg = ((uint32_t)frame.frame.data[5] & 0x0C) >> 2;

            if ((frame.frame.data[5] & 0x10) == 0x10) {
                mRadarConfigStatus.SendQualityCfg = true;
            } else {
                mRadarConfigStatus.SendQualityCfg = false;
            }

            if ((frame.frame.data[5] & 0x20) == 0x20) {
                mRadarConfigStatus.SendExtInfoCfg = true;
            } else {
                mRadarConfigStatus.SendExtInfoCfg = false;
            }

            mRadarConfigStatus.MotionRxState = ((uint32_t)frame.frame.data[5] & 0xC0) >> 6;
            mRadarConfigStatus.RCS_Threshold = ((uint32_t)frame.frame.data[7] & 0x1C) >> 2;
            mRadarConfigStatus.InvalidClusters = (uint32_t)frame.frame.data[6];
        } else if (can_id == CAN_O_FILTER_STATE_HEAD) {
            mRadarObjectFilter.NofObjectFilterCfg = (uint32_t)((frame.frame.data[1] & 0xF8) >> 3);
            mRadarClusterFilter.NofClusterFilterCfg = (uint32_t)((frame.frame.data[0] & 0xF8) >> 3);
        } else if (can_id == CAN_O_FILTER_CFG) {
            bool dataType = (frame.frame.data[0] & 0x80) == 0x80;
            uint32_t filterIndex = (frame.frame.data[0] & 0b1111000) >> 3;

            if (dataType) {     /* 目标模式 */
                switch (filterIndex) {
                    case FILTER_NofObj: {
                        mRadarObjectFilter.NofObj_Active = ((frame.frame.data[0] & 0x04) == 0x04);
                        mRadarObjectFilter.Min_NofObj    = (((uint32_t)frame.frame.data[1] & 0b1111) << 8) + (uint32_t)frame.frame.data[2];
                        mRadarObjectFilter.Max_NofObj    = (((uint32_t)frame.frame.data[3] & 0b1111) << 8) + (uint32_t)frame.frame.data[4];
                    }
                    break;

                    case FILTER_Distance: {
                        mRadarObjectFilter.Distance_Active = ((frame.frame.data[0] & 0x04) == 0x04);
                        mRadarObjectFilter.Min_Distance    = ((((uint32_t)frame.frame.data[1] & 0b1111) << 8) + (uint32_t)frame.frame.data[2]) * 0.1;
                        mRadarObjectFilter.Max_Distance    = ((((uint32_t)frame.frame.data[3] & 0b1111) << 8) + (uint32_t)frame.frame.data[4]) * 0.1;
                    }
                    break;

                    case FILTER_Azimuth: {
                        mRadarObjectFilter.Azimuth_Active = ((frame.frame.data[0] & 0x04) == 0x04);
                        mRadarObjectFilter.Min_Azimuth    = ((((uint32_t)frame.frame.data[1] & 0b1111) << 8) + (uint32_t)frame.frame.data[2]) * 0.025 - 50.000;
                        mRadarObjectFilter.Max_Azimuth    = ((((uint32_t)frame.frame.data[3] & 0b1111) << 8) + (uint32_t)frame.frame.data[4]) * 0.025 - 50.000;
                    }
                    break;

                    case FILTER_VrelOncome: {
                        mRadarObjectFilter.VrelOncome_Active = ((frame.frame.data[0] & 0x04) == 0x04);
                        mRadarObjectFilter.Min_VrelOncome    = ((((uint32_t)frame.frame.data[1] & 0b1111) << 8) + (uint32_t)frame.frame.data[2]) * 0.0315;
                        mRadarObjectFilter.Max_VrelOncome    = ((((uint32_t)frame.frame.data[3] & 0b1111) << 8) + (uint32_t)frame.frame.data[4]) * 0.0315;
                    }
                    break;

                    case FILTER_VrelDepart: {
                        mRadarObjectFilter.VrelDepart_Active = ((frame.frame.data[0] & 0x04) == 0x04);
                        mRadarObjectFilter.Min_VrelDepart    = ((((uint32_t)frame.frame.data[1] & 0b1111) << 8) + (uint32_t)frame.frame.data[2]) * 0.0315;
                        mRadarObjectFilter.Max_VrelDepart    = ((((uint32_t)frame.frame.data[3] & 0b1111) << 8) + (uint32_t)frame.frame.data[4]) * 0.0315;
                    }
                    break;

                    case FILTER_RCS: {
                        mRadarObjectFilter.RCS_Active = ((frame.frame.data[0] & 0x04) == 0x04);
                        mRadarObjectFilter.Min_RCS    = ((((uint32_t)frame.frame.data[1] & 0b1111) << 8) + (uint32_t)frame.frame.data[2]) * 0.025 - 50.000;
                        mRadarObjectFilter.Max_RCS    = ((((uint32_t)frame.frame.data[3] & 0b1111) << 8) + (uint32_t)frame.frame.data[4]) * 0.025 - 50.000;
                    }
                    break;

                    case FILTER_Lifetime: {
                        mRadarObjectFilter.Lifetime_Active = ((frame.frame.data[0] & 0x04) == 0x04);
                        mRadarObjectFilter.Min_Lifetime    = ((((uint32_t)frame.frame.data[1] & 0b1111) << 8) + (uint32_t)frame.frame.data[2]) * 0.1;
                        mRadarObjectFilter.Max_Lifetime    = ((((uint32_t)frame.frame.data[3] & 0b1111) << 8) + (uint32_t)frame.frame.data[4]) * 0.1;
                    }
                    break;

                    case FILTER_Size: {
                        mRadarObjectFilter.Size_Active = ((frame.frame.data[0] & 0x04) == 0x04);
                        mRadarObjectFilter.Min_Size    = ((((uint32_t)frame.frame.data[1] & 0b1111) << 8) + (uint32_t)frame.frame.data[2]) * 0.025;
                        mRadarObjectFilter.Max_Size    = ((((uint32_t)frame.frame.data[3] & 0b1111 )<< 8) + (uint32_t)frame.frame.data[4]) * 0.025;
                    }
                    break;

                    case FILTER_ProbExists: {
                        mRadarObjectFilter.ProbExists_Active = ((frame.frame.data[0] & 0x04) == 0x04);
                        mRadarObjectFilter.Min_ProbExists    = (((uint32_t)frame.frame.data[1] & 0b1111) << 8) + (uint32_t)frame.frame.data[2];
                        mRadarObjectFilter.Max_ProbExists    = (((uint32_t)frame.frame.data[3] & 0b1111) << 8) + (uint32_t)frame.frame.data[4];
                    }
                    break;

                    case FILTER_Y: {
                        mRadarObjectFilter.Y_Active = ((frame.frame.data[0] & 0x04) == 0x04);
                        mRadarObjectFilter.Min_Y    = ((((uint32_t)frame.frame.data[1] & 0b1111) << 8) + (uint32_t)frame.frame.data[2]) * 0.2 - 409.5;
                        mRadarObjectFilter.Max_Y    = ((((uint32_t)frame.frame.data[3] & 0b1111) << 8) + (uint32_t)frame.frame.data[4]) * 0.2 - 409.5;
                    }
                    break;

                    case FILTER_X: {
                        mRadarObjectFilter.X_Active = ((frame.frame.data[0] & 0x04) == 0x04);
                        mRadarObjectFilter.Min_X    = ((((uint32_t)frame.frame.data[1] & 0b11111) << 8) + (uint32_t)frame.frame.data[2]) * 0.2 - 500.0;
                        mRadarObjectFilter.Max_X    = ((((uint32_t)frame.frame.data[3] & 0b11111) << 8) + (uint32_t)frame.frame.data[4]) * 0.2 - 500.0;
                    }
                    break;

                    case FILTER_VYRightLeft: {
                        mRadarObjectFilter.VYRightLeft_Active = ((frame.frame.data[0] & 0x04) == 0x04);
                        mRadarObjectFilter.Min_VYRightLeft    = ((((uint32_t)frame.frame.data[1] & 0b1111) << 8) + (uint32_t)frame.frame.data[2]) * 0.0315;
                        mRadarObjectFilter.Max_VYRightLeft    = ((((uint32_t)frame.frame.data[3] & 0b1111) << 8) + (uint32_t)frame.frame.data[4]) * 0.0315;
                    }
                    break;

                    case FILTER_VXOncome: {
                        mRadarObjectFilter.VXOncome_Active = ((frame.frame.data[0] & 0x04) == 0x04);
                        mRadarObjectFilter.Min_VXOncome    = ((((uint32_t)frame.frame.data[1] & 0b1111) << 8) + (uint32_t)frame.frame.data[2]) * 0.0315;
                        mRadarObjectFilter.Max_VXOncome    = ((((uint32_t)frame.frame.data[3] & 0b1111) << 8) + (uint32_t)frame.frame.data[4]) * 0.0315;
                    }
                    break;

                    case FILTER_VYLeftRight: {
                        mRadarObjectFilter.VYLeftRight_Active = ((frame.frame.data[0] & 0x04) == 0x04);
                        mRadarObjectFilter.Min_VYLeftRight    = ((((uint32_t)frame.frame.data[1] & 0b1111) << 8) + (uint32_t)frame.frame.data[2]) * 0.0315;
                        mRadarObjectFilter.Max_VYLeftRight    = ((((uint32_t)frame.frame.data[3] & 0b1111) << 8) + (uint32_t)frame.frame.data[4]) * 0.0315;
                    }
                    break;

                    case FILTER_VXDepart: {
                        mRadarObjectFilter.VXDepart_Active = ((frame.frame.data[0] & 0x04) == 0x04);
                        mRadarObjectFilter.Min_VXDepart    = ((((uint32_t)frame.frame.data[1] & 0b1111) << 8) + (uint32_t)frame.frame.data[2]) * 0.0315;
                        mRadarObjectFilter.Max_VXDepart    = ((((uint32_t)frame.frame.data[3] & 0b1111) << 8) + (uint32_t)frame.frame.data[4]) * 0.0315;
                    }
                    break;

                    default:
                        break;
                }
            } else {            /* 点云模式 */
                switch (filterIndex) {
                    case FILTER_NofObj: {
                        mRadarClusterFilter.NofObj_Active = ((frame.frame.data[0] & 0x04) == 0x04);
                        mRadarClusterFilter.Min_NofObj    = (((uint32_t)frame.frame.data[1] & 0b1111) << 8) + (uint32_t)frame.frame.data[2];
                        mRadarClusterFilter.Max_NofObj    = (((uint32_t)frame.frame.data[3] & 0b1111) << 8) + (uint32_t)frame.frame.data[4];
                    }
                    break;

                    case FILTER_Distance: {
                        mRadarClusterFilter.Distance_Active = ((frame.frame.data[0] & 0x04) == 0x04);
                        mRadarClusterFilter.Min_Distance    = ((((uint32_t)frame.frame.data[1] & 0b1111) << 8) + (uint32_t)frame.frame.data[2]) * 0.1;
                        mRadarClusterFilter.Max_Distance    = ((((uint32_t)frame.frame.data[3] & 0b1111) << 8) + (uint32_t)frame.frame.data[4]) * 0.1;
                    }
                    break;

                    case FILTER_Azimuth: {
                        mRadarClusterFilter.Azimuth_Active = ((frame.frame.data[0] & 0x04) == 0x04);
                        mRadarClusterFilter.Min_Azimuth    = ((((uint32_t)frame.frame.data[1] & 0b1111) << 8) + (uint32_t)frame.frame.data[2]) * 0.025 - 50.000;
                        mRadarClusterFilter.Max_Azimuth    = ((((uint32_t)frame.frame.data[3] & 0b1111) << 8) + (uint32_t)frame.frame.data[4]) * 0.025 - 50.000;
                    }
                    break;

                    case FILTER_VrelOncome: {
                        mRadarClusterFilter.VrelOncome_Active = ((frame.frame.data[0] & 0x04) == 0x04);
                        mRadarClusterFilter.Min_VrelOncome    = ((((uint32_t)frame.frame.data[1] & 0b1111) << 8) + (uint32_t)frame.frame.data[2]) * 0.0315;
                        mRadarClusterFilter.Max_VrelOncome    = ((((uint32_t)frame.frame.data[3] & 0b1111) << 8) + (uint32_t)frame.frame.data[4]) * 0.0315;
                    }
                    break;

                    case FILTER_VrelDepart: {
                        mRadarClusterFilter.VrelDepart_Active = ((frame.frame.data[0] & 0x04) == 0x04);
                        mRadarClusterFilter.Min_VrelDepart    = ((((uint32_t)frame.frame.data[1] & 0b1111) << 8) + (uint32_t)frame.frame.data[2]) * 0.0315;
                        mRadarClusterFilter.Max_VrelDepart    = ((((uint32_t)frame.frame.data[3] & 0b1111) << 8) + (uint32_t)frame.frame.data[4]) * 0.0315;
                    }
                    break;

                    case FILTER_RCS: {
                        mRadarClusterFilter.RCS_Active = ((frame.frame.data[0] & 0x04) == 0x04);
                        mRadarClusterFilter.Min_RCS    = ((((uint32_t)frame.frame.data[1] & 0b1111) << 8) + (uint32_t)frame.frame.data[2]) * 0.025 - 50.000;
                        mRadarClusterFilter.Max_RCS    = ((((uint32_t)frame.frame.data[3] & 0b1111) << 8) + (uint32_t)frame.frame.data[4]) * 0.025 - 50.000;
                    }
                    break;

                    case FILTER_Lifetime: {
                        mRadarClusterFilter.Lifetime_Active = ((frame.frame.data[0] & 0x04) == 0x04);
                        mRadarClusterFilter.Min_Lifetime    = ((((uint32_t)frame.frame.data[1] & 0b1111) << 8) + (uint32_t)frame.frame.data[2]) * 0.1;
                        mRadarClusterFilter.Max_Lifetime    = ((((uint32_t)frame.frame.data[3] & 0b1111) << 8) + (uint32_t)frame.frame.data[4]) * 0.1;
                    }
                    break;

                    case FILTER_Size: {
                        mRadarClusterFilter.Size_Active = ((frame.frame.data[0] & 0x04) == 0x04);
                        mRadarClusterFilter.Min_Size    = ((((uint32_t)frame.frame.data[1] & 0b1111) << 8) + (uint32_t)frame.frame.data[2]) * 0.025;
                        mRadarClusterFilter.Max_Size    = ((((uint32_t)frame.frame.data[3] & 0b1111) << 8) + (uint32_t)frame.frame.data[4]) * 0.025;
                    }
                    break;

                    case FILTER_ProbExists: {
                        mRadarClusterFilter.ProbExists_Active = ((frame.frame.data[0] & 0x04) == 0x04);
                        mRadarClusterFilter.Min_ProbExists    = (((uint32_t)frame.frame.data[1] & 0b1111) << 8) + (uint32_t)frame.frame.data[2];
                        mRadarClusterFilter.Max_ProbExists    = (((uint32_t)frame.frame.data[3] & 0b1111) << 8) + (uint32_t)frame.frame.data[4];
                    }
                    break;

                    case FILTER_Y: {
                        mRadarClusterFilter.Y_Active = ((frame.frame.data[0] & 0x04) == 0x04);
                        mRadarClusterFilter.Min_Y    = ((((uint32_t)frame.frame.data[1] & 0b1111) << 8) + (uint32_t)frame.frame.data[2]) * 0.2 - 409.5;
                        mRadarClusterFilter.Max_Y    = ((((uint32_t)frame.frame.data[3] & 0b1111) << 8) + (uint32_t)frame.frame.data[4]) * 0.2 - 409.5;
                    }
                    break;

                    case FILTER_X: {
                        mRadarClusterFilter.X_Active = ((frame.frame.data[0] & 0x04) == 0x04);
                        mRadarClusterFilter.Min_X    = ((((uint32_t)frame.frame.data[1] & 0b11111) << 8) + (uint32_t)frame.frame.data[2]) * 0.2 - 500.0;
                        mRadarClusterFilter.Max_X    = ((((uint32_t)frame.frame.data[3] & 0b11111) << 8) + (uint32_t)frame.frame.data[4]) * 0.2 - 500.0;
                    }
                    break;

                    case FILTER_VYRightLeft: {
                        mRadarClusterFilter.VYRightLeft_Active = ((frame.frame.data[0] & 0x04) == 0x04);
                        mRadarClusterFilter.Min_VYRightLeft    = ((((uint32_t)frame.frame.data[1] & 0b1111) << 8) + (uint32_t)frame.frame.data[2]) * 0.0315;
                        mRadarClusterFilter.Max_VYRightLeft    = ((((uint32_t)frame.frame.data[3] & 0b1111) << 8) + (uint32_t)frame.frame.data[4]) * 0.0315;
                    }
                    break;

                    case FILTER_VXOncome: {
                        mRadarClusterFilter.VXOncome_Active = ((frame.frame.data[0] & 0x04) == 0x04);
                        mRadarClusterFilter.Min_VXOncome    = ((((uint32_t)frame.frame.data[1] & 0b1111) << 8) + (uint32_t)frame.frame.data[2]) * 0.0315;
                        mRadarClusterFilter.Max_VXOncome    = ((((uint32_t)frame.frame.data[3] & 0b1111) << 8) + (uint32_t)frame.frame.data[4]) * 0.0315;
                    }
                    break;

                    case FILTER_VYLeftRight: {
                        mRadarClusterFilter.VYLeftRight_Active = ((frame.frame.data[0] & 0x04) == 0x04);
                        mRadarClusterFilter.Min_VYLeftRight    = ((((uint32_t)frame.frame.data[1] & 0b1111) << 8) + (uint32_t)frame.frame.data[2]) * 0.0315;
                        mRadarClusterFilter.Max_VYLeftRight    = ((((uint32_t)frame.frame.data[3] & 0b1111) << 8) + (uint32_t)frame.frame.data[4]) * 0.0315;
                    }
                    break;

                    case FILTER_VXDepart:
                        mRadarClusterFilter.VXDepart_Active = ((frame.frame.data[0] & 0x04) == 0x04);
                        mRadarClusterFilter.Min_VXDepart    = ((((uint32_t)frame.frame.data[1] & 0b1111) << 8) + (uint32_t)frame.frame.data[2]) * 0.0315;
                        mRadarClusterFilter.Max_VXDepart    = ((((uint32_t)frame.frame.data[3] & 0b1111) << 8) + (uint32_t)frame.frame.data[4]) * 0.0315;
                    break;

                    default:
                        break;
                }
            }
        } else if (can_id == CAN_O_POLY_STATE) {
            mRadarPolygonStatus.StoredInNvm   = ((frame.frame.data[0] & 0x01) == 0x01);
            mRadarPolygonStatus.NumOfVertices = ((uint32_t)frame.frame.data[0] & 0b11110) >> 1;
            mRadarPolygonStatus.LastCfgValid  = ((frame.frame.data[0] & 0x20) == 0x20);
            mRadarPolygonStatus.FilterInUse   = ((frame.frame.data[0] & 0x40) == 0x40);
            mRadarPolygonStatus.Active        = ((frame.frame.data[0] & 0x80) == 0x80);
        } else if (can_id == CAN_O_COLLDET_RGN_STATE) {
            uint8_t id = frame.frame.data[0] >> 5;
            if (id < 8) {
                mRadarCollisionRgnStatus[id].RegionID     = id;
                mRadarCollisionRgnStatus[id].WarningLevel = ((uint32_t)frame.frame.data[0] & 0b11000) >> 3;
                mRadarCollisionRgnStatus[id].Point1X      = (((uint32_t)frame.frame.data[1] << 5) + ((uint32_t)frame.frame.data[2] >> 3)) * 0.2 - 500.0;
                mRadarCollisionRgnStatus[id].Point1Y      = ((((uint32_t)frame.frame.data[2] & 0b111) << 8) + (uint32_t)frame.frame.data[3]) * 0.2 - 204.6;
                mRadarCollisionRgnStatus[id].Point2X      = (((uint32_t)frame.frame.data[4] << 5) + ((uint32_t)frame.frame.data[5] >> 3)) * 0.2 - 500.0;
                mRadarCollisionRgnStatus[id].Point2Y      = ((((uint32_t)frame.frame.data[5] & 0b111) << 8) + (uint32_t)frame.frame.data[6]) * 0.2 - 204.6;
                mRadarCollisionRgnStatus[id].NofObjects   = (uint32_t)frame.frame.data[7];
            }
        } else if (can_id == CAN_O_COLLDET_STATE) {
            mRadarCollisionStatus.Activation    = ((frame.frame.data[0] & 0x02) == 0x02);
            mRadarCollisionStatus.NofRegions    = (uint32_t)frame.frame.data[0] > 4;
            mRadarCollisionStatus.MinDetectTime = (uint32_t)frame.frame.data[1] * 0.1;
            mRadarCollisionStatus.MeasCounter   = (uint32_t)(frame.frame.data[2] << 8) | (frame.frame.data[3]);
        } else if (can_id == CAN_O_VERSION_ID) {
            mRadarSWVersion.MajorRelease = (uint32_t)frame.frame.data[0];
            mRadarSWVersion.MinorRelease = (uint32_t)frame.frame.data[1];
            mRadarSWVersion.PatchLevel   = (uint32_t)frame.frame.data[2];

            if ((frame.frame.data[3] & 0x02) == 0x02) {
                mRadarSWVersion.ExtendedRange = true;
            } else {
                mRadarSWVersion.ExtendedRange = false;
            }

            if ((frame.frame.data[3] & 0x01) == 0x01) {
                mRadarSWVersion.CountryCode = true;
            } else {
                mRadarSWVersion.CountryCode = false;
            }
        } else if ((can_id >= CAN_O_OBJECT_0_STATUS) && (can_id <= CAN_O_OBJECT_4_WARNING)) {
            switch (mRadarObjectRecvState) {
                case OBJECT_RADAR_IDLE: {
                    if (can_id == CAN_O_OBJECT_0_STATUS) {
                        mRadarObjectCounter = 0;
                        mObjectTimestamp = frame.ts;
                        mNumOfObject = frame.frame.data[0];
                        if (mNumOfObject > MAX_RADAR_OBJECTS) {
                            mNumOfObject = MAX_RADAR_OBJECTS;
                        }

                        mRadarObjectRecvState = mNumOfObject == 0 ? OBJECT_RADAR_IDLE : OBJECT_RADAR_GET_GENERAL_INFO;
                    }
                }
                break;

                case OBJECT_RADAR_GET_GENERAL_INFO: {
                    if (can_id == CAN_O_OBJECT_0_STATUS) {
                        mRadarObjectCounter = 0;
                        mObjectTimestamp = frame.ts;
                        mNumOfObject = frame.frame.data[0];
                        if (mNumOfObject > MAX_RADAR_OBJECTS) {
                            mNumOfObject = MAX_RADAR_OBJECTS;
                        }

                        mRadarObjectRecvState = mNumOfObject == 0 ? OBJECT_RADAR_IDLE : OBJECT_RADAR_GET_GENERAL_INFO;
                    } else if (can_id == CAN_O_OBJECT_1_GENERAL) {
                        radar_object_info_t obj_temp;

                        memset(&obj_temp, 0, sizeof(obj_temp));
                        obj_temp.timestamp = mObjectTimestamp;
                        obj_temp.ID        = (uint32_t)frame.frame.data[0];
                        obj_temp.DistLong  = (((uint32_t)frame.frame.data[1] << 5) + ((uint32_t)frame.frame.data[2] >> 3)) * 0.2 - 500.0;
                        obj_temp.DistLat   = ((((uint32_t)frame.frame.data[2] & 0b111) << 8) + (uint32_t)frame.frame.data[3]) * 0.2 - 204.6;
                        obj_temp.VrelLong  = (((uint32_t)frame.frame.data[4] << 2) + ((uint32_t)frame.frame.data[5] >> 6)) * 0.25 - 128.00;
                        obj_temp.VrelLat   = ((((uint32_t)frame.frame.data[5] & 0b111111) << 3) + ((uint32_t)frame.frame.data[6] >> 5)) * 0.25 - 64.00;
                        obj_temp.DynProp   = (uint32_t)frame.frame.data[6] & 0b111;
                        obj_temp.RCS       = (uint32_t)frame.frame.data[7] * 0.5 - 64.00;
                        rawObjectLists.push_back(std::move(obj_temp));

                        mRadarObjectCounter += 1;
                        if (mRadarObjectCounter >= mNumOfObject) {
                            mRadarObjectCounter = 0;
                            if (mRadarConfigStatus.SendQualityCfg) {
                                mRadarObjectRecvState = OBJECT_RADAR_GET_QUALITY_INFO;
                            } else if (mRadarConfigStatus.SendExtInfoCfg) {
                                mRadarObjectRecvState = OBJECT_RADAR_GET_EXTENDED_INFO;
                            } else if (mRadarCollisionStatus.Activation) {
                                mRadarObjectRecvState = OBJECT_RADAR_GET_COLLISION_INFO;
                            } else {
                                mRadarObjectRecvState = OBJECT_RADAR_IDLE;
                                {
                                    std::unique_lock<std::mutex> lock(mCanObjectMutex);
                                    std::vector<radar_object_info_t> temp(rawObjectLists);
                                    mCanObjectQueue.push_back(std::move(temp));
                                    mCanObjectCond.notify_one();
                                    rawObjectLists.clear();
                                }
                            }
                        }
                    }
                }
                break;

                case OBJECT_RADAR_GET_QUALITY_INFO: {
                    if (can_id == CAN_O_OBJECT_0_STATUS) {
                        mRadarObjectCounter = 0;
                        mObjectTimestamp = frame.ts;
                        mNumOfObject = frame.frame.data[0];
                        if (mNumOfObject > MAX_RADAR_OBJECTS) {
                            mNumOfObject = MAX_RADAR_OBJECTS;
                        }

                        mRadarObjectRecvState = mNumOfObject == 0 ? OBJECT_RADAR_IDLE : OBJECT_RADAR_GET_GENERAL_INFO;
                    } else if (can_id == CAN_O_OBJECT_2_QUALITY) {
                        radar_object_info_t obj_temp;

                        memset(&obj_temp, 0, sizeof(obj_temp));
                        obj_temp.DistLong_rms    = ((uint32_t)frame.frame.data[1] >> 3);
                        obj_temp.DistLat_rms     = (((uint32_t)frame.frame.data[1] & 0b111) << 2) + ((uint32_t)frame.frame.data[2] >> 6);
                        obj_temp.VrelLong_rms    = ((uint32_t)frame.frame.data[2] & 0b00111110)>>1;
                        obj_temp.VrelLat_rms     = (((uint32_t)frame.frame.data[2] & 0b1) << 4) + ((uint32_t)frame.frame.data[3] >> 4);
                        obj_temp.ArelLong_rms    = (((uint32_t)frame.frame.data[3] & 0b1111) << 1) + ((uint32_t)frame.frame.data[4] >> 7);
                        obj_temp.ArelLat_rms     = ((uint32_t)frame.frame.data[4] & 0b01111100) >> 2;
                        obj_temp.Orientation_rms = (((uint32_t)frame.frame.data[4] & 0b11) << 3) + ((uint32_t)frame.frame.data[5] >> 5);
                        obj_temp.MeasState       = ((uint32_t)frame.frame.data[6] & 0b11100) >> 2;
                        obj_temp.ProbOfExist     = ((uint32_t)frame.frame.data[6] >> 5);

                        if (mRadarObjectCounter < rawObjectLists.size()) {
                            rawObjectLists.at(mRadarObjectCounter).DistLong_rms    = obj_temp.DistLong_rms;
                            rawObjectLists.at(mRadarObjectCounter).DistLat_rms     = obj_temp.DistLat_rms;
                            rawObjectLists.at(mRadarObjectCounter).VrelLong_rms    = obj_temp.VrelLong_rms;
                            rawObjectLists.at(mRadarObjectCounter).VrelLat_rms     = obj_temp.VrelLat_rms;
                            rawObjectLists.at(mRadarObjectCounter).ArelLong_rms    = obj_temp.ArelLong_rms;
                            rawObjectLists.at(mRadarObjectCounter).ArelLat_rms     = obj_temp.ArelLat_rms;
                            rawObjectLists.at(mRadarObjectCounter).Orientation_rms = obj_temp.Orientation_rms;
                            rawObjectLists.at(mRadarObjectCounter).MeasState       = obj_temp.MeasState;
                            rawObjectLists.at(mRadarObjectCounter).ProbOfExist     = obj_temp.ProbOfExist;
                        }

                        mRadarObjectCounter += 1;
                        if (mRadarObjectCounter >= mNumOfObject) {
                            mRadarObjectCounter = 0;
                            if (mRadarConfigStatus.SendExtInfoCfg) {
                                mRadarObjectRecvState = OBJECT_RADAR_GET_EXTENDED_INFO;
                            } else if (mRadarCollisionStatus.Activation) {
                                mRadarObjectRecvState = OBJECT_RADAR_GET_COLLISION_INFO;
                            } else {
                                mRadarObjectRecvState = OBJECT_RADAR_IDLE;
                                {
                                    std::unique_lock<std::mutex> lock(mCanObjectMutex);
                                    std::vector<radar_object_info_t> temp(rawObjectLists);
                                    mCanObjectQueue.push_back(std::move(temp));
                                    mCanObjectCond.notify_one();
                                    rawObjectLists.clear();
                                }
                            }
                        }
                    }
                }
                break;

                case OBJECT_RADAR_GET_EXTENDED_INFO: {
                    if (can_id == CAN_O_OBJECT_0_STATUS) {
                        mRadarObjectCounter = 0;
                        mObjectTimestamp = frame.ts;
                        mNumOfObject = frame.frame.data[0];
                        if (mNumOfObject > MAX_RADAR_OBJECTS) {
                            mNumOfObject = MAX_RADAR_OBJECTS;
                        }

                        mRadarObjectRecvState = mNumOfObject == 0 ? OBJECT_RADAR_IDLE : OBJECT_RADAR_GET_GENERAL_INFO;
                    } else if (can_id == CAN_O_OBJECT_3_EXTENDED) {
                        radar_object_info_t obj_temp;

                        memset(&obj_temp, 0, sizeof(obj_temp));
                        obj_temp.ArelLong         = (((uint32_t)frame.frame.data[1] << 3) + ((uint32_t)frame.frame.data[2] >> 5)) * 0.01 - 10.00;
                        obj_temp.ArelLat          = ((((uint32_t)frame.frame.data[2] & 0b11111) << 4) + ((uint32_t)frame.frame.data[3] >> 4)) * 0.01 - 2.50;
                        obj_temp.Class            = ((uint32_t)frame.frame.data[3] & 0b111);
                        obj_temp.OrientationAngel = (((uint32_t)frame.frame.data[4] << 2) + ((uint32_t)frame.frame.data[5] >> 6)) * 0.4 - 180.0;
                        obj_temp.Length           = frame.frame.data[6] * 0.2;
                        obj_temp.Width            = frame.frame.data[7] * 0.2;

                        if (mRadarObjectCounter < rawObjectLists.size()) {
                            rawObjectLists.at(mRadarObjectCounter).ArelLong         = obj_temp.ArelLong;
                            rawObjectLists.at(mRadarObjectCounter).ArelLat          = obj_temp.ArelLat;
                            rawObjectLists.at(mRadarObjectCounter).Class            = obj_temp.Class;
                            rawObjectLists.at(mRadarObjectCounter).OrientationAngel = obj_temp.OrientationAngel;
                            rawObjectLists.at(mRadarObjectCounter).Length           = obj_temp.Length;
                            rawObjectLists.at(mRadarObjectCounter).Width            = obj_temp.Width;
                        }

                        mRadarObjectCounter += 1;
                        if (mRadarObjectCounter >= mNumOfObject) {
                            mRadarObjectCounter = 0;
                            if (mRadarCollisionStatus.Activation) {
                                mRadarObjectRecvState = OBJECT_RADAR_GET_COLLISION_INFO;
                            } else {
                                mRadarObjectRecvState = OBJECT_RADAR_IDLE;
                                {
                                    std::unique_lock<std::mutex> lock(mCanObjectMutex);
                                    std::vector<radar_object_info_t> temp(rawObjectLists);
                                    mCanObjectQueue.push_back(std::move(temp));
                                    mCanObjectCond.notify_one();
                                    rawObjectLists.clear();
                                }
                            }
                        }
                    }
                }
                break;

                case OBJECT_RADAR_GET_COLLISION_INFO: {
                    if (can_id == CAN_O_OBJECT_0_STATUS) {
                        mRadarObjectCounter = 0;
                        mObjectTimestamp = frame.ts;
                        mNumOfObject = frame.frame.data[0];
                        if (mNumOfObject > MAX_RADAR_OBJECTS) {
                            mNumOfObject = MAX_RADAR_OBJECTS;
                        }

                        mRadarObjectRecvState = mNumOfObject == 0 ? OBJECT_RADAR_IDLE : OBJECT_RADAR_GET_GENERAL_INFO;
                    } else if (can_id == CAN_O_OBJECT_4_WARNING) {
                        if (mRadarObjectCounter < rawObjectLists.size()) {
                            rawObjectLists.at(mRadarObjectCounter).CollDetRegionBitfield = (uint32_t)frame.frame.data[1];
                        }

                        mRadarObjectCounter += 1;
                        if (mRadarObjectCounter >= mNumOfObject) {
                            mRadarObjectCounter = 0;
                            mRadarObjectRecvState = OBJECT_RADAR_IDLE;
                            {
                                std::unique_lock<std::mutex> lock(mCanObjectMutex);
                                std::vector<radar_object_info_t> temp(rawObjectLists);
                                mCanObjectQueue.push_back(std::move(temp));
                                mCanObjectCond.notify_one();
                                rawObjectLists.clear();
                            }
                        }
                    }
                }
                break;

                default: {
                    mRadarObjectRecvState = OBJECT_RADAR_IDLE;
                }
                break;
            }
        } else if ((can_id == CAN_O_CLUSTER_0_STATUS) || (can_id == CAN_O_CLUSTER_1_GENERAL) || (can_id == CAN_O_CLUSTER_2_QUALITY)) {
            switch (mRadarClusterRecvState) {
                case CLUSTER_RADAR_IDLE: {
                    if (can_id == CAN_O_CLUSTER_0_STATUS) {
                        mRadarClusterCounter = 0;
                        mClustersTimestamp = frame.ts;
                        mNumOfClusters = frame.frame.data[0] + frame.frame.data[1];
                        if (mNumOfClusters > MAX_RADAR_CLUTSERS) {
                            mNumOfClusters = MAX_RADAR_CLUTSERS;
                        }

                        mRadarClusterRecvState = mNumOfClusters == 0 ? CLUSTER_RADAR_IDLE : CLUSTER_RADAR_GET_GENERAL_INFO;
                    }
                }
                break;

                case CLUSTER_RADAR_GET_GENERAL_INFO: {
                    if (can_id == CAN_O_CLUSTER_0_STATUS) {
                        mRadarClusterCounter = 0;
                        mClustersTimestamp = frame.ts;
                        mNumOfClusters = frame.frame.data[0] + frame.frame.data[1];
                        if (mNumOfClusters > MAX_RADAR_CLUTSERS) {
                            mNumOfClusters = MAX_RADAR_CLUTSERS;
                        }

                        mRadarClusterRecvState = mNumOfClusters == 0 ? CLUSTER_RADAR_IDLE : CLUSTER_RADAR_GET_GENERAL_INFO;
                    } else if (can_id == CAN_O_CLUSTER_1_GENERAL) {
                        radar_cluster_info_t cluster_temp;

                        memset(&cluster_temp, 0, sizeof(cluster_temp));
                        cluster_temp.timestamp = mClustersTimestamp;
                        cluster_temp.ID        = (uint32_t)frame.frame.data[0];
                        cluster_temp.DistLong  = (((uint32_t)frame.frame.data[1] << 5) + ((uint32_t)frame.frame.data[2] >> 3)) * 0.2 - 500.0;
                        cluster_temp.DistLat   = ((((uint32_t)frame.frame.data[2] & 0b11) << 8) + (uint32_t)frame.frame.data[3]) * 0.2 - 102.3;
                        cluster_temp.VrelLong  = (((uint32_t)frame.frame.data[4] << 2) + ((uint32_t)frame.frame.data[5] >> 6)) * 0.25 - 128.00;
                        cluster_temp.VrelLat   = ((((uint32_t)frame.frame.data[5] & 0b111111) << 3) + ((uint32_t)frame.frame.data[6] >> 5)) * 0.25 - 64.00;
                        cluster_temp.DynProp   = (uint32_t)frame.frame.data[6] & 0b111;
                        cluster_temp.RCS       = (uint32_t)frame.frame.data[7] * 0.5 - 64.00;
                        rawClusterLists.push_back(std::move(cluster_temp));

                        mRadarClusterCounter += 1;
                        if (mRadarClusterCounter >= mNumOfClusters) {
                            mRadarClusterCounter = 0;
                            if (mRadarConfigStatus.SendQualityCfg) {
                                mRadarClusterRecvState = CLUSTER_RADAR_GET_QUALITY_INFO;
                            } else {
                                mRadarClusterRecvState = CLUSTER_RADAR_IDLE;
                                {
                                    std::unique_lock<std::mutex> lock(mCanClusterMutex);
                                    std::vector<radar_cluster_info_t> temp(rawClusterLists);
                                    mCanClusterQueue.push_back(std::move(temp));
                                    mCanClusterCond.notify_one();
                                    rawClusterLists.clear();
                                }
                            }
                        }
                    }
                }
                break;

                case CLUSTER_RADAR_GET_QUALITY_INFO: {
                    if (can_id == CAN_O_CLUSTER_0_STATUS) {
                        mRadarClusterCounter = 0;
                        mClustersTimestamp = frame.ts;
                        mNumOfClusters = frame.frame.data[0] + frame.frame.data[1];
                        if (mNumOfClusters > MAX_RADAR_CLUTSERS) {
                            mNumOfClusters = MAX_RADAR_CLUTSERS;
                        }

                        mRadarClusterRecvState = mNumOfClusters == 0 ? CLUSTER_RADAR_IDLE : CLUSTER_RADAR_GET_GENERAL_INFO;
                    } else if (can_id == CAN_O_CLUSTER_2_QUALITY) {
                        radar_cluster_info_t cluster_temp;

                        memset(&cluster_temp, 0, sizeof(cluster_temp));
                        cluster_temp.DistLong_rms = ((uint32_t)frame.frame.data[1] >> 3);
                        cluster_temp.DistLat_rms  = (((uint32_t)frame.frame.data[1] & 0b111) << 2) + ((uint32_t)frame.frame.data[2] >> 6);
                        cluster_temp.VrelLong_rms = ((uint32_t)frame.frame.data[2] & 0b00111110) >> 1;
                        cluster_temp.VrelLat_rms  = (((uint32_t)frame.frame.data[2] & 0b1) << 4) + ((uint32_t)frame.frame.data[3] >> 4);
                        cluster_temp.Pdh0         = (uint32_t)frame.frame.data[3] & 0b111;
                        cluster_temp.AmbigState   = (uint32_t)frame.frame.data[4] & 0b111;
                        cluster_temp.InvalidState = (uint32_t)frame.frame.data[4] >> 3;

                        if (mRadarClusterCounter < rawClusterLists.size()) {
                            rawClusterLists.at(mRadarClusterCounter).DistLong_rms = cluster_temp.DistLong_rms;
                            rawClusterLists.at(mRadarClusterCounter).DistLat_rms  = cluster_temp.DistLat_rms;
                            rawClusterLists.at(mRadarClusterCounter).VrelLong_rms = cluster_temp.VrelLong_rms;
                            rawClusterLists.at(mRadarClusterCounter).VrelLat_rms  = cluster_temp.VrelLat_rms;
                            rawClusterLists.at(mRadarClusterCounter).Pdh0         = cluster_temp.Pdh0;
                            rawClusterLists.at(mRadarClusterCounter).AmbigState   = cluster_temp.AmbigState;
                            rawClusterLists.at(mRadarClusterCounter).InvalidState = cluster_temp.InvalidState;
                        }

                        mRadarClusterCounter += 1;
                        if (mRadarClusterCounter >= mNumOfClusters) {
                            mRadarClusterCounter = 0;
                            mRadarClusterRecvState = CLUSTER_RADAR_IDLE;
                            {
                                std::unique_lock<std::mutex> lock(mCanClusterMutex);
                                std::vector<radar_cluster_info_t> temp(rawClusterLists);
                                mCanClusterQueue.push_back(std::move(temp));
                                mCanClusterCond.notify_one();
                                rawClusterLists.clear();
                            }
                        }
                    }
                }
                break;

                default: {
                    mRadarClusterRecvState = CLUSTER_RADAR_IDLE;
                }
                break;
            }
        }
    });
}

static void can_capture_manager()
{
    pthread_setname_np(pthread_self(), "can_capture");

    while (1) {
        std::unique_lock<std::mutex> lock(mCanFrameMutex);
        mCanFrameCond.wait_for(lock, std::chrono::microseconds(1000), [&] {
            return !mCanFrameQueue.empty();
        });
        lock.unlock();

        if (!mCanFrameQueue.empty()) {
            auto frame = std::move(mCanFrameQueue.front());
            mCanFrameQueue.pop_front();

            auto canFrameFuture = ProcessCanFrameAsync(std::move(frame));
        }
    }
}

static void can_cluster_manager()
{
    pthread_setname_np(pthread_self(), "can_cluster");

    while (1) {
        std::unique_lock<std::mutex> lock(mCanClusterMutex);
        mCanClusterCond.wait_for(lock, std::chrono::microseconds(1000), [&] {
            return !mCanClusterQueue.empty();
        });
        lock.unlock();

        if (!mCanClusterQueue.empty()) {
            auto frame = std::move(mCanClusterQueue.front());
            mCanClusterQueue.pop_front();

            std::vector<ars408_cluster_t> cluster{};
            for (const auto &item : frame) {
                ars408_cluster_t c;
                c.Cluster_timestamp    = item.timestamp;
                c.Cluster_ID           = item.ID;
                c.Cluster_DistLong     = item.DistLong;
                c.Cluster_DistLat      = item.DistLat;
                c.Cluster_VrelLong     = item.VrelLong;
                c.Cluster_VrelLat      = item.VrelLat;
                c.Cluster_DynProp      = item.DynProp;
                c.Cluster_RCS          = item.RCS;
                c.Cluster_DistLong_rms = item.DistLong_rms;
                c.Cluster_DistLat_rms  = item.DistLat_rms;
                c.Cluster_VrelLong_rms = item.VrelLong_rms;
                c.Cluster_VrelLat_rms  = item.VrelLat_rms;
                c.Cluster_Pdh0         = item.Pdh0;
                c.Cluster_AmbigState   = item.AmbigState;
                c.Cluster_InvalidState = item.InvalidState;
                cluster.emplace_back(std::move(c));
            }

            if (cluster.size() > 0) {
                ars408_cluster_callback(std::move(cluster));
            }
        }
    }
}

static void can_objects_manager()
{
    pthread_setname_np(pthread_self(), "can_objects");

    while (1) {
        std::unique_lock<std::mutex> lock(mCanObjectMutex);
        mCanObjectCond.wait_for(lock, std::chrono::microseconds(5000), [&] {
            return !mCanObjectQueue.empty();
        });
        lock.unlock();

        if (!mCanObjectQueue.empty()) {
            auto frame = std::move(mCanObjectQueue.front());
            mCanObjectQueue.pop_front();

            std::vector<ars408_object_t> objects{};
            for (const auto &item : frame) {
                ars408_object_t o;
                o.timestamp             = item.timestamp;
                o.ID                    = item.ID;
                o.DistLong              = item.DistLong;
                o.DistLat               = item.DistLat;
                o.VrelLong              = item.VrelLong;
                o.VrelLat               = item.VrelLat;
                o.DynProp               = item.DynProp;
                o.RCS                   = item.RCS;
                o.DistLong_rms          = item.DistLong_rms;
                o.DistLat_rms           = item.DistLat_rms;
                o.VrelLong_rms          = item.VrelLong_rms;
                o.VrelLat_rms           = item.VrelLat_rms;
                o.ArelLong_rms          = item.ArelLong_rms;
                o.ArelLat_rms           = item.ArelLat_rms;
                o.Orientation_rms       = item.Orientation_rms;
                o.MeasState             = item.MeasState;
                o.Orientation_rms       = item.Orientation_rms;
                o.MeasState             = item.MeasState;
                o.ProbOfExist           = item.ProbOfExist;
                o.ArelLong              = item.ArelLong;
                o.ArelLat               = item.ArelLat;
                o.Class                 = item.Class;
                o.OrientationAngel      = item.OrientationAngel;
                o.Length                = item.Length;
                o.Width                 = item.Width;
                o.CollDetRegionBitfield = item.CollDetRegionBitfield;

                objects.emplace_back(std::move(o));
            }

            if (objects.size() > 0) {
                ars408_ojects_callback(std::move(objects));
            }
        }
    }
}

int can_frame_analysis_init(void)
{
    can_capture_tid = std::thread(can_capture_manager);
    can_cluster_tid = std::thread(can_cluster_manager);
    can_objects_tid = std::thread(can_objects_manager);

    return 0;
}

int can_frame_analysis_exit(void)
{
    if (can_capture_tid.joinable()) {
        can_capture_tid.join();
    }

    if (can_cluster_tid.joinable()) {
        can_cluster_tid.join();
    }

    if (can_objects_tid.joinable()) {
        can_objects_tid.join();
    }
}
#endif
