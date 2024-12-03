#ifndef __CAN_FRAME_ANALYSIS_H__
#define __CAN_FRAME_ANALYSIS_H__

#include <stdint.h>
#include <linux/can.h>

#include <mutex>
#include <deque>
#include <condition_variable>

typedef struct {
    uint64_t         ts;
    struct can_frame frame;
} can_frame_t;

typedef struct {
    double       Cluster_timestamp;
    unsigned int Cluster_ID;
    float        Cluster_DistLong;
    float        Cluster_DistLat;
    float        Cluster_VrelLong;
    float        Cluster_VrelLat;
    unsigned int Cluster_DynProp;
    float        Cluster_RCS;
    unsigned int Cluster_DistLong_rms;
    unsigned int Cluster_DistLat_rms;
    unsigned int Cluster_VrelLong_rms;
    unsigned int Cluster_VrelLat_rms;
    unsigned int Cluster_Pdh0;
    unsigned int Cluster_AmbigState;
    unsigned int Cluster_InvalidState;
} __attribute__((packed)) ars408_cluster_t;

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
} ars408_object_t;

extern std::mutex mCanFrameMutex;
extern std::condition_variable mCanFrameCond;
extern std::deque<can_frame_t> mCanFrameQueue;

#ifdef __cplusplus
extern "C" {
#endif

int can_frame_analysis_init(void);
int can_frame_analysis_exit(void);

#ifdef __cplusplus
}
#endif

#endif
