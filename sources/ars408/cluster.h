/********************************************************************************
 * @File name:cluster.h
 * @Author:李军
 * @Version: 1.0
 * @Date:2023.05.12
 * @Description:支持点云数据的存储
 ********************************************************************************/

#ifndef USER_CLUSTER_H
#define USER_CLUSTER_H

#include <stdio.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct RadarClusterInfoARS_ {
  double Cluster_timestamp;
  unsigned int Cluster_ID;
  float Cluster_DistLong;
  float Cluster_DistLat;
  float Cluster_VrelLong;
  float Cluster_VrelLat;
  unsigned int Cluster_DynProp;
  float Cluster_RCS;
  unsigned int Cluster_DistLong_rms;
  unsigned int Cluster_DistLat_rms;
  unsigned int Cluster_VrelLong_rms;
  unsigned int Cluster_VrelLat_rms;
  unsigned int Cluster_Pdh0;
  unsigned int Cluster_AmbigState;
  unsigned int Cluster_InvalidState;
} __attribute__((packed)) RadarClusterInfoARS;

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
} RadarObjectsInfoARS;

#define MAX_CLUSTER_INFO 250
#define MAX_CLUSTERFILE_SIZE 1024 * 1024 * 1024   // 1024M
#define MAX_ALGOBJECTFILE_SIZE 1024 * 1024 * 1024 // 1024M

//extern RadarClusterInfoARS radar_cluster_algo[MAX_CLUSTER_INFO];
extern RadarClusterInfoARS radar_cluster_list1[MAX_CLUSTER_INFO];
extern int Cluster_NofClustersNear;
extern int Cluster_NofClustersFar;
extern int saveClusterInfoFlg; // 0:do not save, 1:save
extern int saveAlgObjectInfoFlg;
extern int clusterList1CanWrFlg;
extern int radar_cluster_counter;
extern int clusterFd;
extern FILE *clusterFp;
extern FILE *algObjectFp;
extern char clusterFilePath[128];
extern char algObjectFilePath[128];
extern int clusterInfoValidFlg;
extern const char *str_Cluster_DynProp[];
extern const char *str_Cluster_DistVrel_rms[];
extern const char *str_Cluster_Pdh0[];
extern const char *str_Cluster_AmbigState[];
extern int gotoSendClusterInfoFlg;
extern int gotoSendCluster_TotalCount;
extern int gotoSendCluster_TotalCountBak;
extern RadarClusterInfoARS radar_SendCluster_list1[MAX_CLUSTER_INFO];
extern int gotoSendCluster_CurCount;
extern int gotoSendCluster_CanSend;
extern int g_actualCurClusterCnt;
extern int g_actualCurClusterCnt_702;
extern int g_clustTstpBgnFlg;
extern double g_clustTstp;
extern int g_canTstpBgnFlg;
extern int g_algObjectFileSize;

extern int user_writeClusterTitle(char *pBuf, int len, FILE *fp);
extern int user_writeAlgObjectTitle(char *pBuf, int len, FILE *fp);
extern int user_writeClusterInfo(RadarClusterInfoARS *pInfo, int cnt, FILE *fp);
extern int startSaveClusterInfo(char *cont);
extern int startSaveAlgObjectInfo(char *cont);
extern int endSaveClusterInfo(char *cont);
extern int endSaveAlgObjectInfo(char *cont);

#ifdef __cplusplus
}
#endif

#endif
