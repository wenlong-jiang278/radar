/********************************************************************************
 * @File name:cluster.c
 * @Author:李军
 * @Version: 1.0
 * @Date:2023.05.12
 * @Description:支持点云数据的存储
 ********************************************************************************/

#include "cluster.h"
#include "can_tcp.h"
#include "gps.h"
#include "http.h"
#include "ntp.h"
#include "sqlite3.h"
#include "upgrade.h"
#include <arpa/inet.h>
#include <assert.h>
#include <dirent.h>
#include <errno.h>
#include <fcntl.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <pthread.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <termios.h>
#include <time.h>
#include <unistd.h>
#include <spdlog/spdlog.h>

int clusterFd = 0;                                          // file fd
FILE *clusterFp = NULL;                                     // file fp
FILE *algObjectFp = NULL;                                   // file fp
char clusterFilePath[128];                                  // 文件名
int Cluster_NofClustersFar;                                 // 远程扫描中检测到的clusters点数量
int saveClusterInfoFlg = 0;                                 // 0:do not save, 1:save
int Cluster_NofClustersNear;                                // 近距离扫描中检测到的clusters点数
char algObjectFilePath[128];                                // 文件名
int gotoSendCluster_CanSend;                                // 0:不能传输给后端，1：可以传输给后端
int clusterInfoValidFlg = 0;                                // 0:invalid, 1:valid
int saveAlgObjectInfoFlg = 0;                               // 0:do not save, 1:save
int clusterList1CanWrFlg = 0;                               // 0:can not write, 1:can write
int gotoSendCluster_CurCount;                               // 当前计数
int radar_cluster_counter = 0;                              // 当前clusters点下标
int gotoSendClusterInfoFlg = 0;                             // 开始传输点云数据
int gotoSendCluster_TotalCount = 0;                         // 当次传输的总数量
int gotoSendCluster_TotalCountBak = 0;                      // 当次传输的总数量(用于实际传输)
//RadarClusterInfoARS radar_cluster_algo[MAX_CLUSTER_INFO];   // 当次扫描的clusters点信息
RadarClusterInfoARS radar_cluster_list1[MAX_CLUSTER_INFO];  // 当次扫描的clusters点信息

double g_clustTstp;                                         // 单次扫描点云时间戳
int g_canTstpBgnFlg = 0;                                    // 单次扫描点云时间戳启用标志，0：未启用，1：启用
int g_clustFileSize = 0;                                    // 当前点云文件大小，单位：字节
int g_clustTstpBgnFlg = 0;                                  // 单次扫描点云时间戳启用标志，0：未启用，1：启用
int g_algObjectFileSize = 0;                                // 当前保存经算法计算的目标的文件大小，单位：字节
int g_actualCurClusterCnt = 0;                              // 当次实际读取到的点云数据，目前Linux read的方式会出现丢数据的情况
int g_actualCurClusterCnt_702 = 0;                          // cluster 0x702数据个数

// cluster动态属性表示是否移动
const char *str_Cluster_DynProp[] = {
    "moving",  "stationary",          "oncoming",        "stationary candidate",
    "unknown", "crossing stationary", "crossing moving", "stopped"
};

// 信号数值列表
const char *str_Cluster_DistVrel_rms[] = {
    "<0.005", "<0.006", "<0.008",  "<0.011", "<0.014", "<0.018", "<0.023",
    "<0.029", "<0.038", "<0.049",  "<0.063", "<0.081", "<0.105", "<0.135",
    "<0.174", "<0.224", "<0.288",  "<0.371", "<0.478", "<0.616", "<0.794",
    "<1.023", "<1.317", "<1.697",  "<2.187", "<2.817", "<3.630", "<4.676",
    "<6.025", "<7.762", "<10.000", "invalid"
};

// Cluster的误报概率
const char *str_Cluster_Pdh0[] = {
    "invalid", "<25%", "<50%",   "<75%",
    "<90%",    "<99%", "<99.9%", "<=100%"
};

// 多普勒状态（径向速度）模糊度
const char *str_Cluster_AmbigState[] = {
    "invalid", "ambiguous", "staggered ramp",
    "unambiguous", "stationary candidates"
};

char g_setClusterFpBuf[128 * 1024] = {0};
char g_setAlgObjectFpBuf[128 * 1024] = {0};

/* 写文件表头 */
int user_writeClusterTitle(char *pBuf, int len, FILE *fp)
{
    int ret = 0;
    ret |= fwrite(pBuf, len, 1, fp);

    g_clustFileSize += len;
    return ret;
}

/* 写文件表头 */
int user_writeAlgObjectTitle(char *pBuf, int len, FILE *fp)
{
    int ret = 0;

    ret |= fwrite(pBuf, len, 1, fp);
    g_algObjectFileSize += len;

    return ret;
}

/* 写clusters点信息 */
int user_writeClusterInfo(RadarClusterInfoARS *pInfo, int cnt, FILE *fp)
{
    int ret = 0;
    int i, length;
    char buf[256] = {0};

    if (pInfo == NULL) {
        return -1;
    }

    for (i = 0; i < cnt; ++i) {
        memset(buf, 0, sizeof(buf));

        snprintf(buf, sizeof(buf), "%.3lf,%u,%.1f,%.1f,%.1f,%.1f,%.1f\n",
                pInfo[i].Cluster_timestamp, pInfo[i].Cluster_ID,
                pInfo[i].Cluster_DistLong, pInfo[i].Cluster_DistLat,
                pInfo[i].Cluster_VrelLong, pInfo[i].Cluster_VrelLat,
                pInfo[i].Cluster_RCS);

        length = strlen(buf);
        ret |= fwrite(buf, length, 1, fp);
        if (ret < 0) {
            return ret;
        }

        g_clustFileSize += length;
    }

    if (g_clustFileSize >= MAX_CLUSTERFILE_SIZE) {
        endSaveClusterInfo(NULL);
    }

    return ret;
}

/* 开始导出cluster信息 */
int startSaveClusterInfo(char *cont)
{
    int fd;
    int fdFlags;
    int ret = 0;
    char filePath[128];

    if (cont == NULL) {
        return -1;
    }

    if ((saveClusterInfoFlg == 1) || (saveAlgObjectInfoFlg == 1)) {
        ret = 1;
        return ret;
    }

    ret = access("/run/media/mmcblk0p1/", F_OK);
    if (ret != 0) {
        ret = 2;
        return ret;
    }

    struct timespec sttmspc = {0};
    char buf[256] = {0};
    clock_gettime(CLOCK_REALTIME_COARSE, &sttmspc);

    memset(buf, 0, sizeof(buf));
    snprintf(buf, sizeof(buf), "/run/media/mmcblk0p1/%d.csv", sttmspc.tv_sec);
    memset(filePath, 0, sizeof(filePath));
    strncpy(filePath, buf, strlen(buf));

    clusterFp = fopen(buf, "w+");
    if (clusterFp == NULL) {
        spdlog::error("open {0} failed", buf);
        endSaveClusterInfo(NULL);
        ret = 3;
        return ret;
    }

    fd = fileno(clusterFp);
    fdFlags = fcntl(fd, F_GETFL);
    if (fdFlags == -1) {

    } else {
        if (fcntl(fd, F_SETFL, fdFlags | O_NONBLOCK) == -1) {

        }
    }

    if (setvbuf(clusterFp, g_setClusterFpBuf, _IOFBF, sizeof(g_setClusterFpBuf)) != 0) {
        endSaveClusterInfo(NULL);
        ret = 3;
        return ret;
    }

    g_clustFileSize = 0;
    memset(buf, '\0', sizeof(buf));

    snprintf(buf, sizeof(buf), "timestamp(s),ID,DistLong(m),DistLat(m),VrelLong(m/s),VrelLat(m/s),RCS(dBm2)\n");
    //pthread_rwlock_rdlock(&g_canBuf.mtxCanBuf);
    //pthread_rwlock_unlock(&g_canBuf.mtxCanBuf);
    ret = user_writeClusterTitle(buf, strlen(buf), clusterFp);
    if (ret < 0) {
        endSaveClusterInfo(NULL);
        ret = 4;
        return ret;
    }

    memset(clusterFilePath, 0, sizeof(clusterFilePath));
    strncpy(clusterFilePath, filePath, strlen(filePath));
    radar_cluster_counter = 0;
    clusterList1CanWrFlg = 0;
    clusterInfoValidFlg = 0;
    saveClusterInfoFlg = 1;

    return 0;
}

/* 开始导出经算法计算之后的目标信息 */
int startSaveAlgObjectInfo(char *cont)
{
    int fd;
    int fdFlags;
    int ret = 0;
    char filePath[128];

    if (cont == NULL) {
        return -1;
    }

    if ((saveClusterInfoFlg == 1) || (saveAlgObjectInfoFlg == 1)) {
        ret = 1;
        return ret;
    }

    ret = access("/run/media/mmcblk0p1/", F_OK);
    if (ret != 0) {
        ret = 2;
        return ret;
    }

    char buf[256] = {0};
    struct timespec sttmspc = {0};

    clock_gettime(CLOCK_REALTIME_COARSE, &sttmspc);

    memset(buf, 0, sizeof(buf));
    snprintf(buf, sizeof(buf), "/run/media/mmcblk0p1/%d.csv", sttmspc.tv_sec);
    memset(filePath, 0, sizeof(filePath));
    strncpy(filePath, buf, strlen(buf));

    algObjectFp = fopen(buf, "w+");
    if (algObjectFp == NULL) {
        spdlog::error("open {0} failed", buf);
        endSaveAlgObjectInfo(NULL);
        ret = 3;
        return ret;
    }

    fd = fileno(algObjectFp);
    fdFlags = fcntl(fd, F_GETFL);
    if (fdFlags == -1) {

    } else {
        if (fcntl(fd, F_SETFL, fdFlags | O_NONBLOCK) == -1) {

        }
    }

    if (setvbuf(algObjectFp, g_setAlgObjectFpBuf, _IOFBF, sizeof(g_setAlgObjectFpBuf)) != 0) {
        endSaveAlgObjectInfo(NULL);
        ret = 3;
        return ret;
    }

    g_algObjectFileSize = 0;
    memset(buf, 0, sizeof(buf));
    snprintf(buf, sizeof(buf),
            "timestamp,ID,DistLong,DistLat,VrelLong,VrelLat,ArelLong,ArelLat,"
            "RCS,longitude,latitude,altitude,Class,OrientationAngel,Length,"
            "Width,Lane,Event\n");
    //pthread_rwlock_rdlock(&g_canBuf.mtxCanBuf);
    //pthread_rwlock_unlock(&g_canBuf.mtxCanBuf);
    ret = user_writeAlgObjectTitle(buf, strlen(buf), algObjectFp);
    if (ret < 0) {
        endSaveAlgObjectInfo(NULL);
        ret = 4;
        return ret;
    }

    memset(algObjectFilePath, '\0', sizeof(algObjectFilePath));
    strncpy(algObjectFilePath, filePath, strlen(filePath));
    saveAlgObjectInfoFlg = 1;

    return 0;
}

/* 停止导出cluster信息 */
int endSaveClusterInfo(char *cont)
{
    saveClusterInfoFlg = 0;
    radar_cluster_counter = 0;
    clusterList1CanWrFlg = 0;
    clusterInfoValidFlg = 0;

    if (clusterFp != NULL) {
        fflush(clusterFp);
        fclose(clusterFp);
        clusterFp = NULL;
    }

    return 0;
}

/* 停止导出经算法计算出来的目标信息 */
int endSaveAlgObjectInfo(char *cont)
{
    saveAlgObjectInfoFlg = 0;

    if (algObjectFp != NULL) {
        fflush(algObjectFp);
        fclose(algObjectFp);
        algObjectFp = NULL;
    }

    return 0;
}
