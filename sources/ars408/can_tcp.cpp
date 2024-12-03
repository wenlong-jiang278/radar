/********************************************************************************
 * @File name:can_tcp.c
 * @Author:李军
 * @Version: 1.0
 * @Date:2023.05.15
 * @Description:与雷达板子的CAN通讯，与后端的TCP通讯
 ********************************************************************************/

#include <arpa/inet.h>
#include <assert.h>
#include <dirent.h>
#include <errno.h>
#include <fcntl.h>
#include <semaphore.h>
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
#include <sys/epoll.h>
#include <sys/ioctl.h>
#include <sys/select.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <termios.h>
#include <time.h>
#include <unistd.h>

#include "gps.h"
#include "ntp.h"
#include "http.h"
#include "frame.h"
#include "Track.h"
#include "cJSON.h"
#include "serial.h"
#include "cluster.h"
#include "can_tcp.h"
#include "sqlite3.h"
#include "upgrade.h"

#include <map>
#include <vector>
#include <chrono>
#include <spdlog/spdlog.h>

#include "can_frame.h"
#include "radar_cfg.h"

int frameFlag = -1;
int recordMaxFlag = -1;
bool got_serial = false;
bool needRecord = false;
bool preNeedRecord = false;
char serial_buf[32] = {0, };
pthread_mutex_t mutex = PTHREAD_MUTEX_INITIALIZER;

const char *tableName = "car_data_table";
const char *fileName = "/run/media/mmcblk0p1/file/carDB.db";

int Canfd;
int clientfd;
char deviceID;
int preMilli = 0;
int insertCount = 0;
int section_num = 0;
int polyRoi_num = 0;
int g_curCanBufRdIdx;
int ObjectNumber = 0;                                                               // 原始点云数据经过算法处理之后的目标数量
long pthreadCount = 0;
CalbParams g_CalbPara;                                                              // 动态标定相关参数
sqlite3 *record_db = NULL;
Event_Params event_params;
int ars408_detect_clusters = 0;
StaticCalbParams g_StaticCalbPara;                                                  // 静态标定相关参数
bool respondRadarBasicConfig = false;
SectionAndCoilRegionDef sections[LEN] = {0};
UserPolyRegion polyRoi[userMaxPloyRoiNum] = {0};
ARS408RadarObjectInfo RadarObjectData[250] = {0};                                   // 原始点云数据经过算法处理之后的目标信息
ARS408RadarObjectInfo_Wr RadarObjectData_Wr[250] = {0};                             // 原始点云数据经过算法处理之后的目标信息，保存时使用
UserOffsetParam gOffsetParam = {0, 0.0f, 0.0f, 0, 0};
USER_CANBUF g_canBuf = {.mtxCanBuf = PTHREAD_RWLOCK_INITIALIZER};
pthread_mutex_t g_mtxTrafficFlowDetectionStatistics = PTHREAD_MUTEX_INITIALIZER;    // 对交通流统计结果的访问

std::map<int, RoadLaneParam> gRoadLaneParam;

// 计算函数执行时间的函数
double calculateElapsedTime(struct timeval start, struct timeval end)
{
    double elapsed;
    long seconds, microseconds;

    seconds = end.tv_sec - start.tv_sec;
    microseconds = end.tv_usec - start.tv_usec;
    elapsed = seconds * 1000 + microseconds / 1000.0;

    return elapsed;
}

/**
 * 函数名称: getCurrentTimestamp
 * 功能描述: 获取当前时间戳
 * 输入参数: time_level - 时间精度等级
 *              0 - 秒级时间戳(10位)
 *              1 - 毫秒级时间戳(13位)
 *              2 - 微秒级时间戳(16位)
 *              3 - 纳秒级时间戳(19位)
 * 输出参数: 无
 * 返回说明: 时间戳
 */
uint64_t getCurrentTimestamp(int time_level)
{
    uint64_t current_time_stamp;
    auto timeinfo = std::chrono::system_clock::now().time_since_epoch();

    switch (time_level) {
        case 1:
            current_time_stamp = std::chrono::duration_cast<std::chrono::milliseconds>(timeinfo).count();
            break;

        case 2:
            current_time_stamp = std::chrono::duration_cast<std::chrono::microseconds>(timeinfo).count();
            break;

        case 3:
            current_time_stamp = std::chrono::duration_cast<std::chrono::nanoseconds>(timeinfo).count();
            break;

        case 0:
        default:
            current_time_stamp = std::chrono::duration_cast<std::chrono::seconds>(timeinfo).count();
            break;
    }

    return current_time_stamp;
}

/* 写经算法计算出来的目标信息 */
int user_writeAlgObjectInfo(ARS408RadarObjectInfo *pInfo, int cnt, FILE *fp)
{
    int length;
    int i, ret = 0;
    char buf[512] = {0};

    if (pInfo == NULL) {
        return -1;
    }

    for (i = 0; i < cnt; ++i) {
        memset(buf, 0, sizeof(buf));
        snprintf(buf, sizeof(buf),
                "%.3lf,%d,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.10f,%.10f,%.10f,%d,"
                "%.2f,%.2f,%.2f,%d,%d\n",
                pInfo[i].Object_timestamp, pInfo[i].Object_ID,
                pInfo[i].Object_DistLong, pInfo[i].Object_DistLat,
                pInfo[i].Object_VrelLong, pInfo[i].Object_VrelLat,
                pInfo[i].Object_ArelLong, pInfo[i].Object_ArelLat,
                pInfo[i].Object_RCS, pInfo[i].Object_Longitude,
                pInfo[i].Object_Latitude, pInfo[i].Object_Altitude,
                pInfo[i].Object_Class, pInfo[i].Object_OrientationAngel,
                pInfo[i].Object_Length, pInfo[i].Object_Width,
                pInfo[i].Object_Lane, pInfo[i].Object_Event);
        length = strlen(buf);
        ret |= fwrite(buf, length, 1, fp);
        if (ret < 0) {
            return ret;
        }

        g_algObjectFileSize += length;
    }

    if (g_algObjectFileSize >= MAX_ALGOBJECTFILE_SIZE) {
        endSaveAlgObjectInfo(NULL);
    }

    return ret;
}

/* 写经算法计算出来的目标信息 */
int user_writeAlgObjectInfo_Wr(ARS408RadarObjectInfo_Wr *pInfo, int cnt, FILE *fp)
{
    int ret = 0;
    int i, length;
    char buf[512] = {0};

    for (i = 0; i < cnt; ++i) {
        memset(buf, 0, sizeof(buf));
        snprintf(buf, sizeof(buf),
                "%.3lf,%u,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.10f,%.10f,%.10f,%u,"
                "%.2f,%.2f,%.2f,%u,%u\n",
                pInfo[i].Object_timestamp, pInfo[i].Object_ID,
                pInfo[i].Object_DistLong, pInfo[i].Object_DistLat,
                pInfo[i].Object_VrelLong, pInfo[i].Object_VrelLat,
                pInfo[i].Object_ArelLong, pInfo[i].Object_ArelLat,
                pInfo[i].Object_RCS, pInfo[i].Object_Longitude,
                pInfo[i].Object_Latitude, pInfo[i].Object_Altitude,
                pInfo[i].Object_Class, pInfo[i].Object_OrientationAngel,
                pInfo[i].Object_Length, pInfo[i].Object_Width,
                pInfo[i].Object_Lane, pInfo[i].Object_Event);

        length = strlen(buf);
        ret |= fwrite(buf, length, 1, fp);
        if (ret < 0) {
            return ret;
        }

        g_algObjectFileSize += length;
    }

    if (g_algObjectFileSize >= MAX_ALGOBJECTFILE_SIZE) {
        endSaveAlgObjectInfo(NULL);
    }

    return ret;
}

// CRC数组
static unsigned short crc16_table[256] = {
    0x0000, 0x1021, 0x2042, 0x3063, 0x4084, 0x50a5, 0x60c6, 0x70e7, 0x8108,
    0x9129, 0xa14a, 0xb16b, 0xc18c, 0xd1ad, 0xe1ce, 0xf1ef, 0x1231, 0x0210,
    0x3273, 0x2252, 0x52b5, 0x4294, 0x72f7, 0x62d6, 0x9339, 0x8318, 0xb37b,
    0xa35a, 0xd3bd, 0xc39c, 0xf3ff, 0xe3de, 0x2462, 0x3443, 0x0420, 0x1401,
    0x64e6, 0x74c7, 0x44a4, 0x5485, 0xa56a, 0xb54b, 0x8528, 0x9509, 0xe5ee,
    0xf5cf, 0xc5ac, 0xd58d, 0x3653, 0x2672, 0x1611, 0x0630, 0x76d7, 0x66f6,
    0x5695, 0x46b4, 0xb75b, 0xa77a, 0x9719, 0x8738, 0xf7df, 0xe7fe, 0xd79d,
    0xc7bc, 0x48c4, 0x58e5, 0x6886, 0x78a7, 0x0840, 0x1861, 0x2802, 0x3823,
    0xc9cc, 0xd9ed, 0xe98e, 0xf9af, 0x8948, 0x9969, 0xa90a, 0xb92b, 0x5af5,
    0x4ad4, 0x7ab7, 0x6a96, 0x1a71, 0x0a50, 0x3a33, 0x2a12, 0xdbfd, 0xcbdc,
    0xfbbf, 0xeb9e, 0x9b79, 0x8b58, 0xbb3b, 0xab1a, 0x6ca6, 0x7c87, 0x4ce4,
    0x5cc5, 0x2c22, 0x3c03, 0x0c60, 0x1c41, 0xedae, 0xfd8f, 0xcdec, 0xddcd,
    0xad2a, 0xbd0b, 0x8d68, 0x9d49, 0x7e97, 0x6eb6, 0x5ed5, 0x4ef4, 0x3e13,
    0x2e32, 0x1e51, 0x0e70, 0xff9f, 0xefbe, 0xdfdd, 0xcffc, 0xbf1b, 0xaf3a,
    0x9f59, 0x8f78, 0x9188, 0x81a9, 0xb1ca, 0xa1eb, 0xd10c, 0xc12d, 0xf14e,
    0xe16f, 0x1080, 0x00a1, 0x30c2, 0x20e3, 0x5004, 0x4025, 0x7046, 0x6067,
    0x83b9, 0x9398, 0xa3fb, 0xb3da, 0xc33d, 0xd31c, 0xe37f, 0xf35e, 0x02b1,
    0x1290, 0x22f3, 0x32d2, 0x4235, 0x5214, 0x6277, 0x7256, 0xb5ea, 0xa5cb,
    0x95a8, 0x8589, 0xf56e, 0xe54f, 0xd52c, 0xc50d, 0x34e2, 0x24c3, 0x14a0,
    0x0481, 0x7466, 0x6447, 0x5424, 0x4405, 0xa7db, 0xb7fa, 0x8799, 0x97b8,
    0xe75f, 0xf77e, 0xc71d, 0xd73c, 0x26d3, 0x36f2, 0x0691, 0x16b0, 0x6657,
    0x7676, 0x4615, 0x5634, 0xd94c, 0xc96d, 0xf90e, 0xe92f, 0x99c8, 0x89e9,
    0xb98a, 0xa9ab, 0x5844, 0x4865, 0x7806, 0x6827, 0x18c0, 0x08e1, 0x3882,
    0x28a3, 0xcb7d, 0xdb5c, 0xeb3f, 0xfb1e, 0x8bf9, 0x9bd8, 0xabbb, 0xbb9a,
    0x4a75, 0x5a54, 0x6a37, 0x7a16, 0x0af1, 0x1ad0, 0x2ab3, 0x3a92, 0xfd2e,
    0xed0f, 0xdd6c, 0xcd4d, 0xbdaa, 0xad8b, 0x9de8, 0x8dc9, 0x7c26, 0x6c07,
    0x5c64, 0x4c45, 0x3ca2, 0x2c83, 0x1ce0, 0x0cc1, 0xef1f, 0xff3e, 0xcf5d,
    0xdf7c, 0xaf9b, 0xbfba, 0x8fd9, 0x9ff8, 0x6e17, 0x7e36, 0x4e55, 0x5e74,
    0x2e93, 0x3eb2, 0x0ed1, 0x1ef0
};

int user_readRoadLaneParam(void)
{
    int ret = -1;
    sqlite3 *db1 = NULL;
    char *err_msg = NULL;
    char **db_result = NULL;
    char sql_str[2048] = {0};
    int nrow = 0, ncolumn = 0;

    ret = sqlite3_open(CARDB_DB_FILE, &db1);
    if (ret != SQLITE_OK) {
        spdlog::error("open database {0} failed", CARDB_DB_FILE);
        return -1;
    }

    /* 如果数据表不存在，则创建数据表 */
    memset(sql_str, 0, sizeof(sql_str));
    sprintf(sql_str,
        "CREATE TABLE IF NOT EXISTS road_lane("
        "id INTEGER PRIMARY KEY,"
        "lane INTEGER DEFAULT 1,"
        "dire INTEGER DEFAULT 0,"
        "turn INTEGER DEFAULT 0,"
        "width REAL DEFAULT 3.6,"
        "radius REAL DEFAULT 0.0,"
        "lt_x REAL DEFAULT 300.0,"
        "lt_y REAL DEFAULT 3.6,"
        "rt_x REAL DEFAULT 300.0,"
        "rt_y REAL DEFAULT 0.0,"
        "rb_x REAL DEFAULT 0.0,"
        "rb_y REAL DEFAULT 0.0,"
        "lb_x REAL DEFAULT 0.0,"
        "lb_y REAL DEFAULT 3.6); "
        "INSERT OR IGNORE INTO road_lane (id, lane, dire, turn, width, radius, lt_x, lt_y, rt_x, rt_y, rb_x, rb_y, lb_x, lb_y) "
        "VALUES (1, 1, 0, 0, 3.6, 0.0, 300.0, 3.6, 300.0, 0.0, 0.0, 0.0, 0.0, 3.6);"
    );
    ret = sqlite3_exec(db1, sql_str, 0, 0, &err_msg);
    if (ret != SQLITE_OK) {
        sqlite3_free(err_msg);
        sqlite3_close(db1);
        spdlog::error("{0}", sql_str);
        return -2;
    }

    /* 执行查询 */
    memset(sql_str, 0, sizeof(sql_str));
    sprintf(sql_str, "SELECT lane, dire, turn, width, radius, lt_x, lt_y, rt_x, rt_y, rb_x, rb_y, lb_x, lb_y FROM road_lane;");
    ret = sqlite3_get_table(db1, sql_str, &db_result, &nrow, &ncolumn, &err_msg);
    if (ret != SQLITE_OK) {
        sqlite3_free(err_msg);
        sqlite3_close(db1);
        spdlog::error("{0}", sql_str);
        return -3;
    }

    /* 查询结果 */
    for (int i = 1; i <= nrow; i++) {
        RoadLaneParam param;
        param.lane   = std::stoi(db_result[i * ncolumn + 0]);
        param.dire   = std::stoi(db_result[i * ncolumn + 1]);
        param.turn   = std::stoi(db_result[i * ncolumn + 2]);
        param.width  = std::stof(db_result[i * ncolumn + 3]);
        param.radius = std::stof(db_result[i * ncolumn + 4]);
        param.lt_x   = std::stof(db_result[i * ncolumn + 5]);
        param.lt_y   = std::stof(db_result[i * ncolumn + 6]);
        param.rt_x   = std::stof(db_result[i * ncolumn + 7]);
        param.rt_y   = std::stof(db_result[i * ncolumn + 8]);
        param.rb_x   = std::stof(db_result[i * ncolumn + 9]);
        param.rb_y   = std::stof(db_result[i * ncolumn + 10]);
        param.lb_x   = std::stof(db_result[i * ncolumn + 11]);
        param.lb_y   = std::stof(db_result[i * ncolumn + 12]);

        gRoadLaneParam[param.lane] = param;

        spdlog::info("lane:({}), dir:({}), turn:({}), width:({}), radius:({}), LT:({},{}), RT:({},{}), RB:({},{}), LB:({},{})",
            param.lane, param.dire, param.turn, param.width, param.radius, param.lt_x, param.lt_y, param.rt_x, param.rt_y, param.rb_x, param.rb_y, param.lb_x, param.lb_y);
    }

    sqlite3_free_table(db_result);
    sqlite3_close(db1);

    return 0;
}

int user_readOffsetParameters(void)
{
    int ret = 0;
    int nrow, ncolumn;
    sqlite3 *db1 = NULL;
    char *err_msg = NULL;
    char **db_result = NULL;
    char sql_str[2048] = {0};

    ret = sqlite3_open(CARDB_DB_FILE, &db1);
    if (ret != SQLITE_OK) {
        spdlog::error("open database {0} failed", CARDB_DB_FILE);
        return 1;
    }

    /* 如果数据表不存在，则创建数据表 */
    memset(sql_str, 0, sizeof(sql_str));
    sprintf(sql_str,
            "CREATE TABLE IF NOT EXISTS offset_param(id INTEGER PRIMARY "
            "KEY,flipXY VARCHAR(10) DEFAULT '0',xOffset VARCHAR(15) DEFAULT "
            "'0.0',yOffset VARCHAR(15) DEFAULT '0.0',xDirect VARCHAR(15) DEFAULT "
            "'0',yDirect VARCHAR(15) DEFAULT '0'); INSERT OR IGNORE INTO "
            "offset_param (id, flipXY, xOffset, yOffset, xDirect, yDirect) "
            "VALUES (1, '0', '0.0', '0.0', '0', '0');");
    ret = sqlite3_exec(db1, sql_str, 0, 0, &err_msg);
    if (ret != SQLITE_OK) {
        sqlite3_free(err_msg);
        sqlite3_close(db1);
        spdlog::error("{0}", sql_str);
        return -1;
    }

    /* 读取偏移参数 */
    memset(sql_str, 0, sizeof(sql_str));
    sprintf(sql_str, "select * from offset_param");
    ret = sqlite3_exec(db1, "begin transaction", 0, 0, &err_msg);
    if (ret != SQLITE_OK) {
        sqlite3_free(err_msg);
    }

    ret = sqlite3_get_table(db1, sql_str, &db_result, &nrow, &ncolumn, &err_msg);
    if (ret != SQLITE_OK) {
        sqlite3_free(err_msg);
        sqlite3_close(db1);
        spdlog::error("{0}", sql_str);
        return -1;
    }

    ret = sqlite3_exec(db1, "commit transaction", 0, 0, &err_msg);
    if (ret != SQLITE_OK) {
        sqlite3_free(err_msg);
    }

    if (db_result != NULL) {
        int offset = 1;
        if (db_result[1 * ncolumn + offset] != NULL) {
            gOffsetParam.flipXY = atoi(db_result[1 * ncolumn + offset]);
            offset++;
        }

        if (db_result[1 * ncolumn + offset] != NULL) {
            gOffsetParam.xOffset = strtold(db_result[1 * ncolumn + offset], NULL);
            offset++;
        }

        if (db_result[1 * ncolumn + offset] != NULL) {
            gOffsetParam.yOffset = strtold(db_result[1 * ncolumn + offset], NULL);
            offset++;
        }

        if (db_result[1 * ncolumn + offset] != NULL) {
            gOffsetParam.xDirect = atoi(db_result[1 * ncolumn + offset]);
            offset++;
        }

        if (db_result[1 * ncolumn + offset] != NULL) {
            gOffsetParam.yDirect = atoi(db_result[1 * ncolumn + offset]);
            offset++;
        }

        spdlog::info("flipXY:{0}, xOffset:{1}, yOffset:{2}, xDirect:{3}, yDirect:{4}", gOffsetParam.flipXY, gOffsetParam.xOffset, gOffsetParam.yOffset, gOffsetParam.xDirect, gOffsetParam.yDirect);
        sqlite3_free_table(db_result);
    }

    sqlite3_close(db1);
    return 0;
}

int user_readCalibrationPara(void)
{
    int ret = 0;
    int nrow, ncolumn;
    sqlite3 *db1 = NULL;
    char *err_msg = NULL;
    char **db_result = NULL;
    char sql_find_Calibration[512] = {0};
    sprintf(sql_find_Calibration, "select * from markers_model");

    /* 连接数据库 */
    ret = sqlite3_open(CARDB_DB_FILE, &db1);
    if (ret != SQLITE_OK) {
        printf("无法打开数据库 %s\n", CARDB_DB_FILE);
        return 1;
    }

    /* 读取校准参数 */
    ret = sqlite3_exec(db1, "begin transaction", 0, 0, &err_msg);
    if (ret != SQLITE_OK) {
        sqlite3_free(err_msg);
    }

    ret = sqlite3_get_table(db1, sql_find_Calibration, &db_result, &nrow, &ncolumn, &err_msg);
    if (ret != SQLITE_OK) {
        sqlite3_close(db1);
        sqlite3_free(err_msg);
        return -1;
    }

    ret = sqlite3_exec(db1, "commit transaction", 0, 0, &err_msg);
    if (ret != SQLITE_OK) {
        sqlite3_free(err_msg);
    }

    if (db_result != NULL) {
        int offset = 1;
        if (db_result[1 * ncolumn + offset] != NULL) {
            g_CalbPara.matrix_1 = strtold(db_result[1 * ncolumn + offset], NULL);
            offset++;
        }

        if (db_result[1 * ncolumn + offset] != NULL) {
            g_CalbPara.matrix_2 = strtold(db_result[1 * ncolumn + offset], NULL);
            offset++;
        }

        if (db_result[1 * ncolumn + offset] != NULL) {
            g_CalbPara.matrix_3 = strtold(db_result[1 * ncolumn + offset], NULL);
            offset++;
        }

        if (db_result[1 * ncolumn + offset] != NULL) {
            g_CalbPara.matrix_4 = strtold(db_result[1 * ncolumn + offset], NULL);
            offset++;
        }

        if (db_result[1 * ncolumn + offset] != NULL) {
            g_CalbPara.matrix_5 = strtold(db_result[1 * ncolumn + offset], NULL);
            offset++;
        }

        if (db_result[1 * ncolumn + offset] != NULL) {
            g_CalbPara.matrix_6 = strtold(db_result[1 * ncolumn + offset], NULL);
            offset++;
        }

        if (db_result[1 * ncolumn + offset] != NULL) {
            g_CalbPara.p00 = strtold(db_result[1 * ncolumn + offset], NULL);
            offset++;
        }

        if (db_result[1 * ncolumn + offset] != NULL) {
            g_CalbPara.p10 = strtold(db_result[1 * ncolumn + offset], NULL);
            offset++;
        }

        if (db_result[1 * ncolumn + offset] != NULL) {
            g_CalbPara.p01 = strtold(db_result[1 * ncolumn + offset], NULL);
            offset++;
        }

        if (db_result[1 * ncolumn + offset] != NULL) {
            g_CalbPara.p20 = strtold(db_result[1 * ncolumn + offset], NULL);
            offset++;
        }

        if (db_result[1 * ncolumn + offset] != NULL) {
            g_CalbPara.p11 = strtold(db_result[1 * ncolumn + offset], NULL);
            offset++;
        }

        if (db_result[1 * ncolumn + offset] != NULL) {
            g_CalbPara.p02 = strtold(db_result[1 * ncolumn + offset], NULL);
            offset++;
        }

        if (db_result[1 * ncolumn + offset] != NULL) {
            g_CalbPara.origin_longitude = strtold(db_result[1 * ncolumn + offset], NULL);
            offset++;
        }

        if (db_result[1 * ncolumn + offset] != NULL) {
            g_CalbPara.origin_latitude = strtold(db_result[1 * ncolumn + offset], NULL);
            offset++;
        }

        if (db_result[1 * ncolumn + offset] != NULL) {
            g_CalbPara.angle = strtold(db_result[1 * ncolumn + offset], NULL);
            offset++;
        }

        if (db_result[1 * ncolumn + offset] != NULL) {
            g_CalbPara.is_del = strtold(db_result[1 * ncolumn + offset], NULL);
            offset++;
        }

        if (db_result[1 * ncolumn + offset] != NULL) {
            g_CalbPara.status = strtold(db_result[1 * ncolumn + offset], NULL);
            offset++;
        }

        if (db_result[1 * ncolumn + offset] != NULL) {
            g_CalbPara.create_by = strtold(db_result[1 * ncolumn + offset], NULL);
            offset++;
        }

        if (db_result[1 * ncolumn + offset] != NULL) {
            g_CalbPara.create_date = strtold(db_result[1 * ncolumn + offset], NULL);
            offset++;
        }

        if (db_result[1 * ncolumn + offset] != NULL) {
            g_CalbPara.update_by = strtold(db_result[1 * ncolumn + offset], NULL);
            offset++;
        }

        if (db_result[1 * ncolumn + offset] != NULL) {
            g_CalbPara.update_date = strtold(db_result[1 * ncolumn + offset], NULL);
            offset++;
        }

        sqlite3_free_table(db_result);
    }

    sqlite3_close(db1);
    return 0;
}

int user_readStaticCalibrationPara(void)
{
    int ret = 0;
    int nrow, ncolumn;
    sqlite3 *db1 = NULL;
    char *err_msg = NULL;
    char **db_result = NULL;
    char sql_find_Calibration[512] = {0};

    sprintf(sql_find_Calibration, "select * from markers_staticModel");

    /* 连接数据库 */
    ret = sqlite3_open(CARDB_DB_FILE, &db1);
    if (ret != SQLITE_OK) {
        spdlog::error("open {0} sqlite db failed", CARDB_DB_FILE);
        return 1;
    }

    /* 读取校准参数 */
    ret = sqlite3_exec(db1, "begin transaction", 0, 0, &err_msg);
    if (ret != SQLITE_OK) {
        sqlite3_free(err_msg);
    }

    ret = sqlite3_get_table(db1, sql_find_Calibration, &db_result, &nrow, &ncolumn, &err_msg);
    if (ret != SQLITE_OK) {
        sqlite3_close(db1);
        sqlite3_free(err_msg);
        return -1;
    }

    ret = sqlite3_exec(db1, "commit transaction", 0, 0, &err_msg);
    if (ret != SQLITE_OK) {
        sqlite3_free(err_msg);
    }

    if (db_result != NULL) {
        int offset = 1;
        if (db_result[1 * ncolumn + offset] != NULL) {
            g_StaticCalbPara.matrix_1 = strtold(db_result[1 * ncolumn + offset], NULL);
            offset++;
        }

        if (db_result[1 * ncolumn + offset] != NULL) {
            g_StaticCalbPara.matrix_2 = strtold(db_result[1 * ncolumn + offset], NULL);
            offset++;
        }

        if (db_result[1 * ncolumn + offset] != NULL) {
            g_StaticCalbPara.matrix_3 = strtold(db_result[1 * ncolumn + offset], NULL);
            offset++;
        }

        if (db_result[1 * ncolumn + offset] != NULL) {
            g_StaticCalbPara.matrix_4 = strtold(db_result[1 * ncolumn + offset], NULL);
            offset++;
        }

        if (db_result[1 * ncolumn + offset] != NULL) {
            g_StaticCalbPara.matrix_5 = strtold(db_result[1 * ncolumn + offset], NULL);
            offset++;
        }

        if (db_result[1 * ncolumn + offset] != NULL) {
            g_StaticCalbPara.matrix_6 = strtold(db_result[1 * ncolumn + offset], NULL);
            offset++;
        }

        if (db_result[1 * ncolumn + offset] != NULL) {
            g_StaticCalbPara.p00 = strtold(db_result[1 * ncolumn + offset], NULL);
            offset++;
        }

        if (db_result[1 * ncolumn + offset] != NULL) {
            g_StaticCalbPara.p10 = strtold(db_result[1 * ncolumn + offset], NULL);
            offset++;
        }

        if (db_result[1 * ncolumn + offset] != NULL) {
            g_StaticCalbPara.p01 = strtold(db_result[1 * ncolumn + offset], NULL);
            offset++;
        }

        if (db_result[1 * ncolumn + offset] != NULL) {
            g_StaticCalbPara.p20 = strtold(db_result[1 * ncolumn + offset], NULL);
            offset++;
        }

        if (db_result[1 * ncolumn + offset] != NULL) {
            g_StaticCalbPara.p11 = strtold(db_result[1 * ncolumn + offset], NULL);
            offset++;
        }

        if (db_result[1 * ncolumn + offset] != NULL) {
            g_StaticCalbPara.p02 = strtold(db_result[1 * ncolumn + offset], NULL);
            offset++;
        }

        if (db_result[1 * ncolumn + offset] != NULL) {
            g_StaticCalbPara.origin_latitude = strtold(db_result[1 * ncolumn + offset], NULL);
            offset++;
        }

        if (db_result[1 * ncolumn + offset] != NULL) {
            g_StaticCalbPara.origin_longitude = strtold(db_result[1 * ncolumn + offset], NULL);
            offset++;
        }

        if (db_result[1 * ncolumn + offset] != NULL) {
            g_StaticCalbPara.angle = strtold(db_result[1 * ncolumn + offset], NULL);
            offset++;
        }

        if (db_result[1 * ncolumn + offset] != NULL) {
            g_StaticCalbPara.is_del = strtold(db_result[1 * ncolumn + offset], NULL);
            offset++;
        }

        if (db_result[1 * ncolumn + offset] != NULL) {
            g_StaticCalbPara.status = strtold(db_result[1 * ncolumn + offset], NULL);
            offset++;
        }

        if (db_result[1 * ncolumn + offset] != NULL) {
            g_StaticCalbPara.create_by = strtold(db_result[1 * ncolumn + offset], NULL);
            offset++;
        }

        if (db_result[1 * ncolumn + offset] != NULL) {
            g_StaticCalbPara.create_date = strtold(db_result[1 * ncolumn + offset], NULL);
            offset++;
        }

        if (db_result[1 * ncolumn + offset] != NULL) {
            g_StaticCalbPara.update_by = strtold(db_result[1 * ncolumn + offset], NULL);
            offset++;
        }

        if (db_result[1 * ncolumn + offset] != NULL) {
            g_StaticCalbPara.update_date = strtold(db_result[1 * ncolumn + offset], NULL);
            offset++;
        }

        sqlite3_free_table(db_result);
    }

    sqlite3_close(db1);
    return 0;
}

int user_flushParams(void *ptr)
{
    int ret = 0;
    sqlite3 *db1 = NULL;

    /* 连接数据库 */
    ret = sqlite3_open(ADJUST_DB_FILE, &db1);
    if (ret != SQLITE_OK) {
        spdlog::error("open {0} sqlite db failed", ADJUST_DB_FILE);
        return 1;
    }

    ret = ReadParams(db1, &adjust_Params, sections, &section_num);
    if (ret < 0) {
        spdlog::error("dynamic parameter initialization error");
    } else {
        sqlite3_close(db1);
    }

    return 0;
}

/**
 *@brief CRC16 MSB First 多项式 1021
 *@param puchMsg：参与校验的数据
 *@param usDataLen：参与校验数据的长度
 *@return crc 检验码
 */
unsigned short CRC16_MSB1021(unsigned char *puchMsg, int usDataLen)
{
    int i = 0;
    unsigned short crc = 0;

    for (i = 0; i < usDataLen; i++) {
        crc = crc16_table[((crc >> 8) ^ puchMsg[i]) & 0xFF] ^ (crc << 8);
    }

    return crc;
}

void respondUpperComputerConfigCMD(UINT id, char *data, int dataLen)
{
    unsigned short crc16;
    char packetFrame[256];

    packetFrame[0] = 0xDD;
    packetFrame[1] = 0xFF;
    packetFrame[2] = 0xAA;
    packetFrame[3] = 0x55;
    packetFrame[4] = dataLen >> 8;
    packetFrame[5] = dataLen & 0xff;
    packetFrame[7] = deviceID;

    switch (id) {
        case 0x201:
            packetFrame[6] = FENGHAI_CMD_RADAR_BASIC_CONFIG;
            break;

        case 0x204:
            packetFrame[6] = FENGHAI_CMD_RADAR_FILTER_CONFIG;
            break;

        case 0x206:
            packetFrame[6] = FENGHAI_CMD_RADAR_REGION_CONFIG;
            break;

        default:
            break;
    }

    memcpy(&packetFrame[8], data, dataLen);
    crc16 = CRC16_MSB1021((unsigned char *)&packetFrame[4], dataLen + 4);
    packetFrame[dataLen + 8] = crc16 >> 8;
    packetFrame[dataLen + 9] = crc16 & 0xff;
    packetFrame[dataLen + 10] = 0xEA;
    packetFrame[dataLen + 11] = 0xEB;
    packetFrame[dataLen + 12] = 0xEC;
    packetFrame[dataLen + 13] = 0xED;

    radar_response_send((const void *)&packetFrame, dataLen + 14);
}

void getDevId(uint8_t *devId)
{
    if (!got_serial) {
        if (get_serial_number(serial_buf, sizeof(serial_buf)) == 0) {
            got_serial = true;
        }
    }

    if (got_serial && devId) {
        memcpy(devId, serial_buf, 20);
    }
}

void getDevId_cluster(struct FH_DEV_ID_CLUSTER *devId)
{
    if (!got_serial) {
        if (get_serial_number(serial_buf, sizeof(serial_buf)) == 0) {
            got_serial = true;
        }
    }

    if (got_serial && devId) {
        memcpy(&devId->dev[0], serial_buf, sizeof(devId->dev));
    }
}

/* 管理坐标系相关的一些参数转换 */
void manager_output_parameter(OffsetAdjustParams *origin, OffsetAdjustParams *result)
{
    float xOffset = gOffsetParam.xOffset;
    float yOffset = gOffsetParam.yOffset;
    bool flipXY = gOffsetParam.flipXY != 0;
    bool yUporLeftPostive = gOffsetParam.yDirect == 0;
    bool xRightorUpPostive = gOffsetParam.xDirect == 0;

    /* 雷达坐标系：向前为X的正方向，向左为Y的正方向 */

    // 偏移量坐标系：X横轴（左正右负），Y纵轴（上正下负）
    // 偏移量的坐标：可以认为应安装点位在当前雷达安装点位的相对坐标
    // （例如，雷达应安装坐标在当前雷达位置的右前方分别1m，则偏移量可以设置(xOffset = 1, yOffset = 1)）
    origin->x -= yOffset;
    origin->y += xOffset;

    if (flipXY) {
        /* 需要翻转X和Y */
        if (xRightorUpPostive) {
            /* X轴向右为正 - X轴向左为负 */
            result->x = -origin->y;
            result->sx = -origin->sy;
            result->ax = -origin->ay;
        } else {
            /* X轴向左为正 - X轴向右为负 */
            result->x = origin->y;
            result->sx = origin->sy;
            result->ax = origin->ay;
        }
        result->s = origin->sx;

        if (yUporLeftPostive) {
            /* Y轴向前为正 - Y轴向后为负 */
            result->y = origin->x;
            result->sy = origin->sx;
            result->ay = origin->ax;
        } else {
            /* Y轴向后为正 - Y轴向前为负 */
            result->y = -origin->x;
            result->sy = -origin->sx;
            result->ay = -origin->ax;
        }
    } else {
        /* 不需要翻转X和Y */
        if (xRightorUpPostive) {
            /* X轴向前为正 - X轴向后为负 */
            result->x = origin->x;
            result->sx = origin->sx;
            result->ax = origin->ax;
            result->s = result->sx;
        } else {
            /* X轴向后为正 - X轴向前为负 */
            result->x = -origin->x;
            result->sx = -origin->sx;
            result->ax = -origin->ax;
        }
        result->s = result->sx;

        if (yUporLeftPostive) {
            /* Y轴向左为正 - Y轴向右为负 */
            result->y = origin->y;
            result->sy = origin->sy;
            result->ay = origin->ay;
        } else {
            /* Y轴向右为正 - Y轴向左为负 */
            result->y = -origin->y;
            result->sy = -origin->sy;
            result->ay = -origin->ay;
        }
    }
}

// 雷达获取的目标数据转为要输出的数据
void getObject(ARS408RadarObjectInfo *objectData, FH_OBJ_DATA *data, int id, trajectory_data_t *trajectory)
{
    float object_height = 1.6f;
    OffsetAdjustParams params, origin;
    ARS408RadarObjectInfo radar_object;

    memset(&params, 0, sizeof(params));
    memset(&origin, 0, sizeof(origin));
    memset(&radar_object, 0, sizeof(radar_object));

    radar_object = objectData[id];
    origin.x  = params.x  = radar_object.Object_DistLong;   /* X坐标 */
    origin.y  = params.y  = radar_object.Object_DistLat;    /* Y坐标 */
    origin.s  = params.s  = radar_object.Object_VrelLat;    /* 速度 */
    origin.sx = params.sx = radar_object.Object_VrelLong;   /* X轴速度 */
    origin.sy = params.sy = radar_object.Object_VrelLat;    /* Y轴速度 */
    origin.a  = params.a  = radar_object.Object_ArelLat;    /* 加速度 */
    origin.ax = params.ax = radar_object.Object_ArelLong;   /* X轴加速度 */
    origin.ay = params.ay = radar_object.Object_ArelLat;    /* Y轴加速度 */
    manager_output_parameter(&origin, &params);

    radar_object.Object_Width = 1.8f;
    if (radar_object.Object_Class == OBJECT_SmallCar) {
        radar_object.Object_Length = 4.8f;
    } else if (radar_object.Object_Class == OBJECT_MediumCar) {
        object_height = 4.0f;
        radar_object.Object_Length = 9.8f;
    } else if (radar_object.Object_Class == OBJECT_BigCar) {
        object_height = 4.0f;
        radar_object.Object_Length = 20.0f;
    }

    uint64_t ts = (uint64_t)(radar_object.Object_timestamp);
    uint32_t detect_sec = (uint32_t)(ts / 1000);
    uint16_t detect_msec = (uint16_t)(ts % 1000);

    data->second            = detect_sec;
    data->millisecond       = detect_msec;
    // data->second            = (UINT)radar_object.Object_timestamp;                           // 待添加时间戳功能
    // data->millisecond       = (UINT)((radar_object.Object_timestamp - data->second) * 1000); // 待添加时间戳功能
    data->id                = radar_object.Object_ID;                                        // 目标编号 循环 ID 分配 取值：0~65535
    data->orientation_angel = radar_object.Object_OrientationAngel * 10;                     // 目标偏航角
    data->object_class      = radar_object.Object_Class;                                     // 目标类型
    data->obj_length        = (UINT)(radar_object.Object_Length * 100);                      // 目标长度
    data->obj_width         = (UINT)(radar_object.Object_Width * 100);                       // 目标宽度
    data->x                 = params.x * 100;                                                // X 坐标
    data->y                 = params.y * 100;                                                // Y 坐标
    data->x_speed           = params.sx * 100;                                               // X 轴速度 m/s
    data->y_speed           = params.sy * 100;                                               // Y 轴速度 m/s
    data->speed             = abs(params.s * 100 * 3.6);                                     // 车速 km/h
    data->a_speed           = params.a * 100;                                                // 目标运动方向加速度
    data->a_x_speed         = params.ax * 100;                                               // X 轴加速度
    data->a_y_speed         = params.ay * 100;                                               // Y 轴加速度
    data->Object_RCS        = radar_object.Object_RCS;
    data->event             = radar_object.Object_Event;                                     // 事件类型
    data->lane              = radar_object.Object_Lane;                                      // 车道
    data->Object_Latitude   = radar_object.Object_Latitude;
    data->Object_Longitude  = radar_object.Object_Longitude;
    data->Object_Altitude   = radar_object.Object_Altitude;

    uint8_t direction = 0;                                                                   /* 0x01: 来向，0x02: 去向 */
    for (auto laneItem : gRoadLaneParam) {
        if (laneItem.first == data->lane) {
            direction = laneItem.second.dire;
            break;
        }
    }

    if (direction == 0) {
        direction = origin.sx < 0 ? 0x01 : origin.sx > 0 ? 0x02 : 0x00;
    }

    if (trajectory != NULL) {
        trajectory->detect_sec       = htonl(detect_sec);
        trajectory->detect_msec      = htons(detect_msec);
        trajectory->object_id        = htons(data->id);
        trajectory->event_type       = htonl(radar_object.Object_Event);
        trajectory->object_lane      = (uint8_t)data->lane;
        trajectory->object_type      = (uint8_t)data->object_class;
        trajectory->direction        = (uint8_t)(direction);
        trajectory->object_length    = (uint8_t)(data->obj_length / 10);
        trajectory->object_width     = (uint8_t)(data->obj_width / 10);
        trajectory->object_height    = (uint8_t)(object_height * 10);
        trajectory->object_rcs       = (int16_t)htons((data->Object_RCS * 10));
        trajectory->object_yaw_angle = htons(data->orientation_angel);
        trajectory->object_x_coord   = (int32_t)htonl(data->x);
        trajectory->object_y_coord   = (int32_t)htonl(data->y);
        trajectory->object_x_speed   = (int16_t)htons(data->x_speed);
        trajectory->object_y_speed   = (int16_t)htons(data->y_speed);
        trajectory->object_speed     = (uint16_t)htons(abs(params.s * 100 * 3.6));
        trajectory->object_accel     = (int16_t)htons(data->a_speed);
        trajectory->object_x_accel   = (int16_t)htons(data->a_x_speed);
        trajectory->object_y_accel   = (int16_t)htons(data->a_y_speed);
        trajectory->object_coord_lon = (int32_t)htonl((data->Object_Longitude * 10000000));
        trajectory->object_coord_lat = (int32_t)htonl((data->Object_Latitude * 10000000));
    }
}

int construct_radar_cluster_frame(FH_CLUSTER *frame, RadarClusterInfoARS *clusterData, uint8_t count)
{
#if defined(CONFIG_SENDACTUALCLUSTER)
    int offset = 0;
#endif

    frame->head = 0xDDFFAA55;
    frame->type = 2;
    getDevId_cluster(&frame->dev_id);
    frame->length = count;
    g_actualCurClusterCnt = 0;
    g_actualCurClusterCnt_702 = 0;
    g_clustTstpBgnFlg = 0;

    int i;
    std::vector<radar_cluster_t> radarClusterMap{};

    for (i = 0; i < frame->length; i++) {
        radar_cluster_t cluster;
        memset(&cluster, 0, sizeof(cluster));

        uint64_t ts = (uint64_t)(clusterData[i].Cluster_timestamp);
        uint32_t detect_sec = (uint32_t)(ts / 1000);
        uint16_t detect_msec = (uint16_t)(ts % 1000);

        cluster.detect_sec      = htonl(detect_sec);
        cluster.detect_msec     = htons(detect_msec);
        // cluster.detect_sec      = htonl(clusterData[i].Cluster_timestamp);
        // cluster.detect_msec     = htons((clusterData[i].Cluster_timestamp - cluster.detect_sec) * 1000);
        cluster.object_id       = htons(clusterData[i].Cluster_ID);
        cluster.object_x_coord  = (int32_t)htonl(clusterData[i].Cluster_DistLong * 100);
        cluster.object_y_coord  = (int32_t)htonl(clusterData[i].Cluster_DistLat * 100);
        cluster.object_x_speed  = (int16_t)htons(clusterData[i].Cluster_VrelLong * 100);
        cluster.object_y_speed  = (int16_t)htons(clusterData[i].Cluster_VrelLat * 100);
        cluster.cluster_dynprop = htons(clusterData[i].Cluster_DynProp);
        cluster.object_rcs      = (int16_t)htons(clusterData[i].Cluster_RCS * 10);
        cluster.x_rms           = htons(clusterData[i].Cluster_DistLong_rms);
        cluster.y_rms           = htons(clusterData[i].Cluster_DistLat_rms);
        cluster.vx_rms          = htons(clusterData[i].Cluster_VrelLong_rms);
        cluster.vy_rms          = htons(clusterData[i].Cluster_VrelLat_rms);
        cluster.pdh0            = htons(clusterData[i].Cluster_Pdh0);
        cluster.ambig_state     = htons(clusterData[i].Cluster_AmbigState);
        cluster.invalid_state   = htons(clusterData[i].Cluster_InvalidState);
        radarClusterMap.emplace_back(std::move(cluster));

        memcpy(&frame->Data[i], &clusterData[i], sizeof(RadarClusterInfoARS));
    }

    if (tcp_server_clients(1502) > 0) {
        if (radarClusterMap.size() > 0) {
            frameQueue.emplace_back(std::make_tuple(RADAR_CLUSTER, std::make_shared<linb::any>(std::move(radarClusterMap))));
        }
    }

    // frame->time = GetLocalTimeWithMs_cluster(frame->time);
    GetLocalTimeWithMsEx_cluster(&(frame->time));
    frame->crc = getCRC_cluster(frame);
    frame->end = 0xEAEBECED;

#if defined(CONFIG_SENDACTUALCLUSTER)
    offset = sizeof(frame->head) + sizeof(frame->length) + sizeof(frame->type) + sizeof(frame->dev_id) + sizeof(frame->time) + sizeof(RadarClusterInfoARS) * (frame->length);
    memcpy((char *)frame + offset, &(frame->crc), sizeof(frame->crc));
    offset += sizeof(frame->crc);
    memcpy((char *)frame + offset, &(frame->end), sizeof(frame->end));
    offset += sizeof(frame->end);
    return offset;
#else
    return 0;
#endif
}

int construct_radar_object_frame(FH_OBJ *frame, ARS408RadarObjectInfo *objectData, uint8_t objects)
{
    frame->head = 0xDDFFAA55;
    frame->type = 1;
    getDevId(frame->dev_id.dev);
    frame->length = objects;

    g_actualCurClusterCnt = 0;
    g_actualCurClusterCnt_702 = 0;
    g_clustTstpBgnFlg = 0;

    std::vector<trajectory_data_t> radarObjMap{};
    std::vector<traffic_event_t> radarObjEvent{};

    for (int i = 0; i < frame->length; i++) {
        trajectory_data_t trajectory;
        memset(&trajectory, 0, sizeof(trajectory_data_t));

        getObject(objectData, &frame->Data[i], i, &trajectory);

        uint32_t ev = frame->Data[i].event;
        if (ev != 0) {
            traffic_event_t event;
            event.event_id    = htons(frame->Data[i].id);
            event.event_type  = htonl(ev);
            event.direction   = trajectory.direction;
            event.object_type = frame->Data[i].object_class;
            event.lane_id     = frame->Data[i].lane;
            event.longitude   = (int32_t)htonl(frame->Data[i].Object_Longitude * 10000000);
            event.latitude    = (int32_t)htonl(frame->Data[i].Object_Latitude * 10000000);
            radarObjEvent.emplace_back(std::move(event));
        }

        radarObjMap.emplace_back(std::move(trajectory));
    }

    if (tcp_server_clients(1502) > 0) {
        if (radarObjMap.size() > 0) {
            frameQueue.emplace_back(std::make_tuple(TRAJECTORY_DATA, std::make_shared<linb::any>(std::move(radarObjMap))));
        }

        if (radarObjEvent.size() > 0) {
            frameQueue.emplace_back(std::make_tuple(TRAFFIC_EVENT, std::make_shared<linb::any>(std::move(radarObjEvent))));
        }
    }

    // frame->time = GetLocalTimeWithMs(frame->time);
    GetLocalTimeWithMsEx(&(frame->time));
    frame->crc = getCRC(frame);
    frame->end = 0xEAEBECED;

    if (tcp_server_clients(1500) > 0) {
        tcp_server_send(1500, (const char *)frame, sizeof(FH_OBJ));
    }

    return objects;
}

void GetLocalTimeWithMsEx(FH_TIME *fh_time)
{
    if (fh_time != NULL) {
        /* 获取当前系统时间的毫秒级时间戳 */
        uint64_t milliseconds = getCurrentTimestamp(1);

        /* 将毫秒级时间戳转换为时间结构 */
        time_t seconds = milliseconds / 1000;
        /* 使用gmtime获取UTC时间 */
        std::tm *now_tm = std::gmtime(&seconds);

        fh_time->year  = now_tm->tm_year + 1900 - 2000;
        fh_time->month = now_tm->tm_mon + 1;
        fh_time->day   = now_tm->tm_mday;
        fh_time->hour  = now_tm->tm_hour;
        fh_time->min   = now_tm->tm_min;
        fh_time->sec   = now_tm->tm_sec;
        fh_time->ms    = milliseconds % 1000;
    }
}

void GetLocalTimeWithMsEx_cluster(struct FH_TIME_CLUSTER *fh_time)
{
    if (fh_time != NULL) {
        /* 获取当前系统时间的毫秒级时间戳 */
        uint64_t milliseconds = getCurrentTimestamp(1);

        /* 将毫秒级时间戳转换为时间结构 */
        time_t seconds = milliseconds / 1000;
        /* 使用gmtime获取UTC时间 */
        std::tm *now_tm = std::gmtime(&seconds);

        fh_time->year  = now_tm->tm_year + 1900 - 2000;
        fh_time->month = now_tm->tm_mon + 1;
        fh_time->day   = now_tm->tm_mday;
        fh_time->hour  = now_tm->tm_hour;
        fh_time->min   = now_tm->tm_min;
        fh_time->sec   = now_tm->tm_sec;
        fh_time->ms    = milliseconds % 1000;
    }
}

// 获取本地时间
FH_TIME GetLocalTimeWithMs(FH_TIME fh_time)
{
    struct timeval curTime;
    gettimeofday(&curTime, NULL);

    int milli = curTime.tv_usec / 1000;
    struct tm timeNow;
    localtime_r(&curTime.tv_sec, &timeNow);

    fh_time.year = timeNow.tm_year + 1900 - 2000;
    fh_time.month = timeNow.tm_mon + 1;
    fh_time.day = timeNow.tm_mday;
    fh_time.hour = timeNow.tm_hour;
    fh_time.min = timeNow.tm_min;
    fh_time.sec = timeNow.tm_sec;
    fh_time.ms = milli;

    return fh_time;
}

struct FH_TIME_CLUSTER GetLocalTimeWithMs_cluster(struct FH_TIME_CLUSTER fh_time)
{
    struct timeval curTime;
    gettimeofday(&curTime, NULL);

    int milli = curTime.tv_usec / 1000;
    struct tm timeNow;
    localtime_r(&curTime.tv_sec, &timeNow);

    fh_time.year = timeNow.tm_year + 1900 - 2000;
    fh_time.month = timeNow.tm_mon + 1;
    fh_time.day = timeNow.tm_mday;
    fh_time.hour = timeNow.tm_hour;
    fh_time.min = timeNow.tm_min;
    fh_time.sec = timeNow.tm_sec;
    fh_time.ms = milli;

    return fh_time;
}

void *closeDB(void *frame)
{
    pthread_detach(pthread_self());

    pthread_setname_np(pthread_self(), "closeDB");

    while (insertCount > 0) {
        sleep(1);
    }

    if (record_db) {
        sqlite3_close(record_db);
        record_db = NULL;
    }
}

/**
 * 功能：向表中插入一帧数据
 * 输入参数：表名、数据库、帧数据、标志位、视频标志位
 * 返回：成功或失败
 */
void insertTable(const char *tableName, FH_OBJ frame, int flag, int recordFlag, sqlite3 *dtb)
{
    int i;
    int ret;
    struct tm tm;
    char sql[1024];
    struct timeval t;
    char *zErrMsg = NULL;
    char *zErrorMsg = NULL;
    unsigned char timeStr[64];

    if ((tableName == NULL) || (dtb == NULL)) {
        return;
    }

    getTimeStr(&frame, timeStr);
    gettimeofday(&t, NULL);
    localtime_r(&t.tv_sec, &tm);

    int preMilli = t.tv_usec / 1000 + tm.tm_sec * 1000;
    ret = sqlite3_exec(dtb, "begin transaction", 0, 0, &zErrorMsg);
    if (ret != SQLITE_OK) {
        sqlite3_free(zErrorMsg);
    }

    for (i = 0; i < frame.length; i++) {
        FH_OBJ_DATA data = frame.Data[i];
        sprintf(sql,
                "insert into %s values(NULL, %d, %d, %d, %d, %d, %d, %d, %d, %f, "
                "%f, %d, %d, %d, %d, %d, %d, %d, %d, '%s')",
                tableName, data.id, flag, recordFlag, data.road, data.object_class,
                data.obj_length, data.obj_width, data.orientation_angel,
                (float)data.x / 100, (float)data.y / 100, data.x_speed,
                data.y_speed, data.speed, data.a_speed, data.a_x_speed,
                data.a_y_speed, data.longitude, data.latitude, timeStr);

        ret = sqlite3_exec(dtb, (const char *)sql, 0, 0, &zErrMsg);
        if (ret != SQLITE_OK) {
            sqlite3_free(zErrMsg);
        }
    }

    ret = sqlite3_exec(dtb, "commit transaction", 0, 0, &zErrorMsg);
    if (ret != SQLITE_OK) {
        sqlite3_free(zErrorMsg);
    }
}

unsigned char *getTimeStr(FH_OBJ *frame, unsigned char timeStr[])
{
    FH_OBJ sendFrame = *(FH_OBJ *)frame;
    FH_TIME time = sendFrame.time;
    sprintf((char *)timeStr, "%02d-%02d-%02d %02d:%02d:%02d:%03d", time.year, time.month, time.day, time.hour, time.min, time.sec, time.ms);

    return timeStr;
}

/**
 * 功能：返回表中recordFlag最大值
 * 输入参数：表名、数据库
 */
int findMaxRecordFlag(const char *tableName, sqlite3 *dtb)
{
    int rc;
    char sql[256];
    char *zErrMsg = NULL;
    char **azResult = NULL;
    int nrow = 0, ncolumn = 0, index = 0;

    sprintf(sql, "SELECT max(record_flag) from %s", tableName);

    rc = sqlite3_get_table(dtb, sql, &azResult, &nrow, &ncolumn, &zErrMsg);
    index = ncolumn;
    if (rc != SQLITE_OK) {
        sqlite3_free(zErrMsg);
    }

    char *res = azResult != NULL ? azResult[ncolumn] : NULL;
    int ans = res == NULL ? 0 : *res - '0';

    if (azResult != NULL) {
        sqlite3_free_table(azResult);
    }

    return ans;
}

void *openOrInsertDB(void *frame)
{
    insertCount++;

    pthread_detach(pthread_self());

    pthread_setname_np(pthread_self(), "openOrInsertDB");

    FH_OBJ sendFrame = *(FH_OBJ *)frame;
    if (needRecord && !preNeedRecord) {
        preNeedRecord = needRecord;

        spdlog::info("record starting ......");

        int rc = sqlite3_open(fileName, &record_db);
        if (rc) {
            spdlog::error("can't open database:[{0}], errstr:[{1}]", fileName, sqlite3_errmsg(record_db));
            return NULL;
        } else {
            spdlog::error("open database:[{0}] successfully", fileName);
        }

        // 找到当前最大录制次数
        recordMaxFlag = findMaxRecordFlag(tableName, record_db);
        recordMaxFlag++;
        frameFlag = 1;
        insertTable(tableName, sendFrame, frameFlag++, recordMaxFlag, record_db);
    } else if (needRecord && preNeedRecord) {
        insertTable(tableName, sendFrame, frameFlag++, recordMaxFlag, record_db);
    }

    insertCount--;
}

void dbOperate(FH_OBJ *frame)
{
    if (!needRecord) {
        if (!needRecord && preNeedRecord) {
            preNeedRecord = needRecord;

            pthread_t id;
            pthread_create(&id, NULL, closeDB, NULL);
        }
    } else {
        pthread_t id;
        pthread_create(&id, NULL, openOrInsertDB, (void *)frame);
    }
}

bool isFHCLUSTER(FH_CLUSTER cluster, FH_CLUSTER preCluster)
{
    if (memcmp(&cluster, &preCluster, sizeof(FH_CLUSTER)) != 0) {
        return false;
    }

    return true;
}

bool isFHOBJ(FH_OBJ frame, FH_OBJ preFrame)
{
    if (frame.length != preFrame.length) {
        return false;
    }

    int i;
    for (i = 0; i < frame.length; i++) {
        if (frame.Data[i].x != preFrame.Data[i].x) {
            return false;
        }

        if (frame.Data[i].y != preFrame.Data[i].y) {
            return false;
        }
    }

    return true;
}

int getCRC_cluster(FH_CLUSTER *frame)
{
    int len;
    unsigned char *p_char = (unsigned char *)&frame->length;

#if !defined(CONFIG_SENDACTUALCLUSTER)
    len = sizeof(frame->length) + sizeof(frame->type) + sizeof(frame->dev_id) + sizeof(frame->time) + sizeof(frame->Data);
#else
    len = sizeof(frame->length) + sizeof(frame->type) + sizeof(frame->dev_id) + sizeof(frame->time) + sizeof(RadarClusterInfoARS) * (frame->length);
#endif

    return CRC16_MSB1021(p_char, len);
}

int getCRC(FH_OBJ *frame)
{
    unsigned char *p_char = (unsigned char *)&frame->length;
    int len = sizeof(frame->length) + sizeof(frame->type) + sizeof(frame->dev_id) + sizeof(frame->time) + sizeof(frame->Data);
    return CRC16_MSB1021(p_char, len);
}

int dealRadarConfigData(char dataType, char *dataBuf, int dataLen)
{
    int i;
    int ret;
    struct can_frame frame = {0};

    if (dataLen > 8) {
        spdlog::error("incorrect effective data length:[{0}] greater than 8", dataLen);
        return -1;
    }

    switch (dataType) {
        case FENGHAI_CMD_RADAR_BASIC_CONFIG:
            frame.can_id = 0x0200;
            break;

        case FENGHAI_CMD_RADAR_FILTER_CONFIG:
            frame.can_id = 0x0202;
            break;

        case FENGHAI_CMD_RADAR_REGION_CONFIG:
            frame.can_id = 0x0205;
            break;

        default:
            break;
    }

    memcpy(frame.data, dataBuf, dataLen);
    frame.can_dlc = dataLen;
    ret = write(Canfd, &frame, sizeof(frame));
    if (sizeof(frame) != ret) {
        spdlog::error("can write failed, return:[{0}], errstr:[{1}]", ret, strerror(errno));
        return -1;
    }

    printf("can frame:[%d]->[", frame.can_dlc);
    for (i = 0; i < dataLen; i++) {
        if (i == (dataLen - 1)) {
            printf("%02X", frame.data[i]);
        } else {
            printf("%02X ", frame.data[i]);
        }
    }
    printf("]\n");

    return 0;
}

int decodeUpperComputerData(char *receiveData, int receiveNum)
{
    S_FHCOMMFORM receiveFrame;
    unsigned short crc16, Rxcrc16;

    // 检查数据长度
    if (receiveNum < 14) {
        spdlog::error("incorrect data length:[{0}] less than 14 bytes", receiveNum);
        return -1;
    }

    // 检查帧头
    if (receiveData[0] != 0xDD && receiveData[1] != 0xFF && receiveData[2] != 0xAA && receiveData[3] != 0x55) {
        spdlog::error("frame header error");
        return -1;
    }

    // 检查帧尾
    if (receiveData[receiveNum - 4] != 0xEA &&
        receiveData[receiveNum - 3] != 0xEB &&
        receiveData[receiveNum - 2] != 0xEC &&
        receiveData[receiveNum - 1] != 0xED)
    {
        spdlog::error("end of frame error");
        return -1;
    }

    // 检查CRC16
    crc16 = CRC16_MSB1021((unsigned char *)&receiveData[4], receiveNum - 10);
    Rxcrc16 = receiveData[receiveNum - 6] << 8 | receiveData[receiveNum - 5];  // 低位在前，高位在后
    if (crc16 != Rxcrc16) {
        spdlog::error("invalid CRC16, calc:[0x{:04X}], recv:[0x{:04X}]", crc16, Rxcrc16);
        return -1;
    }

    // 将接收数据转换为丰海格式
    receiveFrame.length = (receiveData[4] << 8 | receiveData[5]);

    receiveFrame.type = receiveData[6];
    receiveFrame.dev_id = receiveData[7];
    memcpy(&receiveFrame.data, &receiveData[8], receiveNum - 14);

    deviceID = receiveFrame.dev_id;
    switch (receiveFrame.type) {
        case FENGHAI_CMD_RADAR_BASIC_CONFIG:
            dealRadarConfigData(receiveFrame.type, receiveFrame.data, receiveFrame.length);
            respondRadarBasicConfig = true;
            break;

        case FENGHAI_CMD_RADAR_FILTER_CONFIG:
        case FENGHAI_CMD_RADAR_REGION_CONFIG:
            dealRadarConfigData(receiveFrame.type, receiveFrame.data, receiveFrame.length);
            break;

        default:
            break;
    }

    return 0;
}

void radar_upper_computer_config(char *data, int bytes)
{
    decodeUpperComputerData(data, bytes);
}

int user_readClusterAndObjectSend2Rearend_func(void)
{
    FILE *fp = fopen("/home/root/clusterAndObjectSend2Rearend", "r");
    if (fp == NULL) {
        spdlog::error("open /home/root/clusterAndObjectSend2Rearend failed");
        return -1;
    }

    char u8Line[128] = {0};
    while (fgets(u8Line, sizeof(u8Line), fp) != NULL) {
        g_clusterAndObjectSend2RearendMode = atoi(u8Line);
    }

    fclose(fp);
    return 0;
}

int user_writeClusterAndObjectSend2Rearend_func(void)
{
    FILE *fp = fopen("/home/root/clusterAndObjectSend2Rearend", "w");
    if (fp == NULL) {
        spdlog::error("open /home/root/clusterAndObjectSend2Rearend failed");
        return -1;
    }

    char u8Line[128] = {0};
    snprintf(u8Line, sizeof(u8Line), "%d\n", g_clusterAndObjectSend2RearendMode);
    fwrite(u8Line, 1, strlen(u8Line), fp);
    fclose(fp);

    return 0;
}

int user_readDynamicOrStaticCalibrationMode_func(void)
{
    FILE *fp = fopen("/home/root/dynamicOrStaticCalibrationMode", "r");
    if (fp == NULL) {
        spdlog::error("open /home/root/dynamicOrStaticCalibrationMode failed");
        return -1;
    }

    char u8Line[128] = {0};
    while (fgets(u8Line, sizeof(u8Line), fp) != NULL) {
        g_dynamicOrStaticCalibrationMode = atoi(u8Line);
    }

    fclose(fp);
    return 0;
}

int user_writeDynamicOrStaticCalibrationMode_func(void)
{
    FILE *fp = fopen("/home/root/dynamicOrStaticCalibrationMode", "w");
    if (fp == NULL) {
        spdlog::error("open /home/root/dynamicOrStaticCalibrationMode failed");
        return -1;
    }

    char u8Line[128] = {0};
    snprintf(u8Line, sizeof(u8Line), "%d\n", g_dynamicOrStaticCalibrationMode);
    fwrite(u8Line, 1, strlen(u8Line), fp);
    fclose(fp);

    return 0;
}

void usr_copyRadarObjectData(ARS408RadarObjectInfo radar_objdata[], int count)
{
    int i;
    for (i = 0; i < count; ++i) {
        RadarObjectData_Wr[i].Object_timestamp             = radar_objdata[i].Object_timestamp;
        RadarObjectData_Wr[i].Object_ID                    = radar_objdata[i].Object_ID;
        RadarObjectData_Wr[i].Object_DistLong              = radar_objdata[i].Object_DistLong;
        RadarObjectData_Wr[i].Object_DistLat               = radar_objdata[i].Object_DistLat;
        RadarObjectData_Wr[i].Object_DistAlt               = radar_objdata[i].Object_DistAlt;
        RadarObjectData_Wr[i].Object_VrelLong              = radar_objdata[i].Object_VrelLong;
        RadarObjectData_Wr[i].Object_VrelLat               = radar_objdata[i].Object_VrelLat;
        RadarObjectData_Wr[i].Object_VrelAlt               = radar_objdata[i].Object_VrelAlt;
        RadarObjectData_Wr[i].Object_DynProp               = radar_objdata[i].Object_DynProp;
        RadarObjectData_Wr[i].Object_RCS                   = radar_objdata[i].Object_RCS;
        RadarObjectData_Wr[i].Obj_DistLong_rms             = radar_objdata[i].Obj_DistLong_rms;
        RadarObjectData_Wr[i].Obj_DistLat_rms              = radar_objdata[i].Obj_DistLat_rms;
        RadarObjectData_Wr[i].Obj_DistAlt_rms              = radar_objdata[i].Obj_DistAlt_rms;
        RadarObjectData_Wr[i].Obj_VrelLong_rms             = radar_objdata[i].Obj_VrelLong_rms;
        RadarObjectData_Wr[i].Obj_VrelLat_rms              = radar_objdata[i].Obj_VrelLat_rms;
        RadarObjectData_Wr[i].Obj_VrelAlt_rms              = radar_objdata[i].Obj_VrelAlt_rms;
        RadarObjectData_Wr[i].Obj_ArelLong_rms             = radar_objdata[i].Obj_ArelLong_rms;
        RadarObjectData_Wr[i].Obj_ArelLat_rms              = radar_objdata[i].Obj_ArelLat_rms;
        RadarObjectData_Wr[i].Obj_ArelAlt_rms              = radar_objdata[i].Obj_ArelAlt_rms;
        RadarObjectData_Wr[i].Obj_Orientation_rms          = radar_objdata[i].Obj_Orientation_rms;
        RadarObjectData_Wr[i].Obj_MeasState                = radar_objdata[i].Obj_MeasState;
        RadarObjectData_Wr[i].Obj_ProbOfExist              = radar_objdata[i].Obj_ProbOfExist;
        RadarObjectData_Wr[i].Object_ArelLong              = radar_objdata[i].Object_ArelLong;
        RadarObjectData_Wr[i].Object_ArelLat               = radar_objdata[i].Object_ArelLat;
        RadarObjectData_Wr[i].Object_ArelAlt               = radar_objdata[i].Object_ArelAlt;
        RadarObjectData_Wr[i].Object_Class                 = radar_objdata[i].Object_Class;
        RadarObjectData_Wr[i].Object_OrientationAngel      = radar_objdata[i].Object_OrientationAngel;
        RadarObjectData_Wr[i].Object_Length                = radar_objdata[i].Object_Length;
        RadarObjectData_Wr[i].Object_Width                 = radar_objdata[i].Object_Width;
        RadarObjectData_Wr[i].Object_CollDetRegionBitfield = radar_objdata[i].Object_CollDetRegionBitfield;
        RadarObjectData_Wr[i].Object_Lane                  = radar_objdata[i].Object_Lane;
        RadarObjectData_Wr[i].Object_Event                 = radar_objdata[i].Object_Event;
    }
}

/* 接收CAN数据放到缓冲区 */
void *usr_readCanData(void *arg)
{
    int ret;
    int epFd;
    int nbytes;
    int errcnt = 0;
    int lockFlg = 0;
    int fdFlags = 0;
    struct ifreq ifr;
    char lostDataCnt = 0;
    struct timespec sttmspc;
    struct sockaddr_can addr;
    unsigned long long int temp;

    system("ifconfig can0 down");
    usleep(200000);
    system("ip link set can0 up type can bitrate 500000 triple-sampling off");
    usleep(200000);
    system("ifconfig can0 up");
    usleep(400000);

    Canfd = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (Canfd < 0) {
        spdlog::error("socket create failed, errstr:[{0}]", strerror(errno));
        exit(-EXIT_FAILURE);
    }

    strcpy(ifr.ifr_name, "can0");
    ioctl(Canfd, SIOCGIFINDEX, &ifr);
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    if (bind(Canfd, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        close(Canfd);
        spdlog::error("can bind failed, errstr:[{0}]", strerror(errno));
        exit(-EXIT_FAILURE);
    }

    epFd = epoll_create1(EPOLL_CLOEXEC);
    if (epFd < 0) {
        close(Canfd);
        spdlog::error("can epoll create1 failed, errstr:[{0}]", strerror(errno));
        exit(-EXIT_FAILURE);
    }

    struct epoll_event event;
    event.events = EPOLLIN;
    event.data.fd = Canfd;

    ret = epoll_ctl(epFd, EPOLL_CTL_ADD, Canfd, &event);
    if (ret != 0) {
        close(epFd);
        close(Canfd);
        spdlog::error("can epoll_ctl add failed, errstr:[{0}]", strerror(errno));
        exit(-EXIT_FAILURE);
    }

    pthread_setname_np(pthread_self(), "readCanData");

    spdlog::info("waiting for can data now ......");

    struct epoll_event events[1];
    while (1) {
        ret = epoll_wait(epFd, events, 1, -1);
        if (ret > 0) {
#if !defined(CONFIG_CANFRAME)
            if (lockFlg == 0) {
                lockFlg = 1;
                pthread_rwlock_wrlock(&g_canBuf.mtxCanBuf);
            }
#endif
            if (events[0].events & EPOLLIN) {
                struct can_frame oneFrame = {0};
                nbytes = read(Canfd, &oneFrame, sizeof(oneFrame));
                if (nbytes < 0) {
                    spdlog::error("can read failed, errstr:[{0}]", strerror(errno));
                    if (++errcnt > 10) {
                        errcnt = 0;
                        close(epFd);
                        close(Canfd);
                        exit(-EXIT_FAILURE);
                    }
                }

#if defined(CONFIG_CANFRAME)
                {
                    can_frame_t frame;
                    frame.ts = getCurrentTimestamp(1);
                    memcpy(&frame.frame, &oneFrame, sizeof(frame));

                    std::unique_lock<std::mutex> lock(mCanFrameMutex);
                    mCanFrameQueue.push_back(std::move(frame));
                    mCanFrameCond.notify_one();
                }
#else
                memcpy(&g_canBuf.canFrmBuf[g_canBuf.wrIdx], &oneFrame, sizeof(oneFrame));

                if ((g_canTstpBgnFlg == 0) && (oneFrame.can_id == 0x701)) {
                    g_canTstpBgnFlg = 1;
                    // clock_gettime(CLOCK_REALTIME, &sttmspc);
                    // temp = (unsigned long long int)sttmspc.tv_sec * 1000 + (unsigned long long int)sttmspc.tv_nsec / 1000000;
                    // g_canBuf.canTime[g_canBuf.wrIdx] = getCurrentTimestamp(1) * 1.0;
                } else if (oneFrame.can_id == 0x600) {
                    g_canTstpBgnFlg = 0;
                    g_canBuf.canTime[g_canBuf.wrIdx] = getCurrentTimestamp(1) * 1.0;
                }

                g_canBuf.wrIdx++;
                if (g_canBuf.wrIdx >= USER_MAX_CANBUF) {
                    g_canBuf.wrIdx = 0;
                }

                g_canBuf.cnt++;
                if (g_canBuf.cnt >= USER_MAX_CANBUF) {
                    g_canBuf.cnt -= 100;
                }
#endif
            }

#if !defined(CONFIG_CANFRAME)
            if (lockFlg == 1) {
                lockFlg = 0;
                pthread_rwlock_unlock(&g_canBuf.mtxCanBuf);
            }
#endif
        } else if (ret < 0) {
            if (errno == EINTR) {
                spdlog::warn("can got EINTR signal");
                continue;
            } else {
                close(epFd);
                close(Canfd);
                spdlog::error("can epoll_wait failed, errstr:[{0}]", strerror(errno));
                exit(-EXIT_FAILURE);
            }
        } else {
#if !defined(CONFIG_CANFRAME)
            if (lockFlg == 1) {
                lockFlg = 0;
                pthread_rwlock_unlock(&g_canBuf.mtxCanBuf);
            }
#endif
        }
    }

    close(epFd);
    close(Canfd);
    pthread_exit(NULL);
}

#if !defined(CONFIG_CANFRAME)
// 处理帧
void deal_frame(struct can_frame *frame)
{
    UINT id = frame->can_id;

    // 处理配置相关数据
    switch (id) {
        case 0x201:     /* 雷达基本状态 */
        case 0x204:     /* 雷达滤波器状态 */
        case 0x206:     /* 雷达多边形过滤器状态 */
            respondUpperComputerConfigCMD(id, (char *)frame->data, frame->can_dlc);
            break;

        case 0x600: {
            g_clustTstp = g_canBuf.canTime[g_curCanBufRdIdx];
            ars408_detect_clusters = frame->data[0] + frame->data[1];

            gotoSendCluster_CanSend = 1;
            if (saveClusterInfoFlg == 1) {
                int ret = user_writeClusterInfo(radar_cluster_list1, g_actualCurClusterCnt, clusterFp);
                if (ret < 0) {
                    saveClusterInfoFlg = 0;
                    break;
                }
            }
            break;
        }

        case 0x701: {
            // spdlog::info("0x701: g_clustTstp: {}, clusters: {}", g_clustTstp, ars408_detect_clusters);
            // (g_clustTstpBgnFlg == 0) ? (g_clustTstpBgnFlg = 1, g_clustTstp = g_canBuf.canTime[g_curCanBufRdIdx]) : (1);
            radar_cluster_list1[g_actualCurClusterCnt].Cluster_timestamp   = (double)g_clustTstp;
            radar_cluster_list1[g_actualCurClusterCnt].Cluster_ID          = frame->data[0];
            radar_cluster_list1[g_actualCurClusterCnt].Cluster_DistLong    = (((uint32_t)(frame->data[1]) << 5) + ((uint32_t)(frame->data[2]) >> 3)) * 0.2 - 500.0;
            radar_cluster_list1[g_actualCurClusterCnt].Cluster_DistLat     = ((((uint32_t)(frame->data[2]) & 0b11) << 8) + (uint32_t)(frame->data[3])) * 0.2 - 102.3;
            radar_cluster_list1[g_actualCurClusterCnt].Cluster_VrelLong    = (((uint32_t)(frame->data[4]) << 2) + ((uint32_t)(frame->data[5]) >> 6)) * 0.25 - 128.00;
            radar_cluster_list1[g_actualCurClusterCnt].Cluster_VrelLat     = ((((uint32_t)(frame->data[5]) & 0b111111) << 3) + ((uint32_t)(frame->data[6]) >> 5)) * 0.25 - 64.00;
            radar_cluster_list1[g_actualCurClusterCnt].Cluster_DynProp     = (uint32_t)(frame->data[6]) & 0b111;
            radar_cluster_list1[g_actualCurClusterCnt].Cluster_RCS         = (uint32_t)(frame->data[7]) * 0.5 - 64.00;

            g_actualCurClusterCnt += 1;
            if (g_actualCurClusterCnt >= (MAX_CLUSTER_INFO - 1)) {
                g_actualCurClusterCnt = MAX_CLUSTER_INFO - 1;
            }
            break;
        }

        case 0x702: {
            radar_cluster_list1[g_actualCurClusterCnt_702].Cluster_DistLong_rms = ((uint32_t)(frame->data[1]) >> 3);
            radar_cluster_list1[g_actualCurClusterCnt_702].Cluster_DistLat_rms  = (((uint32_t)(frame->data[1]) & 0b111) << 2) + ((uint32_t)(frame->data[2]) >> 6);
            radar_cluster_list1[g_actualCurClusterCnt_702].Cluster_VrelLong_rms = ((uint32_t)(frame->data[2]) & 0b00111110) >> 1;
            radar_cluster_list1[g_actualCurClusterCnt_702].Cluster_VrelLat_rms  = (((uint32_t)(frame->data[2]) & 0b1) << 4) + ((uint32_t)(frame->data[3]) >> 4);
            radar_cluster_list1[g_actualCurClusterCnt_702].Cluster_Pdh0         = (uint32_t)(frame->data[3]) & 0b111;
            radar_cluster_list1[g_actualCurClusterCnt_702].Cluster_AmbigState   = (uint32_t)(frame->data[4]) & 0b111;
            radar_cluster_list1[g_actualCurClusterCnt_702].Cluster_InvalidState = (uint32_t)(frame->data[4]) >> 3;

            g_actualCurClusterCnt_702 += 1;
            if (g_actualCurClusterCnt_702 >= (MAX_CLUSTER_INFO - 1)) {
                g_actualCurClusterCnt_702 = MAX_CLUSTER_INFO - 1;
            }
            break;
        }

        default:
            break;
    }
}

void *user_hdlCanFrameThread(void *arg)
{
    int sendCount;
    int sendSize = 0;
    FH_OBJ sendFrame;
    int haveDataFlg = 0;
    FH_CLUSTER sendCluster;
    struct can_frame oneFrame = {0};

    pthread_setname_np(pthread_self(), "hdlCanFrmThread");

    while (1) {
        haveDataFlg = 0;

        if (g_canBuf.cnt > 0) {
            memcpy(&oneFrame, &g_canBuf.canFrmBuf[g_canBuf.rdIdx], sizeof(struct can_frame));
            g_curCanBufRdIdx = g_canBuf.rdIdx;
            g_canBuf.rdIdx++;
            if (g_canBuf.rdIdx >= USER_MAX_CANBUF) {
                g_canBuf.rdIdx = 0;
            }

            pthread_rwlock_rdlock(&g_canBuf.mtxCanBuf);
            g_canBuf.cnt--;
            pthread_rwlock_unlock(&g_canBuf.mtxCanBuf);
            haveDataFlg = 1;
        }

        /* 取出一帧数据用于处理，解锁 */
        if (haveDataFlg == 1) {
            haveDataFlg = 0;
            deal_frame(&oneFrame);

            if (g_clusterAndObjectSend2RearendMode == 1) {  // 将原始点云数据传输到后端
                if (gotoSendCluster_CanSend == 1) {
                    gotoSendCluster_CanSend = 0;

#if defined(CONFIG_SENDACTUALCLUSTER)
                    sendSize = construct_radar_cluster_frame(&sendCluster, radar_cluster_list1, g_actualCurClusterCnt);
#else
                    construct_radar_cluster_frame(&sendCluster, radar_cluster_list1, g_actualCurClusterCnt);
#endif
                    // pthread_rwlock_rdlock(&g_canBuf.mtxCanBuf);
                    // pthread_rwlock_unlock(&g_canBuf.mtxCanBuf);

                    if (tcp_server_clients(1500) > 0) {
#if defined(CONFIG_SENDACTUALCLUSTER)
                        tcp_server_send(1500, (const void *)&sendCluster, sendSize);
#else
                        tcp_server_send(1500, (const void *)&sendCluster, sizeof(sendCluster));
#endif
                    }
                }
            } else if (g_clusterAndObjectSend2RearendMode == 0) {  // 将原始点云数据经算法处理后得到的目标数据传输到后端
                int track_count = 0;
                memset(RadarObjectData, 0, sizeof(RadarObjectData));

                if (gotoSendCluster_CanSend == 1) {
                    gotoSendCluster_CanSend = 0;

                    pthread_mutex_lock(&g_mtxTrafficFlowDetectionStatistics);
                    try {
                        struct timeval starting, finished;

                        gettimeofday(&starting, NULL);
                        Track_PointCloud_Process(&track, radar_cluster_list1, g_actualCurClusterCnt, RadarObjectData, &ObjectNumber, &event_params);
                        gettimeofday(&finished, NULL);

                        double elapsed = calculateElapsedTime(starting, finished);
                        spdlog::debug("exec Track_PointCloud_Process took: {0} ms\n", elapsed);
                    } catch (std::exception &e) {
                        spdlog::error("Track_PointCloud_Process exception:[{0}]", e.what());
                    }
                    pthread_mutex_unlock(&g_mtxTrafficFlowDetectionStatistics);

                    track_count = ObjectNumber;
                    int objects = construct_radar_object_frame(&sendFrame, RadarObjectData, ObjectNumber);

                    /* 保存 */
                    if (saveAlgObjectInfoFlg == 1) {
                        usr_copyRadarObjectData(RadarObjectData, track_count);
                        sendCount = user_writeAlgObjectInfo_Wr((ARS408RadarObjectInfo_Wr *)&RadarObjectData_Wr[0], ObjectNumber, algObjectFp);
                        if (sendCount < 0) {
                            saveAlgObjectInfoFlg = 0;
                        }
                    }
                }
            } else {
                usleep(1000);
                continue;
            }

            if (!(!needRecord && !preNeedRecord && (insertCount == 0))) {
                if (g_clusterAndObjectSend2RearendMode == 0) {
                    // dbOperate(&sendFrame);
                }
            }
        } else {
            usleep(50000);
        }
    }
}
#else
std::mutex cvClusterMutex;
std::condition_variable cvClusterCond;
std::deque<std::vector<ars408_cluster_t>> cvClusterListDeque;

void deal_frame(std::vector<ars408_cluster_t> cluster)
{
    int i = 0, cluster_size = cluster.size();
    memset(radar_cluster_list1, 0, sizeof(radar_cluster_list1));

    if (cluster_size > 250) {
        cluster_size = 250;
    }

    for (i = 0; i < cluster_size; ++i) {
        memcpy(&radar_cluster_list1[i], &cluster.at(i), sizeof(RadarClusterInfoARS));
    }

    if (saveClusterInfoFlg == 1) {
        g_actualCurClusterCnt = cluster_size;
        if (user_writeClusterInfo(radar_cluster_list1, g_actualCurClusterCnt, clusterFp) < 0) {
            saveClusterInfoFlg = 0;
        }
    }

    if (g_clusterAndObjectSend2RearendMode == 1) {
        FH_CLUSTER sendCluster;

        memset(&sendCluster, 0, sizeof(sendCluster));
        int sendSize = construct_radar_cluster_frame(&sendCluster, radar_cluster_list1, cluster_size);

        spdlog::debug("cluster count: {:2d}", cluster_size);

        if (tcp_server_clients(1500) > 0) {
#if defined(CONFIG_SENDACTUALCLUSTER)
            tcp_server_send(1500, (const void *)&sendCluster, sendSize);
#else
            tcp_server_send(1500, (const void *)&sendCluster, sizeof(sendCluster));
#endif
        }
    } else {
        FH_OBJ sendFrame;
        int track_count = 0;

        try {
            struct timeval starting, finished;

            memset(RadarObjectData, 0, sizeof(RadarObjectData));

            gettimeofday(&starting, NULL);
            Track_PointCloud_Process(&track, radar_cluster_list1, cluster_size, RadarObjectData, &track_count, &event_params);
            gettimeofday(&finished, NULL);

            double elapsed = calculateElapsedTime(starting, finished);
            spdlog::debug("exec Track_PointCloud_Process took: {0} ms\n", elapsed);

            spdlog::debug("cluster count: {:2d}, track result count: {:2d}", cluster_size, track_count);

            memset(&sendFrame, 0, sizeof(sendFrame));
            construct_radar_object_frame(&sendFrame, RadarObjectData, track_count);
        } catch (std::exception &e) {
            spdlog::error("Track_PointCloud_Process exception:[{0}]", e.what());
        }

        /* 保存 */
        if ((saveAlgObjectInfoFlg == 1) && (track_count > 0)) {
            usr_copyRadarObjectData(RadarObjectData, track_count);
            int sendCount = user_writeAlgObjectInfo_Wr((ARS408RadarObjectInfo_Wr *)&RadarObjectData_Wr[0], track_count, algObjectFp);
            if (sendCount < 0) {
                saveAlgObjectInfoFlg = 0;
            }
        }

        if (!(!needRecord && !preNeedRecord && (insertCount == 0))) {
            if ((g_clusterAndObjectSend2RearendMode == 0) && (track_count > 0)) {
                // dbOperate(&sendFrame);
            }
        }
    }
}

void ars408_cluster_callback(std::vector<ars408_cluster_t> cluster)
{
    {
        std::unique_lock<std::mutex> lock(cvClusterMutex);
        cvClusterListDeque.emplace_back(std::move(cluster));
        cvClusterCond.notify_one();
    }
}

void ars408_ojects_callback(std::vector<ars408_object_t> objects)
{

}

void response_upper_computer_config(uint32_t can_id, uint8_t *data, uint8_t len)
{
    respondUpperComputerConfigCMD((UINT)can_id, (char *)data, (int)len);
}

void *user_hdlCanFrameThread(void *arg)
{
    pthread_setname_np(pthread_self(), "hdlCanFrmThread");

    while (1) {
        std::unique_lock<std::mutex> lock(cvClusterMutex);
        cvClusterCond.wait_for(lock, std::chrono::microseconds(1000), [&] {
            return !cvClusterListDeque.empty();
        });
        lock.unlock();

        if (!cvClusterListDeque.empty()) {
            auto frame = std::move(cvClusterListDeque.front());
            cvClusterListDeque.pop_front();

            if (frame.size() > 0) {
                deal_frame(std::move(frame));
            }
        }
    }
}
#endif
