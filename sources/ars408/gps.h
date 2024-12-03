/********************************************************************************
* @File name:gps.h
* @Author:李军
* @Version: 1.0
* @Date:2023.05.12
* @Description:GPS功能
********************************************************************************/

#ifndef USER_GPS_H
#define USER_GPS_H

#include <pthread.h>

#ifdef __cplusplus
extern "C" {
#endif

extern int uart5_fd;
extern unsigned char uart5_pthread_run;
extern pthread_t uart5_pthread_id;
extern char g_GNSSEn;
extern char GNSSMode;
extern int GNSSPeriod; //周期
extern char g_GPSGNRMC[80];

extern int user_writeGNSSCfgFile();
extern int user_readGNSSCfgFile();
extern void *uart5_pthread(void *arg);
extern void *GNSSSetTime_pthread(void *arg);

#ifdef __cplusplus
}
#endif

#endif
