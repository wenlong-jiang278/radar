/********************************************************************************
* @File name:ntp.h
* @Author:李军
* @Version: 1.0
* @Date:2023.05.12
* @Description:NTP功能
********************************************************************************/

#ifndef USER_NTP_H
#define USER_NTP_H

#ifdef __cplusplus
extern "C" {
#endif

extern int NTPEnFlg;
extern int NTPPeriod;
extern char NTPServerIP[128];

extern int user_writeCfgFile();
extern int user_readCfgFile();
extern void *ntpGetDateTime(void *arg);

#ifdef __cplusplus
}
#endif

#endif

