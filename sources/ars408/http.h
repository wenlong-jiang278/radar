/********************************************************************************
* @File name:http.h
* @Author:李军
* @Version: 1.0
* @Date:2023.05.15
* @Description:http服务端
********************************************************************************/

#ifndef USER_HTTP_H
#define USER_HTTP_H

#include <sys/types.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef	struct _REQUEST {
	char *type;
	char *addr;
	char *body;
	char *host;
}REQUEST, *PREQUEST;

extern unsigned char g_u8Data[20];
extern unsigned char g_u8Time[20];
extern unsigned char g_u8Serial[64];
extern unsigned char g_u8Mac[64];
extern unsigned char g_u8CPUPer[4];
extern unsigned char g_u8MemPer[8];
extern unsigned char networkInfo[128];
extern unsigned long int g_luMemUsed;
extern unsigned long int g_luMemAvail;
extern unsigned long int g_luMemTotal;
extern unsigned char g_dspVer[];
extern unsigned char g_kernelVer[];
extern unsigned char g_startTime[64];
extern int g_curRunMode;
extern char g_RestoreMode;
extern char g_clusterAndObjectSend2RearendMode;
extern char g_dynamicOrStaticCalibrationMode;

extern int user_readMac(const char *interface);
extern int user_readTop();
extern int user_readMeminfo();
extern int user_readStartTime();
extern int strStr(char *haystack, char *needle);
extern int getHardinfo(char *res);
extern char *fail(char *msg);
extern char *success(char *msg);
extern int getHardRunStatus(char res[]);
extern int setNetwork(char *network, const char *interface);
extern int getDevDateTime(char *devDate, size_t size);
extern int restoreFactory(char *cont);
extern int getNetworkInfo(const char *interfaces);
extern int clusterAndObjectSend2Rearend(char *cont);
extern int dynamicOrStaticCalibration(char *cont);
extern int user_GetDynamicOrStaticCalibration(char *buf, unsigned int size);
extern void *http_manager_thread(void *ptr);
extern int user_getIni(char *buf);
extern int user_checkupdateRoiArgBody(char *cont);
extern int user_getRoiArg(char *buf);
extern int user_gotoUpgradeEmbededSoft(char *buf);
extern int user_lslhPath(char *cont, char *buf, unsigned int size);
extern int user_httpHdlWatchdog(char *cont, char *buf, unsigned int size);
extern int user_GetClusterAndObjectSend2Rearend(char *buf, unsigned int size);
extern int user_UpdateSections(char *cont, char *buf, unsigned int size);
extern int user_InsertSections(char *cont, char *buf, unsigned int size);
extern int user_DeleteSections(char *cont, char *buf, unsigned int size);
extern int user_UpdateCoils(char *cont, char *buf, unsigned int size);
extern int user_InsertCoils(char *cont, char *buf, unsigned int size);
extern int user_DeleteCoils(char *cont, char *buf, unsigned int size);
extern int user_postCalibrationPara(char *cont);
extern int user_getCalibrationPara(char *buf);
extern int user_postStaticCalibrationPara(char *cont);
extern int user_getStaticCalibrationPara(char *buf);
extern int user_getTrafficFlowDetectionStatistics(char *buf, unsigned int size, int wrFileFlg);
extern int user_InsertPolygonRoi(char *buf);
extern int user_UpdatePolygonRoi(char *buf);
extern int user_SelectPolygonRoi(char *buf);
extern int user_DeletePolygonRoi(char *buf);

extern int user_getOffsetParameters(char *buf);
extern int user_setOffsetParameters(char *cont);

#ifdef __cplusplus
}
#endif

#endif
