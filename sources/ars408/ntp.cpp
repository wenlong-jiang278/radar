/********************************************************************************
 * @File name:ntp.c
 * @Author:李军
 * @Version: 1.0
 * @Date:2023.05.12
 * @Description:NTP功能
 ********************************************************************************/

#include "ntp.h"
#include "can_tcp.h"
#include "cluster.h"
#include "gps.h"
#include "http.h"
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

int NTPEnFlg = 0;
int NTPPeriod = 60;
char NTPServerIP[128];

int user_writeCfgFile()
{
    FILE *fp = fopen("/home/root/cfgFile", "w");
    if (fp == NULL) {
        spdlog::error("write open /home/root/cfgFile failed");
        return -1;
    }

    char u8Line[128] = {0};
    memset(u8Line, 0, sizeof(u8Line));
    snprintf(u8Line, sizeof(u8Line), "%s\n", "{");
    fputs(u8Line, fp);

    if (strlen(NTPServerIP) > 0) {
        snprintf(u8Line, sizeof(u8Line), "ntpServerIP:%s,\n", NTPServerIP);
        fputs(u8Line, fp);
    }

    if (NTPPeriod > 0) {
        snprintf(u8Line, sizeof(u8Line), "ntpPeriod:%d,\n", NTPPeriod);
        fputs(u8Line, fp);
    }

    snprintf(u8Line, sizeof(u8Line), "%s\n", "}");
    fputs(u8Line, fp);
    fclose(fp);

    return 0;
}

int user_readCfgFile()
{
    FILE *fp = fopen("/home/root/cfgFile", "r");
    if (fp == NULL) {
        spdlog::error("read open /home/root/cfgFile failed");
        return -1;
    }

    char u8Line[128] = {0};

    memset(u8Line, 0, sizeof(u8Line));
    while (fgets(u8Line, sizeof(u8Line), fp) != NULL) {
        char *pTmp, *pComma;
        if ((pTmp = strstr(u8Line, "ntpServerIP:")) != NULL) {
            pComma = strchr(pTmp + strlen("ntpServerIP:"), ',');
            if (pComma != NULL) {
                char u8ServerIP[128] = {0};

                memset(u8ServerIP, 0, sizeof(u8ServerIP));
                strncpy(u8ServerIP, pTmp + strlen("ntpServerIP:"), pComma - pTmp - strlen("ntpServerIP:"));
                spdlog::info("u8ServerIP:[{0}]", u8ServerIP);
                strcpy(NTPServerIP, u8ServerIP);
            }
        }

        if ((pTmp = strstr(u8Line, "ntpPeriod:")) != NULL) {
            pComma = strchr(pTmp + strlen("ntpPeriod:"), ',');
            if (pComma != NULL) {
                char u8Period[128] = {0};

                memset(u8Period, 0, sizeof(u8Period));
                strncpy(u8Period, pTmp + strlen("ntpPeriod:"), pComma - pTmp - strlen("ntpPeriod:"));
                spdlog::info("u8Period:[{0}]", u8Period);
                NTPPeriod = atoi(u8Period);

                if (NTPPeriod <= 0) {
                    NTPPeriod = 60;
                }

                NTPEnFlg = 1;
                g_GNSSEn = 0;
            }
        }

        memset(u8Line, 0, sizeof(u8Line));
    }

    fclose(fp);
    return 0;
}

void *ntpGetDateTime(void *arg)
{
    // pthread_detach(pthread_self());

    pthread_setname_np(pthread_self(), "NtpSyncDateTime");

    int cnt = 0;
    char cmd[256] = {0};

    memset(cmd, 0, sizeof(cmd));
    snprintf(cmd, sizeof(cmd), "ntpdate -u %s", NTPServerIP);

    system(cmd);
    system(cmd);
    system("hwclock -w");

    sleep(1);

    while (1) {
        while (NTPEnFlg) {
            if (cnt >= NTPPeriod) {
                cnt = 0;
                memset(cmd, 0, sizeof(cmd));
                snprintf(cmd, sizeof(cmd), "ntpdate -u %s", NTPServerIP);

                system(cmd);
                system(cmd);
                system("hwclock -w");
            }

            sleep(1);
            cnt++;
        }

        sleep(1);
    }

    pthread_exit(NULL);
}
