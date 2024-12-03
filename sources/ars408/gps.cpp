/********************************************************************************
 * @File name:gps.c
 * @Author:李军
 * @Version: 1.0
 * @Date:2023.05.12
 * @Description:GPS功能
 ********************************************************************************/

#include "gps.h"
#include "can_tcp.h"
#include "cluster.h"
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

char GNSSMode;
int GNSSPeriod;

int uart5_fd;
char g_GNSSEn = 0;
pthread_t uart5_pthread_id;
char g_GPSGNRMC[80] = {"\0"};
static char au8HMS[24] = {"\0"};
static char au8Date[4] = {"\0"};
static char au8Year[8] = {"\0"};
static char au8Hour[4] = {"\0"};
static char au8Month[4] = {"\0"};
static char au8Minute[4] = {"\0"};
static char au8Second[4] = {"\0"};
unsigned char uart5_pthread_run = 0;

int user_writeGNSSCfgFile()
{
    FILE *fp = fopen("/home/root/GNSSCfgFile", "w");
    if (fp == NULL) {
        spdlog::error("write open /home/root/GNSSCfgFile failed");
        return -1;
    }

    char u8Line[128] = {0};
    snprintf(u8Line, sizeof(u8Line), "%s\n", "{");
    fputs(u8Line, fp);

    snprintf(u8Line, sizeof(u8Line), "mode:%d,\n", GNSSMode);
    fputs(u8Line, fp);

    snprintf(u8Line, sizeof(u8Line), "period:%d,\n", GNSSPeriod);
    fputs(u8Line, fp);

    snprintf(u8Line, sizeof(u8Line), "%s\n", "}");
    fputs(u8Line, fp);

    fclose(fp);
    return 0;
}

int user_readGNSSCfgFile()
{
#if defined(CONFIG_GPS)
    FILE *fp = fopen("/home/root/GNSSCfgFile", "r");
    if (fp == NULL) {
        spdlog::error("read open /home/root/GNSSCfgFile failed");
        return -1;
    }

    char u8Line[128] = {0};
    memset(u8Line, 0, sizeof(u8Line));

    while (fgets(u8Line, sizeof(u8Line), fp) != NULL) {
        char *pTmp, *pComma;
        if ((pTmp = strstr(u8Line, "mode:")) != NULL) {
            pComma = strchr(pTmp + strlen("mode:"), ',');
            if (pComma != NULL) {
                char mode[128] = {0};

                memset(mode, 0, sizeof(mode));
                strncpy(mode, pTmp + strlen("mode:"), pComma - pTmp - strlen("mode:"));
                GNSSMode = atoi(mode);
            }
        }

        if ((pTmp = strstr(u8Line, "period:")) != NULL) {
            pComma = strchr(pTmp + strlen("period:"), ',');
            if (pComma != NULL) {
                char u8Period[128] = {0};

                memset(u8Period, 0, sizeof(u8Period));
                strncpy(u8Period, pTmp + strlen("period:"), pComma - pTmp - strlen("period:"));
                GNSSPeriod = atoi(u8Period);
                g_GNSSEn = 1;
                NTPEnFlg = 0;
            }
        }

        memset(u8Line, 0, sizeof(u8Line));
    }

    fclose(fp);
#endif

    return 0;
}

#if defined(CONFIG_GPS)
void *uart5_pthread(void *arg)
{
    int res = -1;
    int uart5_rx_len;
    unsigned int ret;
    char *pTmp = NULL;
    char *pTmp2 = NULL;
    char *pTmp3 = NULL;
    struct termios newtio;
    const char *dev = "/dev/ttymxc4";
    unsigned char uart5_rx_cache[1024];

    uart5_fd = open(dev, O_RDWR | O_NOCTTY);
    if (uart5_fd == -1) {
        spdlog::error("Can't Open Serial Port {0}", dev);
        pthread_exit(NULL);
    }

    res = fcntl(uart5_fd, F_SETFL, 0);
    if (res < 0) {
        spdlog::error("fcntl failed, errstr:[{0}]", strerror(errno));
    }

    bzero(&newtio, sizeof(newtio));
    newtio.c_iflag = 0;
    newtio.c_oflag = 0;
    newtio.c_lflag = 0;

    newtio.c_cflag = B9600 | CS8 | CLOCAL | CREAD;
    newtio.c_cc[VTIME] = 3;
    newtio.c_cc[VMIN] = 0;
    tcflush(uart5_fd, TCIFLUSH);
    tcsetattr(uart5_fd, TCSANOW, &newtio);

    uart5_pthread_run = 1;

    pthread_setname_np(pthread_self(), "uart5_thread");

    while (uart5_pthread_run) {
        memset(uart5_rx_cache, '\0', sizeof(uart5_rx_cache));
        uart5_rx_len = read(uart5_fd, uart5_rx_cache, sizeof(uart5_rx_cache));

        if (uart5_rx_len == 0) {
            usleep(200000);
            continue;
        } else {
            if ((pTmp = (char *)strstr((const char *)uart5_rx_cache, "$GNZDA")) != NULL) {
                memset(au8Hour, 0, sizeof(au8Hour));
                memset(au8Minute, 0, sizeof(au8Minute));
                memset(au8Second, 0, sizeof(au8Second));
                memset(au8HMS, 0, sizeof(au8HMS));
                memset(au8Date, 0, sizeof(au8Date));
                memset(au8Month, 0, sizeof(au8Month));
                memset(au8Year, 0, sizeof(au8Year));

                pTmp2 = strchr(pTmp, ',');
                pTmp3 = strchr(pTmp2 + 1, ',');
                if ((pTmp3 - pTmp2) > 1) {
                    strncpy(au8HMS, pTmp2 + 1, pTmp3 - pTmp2 - 1);
                    strncpy(au8Hour, au8HMS, 2);
                    strncpy(au8Minute, &au8HMS[2], 2);
                    strncpy(au8Second, &au8HMS[4], 2);
                }

                pTmp2 = strchr(pTmp3 + 1, ',');
                if ((pTmp2 - pTmp3) > 1) {
                    strncpy(au8Date, pTmp3 + 1, pTmp2 - pTmp3 - 1);
                }

                pTmp3 = strchr(pTmp2 + 1, ',');
                if ((pTmp3 - pTmp2) > 1) {
                    strncpy(au8Month, pTmp2 + 1, pTmp3 - pTmp2 - 1);
                }

                pTmp2 = strchr(pTmp3 + 1, ',');
                if ((pTmp2 - pTmp3) > 1) {
                    strncpy(au8Year, pTmp3 + 1, pTmp2 - pTmp3 - 1);
                }
            }

            if ((pTmp = (char *)strstr((const char *)uart5_rx_cache, "$GNRMC")) != NULL) {
                if ((pTmp2 = strstr(pTmp, "\r\n")) != NULL) {
                    memset(g_GPSGNRMC, '\0', sizeof(g_GPSGNRMC));
                    strncpy(g_GPSGNRMC, pTmp, pTmp2 - pTmp + strlen("\r\n"));
                }
            }
        }
    }

    pthread_exit(NULL);
}

void *GNSSSetTime_pthread(void *arg)
{
    int cnt = 0;
    char au8Buf[256] = {0};

    pthread_setname_np(pthread_self(), "GNSSSetTime");

    while (1) {
        if (g_GNSSEn == 1) {
            sleep(1);

            cnt++;
            if (cnt >= GNSSPeriod) {
                cnt = 0;
                memset(au8Buf, 0, sizeof(au8Buf));
                snprintf(au8Buf, sizeof(au8Buf), "date -s \"%s-%s-%s %s:%s:%s\"", au8Year, au8Month, au8Date, au8Hour, au8Minute, au8Second);
                system(au8Buf);
            }
        } else {
            sleep(1);
            cnt = 0;
        }
    }

    pthread_exit(NULL);
}
#endif
