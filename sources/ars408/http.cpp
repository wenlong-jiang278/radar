/********************************************************************************
 * @File name:http.c
 * @Author:李军
 * @Version: 1.0
 * @Date:2023.05.15
 * @Description:http服务端
 ********************************************************************************/

#include "http.h"
#include "Track.h"
#include "TrafficFlowDetection.h"
#include "cJSON.h"
#include "can_tcp.h"
#include "cluster.h"
#include "gps.h"
#include "ini.h"
#include "ntp.h"
#include "serial.h"
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
#include <sys/file.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <termios.h>
#include <time.h>
#include <unistd.h>

#include <exception>
#include <spdlog/spdlog.h>
#include <nlohmann/json.hpp>

unsigned char g_u8Mac[64];
unsigned char g_u8Data[20];
unsigned char g_u8Time[20];
unsigned char g_u8CPUPer[4];
unsigned char g_u8MemPer[8];
unsigned char g_u8Serial[64];
unsigned long int g_luMemUsed;
unsigned char networkInfo[128];
unsigned long int g_luMemAvail;
unsigned long int g_luMemTotal;
unsigned char g_startTime[64] = {0};
unsigned char g_dspVer[] = {"V1.0.2REVA"};
unsigned char g_kernelVer[] = {"4.1.15"};
char g_embeddedSoftVersion[] = {"V3.2.5"};

int g_curRunMode;
char g_RestoreMode;
static bool had_got_serial = false;
char g_dynamicOrStaticCalibrationMode = 0;      // 0：使用动态标定的数据，1：使用静态标定的数据
char g_clusterAndObjectSend2RearendMode = 0;    // 0：将原始点云数据经算法处理后得到的目标数据传输到后端，1：将原始点云数据传输到后端

extern std::map<int, RoadLaneParam> gRoadLaneParam;

int user_readStartTime()
{
    char cmdLine[128] = {"last reboot | head -n 1 | awk -F \" \" \'{print $5\" \"$6\" \"$7\" \"$8}\' > /run/rebootInfo.txt"};
    system(cmdLine);

    usleep(100000);

    int i = 0;
    FILE *fp = NULL;
    if ((fp = fopen("/run/rebootInfo.txt", "r")) == NULL) {
        spdlog::error("open /run/rebootInfo.txt failed");
        return -1;
    }

    char lineCont[64] = {0};
    if (fgets(lineCont, sizeof(lineCont), fp) == NULL) {
        return -1;
    }

    strncpy((char *)g_startTime, lineCont, strlen(lineCont));
    for (i = 0; i < strlen((char *)g_startTime); ++i) {
        if ((g_startTime[i] == 0x0a) || (g_startTime[i] == 0x0d)) {
            g_startTime[i] = '\0';
        }
    }

    fclose(fp);
}

int user_readMeminfo()
{
    system("cat /proc/meminfo > /run/memFile");
    usleep(50000);

    FILE *fp = NULL;
    fp = fopen("/run/memFile", "r");
    if (fp == NULL) {
        spdlog::error("open /run/memFile failed");
        return -1;
    }

    double total = 1.0, avail = 1.0;
    unsigned char line[10240] = {0};
    char *pTotal = NULL, *pAvail = NULL;

    while (fgets((char *)line, sizeof(line), fp) != NULL) {
        if ((pTotal = (char *)strstr((const char *)line, "MemTotal")) != NULL) {
            pTotal = strtok(pTotal + strlen("MemTotal:") + 1, " ");
            total = strtoul(pTotal, NULL, 10);
        } else if ((pAvail = (char *)strstr((const char *)line, "MemAvailable")) != NULL) {
            pAvail = strtok(pAvail + strlen("MemAvailable:") + 1, " ");
            avail = strtoul(pAvail, NULL, 10);
        }

        memset(line, 0, sizeof(line));
    }

    g_luMemTotal = total;
    g_luMemAvail = avail;
    g_luMemUsed = total - avail;
    sprintf((char *)g_u8MemPer, "%d", ((int)((total - avail) / total * 100)));
    spdlog::info("memPer:[{0}], memTotal:[{1}], memAvail:[{2}], memUsed:[{3}]", (char *)g_u8MemPer, g_luMemTotal, g_luMemAvail, g_luMemUsed);

    fclose(fp);
    return 0;
}

int user_readTop()
{
    system("top -n1 | cat > /run/topFile &");
    usleep(500000);

    FILE *fp = fopen("/run/topFile", "r");
    if (fp == NULL) {
        spdlog::error("open /run/topFile failed");
        return -1;
    }

    char line[2048] = {0};
    while (fgets(line, sizeof(line), fp) != NULL) {
        char *pnic = NULL;
        if ((pnic = (char *)strstr(line, "nic")) != NULL) {
            sprintf((char *)g_u8CPUPer, "%lu", 100 - strtoul(pnic + 3, NULL, 10));
            break;
        }

        memset(line, 0, sizeof(line));
    }

    fclose(fp);
    return 0;
}

int user_readMac(const char *interface)
{
    if (interface == NULL) {
        return -1;
    }

    char command[128] = {0};
    sprintf(command, "ifconfig %s > /run/macFile", interface);
    system(command);
    usleep(50000);

    FILE *fp = fopen("/run/macFile", "r");
    if (fp == NULL) {
        spdlog::error("open /run/macFile failed");
        return -1;
    }

    int i = 0;
    unsigned char line[1024] = {0};

    while (fgets((char *)line, sizeof(line), fp) != NULL) {
        char *pHWaddr = NULL;
        char *pNetNode = NULL;
        if (((pHWaddr = (char *)strstr((const char *)line, interface)) != NULL) && ((pHWaddr = (char *)strstr((const char *)line, "HWaddr")) != NULL)) {
            char *pComm = (char *)strchr(pHWaddr, ' ');
            pComm = pComm + 1;
            for (i = 0; i < strlen(pComm); ++i) {
                if ((pComm[i] == ' ') || (pComm[i] == '\n')) {
                    pComm[i] = '\0';
                }
            }

            strcpy((char *)g_u8Mac, pComm);
            break;
        }

        memset(line, 0, sizeof(line));
    }

    fclose(fp);
    return 0;
}

int user_readSerial()
{
    if (!had_got_serial) {
        if (get_serial_number((char *)g_u8Serial, sizeof(g_u8Serial)) == 0) {
            had_got_serial = true;
        }
    }
}

/* KMP字符串比较，包含needle返回1，否则返回0 */
int strStr(char *haystack, char *needle)
{
    if (!haystack || !needle) {
        return 0;
    }

    int n = strlen(haystack), m = strlen(needle);
    if (m == 0) {
        return 1;
    }

    int pi[m];
    pi[0] = 0;

    for (int i = 1, j = 0; i < m; i++) {
        while (j > 0 && needle[i] != needle[j]) {
            j = pi[j - 1];
        }

        if (needle[i] == needle[j]) {
            j++;
        }

        pi[i] = j;
    }

    for (int i = 0, j = 0; i < n; i++) {
        while (j > 0 && haystack[i] != needle[j]) {
            j = pi[j - 1];
        }

        if (haystack[i] == needle[j]) {
            j++;
        }

        if (j == m) {
            return 1;
        }
    }

    return 0;
}

int getNetworkInfo(const char *interfaces)
{
    if (interfaces == NULL) {
        return -1;
    }

    char command[128];
    char *interface = strtok(strdup(interfaces), ",");

    while (interface != NULL) {
        snprintf(command, sizeof(command), "ifconfig %s", interface);

        FILE *fp = popen(command, "r");
        if (fp == NULL) {
            spdlog::error("failed to run command:[{0}]", command);
            return -1;
        }

        char line[256] = {0};
        char address[64] = {0};
        char netmask[64] = {0};

        while (fgets(line, sizeof(line), fp) != NULL) {
            if (strstr(line, "inet addr:") != NULL) {
                sscanf(line, "          inet addr:%s", address);
                char *mask_ptr = strstr(line, "Mask:");
                if (mask_ptr != NULL) {
                    sscanf(mask_ptr, "Mask:%s", netmask);
                }
            }
        }

        pclose(fp);

        // 检查是否获取到信息
        if ((strlen(address) > 0) && (strlen(netmask) > 0)) {
            // 获取网关地址
            char gateway[64] = {0};
            snprintf(command, sizeof(command), "ip route get 8.8.8.8 | awk '{print $3; exit}'");
            FILE *gw_fp = popen(command, "r");
            if (gw_fp != NULL) {
                fgets(gateway, sizeof(gateway), gw_fp);
                pclose(gw_fp);
            }

            // 清理网关地址中的换行符
            gateway[strcspn(gateway, "\n")] = 0;

            memset(networkInfo, 0, sizeof(networkInfo));
            snprintf((char *)networkInfo, sizeof(networkInfo), "%s;%s;%s;", address, netmask, gateway);
            return 0;
        } else {
            spdlog::error("failed to retrieve network info for interface:[{0}]", interface);
            interface = strtok(NULL, ",");
        }
    }

    return -1;
}

int getHardinfo(char *res)
{
    if (res == NULL) {
        return -1;
    }

    strcat(res, "毫米波雷达ARS408;");

    getNetworkInfo("eth1,eth0");
    // if (getNetworkInfo("eth1,eth0") == -1) {
    //     return -1;
    // }

    user_readMac("eth1");
    // if (user_readMac("eth1") == -1) {
    //     return -1;
    // }

    user_readStartTime();
    // if (user_readStartTime() == -1) {
    //     return -1;
    // }

    strcat(res, (const char *)networkInfo);
    strcat(res, "Dtam d39-v;");
    strcat(res, "广州市丰海科技股份有限公司;");
    strcat(res, "广州市黄埔区开源大道11号B3栋101室、201室;");
    strcat(res, "NULL;");
    strcat(res, g_embeddedSoftVersion);
    strcat(res, ";");
    strcat(res, (const char *)g_startTime);
    strcat(res, ";");
    strcat(res, (const char *)g_kernelVer);
    strcat(res, ";");
    strcat(res, (const char *)g_u8Serial);
    strcat(res, ";");
    strcat(res, (const char *)g_dspVer);
    strcat(res, ";350;");
    strcat(res, (const char *)g_u8Mac);
    strcat(res, ";");
    strcat(res, (const char *)g_u8Data);
    strcat(res, " ");
    strcat(res, (const char *)g_u8Time);
    strcat(res, ";");

    return 0;
}

char *fail(char *msg)
{
    static char failedBuf[3072] = {'\0'};

    if (msg == NULL) {
        return (char *)"";
    }

    memset(failedBuf, 0, sizeof(failedBuf));
    failedBuf[0] = '\0';

    strcat(failedBuf, "HTTP/1.1 500 FAIL\r\n");
    strcat(failedBuf, "Server: myhttp\r\n");
    sprintf(failedBuf + strlen(failedBuf), "Content-Length: %d\r\n", strlen(msg));
    strcat(failedBuf, "\r\n");
    strcat(failedBuf, msg);

    return failedBuf;
}

char *success(char *msg)
{
    static char succeedBuf[3072] = {'\0'};

    if (msg == NULL) {
        return (char *)"";
    }

    memset(succeedBuf, 0, sizeof(succeedBuf));
    succeedBuf[0] = '\0';

    strcat(succeedBuf, "HTTP/1.1 200 OK\r\n");
    strcat(succeedBuf, "Server: infohand\r\n");

    sprintf(succeedBuf + strlen(succeedBuf), "Content-Length: %d\r\n", strlen(msg));
    strcat(succeedBuf, "\r\n");
    strcat(succeedBuf, msg);

    return succeedBuf;
}

int getHardRunStatus(char res[])
{
    user_readTop();
    // if (user_readTop() == -1) {
    //     return -1;
    // }

    user_readMeminfo();
    // if (user_readMeminfo() == -1) {
    //     return -1;
    // }

    user_readMac("eth1");
    // if (user_readMac("eth1") == -1) {
    //     return -1;
    // }

    strcat(res, "正常;");
    strcat(res, "NULL;");
    strcat(res, "NULL;");
    strcat(res, "NULL;");
    strcat(res, (const char *)g_u8CPUPer);
    strcat(res, ";");
    strcat(res, (const char *)g_u8MemPer);
    strcat(res, ";");
    strcat(res, "yaffs2;");

    char temp[128] = {0};
    sprintf(temp, "%d;%d;%d;", g_luMemTotal, g_luMemUsed, g_luMemAvail);
    strcat(res, temp);

    return 0;
}

int setNetwork(char *network, const char *interface)
{
    if (network == NULL) {
        return -1;
    }

    FILE *fpRd = NULL, *fpWr = NULL;
    unsigned char line[2048] = {"\0"};

    fpRd = fopen("/etc/network/interfaces", "r");
    if (fpRd == NULL) {
        spdlog::error("open /etc/network/interfaces failed");
        return -1;
    }

    fpWr = fopen("/run/interfaces", "w");
    if (fpWr == NULL) {
        spdlog::error("open /run/interfaces failed");
        fclose(fpRd);
        return -1;
    }

    int i = 0;
    char *pTmp = NULL;
    char *pColon1 = NULL;
    char *pColon2 = NULL;
    char ipaddr[128] = {0};
    char netmask[128] = {0};
    char gateway[128] = {0};
    char broadcast[128] = {0};
    unsigned char tempStr[256] = {0};

    if ((pTmp = strstr(network, "\"ip\"")) != NULL) {
        if ((pColon1 = strstr(pTmp + strlen("\"ip\":"), "\"")) != NULL) {
            if ((pColon2 = strstr(pColon1 + strlen("\""), "\"")) != NULL) {
                strncpy(ipaddr, pColon1 + 1, pColon2 - pColon1 - 1);
                spdlog::info("ipaddr:[{0}]", ipaddr);
            }
        }
    } else {
        fclose(fpRd);
        fclose(fpWr);
        return -1;
    }

    if ((pTmp = strstr(network, "\"netmask\"")) != NULL) {
        if ((pColon1 = strstr(pTmp + strlen("\"netmask\":"), "\"")) != NULL) {
            if ((pColon2 = strstr(pColon1 + strlen("\""), "\"")) != NULL) {
                strncpy(netmask, pColon1 + 1, pColon2 - pColon1 - 1);
                spdlog::info("netmask:[{0}]", netmask);
            }
        }
    } else {
        fclose(fpRd);
        fclose(fpWr);
        return -1;
    }

    if ((pTmp = strstr(network, "\"gateway\"")) != NULL) {
        if ((pColon1 = strstr(pTmp + strlen("\"gateway\":"), "\"")) != NULL) {
            if ((pColon2 = strstr(pColon1 + strlen("\""), "\"")) != NULL) {
                strncpy(gateway, pColon1 + 1, pColon2 - pColon1 - 1);
                spdlog::info("gateway:[{0}]", gateway);
            }
        }
    } else {
        fclose(fpRd);
        fclose(fpWr);
        return -1;
    }

    if ((pTmp = strstr(network, "\"broadcast\"")) != NULL) {
        if ((pColon1 = strstr(pTmp + strlen("\"broadcast\":"), "\"")) != NULL) {
            if ((pColon2 = strstr(pColon1 + strlen("\""), "\"")) != NULL) {
                strncpy(broadcast, pColon1 + 1, pColon2 - pColon1 - 1);
                spdlog::info("broadcast:[{0}]", broadcast);
            }
        }
    } else {
        fclose(fpRd);
        fclose(fpWr);
        return -1;
    }

    spdlog::info("ipaddr:[{0}], netmask:[{1}], gateway:[{2}], broadcast:[{3}]", ipaddr, netmask, gateway, broadcast);

    while (fgets((char *)line, sizeof(line), fpRd) != NULL) {
        char inter[128] = {0};
        sprintf(inter, "iface %s inet static", interface == NULL ? "eth1" : interface);

        if (strstr((const char *)line, inter) != NULL) {
            fputs((const char *)line, fpWr);
            i = 1;
        } else {
            if ((i >= 1) && (i <= 4)) {
                memset(tempStr, 0, sizeof(tempStr));
                switch (i) {
                    case 1: {
                        sprintf((char *)tempStr, "address %s\n", ipaddr);
                        break;
                    }

                    case 2: {
                        sprintf((char *)tempStr, "netmask %s\n", netmask);
                        break;
                    }

                    case 3: {
                        sprintf((char *)tempStr, "gateway %s\n", gateway);
                        break;
                    }

                    case 4: {
                        sprintf((char *)tempStr, "broadcast %s\n", broadcast);
                        break;
                    }
                }

                fputs((const char *)tempStr, fpWr);
                i++;
            } else {
                fputs((const char *)line, fpWr);
            }
        }
    }

    fclose(fpRd);
    fclose(fpWr);

    system("mv /run/interfaces /etc/network/interfaces");
    return 0;
}

int setNTPServerIp(char *cont)
{
    char *pTmp = NULL;
    char *pColon1 = NULL;
    char *pColon2 = NULL;
    char ipaddr[128] = {0};
    char u8Period[24] = {0};

    if (cont == NULL) {
        return -1;
    }

    if ((pTmp = strstr(cont, "\"ip\"")) != NULL) {
        if ((pColon1 = strstr(pTmp + strlen("\"ip\":"), "\"")) != NULL) {
            if ((pColon2 = strstr(pColon1 + strlen("\""), "\"")) != NULL) {
                strncpy(ipaddr, pColon1 + 1, pColon2 - pColon1 - 1);
                spdlog::info("ntp server ipaddr:[{0}]", ipaddr);
            }
        }

        memset(NTPServerIP, 0, sizeof(NTPServerIP));
        strncpy(NTPServerIP, ipaddr, sizeof(NTPServerIP) - 1);
        spdlog::info("NTPServerIP:[{0}]", NTPServerIP);
    }

    if ((pTmp = strstr(cont, "\"period\"")) != NULL) {
        spdlog::info("ntp sync period:[{0}]", pTmp);
        NTPPeriod = 0;
        NTPPeriod = atoi(pTmp + strlen("\"period\":\""));
        if (NTPPeriod <= 0) {
            NTPPeriod = 60;
            // return -1;
        }

        spdlog::info("NTPPeriod:[{0}]", NTPPeriod);

        NTPEnFlg = 1;
        g_GNSSEn = 0;
        user_writeCfgFile();
        system("rm -rf /home/root/GNSSCfgFile");
    }

    return 0;
}

int setRunMode(char *cont)
{
    if (cont == NULL) {
        return -1;
    }

    int value;
    char *p1 = NULL;
    char *p2 = NULL;
    char *p3 = NULL;
    char au8Val[3] = {0};

    if ((p1 = strstr(cont, "\"mode\"")) != NULL) {
        if ((p2 = strchr(p1, ':')) != NULL) {
            if ((p3 = strchr(p2, '\r')) != NULL) {
                if ((p3 - p2) > sizeof(au8Val)) {
                    return -1;
                }

                strncpy(au8Val, p2 + 1, p3 - p2 - 1);
                value = atoi(au8Val);
                if (value > 2) {
                    return -1;
                }

                g_curRunMode = value;
                spdlog::info("run mode:[{0}]", value);
                return 0;
            }
        }
    }

    return -1;
}

int getDevDateTime(char *devDate, size_t size)
{
    if ((devDate == NULL) || (size == 0)) {
        return -1;
    }

    system("date \'+%Y-%m-%d %H:%M:%S\' > /run/devDateFile");
    usleep(100000);

    FILE *fp = NULL;
    fp = fopen("/run/devDateFile", "r");
    if (fp == NULL) {
        spdlog::error("open /run/devDateFile failed");
        return -1;
    }

    unsigned char line[256] = {0};
    while (fgets((char *)line, sizeof(line), fp) != NULL) {
        size_t linesize = strlen((const char *)line);
        if (linesize > 255) {
            linesize = 255;
        }

        line[linesize - 1] = '\0';
        memcpy(devDate, line, linesize);
        spdlog::info("devDate:[{0}]", devDate);
        break;
    }

    fclose(fp);
    return 0;
}

int setGNSSMode(char *cont)
{
    char *pTmp = NULL;
    char *pColon1 = NULL;
    char *pColon2 = NULL;
    char mode[128] = {"\0"};

    if (cont == NULL) {
        return -1;
    }

    if ((pTmp = strstr(cont, "\"mode\":")) != NULL) {
        pColon1 = pTmp + strlen("\"mode\":\"");
        if ((pColon2 = strstr(pColon1, "\r\n")) != NULL) {
            strncpy(mode, pColon1, pColon2 - pColon1);
            GNSSMode = atoi(mode);
            spdlog::info("mode:[{0}], GNSSMode:[{1}]", mode, GNSSMode);
        }
    }

    if ((pTmp = strstr(cont, "\"period\":")) != NULL) {
        GNSSPeriod = atoi(pTmp + strlen("\"period\":\""));
        spdlog::info("GNSSPeriod:[{0}]", GNSSPeriod);

        user_writeGNSSCfgFile();
        g_GNSSEn = 1;
        NTPEnFlg = 0;
        system("rm -rf /home/root/cfgFile");
    }

    return 0;
}

int restoreFactory(char *cont)
{
    if (cont == NULL) {
        return -1;
    }

    char *pTmp = NULL;
    char *pColon1 = NULL;
    char *pColon2 = NULL;
    char mode[128] = {0};

    g_RestoreMode = 0;
    if ((pTmp = strstr(cont, "\"mode\":")) != NULL) {
        pColon1 = pTmp + strlen("\"mode\":");

        if ((pColon2 = strstr(pColon1, "\r\n")) != NULL) {
            strncpy(mode, pColon1, pColon2 - pColon1);
            g_RestoreMode = atoi(mode + 1);
            spdlog::info("mode:[{0}], GNSSMode:[{1}]", mode, GNSSMode);
        }
    }

    if ((g_RestoreMode == 0) || (g_RestoreMode == 1)) {
        FILE *fpRd = NULL, *fpWr = NULL;
        unsigned char line[2048] = {"\0"};

        fpRd = fopen("/etc/network/interfaces", "r");
        if (fpRd == NULL) {
            spdlog::error("open /etc/network/interfaces failed");
            return -1;
        }

        fpWr = fopen("/run/interfaces", "w");
        if (fpWr == NULL) {
            spdlog::error("open /run/interfaces failed");
            fclose(fpRd);
            return -1;
        }

        int i = 0;
        char *pTmp = NULL;
        char *pColon1 = NULL;
        char *pColon2 = NULL;
        char ipaddr[128] = {0};
        char netmask[128] = {0};
        char gateway[128] = {0};
        char broadcast[128] = {0};
        unsigned char tempStr[256] = {0};

        strcpy(ipaddr, "192.168.16.247");
        strcpy(netmask, "255.255.255.0");
        strcpy(gateway, "192.168.16.1");
        strcpy(broadcast, "192.168.16.255");
        spdlog::info("ipaddr:[{0}], netmask:[{1}], gateway:[{2}], broadcast:[{3}]", ipaddr, netmask, gateway, broadcast);

        while (fgets((char *)line, sizeof(line), fpRd) != NULL) {
            if (strstr((const char *)line, "iface eth1 inet static") != NULL) {
                fputs((const char *)line, fpWr);
                i = 1;
            } else {
                if ((i >= 1) && (i <= 4)) {
                    memset(tempStr, 0, sizeof(tempStr));
                    switch (i) {
                        case 1: {
                            sprintf((char *)tempStr, "address %s\n", ipaddr);
                            break;
                        }

                        case 2: {
                            sprintf((char *)tempStr, "netmask %s\n", netmask);
                            break;
                        }

                        case 3: {
                            sprintf((char *)tempStr, "gateway %s\n", gateway);
                            break;
                        }

                        case 4: {
                            sprintf((char *)tempStr, "broadcast %s\n", broadcast);
                            break;
                        }
                    }

                    fputs((const char *)tempStr, fpWr);
                    i++;
                } else {
                    fputs((const char *)line, fpWr);
                }
            }
        }

        fclose(fpRd);
        fclose(fpWr);

        system("mv /run/interfaces /etc/network/interfaces");
    }

    return 0;
}

int clusterAndObjectSend2Rearend(char *cont)
{
    if (cont == NULL) {
        return -1;
    }

    int send_mode = 0;
    char mode[32] = {0};
    const char *key = "\"mode\":\"";
    char *start = strstr(cont, key);

    g_clusterAndObjectSend2RearendMode = 0;

    if (start) {
        start += strlen(key);
        char *end = strchr(start, '\"');
        if (end) {
            strncpy(mode, start, end - start);
            mode[end - start] = '\0';

            send_mode = atoi(mode);
            spdlog::info("mode:[{0}], send_mode:[{1}]", mode, send_mode);

            g_clusterAndObjectSend2RearendMode = send_mode;
            if ((g_clusterAndObjectSend2RearendMode != 0) && (g_clusterAndObjectSend2RearendMode != 1)) {
                g_clusterAndObjectSend2RearendMode = 0;
                return 1;
            }
        } else {
            return 1;
        }
    } else {
        return 1;
    }

    user_writeClusterAndObjectSend2Rearend_func();
    return 0;
}

int dynamicOrStaticCalibration(char *cont)
{
    if (cont == NULL) {
        return -1;
    }

    char mode[32] = {0};
    int calibra_mode = 0;
    const char *key = "\"mode\":\"";
    char *start = strstr(cont, key);

    g_dynamicOrStaticCalibrationMode = 0;

    if (start) {
        start += strlen(key);
        char *end = strchr(start, '\"');
        if (end) {
            strncpy(mode, start, end - start);
            mode[end - start] = '\0';

            calibra_mode = atoi(mode);
            spdlog::info("mode:[{0}], calibra_mode:[{1}]", mode, calibra_mode);

            g_dynamicOrStaticCalibrationMode = calibra_mode;
            if ((g_dynamicOrStaticCalibrationMode != 0) && (g_dynamicOrStaticCalibrationMode != 1)) {
                g_dynamicOrStaticCalibrationMode = 0;
                return 1;
            }
        } else {
            return 1;
        }
    } else {
        return 1;
    }

    user_writeDynamicOrStaticCalibrationMode_func();
    return 0;
}

int user_GetDynamicOrStaticCalibration(char *buf, unsigned int size)
{
    snprintf(buf, size, "mode:%d", g_dynamicOrStaticCalibrationMode);
    return 0;
}

static const char *pIniKeys[] = {
    "EnableTrajectoryManage",
    "EnableSpeed_Dect",
    "EnableRetrograde_Dect",
    "EnableStopCar_Dect",
    "EnableLaneChange_Dect",
    "EnableCongest_Dect",
    "EnableLane_Judge",
    "EnabletrafficFlow",
    "EnablelaneDividedTrafficFlow",
    "EnablesectionAverageSpeed",
    "EnablelaneDividedAverageSpeed",
    "EnablelaneDividedTimeOccupancy",
    "EnablelaneDividedAverageHeadway",
    "EnablelaneDividedAverageHeadwayGap",
    "AdjustParams_DisLongMin",
    "AdjustParams_DisLongMax",
    "AdjustParams_DisLatMin",
    "AdjustParams_DisLatMax",
    "AdjustParams_VrelLongMin",
    "AdjustParams_VrelLongMax",
    "AdjustParams_VrelLatMin",
    "AdjustParams_VrelLatMax",
    "AdjustParams_RCSMin",
    "AdjustParams_RCSMax",
    "AdjustParams_DBSCANMinPts",
    "AdjustParams_DBSCANBIGCarEps_a",
    "AdjustParams_DBSCANBIGCarEps_b",
    "AdjustParams_DBSCANMEDIUMCarEps_a",
    "AdjustParams_DBSCANMEDIUMCarEps_b",
    "AdjustParams_DBSCANSMALLCarEps_a",
    "AdjustParams_DBSCANSMALLCarEps_b",
    "AdjustParams_EKFPairRNGTHR",
    "AdjustParams_EKFPairVELTHR",
    "AdjustParams_EKFPairANGTHR",
    "AdjustParams_EKFUpdatePreTypeThres",
    "AdjustParams_EKFUpdateValTypeThres",
    "AdjustParams_EKFFilterTimeInt",
    "AdjustParams_EKFFilterACC",
    "AdjustParams_SmallCarRCSThres",
    "AdjustParams_MediumCarRCSThres",
    "AdjustParams_BigCarRCSThres",
    "AdjustParams_V_Thr",
    "AdjustParams_Frame_car_list",
    "AdjustParams_Frame_car_stop",
    "AdjustParams_X_Thr",
    "AdjustParams_Y_Thr",
    "AdjustParams_MAX_SPEED",
    "AdjustParams_LIMIT_SPEED",
    "AdjustParams_LIMIT_DIST",
    "EnableExtra_Judge",
    "AdjustParams_EKFFilterR_RNG",
    "AdjustParams_EKFFilterR_VEL",
    "AdjustParams_EKFFilterR_ANG"
};

static int user_findUpdateIniKey(char *key)
{
    if (key == NULL) {
        return -1;
    }

    int i = 0, size = sizeof(pIniKeys) / sizeof(pIniKeys[0]);
    for (int i = 0; i < size; ++i) {
        if (strcmp(key, pIniKeys[i]) == 0) {
            return 0;
        }
    }

    return -1;
}

int user_checkUpdateIniBody(char *cont)
{
    char *p = NULL;
    char *p2 = NULL;
    char *pComma = NULL;
    char key[64] = {0};
    char value[64] = {0};
    char temp[256] = {0};
    char *pColon1 = NULL, *pColon2 = NULL, *pColon3 = NULL, *pColon4 = NULL;

    if (cont == NULL) {
        return -1;
    }

    for (p = strtok(cont, "\n,"); p != NULL; p = strtok(NULL, "\n,")) {
        memset(temp, 0, sizeof(temp));
        strncpy(temp, p, strlen(p));

        pColon1 = strchr(temp, '"');
        if (pColon1 != NULL) {
            pColon2 = strchr(pColon1 + 1, '"');
            if (pColon2 != NULL) {
                if ((pColon2 - (pColon1 + 1)) > (sizeof(key) - 1)) {
                    return -1;
                }

                memset(key, 0, sizeof(key));
                strncpy(key, pColon1 + 1, pColon2 - (pColon1 + 1));

                pColon3 = strchr(pColon2 + 1, '"');
                if (pColon3 != NULL) {
                    pColon4 = strchr(pColon3 + 1, '"');
                    if (pColon4 != NULL) {
                        if ((pColon4 - (pColon3 + 1)) > (sizeof(value) - 1)) {
                            return -1;
                        }

                        memset(value, '\0', sizeof(value));
                        strncpy(value, pColon3 + 1, pColon4 - (pColon3 + 1));

                        if (user_findUpdateIniKey(key) != 0) {
                            return -1;
                        }

                        int ret = 0;
                        sqlite3 *db1 = NULL;

                        /* 连接数据库 */
                        ret = sqlite3_open(ADJUST_DB_FILE, &db1);
                        if (ret != SQLITE_OK) {
                            spdlog::error("open db {0} failed", ADJUST_DB_FILE);
                            return 1;
                        }

                        ret = UpdateParams(db1, key, value);
                        if (ret == 0) {
                            sqlite3_close(db1);
                        }
                    } else {
                        return -1;
                    }
                } else {
                    return -1;
                }
            } else {
                return -1;
            }
        } else {
            continue;
        }
    }

    user_flushParams(NULL);
    return 0;
}

static const char *pROIKeys[] = {"name", "x1", "x2", "x3", "x4", "y1",   "y2", "y3", "y4"};

static int user_findROIKey(char *key)
{
    if (key == NULL) {
        return -1;
    }

    int i = 0, size = sizeof(pROIKeys) / sizeof(pROIKeys[0]);
    for (i = 0; i < size; ++i) {
        if (strcmp(key, pROIKeys[i]) == 0) {
            return 0;
        }
    }

    return -1;
}

static const char *pROINameValues[] = {"r1", "r2", "r3", "r4"};

static int user_findROINameValue(char *Value)
{
    if (Value == NULL) {
        return -1;
    }

    int i = 0, size = sizeof(pROINameValues) / sizeof(pROINameValues[0]);
    for (i = 0; i < size; ++i) {
        if (strcmp(Value, pROINameValues[i]) == 0) {
            return 0;
        }
    }

    return -1;
}

int user_checkupdateRoiArgBody(char *cont)
{
    char *p = NULL;
    int keyCnt = 0;
    char *p2 = NULL;
    int valueCnt = 0;
    char *pComma = NULL;
    char temp[256] = {"\0"};
    char x1Val[64] = {"\0"};
    char y1Val[64] = {"\0"};
    char x2Val[64] = {"\0"};
    char y2Val[64] = {"\0"};
    char x3Val[64] = {"\0"};
    char y3Val[64] = {"\0"};
    char x4Val[64] = {"\0"};
    char y4Val[64] = {"\0"};
    char key[10][64] = {"\0"};
    char nameVal[64] = {"\0"};
    char value[10][64] = {"\0"};
    char *pColon1 = NULL, *pColon2 = NULL, *pColon3 = NULL, *pColon4 = NULL;
    char *pCont[8] = {&x1Val[0], &y1Val[0], &x2Val[0], &y2Val[0], &x3Val[0], &y3Val[0], &x4Val[0], &y4Val[0]};

    if (cont == NULL) {
        return -1;
    }

    for (p = strtok(cont, "\n,"); p != NULL; p = strtok(NULL, "\n,")) {
        memset(temp, '\0', sizeof(temp));
        strncpy(temp, p, strlen(p));

        pColon1 = strchr(temp, '"');
        if (pColon1 != NULL) {
            pColon2 = strchr(pColon1 + 1, '"');
            if (pColon2 != NULL) {
                if ((pColon2 - (pColon1 + 1)) > (sizeof(key[keyCnt]) - 1)) {
                    return -1;
                }

                memset(key[keyCnt], '\0', sizeof(key[keyCnt]));
                strncpy(key[keyCnt], pColon1 + 1, pColon2 - (pColon1 + 1));

                pColon3 = strchr(pColon2 + 1, '"');
                if (pColon3 != NULL) {
                    pColon4 = strchr(pColon3 + 1, '"');
                    if (pColon4 != NULL) {
                        if ((pColon4 - (pColon3 + 1)) > (sizeof(value) - 1)) {
                            return -1;
                        }

                        memset(value[valueCnt], '\0', sizeof(value[valueCnt]));
                        strncpy(value[valueCnt], pColon3 + 1, pColon4 - (pColon3 + 1));

                        if (user_findROIKey(key[keyCnt]) != 0) {
                            return -1;
                        }

                        if (strcmp(key[keyCnt], "name") == 0) {
                            if (user_findROINameValue(value[valueCnt]) != 0) {
                                return -1;
                            }

                            strcpy(nameVal, value[valueCnt]);
                        }
                    } else {
                        return -1;
                    }
                } else {
                    return -1;
                }
            } else {
                return -1;
            }

            if (strcmp(key[keyCnt], "x1") == 0) {
                strcpy(x1Val, value[valueCnt]);
            } else if (strcmp(key[keyCnt], "x2") == 0) {
                strcpy(x2Val, value[valueCnt]);
            } else if (strcmp(key[keyCnt], "x3") == 0) {
                strcpy(x3Val, value[valueCnt]);
            } else if (strcmp(key[keyCnt], "x4") == 0) {
                strcpy(x4Val, value[valueCnt]);
            } else if (strcmp(key[keyCnt], "y1") == 0) {
                strcpy(y1Val, value[valueCnt]);
            } else if (strcmp(key[keyCnt], "y2") == 0) {
                strcpy(y2Val, value[valueCnt]);
            } else if (strcmp(key[keyCnt], "y3") == 0) {
                strcpy(y3Val, value[valueCnt]);
            } else if (strcmp(key[keyCnt], "y4") == 0) {
                strcpy(y4Val, value[valueCnt]);
            }

            keyCnt++;
            valueCnt++;
            if (keyCnt > 9 || valueCnt > 9) {
                return -1;
            }
        } else {
            continue;
        }
    }

    if (keyCnt != 9 || valueCnt != 9) {
        return -1;
    }

    int ret = 0;
    sqlite3 *db1 = NULL;

    ret = sqlite3_open(ADJUST_DB_FILE, &db1);
    if (ret != SQLITE_OK) {
        spdlog::error("open db {0} failed", ADJUST_DB_FILE);
        return 1;
    }

    ret = UpdateRoi(db1, nameVal, pCont);
    if (ret == 0) {
        sqlite3_close(db1);
    }

    user_flushParams(NULL);
    return 0;
}

int user_getIni(char *buf)
{
    char tempBuf[128] = {0};

    if (buf == NULL) {
        return -1;
    }

    (adjust_Params.EnableTrajectoryManage == true) ? (strcat(buf, "\"EnableTrajectoryManage\":\"true\",")) : (strcat(buf, "\"EnableTrajectoryManage\":\"false\","));
    (adjust_Params.EnableExtra_Judge == true) ? (strcat(buf, "\"EnableExtra_Judge\":\"true\",")) : (strcat(buf, "\"EnableExtra_Judge\":\"false\","));
    (adjust_Params.EnableSpeed_Dect == true) ? (strcat(buf, "\"EnableSpeed_Dect\":\"true\",")) : (strcat(buf, "\"EnableSpeed_Dect\":\"false\","));
    (adjust_Params.EnableRetrograde_Dect == true) ? (strcat(buf, "\"EnableRetrograde_Dect\":\"true\",")) : (strcat(buf, "\"EnableRetrograde_Dect\":\"false\","));
    (adjust_Params.EnableStopCar_Dect == true) ? (strcat(buf, "\"EnableStopCar_Dect\":\"true\",")) : (strcat(buf, "\"EnableStopCar_Dect\":\"false\","));
    (adjust_Params.EnableLaneChange_Dect == true) ? (strcat(buf, "\"EnableLaneChange_Dect\":\"true\",")) : (strcat(buf, "\"EnableLaneChange_Dect\":\"false\","));
    (adjust_Params.EnableCongest_Dect == true) ? (strcat(buf, "\"EnableCongest_Dect\":\"true\",")) : (strcat(buf, "\"EnableCongest_Dect\":\"false\","));
    (adjust_Params.EnableLane_Judge == true) ? (strcat(buf, "\"EnableLane_Judge\":\"true\",")) : (strcat(buf, "\"EnableLane_Judge\":\"false\","));
    (adjust_Params.EnabletrafficFlow == true) ? (strcat(buf, "\"EnabletrafficFlow\":\"true\",")) : (strcat(buf, "\"EnabletrafficFlow\":\"false\","));
    (adjust_Params.EnablelaneDividedTrafficFlow == true) ? (strcat(buf, "\"EnablelaneDividedTrafficFlow\":\"true\",")) : (strcat(buf, "\"EnablelaneDividedTrafficFlow\":\"false\","));
    (adjust_Params.EnablesectionAverageSpeed == true) ? (strcat(buf, "\"EnablesectionAverageSpeed\":\"true\",")) : (strcat(buf, "\"EnablesectionAverageSpeed\":\"false\","));
    (adjust_Params.EnablelaneDividedAverageSpeed == true) ? (strcat(buf, "\"EnablelaneDividedAverageSpeed\":\"true\",")) : (strcat(buf, "\"EnablelaneDividedAverageSpeed\":\"false\","));
    (adjust_Params.EnablelaneDividedTimeOccupancy == true) ? (strcat(buf, "\"EnablelaneDividedTimeOccupancy\":\"true\",")) : (strcat(buf, "\"EnablelaneDividedTimeOccupancy\":\"false\","));
    (adjust_Params.EnablelaneDividedAverageHeadway == true) ? (strcat(buf, "\"EnablelaneDividedAverageHeadway\":\"true\",")) : (strcat(buf, "\"EnablelaneDividedAverageHeadway\":\"false\","));
    (adjust_Params.EnablelaneDividedAverageHeadwayGap == true) ? (strcat(buf, "\"EnablelaneDividedAverageHeadwayGap\":\"true\",")) : (strcat(buf, "\"EnablelaneDividedAverageHeadwayGap\":\"false\","));

    memset(tempBuf, 0, sizeof(tempBuf));
    snprintf(tempBuf, sizeof(tempBuf), "\"AdjustParams_DisLongMin\":\"%lf\",", adjust_Params.AdjustParams_DisLongMin);
    strcat(buf, tempBuf);

    memset(tempBuf, 0, sizeof(tempBuf));
    snprintf(tempBuf, sizeof(tempBuf), "\"AdjustParams_DisLongMax\":\"%lf\",", adjust_Params.AdjustParams_DisLongMax);
    strcat(buf, tempBuf);

    memset(tempBuf, 0, sizeof(tempBuf));
    snprintf(tempBuf, sizeof(tempBuf), "\"AdjustParams_DisLatMin\":\"%lf\",", adjust_Params.AdjustParams_DisLatMin);
    strcat(buf, tempBuf);

    memset(tempBuf, 0, sizeof(tempBuf));
    snprintf(tempBuf, sizeof(tempBuf), "\"AdjustParams_DisLatMax\":\"%lf\",", adjust_Params.AdjustParams_DisLatMax);
    strcat(buf, tempBuf);

    memset(tempBuf, 0, sizeof(tempBuf));
    snprintf(tempBuf, sizeof(tempBuf), "\"AdjustParams_VrelLongMin\":\"%lf\",", adjust_Params.AdjustParams_VrelLongMin);
    strcat(buf, tempBuf);

    memset(tempBuf, 0, sizeof(tempBuf));
    snprintf(tempBuf, sizeof(tempBuf), "\"AdjustParams_VrelLongMax\":\"%lf\",", adjust_Params.AdjustParams_VrelLongMax);
    strcat(buf, tempBuf);

    memset(tempBuf, 0, sizeof(tempBuf));
    snprintf(tempBuf, sizeof(tempBuf), "\"AdjustParams_VrelLatMin\":\"%lf\",", adjust_Params.AdjustParams_VrelLatMin);
    strcat(buf, tempBuf);

    memset(tempBuf, 0, sizeof(tempBuf));
    snprintf(tempBuf, sizeof(tempBuf), "\"AdjustParams_VrelLatMax\":\"%lf\",", adjust_Params.AdjustParams_VrelLatMax);
    strcat(buf, tempBuf);

    memset(tempBuf, 0, sizeof(tempBuf));
    snprintf(tempBuf, sizeof(tempBuf), "\"AdjustParams_RCSMin\":\"%lf\",", adjust_Params.AdjustParams_RCSMin);
    strcat(buf, tempBuf);

    memset(tempBuf, 0, sizeof(tempBuf));
    snprintf(tempBuf, sizeof(tempBuf), "\"AdjustParams_RCSMax\":\"%lf\",", adjust_Params.AdjustParams_RCSMax);
    strcat(buf, tempBuf);

    memset(tempBuf, 0, sizeof(tempBuf));
    snprintf(tempBuf, sizeof(tempBuf), "\"AdjustParams_DBSCANMinPts\":\"%d\",", adjust_Params.AdjustParams_DBSCANMinPts);
    strcat(buf, tempBuf);

    memset(tempBuf, 0, sizeof(tempBuf));
    snprintf(tempBuf, sizeof(tempBuf), "\"AdjustParams_DBSCANBIGCarEps_a\":\"%lf\",", adjust_Params.AdjustParams_DBSCANBIGCarEps_a);
    strcat(buf, tempBuf);

    memset(tempBuf, 0, sizeof(tempBuf));
    snprintf(tempBuf, sizeof(tempBuf), "\"AdjustParams_DBSCANBIGCarEps_b\":\"%lf\",", adjust_Params.AdjustParams_DBSCANBIGCarEps_b);
    strcat(buf, tempBuf);

    memset(tempBuf, 0, sizeof(tempBuf));
    snprintf(tempBuf, sizeof(tempBuf), "\"AdjustParams_DBSCANMEDIUMCarEps_a\":\"%lf\",", adjust_Params.AdjustParams_DBSCANMEDIUMCarEps_a);
    strcat(buf, tempBuf);

    memset(tempBuf, 0, sizeof(tempBuf));
    snprintf(tempBuf, sizeof(tempBuf), "\"AdjustParams_DBSCANMEDIUMCarEps_b\":\"%lf\",", adjust_Params.AdjustParams_DBSCANMEDIUMCarEps_b);
    strcat(buf, tempBuf);

    memset(tempBuf, 0, sizeof(tempBuf));
    snprintf(tempBuf, sizeof(tempBuf), "\"AdjustParams_DBSCANSMALLCarEps_a\":\"%lf\",", adjust_Params.AdjustParams_DBSCANSMALLCarEps_a);
    strcat(buf, tempBuf);

    memset(tempBuf, 0, sizeof(tempBuf));
    snprintf(tempBuf, sizeof(tempBuf), "\"AdjustParams_DBSCANSMALLCarEps_b\":\"%lf\",", adjust_Params.AdjustParams_DBSCANSMALLCarEps_b);
    strcat(buf, tempBuf);

    memset(tempBuf, 0, sizeof(tempBuf));
    snprintf(tempBuf, sizeof(tempBuf), "\"AdjustParams_EKFPairRNGTHR\":\"%lf\",", adjust_Params.AdjustParams_EKFPairRNGTHR);
    strcat(buf, tempBuf);

    memset(tempBuf, 0, sizeof(tempBuf));
    snprintf(tempBuf, sizeof(tempBuf), "\"AdjustParams_EKFPairVELTHR\":\"%lf\",", adjust_Params.AdjustParams_EKFPairVELTHR);
    strcat(buf, tempBuf);

    memset(tempBuf, 0, sizeof(tempBuf));
    snprintf(tempBuf, sizeof(tempBuf), "\"AdjustParams_EKFPairANGTHR\":\"%lf\",", adjust_Params.AdjustParams_EKFPairANGTHR);
    strcat(buf, tempBuf);

    memset(tempBuf, 0, sizeof(tempBuf));
    snprintf(tempBuf, sizeof(tempBuf), "\"AdjustParams_EKFUpdatePreTypeThres\":\"%d\",", adjust_Params.AdjustParams_EKFUpdatePreTypeThres);
    strcat(buf, tempBuf);

    memset(tempBuf, 0, sizeof(tempBuf));
    snprintf(tempBuf, sizeof(tempBuf), "\"AdjustParams_EKFUpdateValTypeThres\":\"%d\",", adjust_Params.AdjustParams_EKFUpdateValTypeThres);
    strcat(buf, tempBuf);

    memset(tempBuf, 0, sizeof(tempBuf));
    snprintf(tempBuf, sizeof(tempBuf), "\"AdjustParams_EKFFilterTimeInt\":\"%lf\",", adjust_Params.AdjustParams_EKFFilterTimeInt);
    strcat(buf, tempBuf);

    memset(tempBuf, 0, sizeof(tempBuf));
    snprintf(tempBuf, sizeof(tempBuf), "\"AdjustParams_EKFFilterACC\":\"%lf\",", adjust_Params.AdjustParams_EKFFilterACC);
    strcat(buf, tempBuf);

    memset(tempBuf, 0, sizeof(tempBuf));
    snprintf(tempBuf, sizeof(tempBuf), "\"AdjustParams_SmallCarRCSThres\":\"%lf\",", adjust_Params.AdjustParams_SmallCarRCSThres);
    strcat(buf, tempBuf);

    memset(tempBuf, 0, sizeof(tempBuf));
    snprintf(tempBuf, sizeof(tempBuf), "\"AdjustParams_MediumCarRCSThres\":\"%lf\",", adjust_Params.AdjustParams_MediumCarRCSThres);
    strcat(buf, tempBuf);

    memset(tempBuf, 0, sizeof(tempBuf));
    snprintf(tempBuf, sizeof(tempBuf), "\"AdjustParams_BigCarRCSThres\":\"%lf\",", adjust_Params.AdjustParams_BigCarRCSThres);
    strcat(buf, tempBuf);

    memset(tempBuf, 0, sizeof(tempBuf));
    snprintf(tempBuf, sizeof(tempBuf), "\"AdjustParams_V_Thr\":\"%lf\",", adjust_Params.AdjustParams_V_Thr);
    strcat(buf, tempBuf);

    memset(tempBuf, 0, sizeof(tempBuf));
    snprintf(tempBuf, sizeof(tempBuf), "\"AdjustParams_Frame_car_list\":\"%lf\",", adjust_Params.AdjustParams_Frame_car_list);
    strcat(buf, tempBuf);

    memset(tempBuf, 0, sizeof(tempBuf));
    snprintf(tempBuf, sizeof(tempBuf), "\"AdjustParams_Frame_car_stop\":\"%lf\",", adjust_Params.AdjustParams_Frame_car_stop);
    strcat(buf, tempBuf);

    memset(tempBuf, 0, sizeof(tempBuf));
    snprintf(tempBuf, sizeof(tempBuf), "\"AdjustParams_X_Thr\":\"%lf\",", adjust_Params.AdjustParams_X_Thr);
    strcat(buf, tempBuf);

    memset(tempBuf, 0, sizeof(tempBuf));
    snprintf(tempBuf, sizeof(tempBuf), "\"AdjustParams_Y_Thr\":\"%lf\",", adjust_Params.AdjustParams_Y_Thr);
    strcat(buf, tempBuf);

    memset(tempBuf, 0, sizeof(tempBuf));
    snprintf(tempBuf, sizeof(tempBuf), "\"AdjustParams_MAX_SPEED\":\"%lf\",", adjust_Params.AdjustParams_MAX_SPEED);
    strcat(buf, tempBuf);

    memset(tempBuf, 0, sizeof(tempBuf));
    snprintf(tempBuf, sizeof(tempBuf), "\"AdjustParams_LIMIT_SPEED\":\"%lf\",", adjust_Params.AdjustParams_LIMIT_SPEED);
    strcat(buf, tempBuf);

    memset(tempBuf, 0, sizeof(tempBuf));
    snprintf(tempBuf, sizeof(tempBuf), "\"AdjustParams_LIMIT_DIST\":\"%lf\",", adjust_Params.AdjustParams_LIMIT_DIST);
    strcat(buf, tempBuf);

    memset(tempBuf, 0, sizeof(tempBuf));
    snprintf(tempBuf, sizeof(tempBuf), "\"AdjustParams_EKFFilterR_RNG\":\"%lf\",", adjust_Params.AdjustParams_EKFFilterR_RNG);
    strcat(buf, tempBuf);

    memset(tempBuf, 0, sizeof(tempBuf));
    snprintf(tempBuf, sizeof(tempBuf), "\"AdjustParams_EKFFilterR_VEL\":\"%lf\",", adjust_Params.AdjustParams_EKFFilterR_VEL);
    strcat(buf, tempBuf);

    memset(tempBuf, 0, sizeof(tempBuf));
    snprintf(tempBuf, sizeof(tempBuf), "\"AdjustParams_EKFFilterR_ANG\":\"%lf\",", adjust_Params.AdjustParams_EKFFilterR_ANG);
    strcat(buf, tempBuf);

    if (strlen(buf) > 3072) {
        return -1;
    }

    return 0;
}

int user_getRoiArg(char *buf)
{
    char tempBuf[128] = {0};

    if (buf == NULL) {
        return -1;
    }

    strcat(buf, "[");
    for (int i = 0; i < adjust_Params.AdjustParams_ROI.UsingNumbofROI; ++i) {
        strcat(buf, "{");

        memset(tempBuf, 0, sizeof(tempBuf));
        snprintf(tempBuf, sizeof(tempBuf), "\"name\":\"%s\",", adjust_Params.AdjustParams_ROI.PositionofROI[i].name);
        strcat(buf, tempBuf);

        memset(tempBuf, 0, sizeof(tempBuf));
        snprintf(tempBuf, sizeof(tempBuf), "\"x1\":\"%lf\",", adjust_Params.AdjustParams_ROI.PositionofROI[i].x1);
        strcat(buf, tempBuf);

        memset(tempBuf, 0, sizeof(tempBuf));
        snprintf(tempBuf, sizeof(tempBuf), "\"y1\":\"%lf\",", adjust_Params.AdjustParams_ROI.PositionofROI[i].y1);
        strcat(buf, tempBuf);

        memset(tempBuf, 0, sizeof(tempBuf));
        snprintf(tempBuf, sizeof(tempBuf), "\"x2\":\"%lf\",", adjust_Params.AdjustParams_ROI.PositionofROI[i].x2);
        strcat(buf, tempBuf);

        memset(tempBuf, 0, sizeof(tempBuf));
        snprintf(tempBuf, sizeof(tempBuf), "\"y2\":\"%lf\",", adjust_Params.AdjustParams_ROI.PositionofROI[i].y2);
        strcat(buf, tempBuf);

        memset(tempBuf, 0, sizeof(tempBuf));
        snprintf(tempBuf, sizeof(tempBuf), "\"x3\":\"%lf\",", adjust_Params.AdjustParams_ROI.PositionofROI[i].x3);
        strcat(buf, tempBuf);

        memset(tempBuf, 0, sizeof(tempBuf));
        snprintf(tempBuf, sizeof(tempBuf), "\"y3\":\"%lf\",", adjust_Params.AdjustParams_ROI.PositionofROI[i].y3);
        strcat(buf, tempBuf);

        memset(tempBuf, 0, sizeof(tempBuf));
        snprintf(tempBuf, sizeof(tempBuf), "\"x4\":\"%lf\",", adjust_Params.AdjustParams_ROI.PositionofROI[i].x4);
        strcat(buf, tempBuf);

        memset(tempBuf, 0, sizeof(tempBuf));
        snprintf(tempBuf, sizeof(tempBuf), "\"y4\":\"%lf\"", adjust_Params.AdjustParams_ROI.PositionofROI[i].y4);
        strcat(buf, tempBuf);

        if (i < adjust_Params.AdjustParams_ROI.UsingNumbofROI - 1) {
            strcat(buf, "},");
        } else {
            strcat(buf, "}");
        }
    }
    strcat(buf, "]");

    if (strlen(buf) > 2048) {
        return -1;
    }

    return 0;
}

int user_getSection(char *buf)
{
    char tempBuf[128] = {0};

    if (buf == NULL) {
        return -1;
    }

    strcat(buf, "[");
    for (int i = 0; i < section_num; ++i) {
        strcat(buf, "{");

        memset(tempBuf, 0, sizeof(tempBuf));
        snprintf(tempBuf, sizeof(tempBuf), "\"id\":\"%d\",", sections[i].section_id);
        strcat(buf, tempBuf);

        memset(tempBuf, 0, sizeof(tempBuf));
        snprintf(tempBuf, sizeof(tempBuf), "\"section_x\":\"%lf\",", sections[i].section_x);
        strcat(buf, tempBuf);

        memset(tempBuf, 0, sizeof(tempBuf));
        snprintf(tempBuf, sizeof(tempBuf), "\"detectionCycle\":\"%lf\",", sections[i].detectionCycle);
        strcat(buf, tempBuf);

        memset(tempBuf, 0, sizeof(tempBuf));
        snprintf(tempBuf, sizeof(tempBuf), "\"coilNum\":\"%d\",", sections[i].coilNum);
        strcat(buf, tempBuf);

        memset(tempBuf, 0, sizeof(tempBuf));
        snprintf(tempBuf, sizeof(tempBuf), "\"section_status\":\"%d\"", sections[i].section_status);
        strcat(buf, tempBuf);

        if (i < (section_num - 1)) {
            strcat(buf, "},");
        } else {
            strcat(buf, "}");
        }
    }
    strcat(buf, "]");

    if (strlen(buf) > 2048) {
        return -1;
    }

    return 0;
}

int user_getCoilId(char *cont)
{
    int id;
    int keyCnt = 0;
    char *p = NULL;
    char *p2 = NULL;
    int valueCnt = 0;
    char *pComma = NULL;
    char idVal[64] = {"\0"};
    char temp[256] = {"\0"};
    char key[1][64] = {"\0"};
    char value[1][64] = {"\0"};
    char *pColon1 = NULL, *pColon2 = NULL, *pColon3 = NULL, *pColon4 = NULL;

    if (cont == NULL) {
        return -1;
    }

    for (p = strtok(cont, "\n,"); p != NULL; p = strtok(NULL, "\n,")) {
        memset(temp, '\0', sizeof(temp));
        strncpy(temp, p, strlen(p));

        pColon1 = strchr(temp, '"');
        if (pColon1 != NULL) {
            pColon2 = strchr(pColon1 + 1, '"');
            if (pColon2 != NULL) {
                if ((pColon2 - (pColon1 + 1)) > (sizeof(key[keyCnt]) - 1)) {
                    return -1;
                }

                memset(key[keyCnt], '\0', sizeof(key[keyCnt]));
                strncpy(key[keyCnt], pColon1 + 1, pColon2 - (pColon1 + 1));

                pColon3 = strchr(pColon2 + 1, '"');
                if (pColon3 != NULL) {
                    pColon4 = strchr(pColon3 + 1, '"');
                    if (pColon4 != NULL) {
                        if ((pColon4 - (pColon3 + 1)) > (sizeof(value) - 1)) {
                            return -1;
                        }

                        memset(value[valueCnt], '\0', sizeof(value[valueCnt]));
                        strncpy(value[valueCnt], pColon3 + 1, pColon4 - (pColon3 + 1));
                    } else {
                        goto goto_user_getCoilId_failed;
                    }
                } else {
                    goto goto_user_getCoilId_failed;
                }
            } else {
                goto goto_user_getCoilId_failed;
            }

            if (strcmp(key[keyCnt], "id") == 0) {
                strcpy(idVal, value[valueCnt]);
            }

            keyCnt++;
            valueCnt++;
            if (keyCnt > 1 || valueCnt > 1) {
                goto goto_user_getCoilId_failed;
            }
        } else {
            continue;
        }
    }

    id = atoi(idVal);
    return id;

goto_user_getCoilId_failed:
    return -1;
}

int user_getCoil(char *cont, char *buf)
{
    char tempBuf[128] = {0};
    int Foreign_section_id = 0;
    Foreign_section_id = user_getCoilId(cont);

    if ((cont == NULL) || (buf == NULL)) {
        return -1;
    }

    strcat(buf, "[");
    for (int i = 0; i < section_num; ++i) {
        if (sections[i].section_id == Foreign_section_id) {
            for (int j = 0; j < sections[i].coilNum; ++j) {
                strcat(buf, "{");

                memset(tempBuf, 0, sizeof(tempBuf));
                snprintf(tempBuf, sizeof(tempBuf), "\"laneNum\":\"%d\",", sections[i].CoilPosition[j].laneNum);
                strcat(buf, tempBuf);

                memset(tempBuf, 0, sizeof(tempBuf));
                snprintf(tempBuf, sizeof(tempBuf), "\"coilX\":\"%lf\",", sections[i].CoilPosition[j].coilX);
                strcat(buf, tempBuf);

                memset(tempBuf, 0, sizeof(tempBuf));
                snprintf(tempBuf, sizeof(tempBuf), "\"coilY\":\"%lf\",", sections[i].CoilPosition[j].coilY);
                strcat(buf, tempBuf);

                memset(tempBuf, 0, sizeof(tempBuf));
                snprintf(tempBuf, sizeof(tempBuf), "\"coilWidth\":\"%lf\",", sections[i].CoilPosition[j].coilWidth);
                strcat(buf, tempBuf);

                memset(tempBuf, 0, sizeof(tempBuf));
                snprintf(tempBuf, sizeof(tempBuf), "\"coilLenth\":\"%lf\",", sections[i].CoilPosition[j].coilLenth);
                strcat(buf, tempBuf);

                memset(tempBuf, 0, sizeof(tempBuf));
                snprintf(tempBuf, sizeof(tempBuf), "\"status\":\"%d\"", sections[i].CoilPosition[j].coil_status);
                strcat(buf, tempBuf);

                if (j < sections[i].coilNum - 1) {
                    strcat(buf, "},");
                } else {
                    strcat(buf, "}");
                }
            }
        }
    }
    strcat(buf, "]");

    if (strlen(buf) > 2048) {
        return -1;
    }

    return 0;
}

int user_gotoUpgradeEmbededSoft(char *buf)
{
    int ret;
    int failedFlg = 1;

    if (buf == NULL) {
        return -1;
    }

    ret = access("/run/media/mmcblk0p1/upgradeFiles.zip", F_OK);
    if (ret != 0) {
        snprintf(buf, 10, "ret:%d", 1);
        return 1;
    }

    if (g_upgradingFlg == 1) {
        snprintf(buf, 10, "ret:%d", 1);
        return 1;
    }

    g_upgradingFlg = 1;
    system("unzip -o /run/media/mmcblk0p1/upgradeFiles.zip -d /run/media/mmcblk0p1/");
    sleep(1);

    DIR *pDir = opendir("/run/media/mmcblk0p1/upgradeFiles");
    if (pDir == NULL) {
        printf("opendir /run/media/mmcblk0p1/upgradeFiles failed\n");
        snprintf(buf, 10, "ret:%d", 1);
        return 1;
    }

    struct dirent *ent = NULL;
    while ((ent = readdir(pDir)) != NULL) {
        if ((strcmp(ent->d_name, ".") == 0) || (strcmp(ent->d_name, "..") == 0)) {
            continue;
        }

        if (strcmp(ent->d_name, "gotoUpgradeShellFile.sh")) {
            failedFlg = 0;
            break;
        }
    }

    if (failedFlg == 1) {
        failedFlg = 0;
        closedir(pDir);
        snprintf(buf, 10, "ret:%d", 1);
        return 1;
    }

    closedir(pDir);
    g_upgradingFlg = 0;
    snprintf(buf, 10, "ret:%d", 0);

    return 0;
}

int user_lslhPath(char *cont, char *buf, unsigned int size)
{
    char *pBgn = NULL;
    char *pEnd = NULL;
    char path[256] = {0};
    char cmdline[286] = {0};

    if ((cont == NULL) || (buf == NULL)) {
        return -1;
    }

    if ((pBgn = strstr(cont, "\"path\":")) != NULL) {
        pBgn += strlen("\"path\":");
        if (*pBgn == '"') {
            pEnd = strchr(pBgn + 1, '"');
            if (pEnd != NULL) {
                strncpy(path, pBgn + 1, pEnd - pBgn - 1);
            }
        } else {
            strcpy(buf, "failed");
            return 0;
        }
    }

    if (strlen(path) > 0) {
        snprintf(cmdline, sizeof(cmdline), "ls -lh %s > /run/lslhPath", path);
        system(cmdline);
        usleep(100000);

        int fd = open("/run/lslhPath", O_RDONLY);
        if (fd < 0) {
            strcpy(buf, "failed");
            return 0;
        }

        if (read(fd, buf, size) < 0) {
            strcpy(buf, "failed");
            close(fd);
            return 0;
        }

        close(fd);
    } else {
        strcpy(buf, "failed");
    }

    return 0;
}

int user_httpHdlWatchdog(char *cont, char *buf, unsigned int size)
{
    int flag = 0;
    int ret = -1;
    char *pTmp = NULL;
    char *pColon1 = NULL;
    char *pColon2 = NULL;
    char flgString[32] = {"\0"};

    if ((cont == NULL) || (buf == NULL)) {
        return -1;
    }

    if ((pTmp = strstr(cont, "\"flag\":\"")) != NULL) {
        pColon1 = pTmp + strlen("\"flag\":\"");
        if ((pColon2 = strstr(pColon1, "\r\n")) != NULL) {
            strncpy(flgString, pColon1, pColon2 - pColon1);

            flag = atoi(flgString);
            if ((flag != 0) && (flag != 1)) {
                goto goto_user_httpHdlWatchdog_failed;
            }

            ret = 0;
            if (ret != 0) {
                goto goto_user_httpHdlWatchdog_failed;
            }
        } else {
            goto goto_user_httpHdlWatchdog_failed;
        }
    } else {
        goto goto_user_httpHdlWatchdog_failed;
    }

    snprintf(buf, size, "ret:%d", 0);
    return 0;

goto_user_httpHdlWatchdog_failed:
    snprintf(buf, size, "ret:%d", 1);
    return 1;
}

int user_GetClusterAndObjectSend2Rearend(char *buf, unsigned int size)
{
    if (buf == NULL) {
        return -1;
    }

    snprintf(buf, size, "mode:%d", g_clusterAndObjectSend2RearendMode);
    return 0;
}

int user_UpdateSections(char *cont, char *buf, unsigned int size)
{
    int id;
    int ret = 0;
    char *p = NULL;
    int keyCnt = 0;
    char *p2 = NULL;
    int valueCnt = 0;
    char *pComma = NULL;
    sqlite3 *db1 = NULL;
    char temp[256] = {"\0"};
    char y1Val[64] = {"\0"};
    char y2Val[64] = {"\0"};
    char idVal[64] = {"\0"};
    char key[5][64] = {"\0"};
    char nameVal[64] = {"\0"};
    char value[5][64] = {"\0"};
    char statusVal[64] = {"\0"};
    char section_xVal[64] = {"\0"};
    char detectionCycleVal[64] = {"\0"};
    char *pColon1 = NULL, *pColon2 = NULL, *pColon3 = NULL, *pColon4 = NULL;
    char *pCont[3] = {&section_xVal[0], &detectionCycleVal[0], &statusVal[0]};

    if ((cont == NULL) || (buf == NULL)) {
        return -1;
    }

    for (p = strtok(cont, "\n,"); p != NULL; p = strtok(NULL, "\n,")) {
        memset(temp, '\0', sizeof(temp));
        strncpy(temp, p, strlen(p));

        pColon1 = strchr(temp, '"');
        if (pColon1 != NULL) {
            pColon2 = strchr(pColon1 + 1, '"');
            if (pColon2 != NULL) {
                if ((pColon2 - (pColon1 + 1)) > (sizeof(key[keyCnt]) - 1)) {
                    return -1;
                }

                memset(key[keyCnt], '\0', sizeof(key[keyCnt]));
                strncpy(key[keyCnt], pColon1 + 1, pColon2 - (pColon1 + 1));

                pColon3 = strchr(pColon2 + 1, '"');
                if (pColon3 != NULL) {
                    pColon4 = strchr(pColon3 + 1, '"');
                    if (pColon4 != NULL) {
                        if ((pColon4 - (pColon3 + 1)) > (sizeof(value) - 1)) {
                            return -1;
                        }

                        memset(value[valueCnt], '\0', sizeof(value[valueCnt]));
                        strncpy(value[valueCnt], pColon3 + 1, pColon4 - (pColon3 + 1));
                    } else {
                        goto goto_user_UpdateSections_failed;
                    }
                } else {
                    goto goto_user_UpdateSections_failed;
                }
            } else {
                goto goto_user_UpdateSections_failed;
            }

            if (strcmp(key[keyCnt], "section_x") == 0) {
                strcpy(section_xVal, value[valueCnt]);
            } else if (strcmp(key[keyCnt], "detectionCycle") == 0) {
                strcpy(detectionCycleVal, value[valueCnt]);
            } else if (strcmp(key[keyCnt], "status") == 0) {
                strcpy(statusVal, value[valueCnt]);
            } else if (strcmp(key[keyCnt], "id") == 0) {
                strcpy(idVal, value[valueCnt]);
            }

            keyCnt++;
            valueCnt++;
            if (keyCnt > 4 || valueCnt > 4) {
                goto goto_user_UpdateSections_failed;
            }
        } else {
            continue;
        }
    }

    ret = sqlite3_open(ADJUST_DB_FILE, &db1);
    if (ret != SQLITE_OK) {
        spdlog::error("open db {0} failed", ADJUST_DB_FILE);
        return 1;
    }

    id = atoi(idVal);
    ret = UpdateSections(db1, id, pCont);
    if (ret == 0) {
        sqlite3_close(db1);
    }

    if (ret != 0) {
        goto goto_user_UpdateSections_failed;
    }

    snprintf(buf, size, "ret:%d", 0);
    user_flushParams(NULL);
    return 0;

goto_user_UpdateSections_failed:
    snprintf(buf, size, "ret:%d", 1);
    return 1;
}

int user_InsertSections(char *cont, char *buf, unsigned int size)
{
    int ret = 0;
    char *p = NULL;
    int keyCnt = 0;
    char *p2 = NULL;
    int valueCnt = 0;
    sqlite3 *db1 = NULL;
    char *pComma = NULL;
    char temp[256] = {"\0"};
    char y1Val[64] = {"\0"};
    char y2Val[64] = {"\0"};
    char key[5][64] = {"\0"};
    char nameVal[64] = {"\0"};
    char value[5][64] = {"\0"};
    char statusVal[64] = {"\0"};
    char coilNumVal[64] = {"\0"};
    char section_xVal[64] = {"\0"};
    char detectionCycleVal[64] = {"\0"};
    char *pColon1 = NULL, *pColon2 = NULL, *pColon3 = NULL, *pColon4 = NULL;
    char *pCont[4] = {&section_xVal[0], &detectionCycleVal[0], &coilNumVal[0], &statusVal[0]};

    if ((cont == NULL) || (buf == NULL)) {
        return -1;
    }

    for (p = strtok(cont, "\n,"); p != NULL; p = strtok(NULL, "\n,")) {
        memset(temp, '\0', sizeof(temp));
        strncpy(temp, p, strlen(p));

        pColon1 = strchr(temp, '"');
        if (pColon1 != NULL) {
            pColon2 = strchr(pColon1 + 1, '"');
            if (pColon2 != NULL) {
                if ((pColon2 - (pColon1 + 1)) > (sizeof(key[keyCnt]) - 1)) {
                    return -1;
                }

                memset(key[keyCnt], '\0', sizeof(key[keyCnt]));
                strncpy(key[keyCnt], pColon1 + 1, pColon2 - (pColon1 + 1));

                pColon3 = strchr(pColon2 + 1, '"');
                if (pColon3 != NULL) {
                    pColon4 = strchr(pColon3 + 1, '"');
                    if (pColon4 != NULL) {
                        if ((pColon4 - (pColon3 + 1)) > (sizeof(value) - 1)) {
                            return -1;
                        }

                        memset(value[valueCnt], '\0', sizeof(value[valueCnt]));
                        strncpy(value[valueCnt], pColon3 + 1, pColon4 - (pColon3 + 1));
                    } else {
                        goto goto_user_InsertSections_failed;
                    }
                } else {
                    goto goto_user_InsertSections_failed;
                }
            } else {
                goto goto_user_InsertSections_failed;
            }

            if (strcmp(key[keyCnt], "section_x") == 0) {
                strcpy(section_xVal, value[valueCnt]);
            } else if (strcmp(key[keyCnt], "detectionCycle") == 0) {
                strcpy(detectionCycleVal, value[valueCnt]);
            } else if (strcmp(key[keyCnt], "status") == 0) {
                strcpy(statusVal, value[valueCnt]);
            } else if (strcmp(key[keyCnt], "coilNum") == 0) {
                strcpy(coilNumVal, value[valueCnt]);
            }

            keyCnt++;
            valueCnt++;
            if (keyCnt > 4 || valueCnt > 4) {
                goto goto_user_InsertSections_failed;
            }
        } else {
            continue;
        }
    }

    ret = sqlite3_open(ADJUST_DB_FILE, &db1);
    if (ret != SQLITE_OK) {
        spdlog::error("open db {0} failed", ADJUST_DB_FILE);
        return 1;
    }

    ret = InsertSections(db1, pCont);
    if (ret == 0) {
        sqlite3_close(db1);
    }

    if (ret != 0) {
        goto goto_user_InsertSections_failed;
    }

    snprintf(buf, size, "ret:%d", 0);
    user_flushParams(NULL);
    return 0;

goto_user_InsertSections_failed:
    snprintf(buf, size, "ret:%d", 1);
    return 1;
}

int user_DeleteSections(char *cont, char *buf, unsigned int size)
{
    int id;
    int ret = 0;
    char *p = NULL;
    int keyCnt = 0;
    char *p2 = NULL;
    int valueCnt = 0;
    sqlite3 *db1 = NULL;
    char *pComma = NULL;
    char temp[256] = {"\0"};
    char idVal[64] = {"\0"};
    char key[1][64] = {"\0"};
    char value[1][64] = {"\0"};
    char *pColon1 = NULL, *pColon2 = NULL, *pColon3 = NULL, *pColon4 = NULL;

    if ((cont == NULL) || (buf == NULL)) {
        return -1;
    }

    for (p = strtok(cont, "\n,"); p != NULL; p = strtok(NULL, "\n,")) {
        memset(temp, '\0', sizeof(temp));
        strncpy(temp, p, strlen(p));

        pColon1 = strchr(temp, '"');
        if (pColon1 != NULL) {
            pColon2 = strchr(pColon1 + 1, '"');
            if (pColon2 != NULL) {
                if ((pColon2 - (pColon1 + 1)) > (sizeof(key[keyCnt]) - 1)) {
                    return -1;
                }

                memset(key[keyCnt], '\0', sizeof(key[keyCnt]));
                strncpy(key[keyCnt], pColon1 + 1, pColon2 - (pColon1 + 1));

                pColon3 = strchr(pColon2 + 1, '"');
                if (pColon3 != NULL) {
                    pColon4 = strchr(pColon3 + 1, '"');
                    if (pColon4 != NULL) {
                        if ((pColon4 - (pColon3 + 1)) > (sizeof(value) - 1)) {
                            return -1;
                        }

                        memset(value[valueCnt], '\0', sizeof(value[valueCnt]));
                        strncpy(value[valueCnt], pColon3 + 1, pColon4 - (pColon3 + 1));
                    } else {
                        goto goto_user_DeleteSections_failed;
                    }
                } else {
                    goto goto_user_DeleteSections_failed;
                }
            } else {
                goto goto_user_DeleteSections_failed;
            }

            if (strcmp(key[keyCnt], "id") == 0) {
                strcpy(idVal, value[valueCnt]);
            }

            keyCnt++;
            valueCnt++;
            if (keyCnt > 1 || valueCnt > 1) {
                goto goto_user_DeleteSections_failed;
            }
        } else {
            continue;
        }
    }

    ret = sqlite3_open(ADJUST_DB_FILE, &db1);
    if (ret != SQLITE_OK) {
        spdlog::error("open db {0} failed", ADJUST_DB_FILE);
        return 1;
    }

    id = atoi(idVal);
    ret = DeleteSections(db1, id);
    if (ret == 0) {
        sqlite3_close(db1);
    }

    if (ret != 0) {
        goto goto_user_DeleteSections_failed;
    }

    snprintf(buf, size, "ret:%d", 0);
    user_flushParams(NULL);
    return 0;

goto_user_DeleteSections_failed:
    snprintf(buf, size, "ret:%d", 1);
    return 1;
}

int user_UpdateCoils(char *cont, char *buf, unsigned int size)
{
    int id;
    int ret = 0;
    int laneNum;
    int keyCnt = 0;
    char *p = NULL;
    char *p2 = NULL;
    int valueCnt = 0;
    sqlite3 *db1 = NULL;
    char *pComma = NULL;
    char y1Val[64] = {"\0"};
    char temp[256] = {"\0"};
    char y2Val[64] = {"\0"};
    char idVal[64] = {"\0"};
    char key[7][64] = {"\0"};
    char nameVal[64] = {"\0"};
    char coilXVal[64] = {"\0"};
    char coilYVal[64] = {"\0"};
    char value[7][64] = {"\0"};
    char statusVal[64] = {"\0"};
    char laneNumVal[64] = {"\0"};
    char coilwidthVal[64] = {"\0"};
    char coilLenthVal[64] = {"\0"};
    char section_xVal[64] = {"\0"};
    char detectionCycleVal[64] = {"\0"};
    char *pColon1 = NULL, *pColon2 = NULL, *pColon3 = NULL, *pColon4 = NULL;
    char *pCont[5] = {&coilXVal[0], &coilYVal[0], &coilwidthVal[0], &coilLenthVal[0], &statusVal[0]};

    if ((cont == NULL) || (buf == NULL)) {
        return -1;
    }

    for (p = strtok(cont, "\n,"); p != NULL; p = strtok(NULL, "\n,")) {
        memset(temp, '\0', sizeof(temp));
        strncpy(temp, p, strlen(p));

        pColon1 = strchr(temp, '"');
        if (pColon1 != NULL) {
            pColon2 = strchr(pColon1 + 1, '"');
            if (pColon2 != NULL) {
                if ((pColon2 - (pColon1 + 1)) > (sizeof(key[keyCnt]) - 1)) {
                    return -1;
                }

                memset(key[keyCnt], '\0', sizeof(key[keyCnt]));
                strncpy(key[keyCnt], pColon1 + 1, pColon2 - (pColon1 + 1));

                pColon3 = strchr(pColon2 + 1, '"');
                if (pColon3 != NULL) {
                    pColon4 = strchr(pColon3 + 1, '"');
                    if (pColon4 != NULL) {
                        if ((pColon4 - (pColon3 + 1)) > (sizeof(value) - 1)) {
                            return -1;
                        }

                        memset(value[valueCnt], '\0', sizeof(value[valueCnt]));
                        strncpy(value[valueCnt], pColon3 + 1, pColon4 - (pColon3 + 1));
                    } else {
                        goto goto_user_UpdateCoils_failed;
                    }
                } else {
                    goto goto_user_UpdateCoils_failed;
                }
            } else {
                goto goto_user_UpdateCoils_failed;
            }

            if (strcmp(key[keyCnt], "coilX") == 0) {
                strcpy(coilXVal, value[valueCnt]);
            } else if (strcmp(key[keyCnt], "coilY") == 0) {
                strcpy(coilYVal, value[valueCnt]);
            } else if (strcmp(key[keyCnt], "coilwidth") == 0) {
                strcpy(coilwidthVal, value[valueCnt]);
            } else if (strcmp(key[keyCnt], "coilLenth") == 0) {
                strcpy(coilLenthVal, value[valueCnt]);
            } else if (strcmp(key[keyCnt], "status") == 0) {
                strcpy(statusVal, value[valueCnt]);
            } else if (strcmp(key[keyCnt], "id") == 0) {
                strcpy(idVal, value[valueCnt]);
            } else if (strcmp(key[keyCnt], "laneNum") == 0) {
                strcpy(laneNumVal, value[valueCnt]);
            }

            keyCnt++;
            valueCnt++;
            if (keyCnt > 7 || valueCnt > 7) {
                goto goto_user_UpdateCoils_failed;
            }
        } else {
            continue;
        }
    }

    ret = sqlite3_open(ADJUST_DB_FILE, &db1);
    if (ret != SQLITE_OK) {
        spdlog::error("open db {0} failed", ADJUST_DB_FILE);
        return 1;
    }

    id = atoi(idVal);
    laneNum = atoi(laneNumVal);
    ret = UpdateCoils(db1, id, laneNum, pCont);
    if (ret == 0) {
        sqlite3_close(db1);
    }

    if (ret != 0) {
        goto goto_user_UpdateCoils_failed;
    }

    snprintf(buf, size, "ret:%d", 0);
    user_flushParams(NULL);
    return 0;

goto_user_UpdateCoils_failed:
    snprintf(buf, size, "ret:%d", 1);
    return 1;
}

int user_InsertCoils(char *cont, char *buf, unsigned int size)
{
    int id;
    int ret = 0;
    int laneNum;
    int keyCnt = 0;
    char *p = NULL;
    char *p2 = NULL;
    int valueCnt = 0;
    sqlite3 *db1 = NULL;
    char *pComma = NULL;
    char y1Val[64] = {"\0"};
    char temp[256] = {"\0"};
    char y2Val[64] = {"\0"};
    char idVal[64] = {"\0"};
    char key[7][64] = {"\0"};
    char nameVal[64] = {"\0"};
    char coilXVal[64] = {"\0"};
    char coilYVal[64] = {"\0"};
    char value[7][64] = {"\0"};
    char statusVal[64] = {"\0"};
    char laneNumVal[64] = {"\0"};
    char coilwidthVal[64] = {"\0"};
    char coilLenthVal[64] = {"\0"};
    char section_xVal[64] = {"\0"};
    char detectionCycleVal[64] = {"\0"};
    char *pColon1 = NULL, *pColon2 = NULL, *pColon3 = NULL, *pColon4 = NULL;
    char *pCont[6] = {&laneNumVal[0], &coilXVal[0], &coilYVal[0], &coilwidthVal[0], &coilLenthVal[0], &statusVal[0]};

    if ((cont == NULL) || (buf == NULL)) {
        return -1;
    }

    for (p = strtok(cont, "\n,"); p != NULL; p = strtok(NULL, "\n,")) {
        memset(temp, '\0', sizeof(temp));
        strncpy(temp, p, strlen(p));

        pColon1 = strchr(temp, '"');
        if (pColon1 != NULL) {
            pColon2 = strchr(pColon1 + 1, '"');
            if (pColon2 != NULL) {
                if ((pColon2 - (pColon1 + 1)) > (sizeof(key[keyCnt]) - 1)) {
                    return -1;
                }

                memset(key[keyCnt], '\0', sizeof(key[keyCnt]));
                strncpy(key[keyCnt], pColon1 + 1, pColon2 - (pColon1 + 1));

                pColon3 = strchr(pColon2 + 1, '"');
                if (pColon3 != NULL) {
                    pColon4 = strchr(pColon3 + 1, '"');
                    if (pColon4 != NULL) {
                        if ((pColon4 - (pColon3 + 1)) > (sizeof(value) - 1)) {
                            return -1;
                        }

                        memset(value[valueCnt], '\0', sizeof(value[valueCnt]));
                        strncpy(value[valueCnt], pColon3 + 1, pColon4 - (pColon3 + 1));
                    } else {
                        goto goto_user_InsertCoils_failed;
                    }
                } else {
                    goto goto_user_InsertCoils_failed;
                }
            } else {
                goto goto_user_InsertCoils_failed;
            }

            if (strcmp(key[keyCnt], "coilX") == 0) {
                strcpy(coilXVal, value[valueCnt]);
            } else if (strcmp(key[keyCnt], "coilY") == 0) {
                strcpy(coilYVal, value[valueCnt]);
            } else if (strcmp(key[keyCnt], "coilwidth") == 0) {
                strcpy(coilwidthVal, value[valueCnt]);
            } else if (strcmp(key[keyCnt], "coilLenth") == 0) {
                strcpy(coilLenthVal, value[valueCnt]);
            } else if (strcmp(key[keyCnt], "status") == 0) {
                strcpy(statusVal, value[valueCnt]);
            } else if (strcmp(key[keyCnt], "id") == 0) {
                strcpy(idVal, value[valueCnt]);
            } else if (strcmp(key[keyCnt], "laneNum") == 0) {
                strcpy(laneNumVal, value[valueCnt]);
            }

            keyCnt++;
            valueCnt++;
            if ((keyCnt > 7) || (valueCnt > 7)) {
                goto goto_user_InsertCoils_failed;
            }
        } else {
            continue;
        }
    }

    ret = sqlite3_open(ADJUST_DB_FILE, &db1);
    if (ret != SQLITE_OK) {
        spdlog::error("open db {0} failed", ADJUST_DB_FILE);
        return 1;
    }

    id = atoi(idVal);
    ret = InsertCoils(db1, id, pCont);
    if (ret == 0) {
        sqlite3_close(db1);
    }

    if (ret != 0) {
        goto goto_user_InsertCoils_failed;
    }

    snprintf(buf, size, "ret:%d", 0);
    user_flushParams(NULL);
    return 0;

goto_user_InsertCoils_failed:
    snprintf(buf, size, "ret:%d", 1);
    return 1;
}

int user_DeleteCoils(char *cont, char *buf, unsigned int size)
{
    int id;
    int ret = 0;
    int laneNum;
    char *p = NULL;
    int keyCnt = 0;
    char *p2 = NULL;
    int valueCnt = 0;
    sqlite3 *db1 = NULL;
    char *pComma = NULL;
    char temp[256] = {"\0"};
    char idVal[64] = {"\0"};
    char key[2][64] = {"\0"};
    char value[2][64] = {"\0"};
    char laneNumVal[64] = {"\0"};
    char *pColon1 = NULL, *pColon2 = NULL, *pColon3 = NULL, *pColon4 = NULL;

    if ((cont == NULL) || (buf == NULL)) {
        return -1;
    }

    for (p = strtok(cont, "\n,"); p != NULL; p = strtok(NULL, "\n,")) {
        memset(temp, '\0', sizeof(temp));
        strncpy(temp, p, strlen(p));

        pColon1 = strchr(temp, '"');
        if (pColon1 != NULL) {
            pColon2 = strchr(pColon1 + 1, '"');
            if (pColon2 != NULL) {
                if ((pColon2 - (pColon1 + 1)) > (sizeof(key[keyCnt]) - 1)) {
                    return -1;
                }

                memset(key[keyCnt], '\0', sizeof(key[keyCnt]));
                strncpy(key[keyCnt], pColon1 + 1, pColon2 - (pColon1 + 1));

                pColon3 = strchr(pColon2 + 1, '"');
                if (pColon3 != NULL) {
                    pColon4 = strchr(pColon3 + 1, '"');
                    if (pColon4 != NULL) {
                        if ((pColon4 - (pColon3 + 1)) > (sizeof(value) - 1)) {
                            return -1;
                        }

                        memset(value[valueCnt], '\0', sizeof(value[valueCnt]));
                        strncpy(value[valueCnt], pColon3 + 1, pColon4 - (pColon3 + 1));
                    } else {
                        goto goto_user_DeleteCoils_failed;
                    }
                } else {
                    goto goto_user_DeleteCoils_failed;
                }
            } else {
                goto goto_user_DeleteCoils_failed;
            }

            if (strcmp(key[keyCnt], "id") == 0) {
                strcpy(idVal, value[valueCnt]);
            } else if (strcmp(key[keyCnt], "laneNum") == 0) {
                strcpy(laneNumVal, value[valueCnt]);
            }

            keyCnt++;
            valueCnt++;
            if ((keyCnt > 2) || (valueCnt > 2)) {
                goto goto_user_DeleteCoils_failed;
            }
        } else {
            continue;
        }
    }

    ret = sqlite3_open(ADJUST_DB_FILE, &db1);
    if (ret != SQLITE_OK) {
        spdlog::error("open db {0} failed", ADJUST_DB_FILE);
        return 1;
    }

    id = atoi(idVal);
    laneNum = atoi(laneNumVal);
    ret = DeleteCoils(db1, id, laneNum);
    if (ret == 0) {
        sqlite3_close(db1);
    }

    if (ret != 0) {
        goto goto_user_DeleteCoils_failed;
    }

    snprintf(buf, size, "ret:%d", 0);
    user_flushParams(NULL);
    return 0;

goto_user_DeleteCoils_failed:
    snprintf(buf, size, "ret:%d", 1);
    return 1;
}

int user_writeTrafficStatInfo(char *buf, int size)
{
    int fd;
    int ret;
    int cnt = 0;

    if ((buf == NULL) || (size <= 0)) {
        return -1;
    }

    fd = open("/var/volatile/tmp/TrafficFlowDetectionStatistics.txt", O_WRONLY | O_CREAT | O_TRUNC, 0666);
    if (fd < 0) {
        spdlog::error("open /var/volatile/tmp/TrafficFlowDetectionStatistics.txt failed");
        return -1;
    }

    while (1) {
        if (flock(fd, LOCK_NB | LOCK_EX) != 0) {
            if (errno == EWOULDBLOCK) {
                usleep(10000);
                cnt++;
                if (cnt >= 10) {
                    flock(fd, LOCK_UN);
                    close(fd);
                    return -1;
                }

                continue;
            }
        } else {
            break;
        }
    }

    write(fd, buf, size);
    flock(fd, LOCK_UN);
    close(fd);

    return 0;
}

int user_getTrafficFlowDetectionStatistics(char *buf, unsigned int size, int wrFileFlg)
{
    int i = 0;
    char tempBuf[128] = {0};

    if (buf == NULL) {
        return -1;
    }

    pthread_mutex_lock(&g_mtxTrafficFlowDetectionStatistics);

    strcat(buf, "{");

    strcat(buf, "\"trafficFlowData\":");
    memset(tempBuf, 0, sizeof(tempBuf));
    snprintf(tempBuf, sizeof(tempBuf), "\"%d\",", trafficFlowDetectionStatisticsDef.trafficFlowData);
    strcat(buf, tempBuf);

    strcat(buf, "\"laneDividedTrafficFlowData\":");
    strcat(buf, "[");
    for (i = 0; i < LEN; ++i) {
        strcat(buf, "{");
        strcat(buf, "\"FlowData\":");
        memset(tempBuf, 0, sizeof(tempBuf));
        snprintf(tempBuf, sizeof(tempBuf), "\"%d\"", trafficFlowDetectionStatisticsDef.laneDividedTrafficFlowData[i]);
        strcat(buf, tempBuf);

        if (i == (LEN - 1)) {
            strcat(buf, "}");
        } else {
            strcat(buf, "},");
        }
    }
    strcat(buf, "],");

    strcat(buf, "\"sectionAverageSpeedData\":");
    memset(tempBuf, 0, sizeof(tempBuf));
    snprintf(tempBuf, sizeof(tempBuf), "\"%f\",", trafficFlowDetectionStatisticsDef.sectionAverageSpeedData);
    strcat(buf, tempBuf);

    strcat(buf, "\"laneDividedAverageSpeedData\":");
    strcat(buf, "[");
    for (i = 0; i < LEN; ++i) {
        strcat(buf, "{");
        strcat(buf, "\"SpeedData\":");
        memset(tempBuf, 0, sizeof(tempBuf));
        snprintf(tempBuf, sizeof(tempBuf), "\"%f\"", trafficFlowDetectionStatisticsDef.laneDividedAverageSpeedData[i]);
        strcat(buf, tempBuf);

        if (i == (LEN - 1)) {
            strcat(buf, "}");
        } else {
            strcat(buf, "},");
        }
    }
    strcat(buf, "],");

    strcat(buf, "\"laneDividedTimeOccupancyData\":");
    strcat(buf, "[");
    for (i = 0; i < LEN; ++i) {
        strcat(buf, "{");
        strcat(buf, "\"OccupancyData\":");
        memset(tempBuf, 0, sizeof(tempBuf));
        snprintf(tempBuf, sizeof(tempBuf), "\"%f\"", trafficFlowDetectionStatisticsDef.laneDividedTimeOccupancyData[i]);
        strcat(buf, tempBuf);

        if (i == (LEN - 1)) {
            strcat(buf, "}");
        } else {
            strcat(buf, "},");
        }
    }
    strcat(buf, "],");

    strcat(buf, "\"laneDividedAverageHeadwayData\":");
    strcat(buf, "[");
    for (i = 0; i < LEN; ++i) {
        strcat(buf, "{");
        strcat(buf, "\"HeadwayData\":");
        memset(tempBuf, 0, sizeof(tempBuf));
        snprintf(tempBuf, sizeof(tempBuf), "\"%f\"", trafficFlowDetectionStatisticsDef.laneDividedAverageHeadwayData[i]);
        strcat(buf, tempBuf);

        if (i == (LEN - 1)) {
            strcat(buf, "}");
        } else {
            strcat(buf, "},");
        }
    }
    strcat(buf, "],");

    strcat(buf, "\"laneDividedAverageHeadwayGapData\":");
    strcat(buf, "[");
    for (i = 0; i < LEN; ++i) {
        strcat(buf, "{");
        strcat(buf, "\"HeadwayGapData\":");
        memset(tempBuf, 0, sizeof(tempBuf));
        snprintf(tempBuf, sizeof(tempBuf), "\"%f\"", trafficFlowDetectionStatisticsDef.laneDividedAverageHeadwayGapData[i]);
        strcat(buf, tempBuf);

        if (i == (LEN - 1)) {
            strcat(buf, "}");
        } else {
            strcat(buf, "},");
        }
    }
    strcat(buf, "]");

    strcat(buf, "}");

    if ((wrFileFlg == 1) && (strlen(buf) < size)) {
        user_writeTrafficStatInfo(buf, strlen(buf));
    }

    pthread_mutex_unlock(&g_mtxTrafficFlowDetectionStatistics);

    if (strlen(buf) > (size - 1)) {
        memset(buf, 0, size);
        strcpy(buf, "failed");
        return -1;
    }

    return 0;
}

static const char *pCalibrationParaStrs[] = {
    "matrix_1",
    "matrix_2",
    "matrix_3",
    "matrix_4",
    "matrix_5",
    "matrix_6",
    "p00",
    "p10",
    "p01",
    "p20",
    "p11",
    "p02",
    "origin_longitude",
    "origin_latitude",
    "angle",
    "is_del",
    "status",
    "create_by",
    "create_date",
    "update_by",
    "update_date"
};

static int user_findCalibrationPara(char *key)
{
    if (key == NULL) {
        return -1;
    }

    int i = 0, size = sizeof(pCalibrationParaStrs) / sizeof(pCalibrationParaStrs[0]);
    for (i = 0; i < size; ++i) {
        if (strcmp(key, pCalibrationParaStrs[i]) == 0) {
            return 0;
        }
    }

    return -1;
}

int user_postCalibrationPara(char *cont)
{
    char *p = NULL;
    char *p2 = NULL;
    char *pComma = NULL;
    char key[64] = {"\0"};
    char value[64] = {"\0"};
    char temp[256] = {"\0"};
    char *pColon1 = NULL, *pColon2 = NULL, *pColon3 = NULL, *pColon4 = NULL;

    if (cont == NULL) {
        return -1;
    }

    for (p = strtok(cont, "\n,"); p != NULL; p = strtok(NULL, "\n,")) {
        memset(temp, '\0', sizeof(temp));
        strncpy(temp, p, strlen(p));

        pColon1 = strchr(temp, '"');
        if (pColon1 != NULL) {
            pColon2 = strchr(pColon1 + 1, '"');
            if (pColon2 != NULL) {
                if ((pColon2 - (pColon1 + 1)) > (sizeof(key) - 1)) {
                    return -1;
                }

                memset(key, '\0', sizeof(key));
                strncpy(key, pColon1 + 1, pColon2 - (pColon1 + 1));

                pColon3 = strchr(pColon2 + 1, '"');
                if (pColon3 != NULL) {
                    pColon4 = strchr(pColon3 + 1, '"');
                    if (pColon4 != NULL) {
                        if ((pColon4 - (pColon3 + 1)) > (sizeof(value) - 1)) {
                            return -1;
                        }

                        memset(value, '\0', sizeof(value));
                        strncpy(value, pColon3 + 1, pColon4 - (pColon3 + 1));

                        if (user_findCalibrationPara(key) != 0) {
                            return -1;
                        }

                        int ret = 0;
                        sqlite3 *db1 = NULL;
                        char *err_msg = NULL;

                        ret = sqlite3_open(CARDB_DB_FILE, &db1);
                        if (ret != SQLITE_OK) {
                            spdlog::error("open db {0} failed", CARDB_DB_FILE);
                            return 1;
                        }

                        {
                            /* 更新 */
                            int ret = 0;
                            char sql_update[512] = {0};

                            sprintf(sql_update, "update markers_model set %s = '%s'", key, value);
                            ret = sqlite3_exec(db1, sql_update, 0, 0, &err_msg);
                            if (ret != SQLITE_OK) {
                                sqlite3_close(db1);
                                sqlite3_free(err_msg);
                                return -1;
                            }
                        }

                        sqlite3_close(db1);
                    } else {
                        return -1;
                    }
                } else {
                    return -1;
                }
            } else {
                return -1;
            }
        } else {
            continue;
        }
    }

    user_readCalibrationPara();
    return 0;
}

int user_getCalibrationPara(char *buf)
{
    char tempBuf[128] = {"\0"};

    if (buf == NULL) {
        return -1;
    }

    strcat(buf, "{\n");

    memset(tempBuf, 0, sizeof(tempBuf));
    snprintf(tempBuf, sizeof(tempBuf), "\"matrix_1\":\"%.15llf\",\n", g_CalbPara.matrix_1);
    strcat(buf, tempBuf);

    memset(tempBuf, 0, sizeof(tempBuf));
    snprintf(tempBuf, sizeof(tempBuf), "\"matrix_2\":\"%.15llf\",\n", g_CalbPara.matrix_2);
    strcat(buf, tempBuf);

    memset(tempBuf, 0, sizeof(tempBuf));
    snprintf(tempBuf, sizeof(tempBuf), "\"matrix_3\":\"%.15llf\",\n", g_CalbPara.matrix_3);
    strcat(buf, tempBuf);

    memset(tempBuf, 0, sizeof(tempBuf));
    snprintf(tempBuf, sizeof(tempBuf), "\"matrix_4\":\"%.15llf\",\n", g_CalbPara.matrix_4);
    strcat(buf, tempBuf);

    memset(tempBuf, 0, sizeof(tempBuf));
    snprintf(tempBuf, sizeof(tempBuf), "\"matrix_5\":\"%.15llf\",\n", g_CalbPara.matrix_5);
    strcat(buf, tempBuf);

    memset(tempBuf, 0, sizeof(tempBuf));
    snprintf(tempBuf, sizeof(tempBuf), "\"matrix_6\":\"%.15llf\",\n", g_CalbPara.matrix_6);
    strcat(buf, tempBuf);

    memset(tempBuf, 0, sizeof(tempBuf));
    snprintf(tempBuf, sizeof(tempBuf), "\"p00\":\"%.15llf\",\n", g_CalbPara.p00);
    strcat(buf, tempBuf);

    memset(tempBuf, 0, sizeof(tempBuf));
    snprintf(tempBuf, sizeof(tempBuf), "\"p10\":\"%.15llf\",\n", g_CalbPara.p10);
    strcat(buf, tempBuf);

    memset(tempBuf, 0, sizeof(tempBuf));
    snprintf(tempBuf, sizeof(tempBuf), "\"p01\":\"%.15llf\",\n", g_CalbPara.p01);
    strcat(buf, tempBuf);

    memset(tempBuf, 0, sizeof(tempBuf));
    snprintf(tempBuf, sizeof(tempBuf), "\"p20\":\"%.15llf\",\n", g_CalbPara.p20);
    strcat(buf, tempBuf);

    memset(tempBuf, 0, sizeof(tempBuf));
    snprintf(tempBuf, sizeof(tempBuf), "\"p11\":\"%.15llf\",\n", g_CalbPara.p11);
    strcat(buf, tempBuf);

    memset(tempBuf, 0, sizeof(tempBuf));
    snprintf(tempBuf, sizeof(tempBuf), "\"p02\":\"%.15llf\",\n", g_CalbPara.p02);
    strcat(buf, tempBuf);

    memset(tempBuf, 0, sizeof(tempBuf));
    snprintf(tempBuf, sizeof(tempBuf), "\"origin_longitude\":\"%.15llf\",\n", g_CalbPara.origin_longitude);
    strcat(buf, tempBuf);

    memset(tempBuf, 0, sizeof(tempBuf));
    snprintf(tempBuf, sizeof(tempBuf), "\"origin_latitude\":\"%.15llf\",\n", g_CalbPara.origin_latitude);
    strcat(buf, tempBuf);

    memset(tempBuf, 0, sizeof(tempBuf));
    snprintf(tempBuf, sizeof(tempBuf), "\"angle\":\"%.15llf\",\n", g_CalbPara.angle);
    strcat(buf, tempBuf);

    memset(tempBuf, 0, sizeof(tempBuf));
    snprintf(tempBuf, sizeof(tempBuf), "\"is_del\":\"%.15llf\",\n", g_CalbPara.is_del);
    strcat(buf, tempBuf);

    memset(tempBuf, 0, sizeof(tempBuf));
    snprintf(tempBuf, sizeof(tempBuf), "\"status\":\"%.15llf\",\n", g_CalbPara.status);
    strcat(buf, tempBuf);

    memset(tempBuf, 0, sizeof(tempBuf));
    snprintf(tempBuf, sizeof(tempBuf), "\"create_by\":\"%.15llf\",\n", g_CalbPara.create_by);
    strcat(buf, tempBuf);

    memset(tempBuf, 0, sizeof(tempBuf));
    snprintf(tempBuf, sizeof(tempBuf), "\"create_date\":\"%.15llf\",\n", g_CalbPara.create_date);
    strcat(buf, tempBuf);

    memset(tempBuf, 0, sizeof(tempBuf));
    snprintf(tempBuf, sizeof(tempBuf), "\"update_by\":\"%.15llf\",\n", g_CalbPara.update_by);
    strcat(buf, tempBuf);

    memset(tempBuf, 0, sizeof(tempBuf));
    snprintf(tempBuf, sizeof(tempBuf), "\"update_date\":\"%.15llf\"\n", g_CalbPara.update_date);
    strcat(buf, tempBuf);

    strcat(buf, "}\n");

    if (strlen(buf) > 3072) {
        return -1;
    }

    return 0;
}

static const char *pStaticCalibrationParaStrs[] = {
    "matrix_1",
    "matrix_2",
    "matrix_3",
    "matrix_4",
    "matrix_5",
    "matrix_6",
    "p00",
    "p10",
    "p01",
    "p20",
    "p11",
    "p02",
    "origin_longitude",
    "origin_latitude",
    "angle",
    "is_del",
    "status",
    "create_by",
    "create_date",
    "update_by",
    "update_date"
};

static int user_findStaticCalibrationPara(char *key)
{
    if (key == NULL) {
        return -1;
    }

    int i = 0, size = sizeof(pStaticCalibrationParaStrs) / sizeof(pStaticCalibrationParaStrs[0]);
    for (i = 0; i < size; ++i) {
        if (strcmp(key, pStaticCalibrationParaStrs[i]) == 0) {
            return 0;
        }
    }

    return -1;
}

int user_postStaticCalibrationPara(char *cont)
{
    char *p = NULL;
    char *p2 = NULL;
    char *pComma = NULL;
    char key[64] = {"\0"};
    char value[64] = {"\0"};
    char temp[256] = {"\0"};
    char *pColon1 = NULL, *pColon2 = NULL, *pColon3 = NULL, *pColon4 = NULL;

    if (cont == NULL) {
        return -1;
    }

    for (p = strtok(cont, "\n,"); p != NULL; p = strtok(NULL, "\n,")) {
        memset(temp, '\0', sizeof(temp));
        strncpy(temp, p, strlen(p));

        pColon1 = strchr(temp, '"');
        if (pColon1 != NULL) {
            pColon2 = strchr(pColon1 + 1, '"');
            if (pColon2 != NULL) {
                if ((pColon2 - (pColon1 + 1)) > (sizeof(key) - 1)) {
                    return -1;
                }

                memset(key, '\0', sizeof(key));
                strncpy(key, pColon1 + 1, pColon2 - (pColon1 + 1));

                pColon3 = strchr(pColon2 + 1, '"');
                if (pColon3 != NULL) {
                    pColon4 = strchr(pColon3 + 1, '"');
                    if (pColon4 != NULL) {
                        if ((pColon4 - (pColon3 + 1)) > (sizeof(value) - 1)) {
                            return -1;
                        }

                        memset(value, '\0', sizeof(value));
                        strncpy(value, pColon3 + 1, pColon4 - (pColon3 + 1));

                        if (user_findStaticCalibrationPara(key) != 0) {
                            return -1;
                        }

                        int ret = 0;
                        sqlite3 *db1 = NULL;
                        char *err_msg = NULL;

                        ret = sqlite3_open(CARDB_DB_FILE, &db1);
                        if (ret != SQLITE_OK) {
                            spdlog::error("open db {0} failed", CARDB_DB_FILE);
                            return 1;
                        }

                        {
                            int ret = 0;
                            char sql_update[512] = {0};

                            sprintf(sql_update, "update markers_staticModel set %s = '%s'", key, value);
                            ret = sqlite3_exec(db1, sql_update, 0, 0, &err_msg);
                            if (ret != SQLITE_OK) {
                                sqlite3_close(db1);
                                sqlite3_free(err_msg);
                                return -1;
                            }
                        }

                        sqlite3_close(db1);
                    } else {
                        return -1;
                    }
                } else {
                    return -1;
                }
            } else {
                return -1;
            }
        } else {
            continue;
        }
    }

    user_readStaticCalibrationPara();
    return 0;
}

int user_getStaticCalibrationPara(char *buf)
{
    char tempBuf[128] = {"\0"};

    if (buf == NULL) {
        return -1;
    }

    strcat(buf, "{\n");

    memset(tempBuf, 0, sizeof(tempBuf));
    snprintf(tempBuf, sizeof(tempBuf), "\"matrix_1\":\"%.15llf\",\n", g_StaticCalbPara.matrix_1);
    strcat(buf, tempBuf);

    memset(tempBuf, 0, sizeof(tempBuf));
    snprintf(tempBuf, sizeof(tempBuf), "\"matrix_2\":\"%.15llf\",\n", g_StaticCalbPara.matrix_2);
    strcat(buf, tempBuf);

    memset(tempBuf, 0, sizeof(tempBuf));
    snprintf(tempBuf, sizeof(tempBuf), "\"matrix_3\":\"%.15llf\",\n", g_StaticCalbPara.matrix_3);
    strcat(buf, tempBuf);

    memset(tempBuf, 0, sizeof(tempBuf));
    snprintf(tempBuf, sizeof(tempBuf), "\"matrix_4\":\"%.15llf\",\n", g_StaticCalbPara.matrix_4);
    strcat(buf, tempBuf);

    memset(tempBuf, 0, sizeof(tempBuf));
    snprintf(tempBuf, sizeof(tempBuf), "\"matrix_5\":\"%.15llf\",\n", g_StaticCalbPara.matrix_5);
    strcat(buf, tempBuf);

    memset(tempBuf, 0, sizeof(tempBuf));
    snprintf(tempBuf, sizeof(tempBuf), "\"matrix_6\":\"%.15llf\",\n", g_StaticCalbPara.matrix_6);
    strcat(buf, tempBuf);

    memset(tempBuf, 0, sizeof(tempBuf));
    snprintf(tempBuf, sizeof(tempBuf), "\"p00\":\"%.15llf\",\n", g_StaticCalbPara.p00);
    strcat(buf, tempBuf);

    memset(tempBuf, 0, sizeof(tempBuf));
    snprintf(tempBuf, sizeof(tempBuf), "\"p10\":\"%.15llf\",\n", g_StaticCalbPara.p10);
    strcat(buf, tempBuf);

    memset(tempBuf, 0, sizeof(tempBuf));
    snprintf(tempBuf, sizeof(tempBuf), "\"p01\":\"%.15llf\",\n", g_StaticCalbPara.p01);
    strcat(buf, tempBuf);

    memset(tempBuf, 0, sizeof(tempBuf));
    snprintf(tempBuf, sizeof(tempBuf), "\"p20\":\"%.15llf\",\n", g_StaticCalbPara.p20);
    strcat(buf, tempBuf);

    memset(tempBuf, 0, sizeof(tempBuf));
    snprintf(tempBuf, sizeof(tempBuf), "\"p11\":\"%.15llf\",\n", g_StaticCalbPara.p11);
    strcat(buf, tempBuf);

    memset(tempBuf, 0, sizeof(tempBuf));
    snprintf(tempBuf, sizeof(tempBuf), "\"p02\":\"%.15llf\",\n", g_StaticCalbPara.p02);
    strcat(buf, tempBuf);

    memset(tempBuf, 0, sizeof(tempBuf));
    snprintf(tempBuf, sizeof(tempBuf), "\"origin_longitude\":\"%.15llf\",\n", g_StaticCalbPara.origin_longitude);
    strcat(buf, tempBuf);

    memset(tempBuf, 0, sizeof(tempBuf));
    snprintf(tempBuf, sizeof(tempBuf), "\"origin_latitude\":\"%.15llf\",\n", g_StaticCalbPara.origin_latitude);
    strcat(buf, tempBuf);

    memset(tempBuf, 0, sizeof(tempBuf));
    snprintf(tempBuf, sizeof(tempBuf), "\"angle\":\"%.15llf\",\n", g_StaticCalbPara.angle);
    strcat(buf, tempBuf);

    memset(tempBuf, 0, sizeof(tempBuf));
    snprintf(tempBuf, sizeof(tempBuf), "\"is_del\":\"%.15llf\",\n", g_StaticCalbPara.is_del);
    strcat(buf, tempBuf);

    memset(tempBuf, 0, sizeof(tempBuf));
    snprintf(tempBuf, sizeof(tempBuf), "\"status\":\"%.15llf\",\n", g_StaticCalbPara.status);
    strcat(buf, tempBuf);

    memset(tempBuf, 0, sizeof(tempBuf));
    snprintf(tempBuf, sizeof(tempBuf), "\"create_by\":\"%.15llf\",\n", g_StaticCalbPara.create_by);
    strcat(buf, tempBuf);

    memset(tempBuf, 0, sizeof(tempBuf));
    snprintf(tempBuf, sizeof(tempBuf), "\"create_date\":\"%.15llf\",\n", g_StaticCalbPara.create_date);
    strcat(buf, tempBuf);

    memset(tempBuf, 0, sizeof(tempBuf));
    snprintf(tempBuf, sizeof(tempBuf), "\"update_by\":\"%.15llf\",\n", g_StaticCalbPara.update_by);
    strcat(buf, tempBuf);

    memset(tempBuf, 0, sizeof(tempBuf));
    snprintf(tempBuf, sizeof(tempBuf), "\"update_date\":\"%.15llf\"\n", g_StaticCalbPara.update_date);
    strcat(buf, tempBuf);

    strcat(buf, "}\n");

    if (strlen(buf) > 3072) {
        return -1;
    }

    return 0;
}

int user_InsertPolygonRoi(char *buf)
{
    int id;
    int ret = 0;
    int laneNum;
    char *p = NULL;
    int keyCnt = 0;
    char *p2 = NULL;
    int valueCnt = 0;
    sqlite3 *db1 = NULL;
    char *pComma = NULL;
    char idVal[64] = {"\0"};
    char temp[256] = {"\0"};
    char P1XVal[64] = {"\0"};
    char P1YVal[64] = {"\0"};
    char P2XVal[64] = {"\0"};
    char P2YVal[64] = {"\0"};
    char P3XVal[64] = {"\0"};
    char P3YVal[64] = {"\0"};
    char P4XVal[64] = {"\0"};
    char P4YVal[64] = {"\0"};
    char key[9][64] = {"\0"};
    char nameVal[64] = {"\0"};
    char value[9][64] = {"\0"};
    char section_xVal[64] = {"\0"};
    char detectionCycleVal[64] = {"\0"};
    char *pColon1 = NULL, *pColon2 = NULL, *pColon3 = NULL, *pColon4 = NULL;
    char *pCont[8] = {&P1XVal[0], &P1YVal[0], &P2XVal[0], &P2YVal[0], &P3XVal[0], &P3YVal[0], &P4XVal[0], &P4YVal[0]};

    if (buf == NULL) {
        return -1;
    }

    for (p = strtok(buf, "\n,"); p != NULL; p = strtok(NULL, "\n,")) {
        memset(temp, '\0', sizeof(temp));
        strncpy(temp, p, strlen(p));

        pColon1 = strchr(temp, '"');
        if (pColon1 != NULL) {
            pColon2 = strchr(pColon1 + 1, '"');
            if (pColon2 != NULL) {
                if ((pColon2 - (pColon1 + 1)) > (sizeof(key[keyCnt]) - 1)) {
                    return -1;
                }

                memset(key[keyCnt], '\0', sizeof(key[keyCnt]));
                strncpy(key[keyCnt], pColon1 + 1, pColon2 - (pColon1 + 1));

                pColon3 = strchr(pColon2 + 1, '"');
                if (pColon3 != NULL) {
                    pColon4 = strchr(pColon3 + 1, '"');
                    if (pColon4 != NULL) {
                        if ((pColon4 - (pColon3 + 1)) > (sizeof(value) - 1)) {
                            return -1;
                        }

                        memset(value[valueCnt], '\0', sizeof(value[valueCnt]));
                        strncpy(value[valueCnt], pColon3 + 1, pColon4 - (pColon3 + 1));
                    } else {
                        goto goto_user_insertPolygonRoi_failed;
                    }
                } else {
                    goto goto_user_insertPolygonRoi_failed;
                }
            } else {
                goto goto_user_insertPolygonRoi_failed;
            }

            if (strcmp(key[keyCnt], "id") == 0) {
                strcpy(idVal, value[valueCnt]);
            } else if (strcmp(key[keyCnt], "P1X") == 0) {
                strcpy(P1XVal, value[valueCnt]);
            } else if (strcmp(key[keyCnt], "P1Y") == 0) {
                strcpy(P1YVal, value[valueCnt]);
            } else if (strcmp(key[keyCnt], "P2X") == 0) {
                strcpy(P2XVal, value[valueCnt]);
            } else if (strcmp(key[keyCnt], "P2Y") == 0) {
                strcpy(P2YVal, value[valueCnt]);
            } else if (strcmp(key[keyCnt], "P3X") == 0) {
                strcpy(P3XVal, value[valueCnt]);
            } else if (strcmp(key[keyCnt], "P3Y") == 0) {
                strcpy(P3YVal, value[valueCnt]);
            } else if (strcmp(key[keyCnt], "P4X") == 0) {
                strcpy(P4XVal, value[valueCnt]);
            } else if (strcmp(key[keyCnt], "P4Y") == 0) {
                strcpy(P4YVal, value[valueCnt]);
            }

            keyCnt++;
            valueCnt++;
            if ((keyCnt > 9) || (valueCnt > 9)) {
                goto goto_user_insertPolygonRoi_failed;
            }
        } else {
            continue;
        }
    }

    ret = sqlite3_open(ADJUST_DB_FILE, &db1);
    if (ret != SQLITE_OK) {
        spdlog::error("open db {0} failed", ADJUST_DB_FILE);
        return 1;
    }

    id = atoi(idVal);
    ret = user_InsertPolygonRoi_func(db1, id, pCont);
    if (ret == 0) {
        sqlite3_close(db1);
    }

    if (ret != 0) {
        goto goto_user_insertPolygonRoi_failed;
    }

    user_flushParams(NULL);
    return 0;

goto_user_insertPolygonRoi_failed:
    return 1;
}

int user_UpdatePolygonRoi(char *buf)
{
    int id;
    int ret = 0;
    int laneNum;
    int keyCnt = 0;
    char *p = NULL;
    char *p2 = NULL;
    int valueCnt = 0;
    sqlite3 *db1 = NULL;
    char *pComma = NULL;
    char *err_msg = NULL;
    char temp[256] = {"\0"};
    char idVal[64] = {"\0"};
    char P1XVal[64] = {"\0"};
    char P1YVal[64] = {"\0"};
    char P2XVal[64] = {"\0"};
    char P2YVal[64] = {"\0"};
    char P3XVal[64] = {"\0"};
    char P3YVal[64] = {"\0"};
    char P4XVal[64] = {"\0"};
    char P4YVal[64] = {"\0"};
    char key[9][64] = {"\0"};
    char nameVal[64] = {"\0"};
    char value[9][64] = {"\0"};
    char sql_update[512] = {0};
    char section_xVal[64] = {"\0"};
    char detectionCycleVal[64] = {"\0"};
    char *pColon1 = NULL, *pColon2 = NULL, *pColon3 = NULL, *pColon4 = NULL;
    char *pCont[8] = {&P1XVal[0], &P1YVal[0], &P2XVal[0], &P2YVal[0], &P3XVal[0], &P3YVal[0], &P4XVal[0], &P4YVal[0]};

    if (buf == NULL) {
        return -1;
    }

    for (p = strtok(buf, "\n,"); p != NULL; p = strtok(NULL, "\n,")) {
        memset(temp, '\0', sizeof(temp));
        strncpy(temp, p, strlen(p));

        pColon1 = strchr(temp, '"');
        if (pColon1 != NULL) {
            pColon2 = strchr(pColon1 + 1, '"');
            if (pColon2 != NULL) {
                if ((pColon2 - (pColon1 + 1)) > (sizeof(key[keyCnt]) - 1)) {
                    return -1;
                }

                memset(key[keyCnt], '\0', sizeof(key[keyCnt]));
                strncpy(key[keyCnt], pColon1 + 1, pColon2 - (pColon1 + 1));

                pColon3 = strchr(pColon2 + 1, '"');
                if (pColon3 != NULL) {
                    pColon4 = strchr(pColon3 + 1, '"');
                    if (pColon4 != NULL) {
                        if ((pColon4 - (pColon3 + 1)) > (sizeof(value) - 1)) {
                            return -1;
                        }

                        memset(value[valueCnt], '\0', sizeof(value[valueCnt]));
                        strncpy(value[valueCnt], pColon3 + 1, pColon4 - (pColon3 + 1));
                    } else {
                        goto goto_user_UpdatePolygonRoi_failed;
                    }
                } else {
                    goto goto_user_UpdatePolygonRoi_failed;
                }
            } else {
                goto goto_user_UpdatePolygonRoi_failed;
            }

            if (strcmp(key[keyCnt], "id") == 0) {
                strcpy(idVal, value[valueCnt]);
            } else if (strcmp(key[keyCnt], "P1X") == 0) {
                strcpy(P1XVal, value[valueCnt]);
            } else if (strcmp(key[keyCnt], "P1Y") == 0) {
                strcpy(P1YVal, value[valueCnt]);
            } else if (strcmp(key[keyCnt], "P2X") == 0) {
                strcpy(P2XVal, value[valueCnt]);
            } else if (strcmp(key[keyCnt], "P2Y") == 0) {
                strcpy(P2YVal, value[valueCnt]);
            } else if (strcmp(key[keyCnt], "P3X") == 0) {
                strcpy(P3XVal, value[valueCnt]);
            } else if (strcmp(key[keyCnt], "P3Y") == 0) {
                strcpy(P3YVal, value[valueCnt]);
            } else if (strcmp(key[keyCnt], "P4X") == 0) {
                strcpy(P4XVal, value[valueCnt]);
            } else if (strcmp(key[keyCnt], "P4Y") == 0) {
                strcpy(P4YVal, value[valueCnt]);
            }

            keyCnt++;
            valueCnt++;
            if ((keyCnt > 9) || (valueCnt > 9)) {
                goto goto_user_UpdatePolygonRoi_failed;
            }
        } else {
            continue;
        }
    }

    ret = sqlite3_open(ADJUST_DB_FILE, &db1);
    if (ret != SQLITE_OK) {
        spdlog::error("open db {0} failed", ADJUST_DB_FILE);
        goto goto_user_UpdatePolygonRoi_failed;
    }

    id = atoi(idVal);
    sprintf(sql_update,
            "update PolygonRoi set "
            "P1X='%s',P1Y='%s',P2X='%s',P2Y='%s',P3X='%s',P3Y='%s',P4X='%s',P4Y='"
            "%s' where id = '%d'",
            P1XVal, P1YVal, P2XVal, P2YVal, P3XVal, P3YVal, P4XVal, P4YVal, id);

    ret = sqlite3_exec(db1, "begin transaction", 0, 0, &err_msg);
    if (ret != SQLITE_OK) {
        sqlite3_free(err_msg);
    }

    ret = sqlite3_exec(db1, sql_update, 0, 0, &err_msg);
    if (ret != SQLITE_OK) {
        sqlite3_close(db1);
        sqlite3_free(err_msg);
        goto goto_user_UpdatePolygonRoi_failed;
    }

    ret = sqlite3_exec(db1, "commit transaction", 0, 0, &err_msg);
    if (ret != SQLITE_OK) {
        sqlite3_free(err_msg);
    }

    sqlite3_close(db1);

    if (ret != 0) {
        goto goto_user_UpdatePolygonRoi_failed;
    }

    user_flushParams(NULL);
    return 0;

goto_user_UpdatePolygonRoi_failed:
    return 1;
}

int user_SelectPolygonRoi(char *buf)
{
    int i = 0;
    char tempBuf[128] = {0};

    if (buf == NULL) {
        return -1;
    }

    strcat(buf, "[");
    for (i = 0; i < polyRoi_num; ++i) {
        strcat(buf, "{");

        memset(tempBuf, 0, sizeof(tempBuf));
        snprintf(tempBuf, sizeof(tempBuf), "\"id\":\"%d\",", polyRoi[i].id);
        strcat(buf, tempBuf);

        memset(tempBuf, 0, sizeof(tempBuf));
        snprintf(tempBuf, sizeof(tempBuf), "\"P1X\":\"%lf\",", polyRoi[i].P1X);
        strcat(buf, tempBuf);

        memset(tempBuf, 0, sizeof(tempBuf));
        snprintf(tempBuf, sizeof(tempBuf), "\"P1Y\":\"%lf\",", polyRoi[i].P1Y);
        strcat(buf, tempBuf);

        memset(tempBuf, 0, sizeof(tempBuf));
        snprintf(tempBuf, sizeof(tempBuf), "\"P2X\":\"%lf\",", polyRoi[i].P2X);
        strcat(buf, tempBuf);

        memset(tempBuf, 0, sizeof(tempBuf));
        snprintf(tempBuf, sizeof(tempBuf), "\"P2Y\":\"%lf\",", polyRoi[i].P2Y);
        strcat(buf, tempBuf);

        memset(tempBuf, 0, sizeof(tempBuf));
        snprintf(tempBuf, sizeof(tempBuf), "\"P3X\":\"%lf\",", polyRoi[i].P3X);
        strcat(buf, tempBuf);

        memset(tempBuf, 0, sizeof(tempBuf));
        snprintf(tempBuf, sizeof(tempBuf), "\"P3Y\":\"%lf\",", polyRoi[i].P3Y);
        strcat(buf, tempBuf);

        memset(tempBuf, 0, sizeof(tempBuf));
        snprintf(tempBuf, sizeof(tempBuf), "\"P4X\":\"%lf\",", polyRoi[i].P4X);
        strcat(buf, tempBuf);

        memset(tempBuf, 0, sizeof(tempBuf));
        snprintf(tempBuf, sizeof(tempBuf), "\"P4Y\":\"%lf\",", polyRoi[i].P4Y);
        strcat(buf, tempBuf);

        if (i < (polyRoi_num - 1)) {
            strcat(buf, "},");
        } else {
            strcat(buf, "}");
        }
    }
    strcat(buf, "]");

    if (strlen(buf) > 2048) {
        return -1;
    }

    return 0;
}

int user_DeletePolygonRoi(char *buf)
{
    int id;
    int ret = 0;
    int laneNum;
    int keyCnt = 0;
    char *p = NULL;
    char *p2 = NULL;
    int valueCnt = 0;
    sqlite3 *db1 = NULL;
    char *pComma = NULL;
    char *err_msg = NULL;
    char sql_del[512] = {0};
    char temp[256] = {"\0"};
    char idVal[64] = {"\0"};
    char key[2][64] = {"\0"};
    char value[2][64] = {"\0"};
    char laneNumVal[64] = {"\0"};
    char *pColon1 = NULL, *pColon2 = NULL, *pColon3 = NULL, *pColon4 = NULL;

    if (buf == NULL) {
        return -1;
    }

    for (p = strtok(buf, "\n,"); p != NULL; p = strtok(NULL, "\n,")) {
        memset(temp, '\0', sizeof(temp));
        strncpy(temp, p, strlen(p));

        pColon1 = strchr(temp, '"');
        if (pColon1 != NULL) {
            pColon2 = strchr(pColon1 + 1, '"');
            if (pColon2 != NULL) {
                if ((pColon2 - (pColon1 + 1)) > (sizeof(key[keyCnt]) - 1)) {
                    return -1;
                }

                memset(key[keyCnt], '\0', sizeof(key[keyCnt]));
                strncpy(key[keyCnt], pColon1 + 1, pColon2 - (pColon1 + 1));

                pColon3 = strchr(pColon2 + 1, '"');
                if (pColon3 != NULL) {
                    pColon4 = strchr(pColon3 + 1, '"');
                    if (pColon4 != NULL) {
                        if ((pColon4 - (pColon3 + 1)) > (sizeof(value) - 1)) {
                            return -1;
                        }

                        memset(value[valueCnt], '\0', sizeof(value[valueCnt]));
                        strncpy(value[valueCnt], pColon3 + 1, pColon4 - (pColon3 + 1));
                    } else {
                        goto goto_user_DeletePolygonRoi_failed;
                    }
                } else {
                    goto goto_user_DeletePolygonRoi_failed;
                }
            } else {
                goto goto_user_DeletePolygonRoi_failed;
            }

            if (strcmp(key[keyCnt], "id") == 0) {
                strcpy(idVal, value[valueCnt]);
            }

            keyCnt++;
            valueCnt++;
            if ((keyCnt > 1) || (valueCnt > 1)) {
                goto goto_user_DeletePolygonRoi_failed;
            }
        } else {
            continue;
        }
    }

    ret = sqlite3_open(ADJUST_DB_FILE, &db1);
    if (ret != SQLITE_OK) {
        spdlog::error("open db {0} failed", ADJUST_DB_FILE);
        return 1;
    }

    id = atoi(idVal);

    sprintf(sql_del, "delete from PolygonRoi where id = '%d'", id);
    ret = sqlite3_exec(db1, "begin transaction", 0, 0, &err_msg);
    if (ret != SQLITE_OK) {
        sqlite3_free(err_msg);
    }

    ret = sqlite3_exec(db1, sql_del, 0, 0, &err_msg);
    if (ret != SQLITE_OK) {
        sqlite3_free(err_msg);
        sqlite3_close(db1);
        return -1;
    }

    ret = sqlite3_exec(db1, "commit transaction", 0, 0, &err_msg);
    if (ret != SQLITE_OK) {
        sqlite3_free(err_msg);
    }

    sqlite3_close(db1);

    if (ret != 0) {
        goto goto_user_DeletePolygonRoi_failed;
    }

    user_flushParams(NULL);
    return 0;

goto_user_DeletePolygonRoi_failed:
    return 1;
}

static const char *offsetParamKeys[] = {"flipXY", "xOffset", "yOffset", "xDirect", "yDirect"};

static int findOffsetParamkeys(const char *key)
{
    if (key == NULL) {
        return -1;
    }

    int i = 0, size = sizeof(offsetParamKeys) / sizeof(offsetParamKeys[0]);
    for (i = 0; i < size; ++i) {
        if (strcmp(key, offsetParamKeys[i]) == 0) {
            return 0;
        }
    }

    return -1;
}

int user_setOffsetParameters(char *cont)
{
    char *p = NULL;
    char *p2 = NULL;
    char *pComma = NULL;
    char key[64] = {"\0"};
    char value[64] = {"\0"};
    char temp[256] = {"\0"};
    char *pColon1 = NULL, *pColon2 = NULL, *pColon3 = NULL, *pColon4 = NULL;

    if (cont == NULL) {
        return -1;
    }

    for (p = strtok(cont, "\n,"); p != NULL; p = strtok(NULL, "\n,")) {
        memset(temp, '\0', sizeof(temp));
        strncpy(temp, p, strlen(p));

        pColon1 = strchr(temp, '"');
        if (pColon1 != NULL) {
            pColon2 = strchr(pColon1 + 1, '"');
            if (pColon2 != NULL) {
                if ((pColon2 - (pColon1 + 1)) > (sizeof(key) - 1)) {
                    return -1;
                }

                memset(key, '\0', sizeof(key));
                strncpy(key, pColon1 + 1, pColon2 - (pColon1 + 1));

                pColon3 = strchr(pColon2 + 1, '"');
                if (pColon3 != NULL) {
                    pColon4 = strchr(pColon3 + 1, '"');
                    if (pColon4 != NULL) {
                        if ((pColon4 - (pColon3 + 1)) > (sizeof(value) - 1)) {
                            return -1;
                        }

                        memset(value, '\0', sizeof(value));
                        strncpy(value, pColon3 + 1, pColon4 - (pColon3 + 1));
                        if (findOffsetParamkeys(key) != 0) {
                            return -1;
                        }

                        sqlite3 *db1 = NULL;
                        char *err_msg = NULL;
                        int ret = sqlite3_open(CARDB_DB_FILE, &db1);
                        if (ret != SQLITE_OK) {
                            spdlog::error("open db {0} failed", CARDB_DB_FILE);
                            return -1;
                        }

                        {
                            char sql_str[1024] = {0};

                            memset(sql_str, 0, sizeof(sql_str));
                            sprintf(sql_str,
                                    "CREATE TABLE IF NOT EXISTS offset_param(id INTEGER "
                                    "PRIMARY KEY,flipXY VARCHAR(10) DEFAULT '0',xOffset "
                                    "VARCHAR(15) DEFAULT '0.0',yOffset VARCHAR(15) DEFAULT "
                                    "'0.0',xDirect VARCHAR(15) DEFAULT '0',yDirect "
                                    "VARCHAR(15) DEFAULT '0'); INSERT OR IGNORE INTO "
                                    "offset_param (id, flipXY, xOffset, yOffset, xDirect, "
                                    "yDirect) VALUES (1, '0', '0.0', '0.0', '0', '0');");
                            ret = sqlite3_exec(db1, sql_str, 0, 0, &err_msg);
                            if (ret != SQLITE_OK) {
                                sqlite3_free(err_msg);
                                sqlite3_close(db1);
                                return -1;
                            }

                            /* 入库数据表不存在则新建表后插入数据，否则是更新数据 */
                            memset(sql_str, 0, sizeof(sql_str));
                            sprintf(sql_str, "UPDATE offset_param SET %s = '%s' WHERE id = 1;", key, value);
                            ret = sqlite3_exec(db1, sql_str, 0, 0, &err_msg);
                            if (ret != SQLITE_OK) {
                                sqlite3_free(err_msg);
                                sqlite3_close(db1);
                                return -1;
                            }

                            sqlite3_close(db1);
                        }
                    } else {
                        return -1;
                    }
                } else {
                    return -1;
                }
            } else {
                return -1;
            }
        } else {
            continue;
        }
    }

    user_readOffsetParameters();
    return 0;
}

int user_getOffsetParameters(char *buf)
{
    char tempBuf[128] = {0};

    if (buf == NULL) {
        spdlog::error("Write param buf is NULL");
        return -1;
    }

    strcat(buf, "{\n");

    memset(tempBuf, 0, sizeof(tempBuf));
    snprintf(tempBuf, sizeof(tempBuf), "\"flipXY\":\"%d\",\n", gOffsetParam.flipXY);
    strcat(buf, tempBuf);

    memset(tempBuf, 0, sizeof(tempBuf));
    snprintf(tempBuf, sizeof(tempBuf), "\"xOffset\":\"%.5f\",\n", gOffsetParam.xOffset);
    strcat(buf, tempBuf);

    memset(tempBuf, 0, sizeof(tempBuf));
    snprintf(tempBuf, sizeof(tempBuf), "\"yOffset\":\"%.5f\",\n", gOffsetParam.yOffset);
    strcat(buf, tempBuf);

    memset(tempBuf, 0, sizeof(tempBuf));
    snprintf(tempBuf, sizeof(tempBuf), "\"xDirect\":\"%d\",\n", gOffsetParam.xDirect);
    strcat(buf, tempBuf);

    memset(tempBuf, 0, sizeof(tempBuf));
    snprintf(tempBuf, sizeof(tempBuf), "\"yDirect\":\"%d\"\n", gOffsetParam.yDirect);
    strcat(buf, tempBuf);

    strcat(buf, "}\n");

    if (strlen(buf) > 3072) {
        return -1;
    }

    return 0;
}

static int writeRoadLaneParam(const std::map<int, RoadLaneParam> &params)
{
    int ret = -1;
    sqlite3 *db1 = NULL;
    char *err_msg = NULL;

    ret = sqlite3_open(CARDB_DB_FILE, &db1);
    if (ret != SQLITE_OK) {
        spdlog::error("open database {0} failed", CARDB_DB_FILE);
        return -1;
    }

    /* 清空原表中的数据 */
    const char *sql_clear = "DELETE FROM road_lane;";
    ret = sqlite3_exec(db1, sql_clear, 0, 0, &err_msg);
    if (ret != SQLITE_OK) {
        sqlite3_free(err_msg);
        sqlite3_close(db1);
        spdlog::error("Failed to clear road_lane table: {0}", err_msg);
        return -2;
    }

    /* 插入新的数据 */
    for (const auto &pair : params) {
        char sql_insert[512];
        const RoadLaneParam &param = pair.second;

        sprintf(sql_insert,
                "INSERT INTO road_lane (lane, dire, turn, width, radius, lt_x, lt_y, rt_x, rt_y, rb_x, rb_y, lb_x, lb_y) "
                "VALUES (%d, %d, %d, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f);",
                param.lane, param.dire, param.turn, param.width, param.radius,
                param.lt_x, param.lt_y, param.rt_x, param.rt_y,
                param.rb_x, param.rb_y, param.lb_x, param.lb_y);
        ret = sqlite3_exec(db1, sql_insert, 0, 0, &err_msg);
        if (ret != SQLITE_OK) {
            sqlite3_free(err_msg);
            sqlite3_close(db1);
            spdlog::error("Failed to insert data into road_lane table: {0}", err_msg);
            return -3;
        }

        spdlog::info("lane:({}), dir:({}), turn:({}), width:({}), radius:({}), LT:({},{}), RT:({},{}), RB:({},{}), LB:({},{})",
            param.lane, param.dire, param.turn, param.width, param.radius, param.lt_x, param.lt_y, param.rt_x, param.rt_y, param.rb_x, param.rb_y, param.lb_x, param.lb_y);
    }

    sqlite3_close(db1);
    return 0;
}

int user_queryRoadLaneParam(char *buff)
{
    if (buff == NULL) {
        return -1;
    }

    if (gRoadLaneParam.size() <= 0) {
        return -2;
    }

    nlohmann::json doc;
    nlohmann::json jlist = nlohmann::json::array();

    for (auto item : gRoadLaneParam) {
        nlohmann::json jtmp;
        jtmp["dire"]   = item.second.dire;
        jtmp["lane"]   = item.second.lane;
        jtmp["turn"]   = item.second.turn;
        jtmp["width"]  = item.second.width;
        jtmp["radius"] = item.second.radius;
        jtmp["lt_x"]   = item.second.lt_x;
        jtmp["lt_y"]   = item.second.lt_y;
        jtmp["rt_x"]   = item.second.rt_x;
        jtmp["rt_y"]   = item.second.rt_y;
        jtmp["rb_x"]   = item.second.rb_x;
        jtmp["rb_y"]   = item.second.rb_y;
        jtmp["lb_x"]   = item.second.lb_x;
        jtmp["lb_y"]   = item.second.lb_y;
        jlist.emplace_back(std::move(jtmp));
    }
    doc["road_lane"] = jlist;

    std::string payload = nlohmann::json(doc).dump();
    if (payload.length() > 3072) {
        return -3;
    }

    strcpy(buff, payload.c_str());
    return 0;
}

int user_modifyRoadLaneParam(char *cont)
{
    if (cont == NULL) {
        return -1;
    }

    try {
        std::string payload = std::string(cont);
        nlohmann::json doc = nlohmann::json::parse(payload);

        if (doc.find("road_lane") != doc.end()) {
            auto road_lane = doc["road_lane"];
            if (road_lane.is_array()) {
                std::map<int, RoadLaneParam> params;

                for (auto &lane : road_lane) {
                    if (
                        ((lane.find("dire") != lane.end()) && lane["dire"].is_number_unsigned()) &&
                        ((lane.find("lane") != lane.end()) && lane["lane"].is_number_unsigned()) &&
                        ((lane.find("turn") != lane.end()) && lane["turn"].is_number_unsigned()) &&
                        ((lane.find("width") != lane.end()) && lane["width"].is_number()) &&
                        ((lane.find("radius") != lane.end()) && lane["radius"].is_number()) &&
                        ((lane.find("lt_x") != lane.end()) && lane["lt_x"].is_number()) &&
                        ((lane.find("lt_y") != lane.end()) && lane["lt_y"].is_number()) &&
                        ((lane.find("rt_x") != lane.end()) && lane["rt_x"].is_number()) &&
                        ((lane.find("rt_y") != lane.end()) && lane["rt_y"].is_number()) &&
                        ((lane.find("rb_x") != lane.end()) && lane["rb_x"].is_number()) &&
                        ((lane.find("rb_y") != lane.end()) && lane["rb_y"].is_number()) &&
                        ((lane.find("lb_x") != lane.end()) && lane["lb_x"].is_number()) &&
                        ((lane.find("lb_y") != lane.end()) && lane["lb_y"].is_number())
                    )
                    {
                        RoadLaneParam param;
                        param.dire   = lane["dire"].get<int>();
                        param.lane   = lane["lane"].get<int>();
                        param.turn   = lane["turn"].get<int>();
                        param.width  = lane["width"].get<float>();
                        param.radius = lane["radius"].get<float>();
                        param.lt_x   = lane["lt_x"].get<float>();
                        param.lt_y   = lane["lt_y"].get<float>();
                        param.rt_x   = lane["rt_x"].get<float>();
                        param.rt_y   = lane["rt_y"].get<float>();
                        param.rb_x   = lane["rb_x"].get<float>();
                        param.rb_y   = lane["rb_y"].get<float>();
                        param.lb_x   = lane["lb_x"].get<float>();
                        param.lb_y   = lane["lb_y"].get<float>();
                        params[param.lane] = param;
                    }
                }

                if (params.size() > 0) {
                    gRoadLaneParam.clear();
                    gRoadLaneParam = params;
                    return writeRoadLaneParam(params);
                }
            }
        }
    } catch (const std::exception &ex) {
        spdlog::error("modifyRoadLaneParam exception: {}", ex.what());
        return -2;
    }

    return -3;
}

void *http_manager_thread(void *ptr)
{
    int c;
    int ret;
    int bodyLen;
    int preRecvLen;
    int sockfd = -1;
    int setSockoptFlg = 0;
    char buff[3072] = {0};
    struct timeval timeout;
    char ackBuf[8192] = {0};
    char bodyBuff[3072] = {0};

    sockfd = socket(AF_INET, SOCK_STREAM, 0);
    if (sockfd < 0) {
        spdlog::error("http_manager_thread create socket failed, errstr:[{0}]", strerror(errno));
        return NULL;
    }

    int opt = 1;
    setsockopt(sockfd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

    // /* 设置keepalive */
    // int keepalive = 1;
    // setsockopt(sockfd, SOL_SOCKET, SO_KEEPALIVE, &keepalive, sizeof(keepalive));

    // int keep_idle = 60;
    // setsockopt(sockfd, SOL_TCP, TCP_KEEPIDLE, &keep_idle, sizeof(keep_idle));

    // int keep_interval = 30;
    // setsockopt(sockfd, SOL_TCP, TCP_KEEPINTVL, &keep_interval, sizeof(keep_interval));

    // int keep_count = 3;
    // setsockopt(sockfd, SOL_TCP, TCP_KEEPCNT, &keep_count, sizeof(keep_count));

    struct sockaddr_in saddr;
    memset(&saddr, 0, sizeof(saddr));
    saddr.sin_family = AF_INET;
    saddr.sin_port = htons(8000);
    saddr.sin_addr.s_addr = INADDR_ANY;

    ret = bind(sockfd, (struct sockaddr *)&saddr, sizeof(saddr));
    if (ret < 0) {
        close(sockfd);
        spdlog::error("http_manager_thread socket bind for 8000 failed, errstr:[{0}]", strerror(errno));
        return NULL;
    }

    ret = listen(sockfd, 5);
    spdlog::debug("http_manager_thread listen return:[{0}]", ret);
    if (ret < 0) {
        close(sockfd);
        spdlog::error("http_manager_thread socket listen for 8000 failed, errstr:[{0}]", strerror(errno));
        return NULL;
    }

    pthread_setname_np(pthread_self(), "http_manager");

    timeout.tv_sec = 0;
    timeout.tv_usec = 300000;
    sprintf((char *)g_u8Data, "%s", __DATE__);
    sprintf((char *)g_u8Time, "%s", __TIME__);

    user_readSerial();
    user_readMac("eth1");
    user_readTop();
    user_readMeminfo();
    user_readStartTime();

    spdlog::info("http manager thread starting ......");

    while (1) {
        struct sockaddr_in caddr;
        socklen_t len = sizeof(caddr);
        c = accept(sockfd, (struct sockaddr *)&caddr, &len);
        if (c < 0) {
            spdlog::error("http manager accept client failed, errstr:[{0}]", strerror(errno));
            continue;
        }

        int result = setsockopt(c, SOL_SOCKET, SO_RCVTIMEO, (char *)&timeout.tv_sec, sizeof(struct timeval));
        if (result < 0) {
            spdlog::error("http manager setsockopt failed, errstr:[{0}]", strerror(errno));
            setSockoptFlg = 0;
        } else {
            setSockoptFlg = 1;
        }

        memset(buff, 0, sizeof(buff));
        int res = recv(c, buff, sizeof(buff), 0);
        if (res <= 0) {
            spdlog::error("http manager recv first failed, errstr:[{0}}]-[{1}]", errno, strerror(errno));
            close(c);
            c = -1;
            continue;
        }
        preRecvLen = res;

        char *p = NULL;
        REQUEST req = {0};
        char *host = NULL;
        char *body = NULL;
        char *typeAndAddr = NULL;
        char newBody[3072] = "\0";

        for (p = strtok(buff, "\r\n"); p != NULL; p = strtok(NULL, "\r\n")) {
            if (strStr(p, (char *)"HTTP/1.1") == 1) {
                typeAndAddr = p;
            } else if (strStr(p, (char *)"Host:") == 1) {
                host = p;
            } else if (strStr(p, (char *)"Content-Length:") == 1) {
                bodyLen = atoi(p + strlen("Content-Length: "));
                if ((bodyLen > 0) && (setSockoptFlg == 1)) {
                    memset(bodyBuff, 0, sizeof(bodyBuff));
                    res = recv(c, bodyBuff, bodyLen, 0);
                    if (res == bodyLen) {
                        body = bodyBuff;
                        goto user_gotohttp;
                    }
                }

                break;
            }
        }

user_gotohttp_1:
        body = strtok(NULL, "");

user_gotohttp:
        req.type = strtok(typeAndAddr, " ");
        req.addr = strtok(NULL, " ");
        strtok(host, " ");
        req.host = strtok(NULL, " ");
#if 0
        for (body = strtok(body, "\r\n,"); body != NULL; body = strtok(NULL, "\r\n,")) {
            if (strlen(body) > 0) {
                strcat(newBody, body);
                strcat(newBody, "\r\n");
            }
        }
#else
        for (body = strtok(body, "\r\n"); body != NULL; body = strtok(NULL, "\r\n")) {
            if (strlen(body) > 0) {
                strcat(newBody, body);
            }
        }
#endif
        req.body = newBody;

        char cmd[256] = "";
        char *head_buff = NULL;

        if (strStr(req.type, (char *)"GET") == 1) {
            if (strStr(req.addr, (char *)"getHardinfo") == 1) {
                char Hardinfo[1024] = {0};
                int ret = getHardinfo(Hardinfo);
                if (ret == -1) {
                    head_buff = fail((char *)"硬件信息获取失败！");
                    send(c, head_buff, strlen(head_buff), 0);
                } else {
                    head_buff = success(Hardinfo);
                    send(c, head_buff, strlen(head_buff), 0);
                }
            } else if (strStr(req.addr, (char *)"getHardRunStatus") == 1) {
                char res[1024] = {0};
                int ret = getHardRunStatus(res);
                if (ret == -1) {
                    head_buff = fail((char *)"运行状态信息获取失败！");
                    send(c, head_buff, strlen(head_buff), 0);
                } else {
                    head_buff = success(res);
                    send(c, head_buff, strlen(head_buff), 0);
                }
            } else if (strStr(req.addr, (char *)"getTime") == 1) {
                /* 获取当前时间点 */
                auto now = std::chrono::system_clock::now();
                auto now_c = std::chrono::system_clock::to_time_t(now);

                /* 获取毫秒部分 */
                auto milliseconds = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()) % 1000;

                /* 将时间格式化为字符串 */
                std::tm *now_tm = std::localtime(&now_c);
                char buffer[80];
                std::strftime(buffer, sizeof(buffer), "%Y-%m-%d %H:%M:%S", now_tm);

                std::ostringstream ostr;
                ostr << "datetime: " << buffer << "." << std::setfill('0') << std::setw(3) << milliseconds.count();

                head_buff = success((char *)(ostr.str().c_str()));
                send(c, head_buff, strlen(head_buff), 0);
            } else if (strStr(req.addr, (char *)"api/getDeviceSerial") == 1) {
                head_buff = success((char *)g_u8Serial);
                send(c, head_buff, strlen(head_buff), 0);
            } else {
                head_buff = fail((char *)"错误！未知接口！");
                send(c, head_buff, strlen(head_buff), 0);
            }
        } else if (strStr(req.type, (char *)"POST") == 1) {
            if (strStr(req.addr, (char *)"getLinkStatus") == 1) {
                char res[256] = {0};
                strcpy(res, "Linked");
                head_buff = success(res);
                send(c, head_buff, strlen(head_buff), 0);
            } else if (strStr(req.addr, (char *)"setTime") == 1) {
                char *pT1, *pT2, *pT3;
                char date[32] = {"\0"};
                char time[32] = {"\0"};
                char dateTime[256] = {"\0"};

                if ((pT1 = strstr(req.body, (char *)"time\":")) != NULL) {
                    if ((pT2 = strchr(pT1 + strlen("time\":"), '"')) != NULL) {
                        if ((pT3 = strchr(pT2 + 1, '"')) != NULL) {
                            strncpy(dateTime, pT2 + 1, pT3 - pT2 - 1);
                        }
                    }
                }

                if ((pT3 != NULL) && (strlen(dateTime) > 0)) {
                    char cmd[256] = {"\0"};
                    strcat(cmd, "date -s \"");
                    strcat(cmd, dateTime);
                    strcat(cmd, "\"");
                    res = system(cmd);
                } else {
                    goto setDatetimeFailed;
                }

                // 发送响应报文
                if (res != 0) {
setDatetimeFailed:
                    head_buff = fail((char *)"failed");
                    send(c, head_buff, strlen(head_buff), 0);
                } else {
                    head_buff = success((char *)"success");
                    send(c, (const char *)head_buff, strlen(head_buff), 0);
                    g_GNSSEn = 0;
                    NTPEnFlg = 0;
                    //system("rm -rf /home/root/GNSSCfgFile");
                    //system("rm -rf /home/root/cfgFile");
                    system("sync");
                }
            } else if (strStr(req.addr, (char *)"reboot") == 1) {
                strcat(cmd, "reboot -f");
                head_buff = success((char *)"success");
                send(c, head_buff, strlen(head_buff), 0);
                system(cmd);
            } else if (strStr(req.addr, (char *)"setNetwork") == 1) {
                int ret = setNetwork(req.body, "eth1");
                if (ret == -1) {
                    head_buff = fail((char *)"failed");
                    send(c, head_buff, strlen(head_buff), 0);
                } else {
                    head_buff = success((char *)"success");
                    send(c, head_buff, strlen(head_buff), 0);
                }
            } else if (strStr(req.addr, (char *)"setNTPServerIp") == 1) {
                int ret = setNTPServerIp(req.body);
                if (ret == -1) {
                    head_buff = fail((char *)"failed");
                    send(c, head_buff, strlen(head_buff), 0);
                } else {
                    head_buff = success((char *)"success");
                    send(c, head_buff, strlen(head_buff), 0);
                }
            } else if (strStr(req.addr, (char *)"setRunMode") == 1) {
                int ret = setRunMode(req.body);
                if (ret == -1) {
                    head_buff = fail((char *)"success");
                    send(c, head_buff, strlen(head_buff), 0);
                } else {
                    head_buff = success((char *)"failed");
                    send(c, head_buff, strlen(head_buff), 0);
                }
            } else if (strStr(req.addr, (char *)"getDevDateTime") == 1) {
                char devDate[256] = {0};
                int ret = getDevDateTime(devDate, 256);
                if (ret == -1) {
                    head_buff = fail((char *)"获取设备时间失败！");
                    send(c, head_buff, strlen(head_buff), 0);
                } else {
                    head_buff = success(devDate);
                    send(c, head_buff, strlen(head_buff), 0);
                }
            } else if (strStr(req.addr, (char *)"setGNSSMode") == 1) {
#if 0
                int ret = setGNSSMode(req.body);
                if (ret == -1) {
                    head_buff = fail((char *)"failed");
                    send(c, head_buff, strlen(head_buff), 0);
                } else {
                    head_buff = success((char *)"success");
                    send(c, head_buff, strlen(head_buff), 0);
                }
#else
                head_buff = fail((char *)"unsupported");
                send(c, head_buff, strlen(head_buff), 0);
#endif
            } else if (strStr(req.addr, (char *)"restoreFactory") == 1) {
                int ret = restoreFactory(req.body);
                if (ret == -1) {
                    head_buff = fail((char *)"failed");
                    send(c, head_buff, strlen(head_buff), 0);
                } else {
                    head_buff = success((char *)"success");
                    send(c, head_buff, strlen(head_buff), 0);
                }
            } else if (strStr(req.addr, (char *)"startSaveClusterInfo") == 1) {
                char tmpBuf[24] = {0};
                int ret = startSaveClusterInfo(req.body);
                if (ret != 0) {
                    memset(tmpBuf, 0, sizeof(tmpBuf));
                    snprintf(tmpBuf, sizeof(tmpBuf), "ret:%d", ret);
                    head_buff = fail(tmpBuf);
                    send(c, head_buff, strlen(head_buff), 0);
                } else {
                    memset(tmpBuf, 0, sizeof(tmpBuf));
                    snprintf(tmpBuf, sizeof(tmpBuf), "ret:%d", ret);
                    head_buff = success(tmpBuf);
                    send(c, head_buff, strlen(head_buff), 0);
                }
            } else if (strStr(req.addr, (char *)"endSaveClusterInfo") == 1) {
                int ret = endSaveClusterInfo(req.body);
                if (ret != 0) {
                    head_buff = fail((char *)"endSaveClusterInfo failed");
                    send(c, head_buff, strlen(head_buff), 0);
                } else {
                    head_buff = success(clusterFilePath);
                    send(c, head_buff, strlen(head_buff), 0);
                }
            } else if (strStr(req.addr, (char *)"clusterAndObjectSend2Rearend") == 1) {
                char tmpBuf[24] = {0};
                int ret = clusterAndObjectSend2Rearend(req.body);
                if (ret != 0) {
                    memset(tmpBuf, 0, sizeof(tmpBuf));
                    snprintf(tmpBuf, sizeof(tmpBuf), "ret:%d", ret);
                    head_buff = fail(tmpBuf);
                    send(c, head_buff, strlen(head_buff), 0);
                } else {
                    memset(tmpBuf, 0, sizeof(tmpBuf));
                    snprintf(tmpBuf, sizeof(tmpBuf), "ret:%d", ret);
                    head_buff = success(tmpBuf);
                    send(c, head_buff, strlen(head_buff), 0);
                }
            } else if (strStr(req.addr, (char *)"embeddedSoftVersion") == 1) {
                char tmpBuf[24] = {0};
                memset(tmpBuf, 0, sizeof(tmpBuf));
                snprintf(tmpBuf, sizeof(tmpBuf), "%s", g_embeddedSoftVersion);
                head_buff = success(tmpBuf);
                send(c, head_buff, strlen(head_buff), 0);
            } else if (strStr(req.addr, (char *)"updateIni") == 1) {
                int ret = 0;

                ret = user_checkUpdateIniBody(req.body);
                if (ret != 0) {
                    head_buff = fail((char *)"failed");
                    send(c, head_buff, strlen(head_buff), 0);
                } else {
                    head_buff = success((char *)"success");
                    send(c, (const char *)head_buff, strlen(head_buff), 0);
                }
            } else if (strStr(req.addr, (char *)"getIni") == 1) {
                int retGetIni = 0;

                memset(ackBuf, 0, sizeof(ackBuf));
                retGetIni = user_getIni(ackBuf);

                if (retGetIni != 0) {
                    head_buff = fail((char *)"failed");
                    send(c, head_buff, strlen(head_buff), 0);
                } else {
                    head_buff = success(ackBuf);
                    send(c, (const char *)head_buff, strlen(head_buff), 0);
                }
            } else if (strStr(req.addr, (char *)"updateRoiArg") == 1) {
                int ret = user_checkupdateRoiArgBody(req.body);
                if (ret != 0) {
                    head_buff = fail((char *)"failed");
                    send(c, head_buff, strlen(head_buff), 0);
                } else {
                    head_buff = success((char *)"success");
                    send(c, (const char *)head_buff, strlen(head_buff), 0);
                }
            } else if (strStr(req.addr, (char *)"getRoiArg") == 1) {
                int getRoiArg = 0;

                memset(ackBuf, '\0', sizeof(ackBuf));
                getRoiArg = user_getRoiArg(ackBuf);

                if (getRoiArg != 0) {
                    head_buff = fail((char *)"获取参数失败！");
                    send(c, head_buff, strlen(head_buff), 0);
                } else {
                    head_buff = success(ackBuf);
                    send(c, (const char *)head_buff, strlen(head_buff), 0);
                }
            } else if (strStr(req.addr, (char *)"getSections") == 1) {
                int getSection = 0;

                memset(ackBuf, '\0', sizeof(ackBuf));
                getSection = user_getSection(ackBuf);

                if (getSection != 0) {
                    head_buff = fail((char *)"failed");
                    send(c, head_buff, strlen(head_buff), 0);
                } else {
                    head_buff = success(ackBuf);
                    send(c, (const char *)head_buff, strlen(head_buff), 0);
                }
            } else if (strStr(req.addr, (char *)"getCoils") == 1) {
                int getCoil = 0;

                memset(ackBuf, '\0', sizeof(ackBuf));
                getCoil = user_getCoil(req.body, ackBuf);

                if (getCoil != 0) {
                    head_buff = fail((char *)"failed");
                    send(c, head_buff, strlen(head_buff), 0);
                } else {
                    head_buff = success(ackBuf);
                    send(c, (const char *)head_buff, strlen(head_buff), 0);
                }
            } else if (strStr(req.addr, (char *)"gotoUpgradeEmbededSoft") == 1) {
                memset(ackBuf, '\0', sizeof(ackBuf));
                ret = user_gotoUpgradeEmbededSoft(ackBuf);
                if (ret != 0) {
                    head_buff = fail(ackBuf);
                    send(c, head_buff, strlen(head_buff), 0);
                } else {
                    head_buff = success(ackBuf);
                    send(c, (const char *)head_buff, strlen(head_buff), 0);
                    system("/run/media/mmcblk0p1/upgradeFiles/gotoUpgradeShellFile.sh");
                }
            } else if (strStr(req.addr, (char *)"lslhPath") == 1) {
                memset(ackBuf, '\0', sizeof(ackBuf));
                ret = user_lslhPath(req.body, ackBuf, sizeof(ackBuf));
                if (ret != 0) {
                    head_buff = fail(ackBuf);
                    send(c, head_buff, strlen(head_buff), 0);
                } else {
                    head_buff = success(ackBuf);
                    send(c, (const char *)head_buff, strlen(head_buff), 0);
                }
            } else if (strStr(req.addr, (char *)"getGPSGNRMC") == 1) {
                head_buff = success(g_GPSGNRMC);
                send(c, head_buff, strlen(head_buff), 0);
            } else if (strStr(req.addr, (char *)"enableWatchdog") == 1) {
                memset(ackBuf, 0, sizeof(ackBuf));
                ret = user_httpHdlWatchdog(req.body, ackBuf, sizeof(ackBuf));
                if (ret != 0) {
                    head_buff = fail(ackBuf);
                    send(c, head_buff, strlen(head_buff), 0);
                } else {
                    head_buff = success(ackBuf);
                    send(c, (const char *)head_buff, strlen(head_buff), 0);
                }
            } else if (strStr(req.addr, (char *)"GetClusterAndObjectSend2Rearend") == 1) {
                memset(ackBuf, '\0', sizeof(ackBuf));
                ret = user_GetClusterAndObjectSend2Rearend(ackBuf, sizeof(ackBuf));

                head_buff = success(ackBuf);
                send(c, (const char *)head_buff, strlen(head_buff), 0);
            } else if (strStr(req.addr, (char *)"UpdateSections") == 1) {
                memset(ackBuf, '\0', sizeof(ackBuf));
                ret = user_UpdateSections(req.body, ackBuf, sizeof(ackBuf));

                if (ret != 0) {
                    head_buff = fail(ackBuf);
                    send(c, head_buff, strlen(head_buff), 0);
                } else {
                    head_buff = success(ackBuf);
                    send(c, (const char *)head_buff, strlen(head_buff), 0);
                }
            } else if (strStr(req.addr, (char *)"InsertSections") == 1) {
                memset(ackBuf, '\0', sizeof(ackBuf));
                ret = user_InsertSections(req.body, ackBuf, sizeof(ackBuf));
                if (ret != 0) {
                    head_buff = fail(ackBuf);
                    send(c, head_buff, strlen(head_buff), 0);
                } else {
                    head_buff = success(ackBuf);
                    send(c, (const char *)head_buff, strlen(head_buff), 0);
                }
            } else if (strStr(req.addr, (char *)"DeleteSections") == 1) {
                memset(ackBuf, '\0', sizeof(ackBuf));
                ret = user_DeleteSections(req.body, ackBuf, sizeof(ackBuf));
                if (ret != 0) {
                    head_buff = fail(ackBuf);
                    send(c, head_buff, strlen(head_buff), 0);
                } else {
                    head_buff = success(ackBuf);
                    send(c, (const char *)head_buff, strlen(head_buff), 0);
                }
            } else if (strStr(req.addr, (char *)"UpdateCoils") == 1) {
                memset(ackBuf, '\0', sizeof(ackBuf));
                ret = user_UpdateCoils(req.body, ackBuf, sizeof(ackBuf));
                if (ret != 0) {
                    head_buff = fail(ackBuf);
                    send(c, head_buff, strlen(head_buff), 0);
                } else {
                    head_buff = success(ackBuf);
                    send(c, (const char *)head_buff, strlen(head_buff), 0);
                }
            } else if (strStr(req.addr, (char *)"InsertCoils") == 1) {
                memset(ackBuf, '\0', sizeof(ackBuf));
                ret = user_InsertCoils(req.body, ackBuf, sizeof(ackBuf));
                if (ret != 0) {
                    head_buff = fail(ackBuf);
                    send(c, head_buff, strlen(head_buff), 0);
                } else {
                    head_buff = success(ackBuf);
                    send(c, (const char *)head_buff, strlen(head_buff), 0);
                }
            } else if (strStr(req.addr, (char *)"DeleteCoils") == 1) {
                memset(ackBuf, '\0', sizeof(ackBuf));
                ret = user_DeleteCoils(req.body, ackBuf, sizeof(ackBuf));

                if (ret != 0) {
                    head_buff = fail(ackBuf);
                    send(c, head_buff, strlen(head_buff), 0);
                } else {
                    head_buff = success(ackBuf);
                    send(c, (const char *)head_buff, strlen(head_buff), 0);
                }
            } else if (strStr(req.addr, (char *)"getTrafficFlowDetectionStatistics") == 1) {
                memset(ackBuf, '\0', sizeof(ackBuf));
                ret = user_getTrafficFlowDetectionStatistics(ackBuf, sizeof(ackBuf), 0);
                if (ret != 0) {
                    head_buff = fail(ackBuf);
                    send(c, head_buff, strlen(head_buff), 0);
                } else {
                    head_buff = success(ackBuf);
                    send(c, (const char *)head_buff, strlen(head_buff), 0);
                }
            } else if (strStr(req.addr, (char *)"postCalibrationPara") == 1) {
                ret = user_postCalibrationPara(req.body);
                if (ret != 0) {
                    head_buff = fail((char *)"failed");
                    send(c, head_buff, strlen(head_buff), 0);
                } else {
                    head_buff = success((char *)"success");
                    send(c, (const char *)head_buff, strlen(head_buff), 0);
                }
            } else if (strStr(req.addr, (char *)"getCalibrationPara") == 1) {
                int retGetCalibrationPara = 0;

                memset(ackBuf, '\0', sizeof(ackBuf));
                retGetCalibrationPara = user_getCalibrationPara(ackBuf);
                if (retGetCalibrationPara != 0) {
                    head_buff = fail((char *)"failed");
                    send(c, head_buff, strlen(head_buff), 0);
                } else {
                    head_buff = success(ackBuf);
                    send(c, (const char *)head_buff, strlen(head_buff), 0);
                }
            } else if (strStr(req.addr, (char *)"postStaticCalibrationPara") == 1) {
                ret = user_postStaticCalibrationPara(req.body);
                if (ret != 0) {
                    head_buff = fail((char *)"failed");
                    send(c, head_buff, strlen(head_buff), 0);
                } else {
                    head_buff = success((char *)"success");
                    send(c, (const char *)head_buff, strlen(head_buff), 0);
                }
            } else if (strStr(req.addr, (char *)"getStaticCalibrationPara") == 1) {
                int retGetCalibrationPara = 0;

                memset(ackBuf, '\0', sizeof(ackBuf));
                retGetCalibrationPara = user_getStaticCalibrationPara(ackBuf);

                if (retGetCalibrationPara != 0) {
                    head_buff = fail((char *)"failed");
                    send(c, head_buff, strlen(head_buff), 0);
                } else {
                    head_buff = success(ackBuf);
                    send(c, (const char *)head_buff, strlen(head_buff), 0);
                }
            } else if (strStr(req.addr, (char *)"dynamicOrStaticCalibration") == 1) {
                char tmpBuf[24] = {0};
                int ret = dynamicOrStaticCalibration(req.body);
                if (ret != 0) {
                    memset(tmpBuf, 0, sizeof(tmpBuf));
                    snprintf(tmpBuf, sizeof(tmpBuf), "ret:%d", ret);
                    head_buff = fail(tmpBuf);
                    send(c, head_buff, strlen(head_buff), 0);
                } else {
                    memset(tmpBuf, 0, sizeof(tmpBuf));
                    snprintf(tmpBuf, sizeof(tmpBuf), "ret:%d", ret);
                    head_buff = success(tmpBuf);
                    send(c, head_buff, strlen(head_buff), 0);
                }
            } else if (strStr(req.addr, (char *)"GetDynamicOrStaticCalibration") == 1) {
                memset(ackBuf, '\0', sizeof(ackBuf));
                user_GetDynamicOrStaticCalibration(ackBuf, sizeof(ackBuf));

                head_buff = success(ackBuf);
                send(c, (const char *)head_buff, strlen(head_buff), 0);
            } else if (strStr(req.addr, (char *)"startSaveObjectInfo") == 1) {
                char tmpBuf[24] = {0};

                int ret = startSaveAlgObjectInfo(req.body);
                if (ret != 0) {
                    memset(tmpBuf, 0, sizeof(tmpBuf));
                    snprintf(tmpBuf, sizeof(tmpBuf), "ret:%d", ret);
                    head_buff = fail(tmpBuf);
                    send(c, head_buff, strlen(head_buff), 0);
                } else {
                    memset(tmpBuf, 0, sizeof(tmpBuf));
                    snprintf(tmpBuf, sizeof(tmpBuf), "ret:%d", ret);
                    head_buff = success(tmpBuf);
                    send(c, head_buff, strlen(head_buff), 0);
                }
            } else if (strStr(req.addr, (char *)"endSaveObjectInfo") == 1) {
                int ret = endSaveAlgObjectInfo(req.body);
                if (ret != 0) {
                    head_buff = fail((char *)"endSaveObjectInfo failed");
                    send(c, head_buff, strlen(head_buff), 0);
                } else {
                    head_buff = success(algObjectFilePath);
                    send(c, head_buff, strlen(head_buff), 0);
                }
            } else if (strStr(req.addr, (char *)"insertPolygonRoi") == 1) {
                ret = user_InsertPolygonRoi(req.body);
                if (ret != 0) {
                    head_buff = fail((char *)"failed");
                    send(c, head_buff, strlen(head_buff), 0);
                } else {
                    head_buff = success((char *)"success");
                    send(c, (const char *)head_buff, strlen(head_buff), 0);
                }
            } else if (strStr(req.addr, (char *)"updatePolygonRoi") == 1) {
                ret = user_UpdatePolygonRoi(req.body);
                if (ret != 0) {;
                    head_buff = fail((char *)"failed");
                    send(c, head_buff, strlen(head_buff), 0);
                } else {
                    head_buff = success((char *)"success");
                    send(c, (const char *)head_buff, strlen(head_buff), 0);
                }
            } else if (strStr(req.addr, (char *)"selectPolygonRoi") == 1) {
                memset(ackBuf, '\0', sizeof(ackBuf));
                ret = user_SelectPolygonRoi(ackBuf);
                if (ret != 0) {
                    head_buff = fail((char *)"failed");
                    send(c, head_buff, strlen(head_buff), 0);
                } else {
                    head_buff = success(ackBuf);
                    send(c, (const char *)head_buff, strlen(head_buff), 0);
                }
            } else if (strStr(req.addr, (char *)"deletePolygonRoi") == 1) {
                ret = user_DeletePolygonRoi(req.body);
                if (ret != 0) {
                    head_buff = fail((char *)"failed");
                    send(c, head_buff, strlen(head_buff), 0);
                } else {
                    head_buff = success((char *)"success");
                    send(c, (const char *)head_buff, strlen(head_buff), 0);
                }
            } else if (strStr(req.addr, (char *)"getOffsetParameters") == 1) {
                int retOffsetParam = 0;

                memset(ackBuf, '\0', sizeof(ackBuf));
                retOffsetParam = user_getOffsetParameters(ackBuf);
                if (retOffsetParam != 0) {
                    head_buff = fail((char *)"failed");
                    send(c, head_buff, strlen(head_buff), 0);
                } else {
                    head_buff = success(ackBuf);
                    send(c, (const char *)head_buff, strlen(head_buff), 0);
                }
            } else if (strStr(req.addr, (char *)"setOffsetParameters") == 1) {
                ret = user_setOffsetParameters(req.body);
                if (ret != 0) {
                    head_buff = fail((char *)"failed");
                    send(c, head_buff, strlen(head_buff), 0);
                } else {
                    head_buff = success((char *)"success");
                    send(c, (const char *)head_buff, strlen(head_buff), 0);
                }
            } else if (strStr(req.addr, (char *)"queryRoadLaneParam") == 1) {       // 查询
                memset(ackBuf, 0, sizeof(ackBuf));
                ret = user_queryRoadLaneParam(ackBuf);
                if (ret != 0) {
                    head_buff = fail((char *)"failed");
                    send(c, head_buff, strlen(head_buff), 0);
                } else {
                    head_buff = success(ackBuf);
                    send(c, (const char *)head_buff, strlen(head_buff), 0);
                }
            } else if (strStr(req.addr, (char *)"modifyRoadLaneParam") == 1) {      // 修改
                ret = user_modifyRoadLaneParam(req.body);
                if (ret != 0) {
                    head_buff = fail((char *)"failed");
                    send(c, head_buff, strlen(head_buff), 0);
                } else {
                    head_buff = success((char *)"success");
                    send(c, (const char *)head_buff, strlen(head_buff), 0);
                }
            } else {
                head_buff = fail((char *)"错误！未知接口！");
                send(c, head_buff, strlen(head_buff), 0);
            }
        } else {
            head_buff = fail((char *)"错误！未知接口！");
            send(c, head_buff, strlen(head_buff), 0);
        }

        if (c > 0) {
            close(c);
        }
    }

    close(sockfd);
    pthread_exit(NULL);
}
