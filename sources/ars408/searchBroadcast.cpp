/********************************************************************************
* @File name:searchBroadcast.c
* @Author:李军
* @Version: 1.0
* @Date:2023.05.25
* @Description:广播搜索功能
********************************************************************************/

#include <stdio.h>
#include <errno.h>
#include <stdlib.h>
#include <netdb.h>
#include <signal.h>
#include <string.h>
#include <unistd.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <pthread.h>
#include <spdlog/spdlog.h>

#include "searchBroadcast.h"
#include "http.h"
#include "upgrade.h"

static int sockfd;
static int sockRst;

void *hdlSearchBroadcast_pthread(void *arg)
{
    int ret;
    int length;
    ssize_t recvLen;
    char tBf[64] = {"\0"};
    char buff[1024] = {"\0"};
    char tmpBuf[1024] = {"\0"};
    char gateway[48] = {"\0"};
    char lineBuf[1024] = {"\0"};
    char sendBuf[1024] = {"\0"};

    sockfd = socket(AF_INET, SOCK_DGRAM, 0);
    if (sockfd < 0) {
        spdlog::error("search broadcast socket error, errstr:[{0}]", strerror(errno));
        pthread_exit(NULL);
    }

    int opt = 1;
    ret = setsockopt(sockfd, SOL_SOCKET, SO_BROADCAST, &opt, sizeof(opt));
    if (ret < 0) {
        close(sockfd);
        spdlog::error("search broadcast setsockopt error, errstr:[{0}]", strerror(errno));
        exit(EXIT_FAILURE);
    }

    struct sockaddr_in serveraddr;
    memset(&serveraddr, 0, sizeof(serveraddr));
    serveraddr.sin_family = AF_INET;
    serveraddr.sin_port = htons(USER_BROADCAST_PORT);
    serveraddr.sin_addr.s_addr = INADDR_ANY;
    if (bind(sockfd, (struct sockaddr *)&serveraddr, sizeof(serveraddr)) < 0){
        close(sockfd);
        spdlog::error("search broadcast bind error, errstr:[{0}]", strerror(errno));
        pthread_exit(NULL);
    }

    pthread_setname_np(pthread_self(), "searchBroadcast");

    int crcVal;
    char ip[24];
    char buffer[256];
    struct sockaddr_in clientaddr;
    socklen_t len = sizeof(clientaddr);

    while (1) {
        memset(buffer, 0, sizeof(buffer));
        memset(&clientaddr, 0, sizeof(clientaddr));

        if ((recvLen = recvfrom(sockfd, buffer, sizeof(buffer), 0, (struct sockaddr *)&clientaddr, &len)) < 0) {
            close(sockfd);
            spdlog::error("search broadcast recvfrom error, errstr:[{0}]", strerror(errno));
            pthread_exit(NULL);
        } else {
            memset(ip, 0, sizeof(ip));
            inet_ntop(AF_INET, &clientaddr.sin_addr.s_addr, ip, sizeof(ip));
        }

        if ((buffer[0] == 0x02) && (buffer[1] == 0x0D) && (buffer[4] == 0x03)) {
            // 02 0d d1 ad 03 //02, 8d, 31, 39, 32, 2e, 31, 36, 38, 2e, 31, 36, 2e, 32, 34, 37, 2c, 31, 39, 32, 2e, 31, 36, 38, 2e, 31, 36, 2e, 32, 35, 35, 2c, 32, 35, 35, 2e, 32, 35, //35, 2e, 32, 35, 35, 2e, 30, 2c, 31, 39, 32, 2e, 31, 36, 38, 2e, 31, 36, 2e, 31, 87, bd, 03
            crcVal = gen_crc((unsigned char *)&buffer[1], 1);
            if (((char)(crcVal >> 8 & 0xff) != buffer[2]) || ((char)(crcVal & 0xff) != buffer[3])) {
                continue;
            }

            int fd;
            struct ifreq ifr = {0};

            fd = socket(AF_INET, SOCK_DGRAM, 0);
            if (fd > 0) {
                ifr.ifr_addr.sa_family = AF_INET;
                strncpy(ifr.ifr_name, "eth1", IFNAMSIZ-1);
                ioctl(fd, SIOCGIFADDR, &ifr);

                char *info = inet_ntoa(((struct sockaddr_in *)&ifr.ifr_addr)->sin_addr);
                memset(tmpBuf, '\0', sizeof(tmpBuf));
                snprintf(tmpBuf, sizeof(tmpBuf), "%s,", info);

                memset(&ifr, '\0', sizeof(ifr));
                ifr.ifr_addr.sa_family = AF_INET;
                strncpy(ifr.ifr_name, "eth1", IFNAMSIZ-1);
                ioctl(fd, SIOCGIFBRDADDR, &ifr);

                info = inet_ntoa(((struct sockaddr_in *)&ifr.ifr_addr)->sin_addr);
                memset(tBf, '\0', sizeof(tBf));
                snprintf(tBf, sizeof(tBf), "%s,", info);
                strncat(tmpBuf, tBf, strlen(tBf));

                memset(&ifr, '\0', sizeof(ifr));
                ifr.ifr_addr.sa_family = AF_INET;
                strncpy(ifr.ifr_name, "eth1", IFNAMSIZ-1);
                ioctl(fd, SIOCGIFNETMASK, &ifr);

                info = inet_ntoa(((struct sockaddr_in *)&ifr.ifr_addr)->sin_addr);
                memset(tBf, '\0', sizeof(tBf));
                snprintf(tBf, sizeof(tBf), "%s,", info);
                strncat(tmpBuf, tBf, strlen(tBf));
                close(fd);
            }

            system("ip route show > /run/iprouteshow");
            usleep(500000);

            FILE *fp = fopen("/run/iprouteshow", "r");
            if (fp == NULL) {
                close(sockfd);
                spdlog::error("open /run/iprouteshow failed");
                return NULL;
            }

            memset(lineBuf, 0, sizeof(lineBuf));
            while (fgets(lineBuf, sizeof(lineBuf), fp) != NULL) {
                if (strstr(lineBuf, "default") != NULL && strstr(lineBuf, "via") != NULL && strstr(lineBuf, "dev") != NULL && strstr(lineBuf, "eth1") != NULL) {
                    memset(gateway, 0, sizeof(gateway));
                    sscanf(lineBuf, "default via %s dev eth1", gateway);
                    break;
                }

                memset(lineBuf, 0, sizeof(lineBuf));
            }

            fclose(fp);
            memset(tBf, 0, sizeof(tBf));
            snprintf(tBf, sizeof(tBf), "%s", gateway);
            strncat(tmpBuf, tBf, strlen(tBf));

            ssize_t size = strlen(tmpBuf) * sizeof(char);
            inet_pton(AF_INET, "255.255.255.255", &clientaddr.sin_addr.s_addr);

            memset(buff, 0, sizeof(buff));
            buff[0] = 0x02;
            buff[1] = 0x8D;
            memcpy(&buff[2], tmpBuf, strlen(tmpBuf));
            crcVal = gen_crc((unsigned char *)&buff[1], strlen(buff) - 1);
            length = strlen(buff);
            buff[length] = (char)(crcVal >> 8 & 0xff);
            buff[length + 1] = (char)(crcVal & 0xff);
            buff[length + 2] = 0x03;

            memset(sendBuf, 0, sizeof(sendBuf));
            length = user_sendTranslateBuf(buff, strlen(buff), sendBuf);

            if (sendto(sockfd, sendBuf, length, 0, (struct sockaddr *)&clientaddr, sizeof(clientaddr)) < 0) {
                close(sockfd);
                spdlog::error("search broadcast sendto error, errstr:[{0}]", strerror(errno));
                pthread_exit(NULL);
            }
        }

        usleep(1000000);
    }

    close(sockfd);
    pthread_exit(NULL);
}

void *hdlSearchAndRestoryBrdcast_pthread(void *arg)
{
    int ret;
    int crcVal;
    int length;
    ssize_t recvLen;
    char tBf[64] = {"\0"};
    char buff[1024] = {"\0"};
    char gateway[48] = {"\0"};
    char tmpBuf[1024] = {"\0"};
    char lineBuf[1024] = {"\0"};
    char sendBuf[1024] = {"\0"};

    sockRst = socket(AF_INET, SOCK_DGRAM, 0);
    if (sockRst < 0){
        spdlog::error("restory broadcast socket error, errstr:[{0}]", strerror(errno));
        pthread_exit(NULL);
    }

    int opt = 1;
    ret = setsockopt(sockRst, SOL_SOCKET, SO_BROADCAST, &opt, sizeof(opt));
    if (ret < 0) {
        close(sockRst);
        spdlog::error("restory broadcast setsockopt error, errstr:[{0}]", strerror(errno));
        exit(EXIT_FAILURE);
    }

    struct sockaddr_in serveraddr;
    memset(&serveraddr, 0, sizeof(serveraddr));
    serveraddr.sin_family = AF_INET;
    serveraddr.sin_port = htons(USER_SEARCHANDRST_PORT);
    serveraddr.sin_addr.s_addr = INADDR_ANY;
    if (bind(sockRst, (struct sockaddr *)&serveraddr, sizeof(serveraddr)) < 0) {
        close(sockRst);
        spdlog::error("restory broadcast bind error, errstr:[{0}]", strerror(errno));
        pthread_exit(NULL);
    }

    pthread_setname_np(pthread_self(), "searchAndReboot");

    char ip[24];
    char buffer[256];
    struct sockaddr_in clientaddr;
    socklen_t len = sizeof(clientaddr);

    while (1) {
        memset(buffer, 0, sizeof(buffer));
        memset(&clientaddr, 0, sizeof(clientaddr));

        if ((recvLen = recvfrom(sockRst, buffer, sizeof(buffer), 0, (struct sockaddr *)&clientaddr, &len)) < 0) {
            close(sockRst);
            spdlog::error("restory broadcast recvfrom error, errstr:[{0}]", strerror(errno));
            pthread_exit(NULL);
        } else {
            memset(ip, 0, sizeof(ip));
            inet_ntop(AF_INET, &clientaddr.sin_addr.s_addr, ip, sizeof(ip));
            spdlog::info("ipaddr:[{0}] receive packet", ip);
        }

        if ((buffer[0] == 0x02) && (buffer[1] == 0x0E) && (buffer[4] == 0x03)) {
            // 02 0e e1 ce 03 //02, 8e, 31, 39, 32, 2e, 31, 36, 38, 2e, 31, 36, 2e, 32, 34, 37, 2c, 31, 39, 32, 2e, 31, 36, 38, 2e, 31, 36, 2e, 32, 35, 35, 2c, 32, 35, 35, 2e, 32, 35, 35, 2e, 32, 35, 35, 2e, 30, 2c, 31, 39, 32, 2e, 31, 36, 38, 2e, 31, 36, 2e, 31, 37, b3, 03
            crcVal = gen_crc((unsigned char *)&buffer[1], 1);
            if (((char)(crcVal >> 8 & 0xff) != buffer[2]) || ((char)(crcVal & 0xff) != buffer[3])) {
                continue;
            }

            int fd;
            struct ifreq ifr = {0};

            fd = socket(AF_INET, SOCK_DGRAM, 0);
            if (fd > 0) {
                ifr.ifr_addr.sa_family = AF_INET;
                strncpy(ifr.ifr_name, "eth1", IFNAMSIZ - 1);
                ioctl(fd, SIOCGIFADDR, &ifr);

                char *info = inet_ntoa(((struct sockaddr_in *)&ifr.ifr_addr)->sin_addr);
                memset(tmpBuf, '\0', sizeof(tmpBuf));
                snprintf(tmpBuf, sizeof(tmpBuf), "%s,", info);

                memset(&ifr, '\0', sizeof(ifr));
                ifr.ifr_addr.sa_family = AF_INET;
                strncpy(ifr.ifr_name, "eth1", IFNAMSIZ - 1);
                ioctl(fd, SIOCGIFBRDADDR, &ifr);

                info = inet_ntoa(((struct sockaddr_in *)&ifr.ifr_addr)->sin_addr);
                memset(tBf, '\0', sizeof(tBf));
                snprintf(tBf, sizeof(tBf), "%s,", info);
                strncat(tmpBuf, tBf, strlen(tBf));

                memset(&ifr, '\0', sizeof(ifr));
                ifr.ifr_addr.sa_family = AF_INET;
                strncpy(ifr.ifr_name, "eth1", IFNAMSIZ - 1);
                ioctl(fd, SIOCGIFNETMASK, &ifr);

                info = inet_ntoa(((struct sockaddr_in *)&ifr.ifr_addr)->sin_addr);
                memset(tBf, '\0', sizeof(tBf));
                snprintf(tBf, sizeof(tBf), "%s,", info);
                strncat(tmpBuf, tBf, strlen(tBf));
                close(fd);
            }

            system("ip route show > /run/iprouteshow");
            usleep(500000);

            FILE *fp = fopen("/run/iprouteshow", "r");
            if (fp == NULL) {
                spdlog::error("open /run/iprouteshow failed");
                close(sockRst);
                return NULL;
            }

            memset(lineBuf, '\0', sizeof(lineBuf));
            while (fgets(lineBuf, sizeof(lineBuf), fp) != NULL) {
                if (strstr(lineBuf, "default") != NULL && strstr(lineBuf, "via") != NULL && strstr(lineBuf, "dev") != NULL && strstr(lineBuf, "eth1") != NULL) {
                    memset(gateway, '\0', sizeof(gateway));
                    sscanf(lineBuf, "default via %s dev eth1", gateway);
                    break;
                }

                memset(lineBuf, '\0', sizeof(lineBuf));
            }

            fclose(fp);
            memset(tBf, '\0', sizeof(tBf));
            snprintf(tBf, sizeof(tBf), "%s", gateway);
            strncat(tmpBuf, tBf, strlen(tBf));

            ssize_t size = strlen(tmpBuf) * sizeof(char);
            inet_pton(AF_INET, "255.255.255.255", &clientaddr.sin_addr.s_addr);

            memset(buff, '\0', sizeof(buff));
            buff[0] = 0x02;
            buff[1] = 0x8E;
            memcpy(&buff[2], tmpBuf, strlen(tmpBuf));
            crcVal = gen_crc((unsigned char *)&buff[1], strlen(buff) - 1);
            length = strlen(buff);
            buff[length] = (char)(crcVal >> 8 & 0xff);
            buff[length + 1] = (char)(crcVal & 0xff);
            buff[length + 2] = 0x03;

            memset(sendBuf, '\0', sizeof(sendBuf));
            length = user_sendTranslateBuf(buff, strlen(buff), sendBuf);

            if (sendto(sockRst, sendBuf, length, 0, (struct sockaddr *)&clientaddr, sizeof(clientaddr)) < 0) {
                close(sockRst);
                spdlog::error("restory broadcast sendto error, errstr:[{0}]", strerror(errno));
                pthread_exit(NULL);
            }
        } else if ((buffer[0] == 0x02) && (buffer[1] == 0x0F) && (buffer[4] == 0x03)) {
            // 02 0f f1 ef 03 //02, 8f, 67, 6f, 69, 6e, 67, 20, 74, 6f, 20, 72, 65, 62, 6f, 6f, 74, 21, 8d, 29, 03
            restoreFactory((char *)"\"mode\":0\r\n");
            memset(tmpBuf, '\0', sizeof(tmpBuf));
            snprintf(tmpBuf, sizeof(tmpBuf), "going to reboot!");
            ssize_t size = strlen(tmpBuf) * sizeof(char);
            inet_pton(AF_INET, "255.255.255.255", &clientaddr.sin_addr.s_addr);

            memset(buff, '\0', sizeof(buff));
            buff[0] = 0x02;
            buff[1] = 0x8F;
            memcpy(&buff[2], tmpBuf, strlen(tmpBuf));
            crcVal = gen_crc((unsigned char *)&buff[1], strlen(buff) - 1);
            length = strlen(buff);
            buff[length] = (char)(crcVal >> 8 & 0xff);
            buff[length + 1] = (char)(crcVal & 0xff);
            buff[length + 2] = 0x03;

            memset(sendBuf, '\0', sizeof(sendBuf));
            length = user_sendTranslateBuf(buff, strlen(buff), sendBuf);

            if (sendto(sockRst, sendBuf, length, 0, (struct sockaddr *)&clientaddr, sizeof(clientaddr)) < 0) {
                close(sockRst);
                spdlog::error("restory broadcast sendto error, errstr:[{0}]", strerror(errno));
                pthread_exit(NULL);
            }

            spdlog::info("searchbroadcast ready to rebooting ......");
            sleep(1);

            system("ifconfig can0 down");
            system("reboot");
        }

        usleep(1000000);
    }

    close(sockRst);
    pthread_exit(NULL);
}
