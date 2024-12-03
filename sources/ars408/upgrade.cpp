/********************************************************************************
 * @File name:upgrade.c
 * @Author:李军
 * @Version: 1.0
 * @Date:2023.05.12
 * @Description:远程升级
 ********************************************************************************/

#include "upgrade.h"
#include "can_tcp.h"
#include "cluster.h"
#include "gps.h"
#include "http.h"
#include "ntp.h"
#include "sqlite3.h"
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

int acceptIdx = 0;
int g_upgradingFlg = 0;
int acceptId[10] = {0};

void *upgradeThred(void *arg)
{
    // system("df /dev/mmcblk0p1 -h");

    pthread_detach(pthread_self());

    int sockfd = socket(AF_INET, SOCK_STREAM, 0);
    if (sockfd == -1) {
        spdlog::error("upgrade socket failed, errstr:[{0}]", strerror(errno));
        pthread_exit(NULL);
    }

    int on = 1;
    int result = setsockopt(sockfd, SOL_SOCKET, SO_REUSEADDR, &on, sizeof(on));
    if (result < 0) {
        spdlog::error("upgrade setsockopt of reuseaddr, errstr:[{0}]", strerror(errno));
    }

    struct sockaddr_in saddr;
    memset(&saddr, 0, sizeof(saddr));
    saddr.sin_family = AF_INET;
    saddr.sin_port = htons(9001);
    saddr.sin_addr.s_addr = INADDR_ANY;
    int res = bind(sockfd, (struct sockaddr *)&saddr, sizeof(saddr));
    if (res == -1) {
        close(sockfd);
        spdlog::error("upgrade bind failed, errstr:[{0}]", strerror(errno));
        pthread_exit(NULL);
    }

    if (listen(sockfd, 5) == -1) {
        close(sockfd);
        spdlog::error("upgrade listen failed, errstr:[{0}]", strerror(errno));
        pthread_exit(NULL);
    }

    pthread_setname_np(pthread_self(), "upgrade_thread");

    struct sockaddr_in caddr;
    socklen_t len = sizeof(caddr);

    while (1) {
        acceptId[acceptIdx] = accept(sockfd, (struct sockaddr *)&caddr, &len);
        if (acceptId[acceptIdx] < 0) {
            usleep(1000000);
            continue;
        }

        spdlog::info("upgrade client acceptId[{0}] = {1}", acceptIdx, acceptId[acceptIdx]);

        pthread_t id;
        int retCreate = pthread_create(&id, NULL, upgradeFile, &acceptId[acceptIdx]);
        if (retCreate != 0) {
            close(acceptId[acceptIdx]);
            spdlog::error("create upgrade file thread failed");
            usleep(1000000);
        }

        (acceptIdx < 10 - 1) ? (acceptIdx++) : (acceptIdx = 0);
    }
}

void *upgradeFile(void *arg)
{
    int fd = *((int *)arg);

    int fileFd;
    int crcVal;
    int ret, res;
    int wrIdx = 0;
    FILE *fp = NULL;
    char failedFlg = 0;
    char failedVal = 0;
    int wrIdxBackup = 0;
    char lastPackFlg = 0;
    double davailSize = 0;
    struct timeval timeout;
    char Size[256] = {"\0"};
    char Used[256] = {"\0"};
    char UsePer[256] = {"\0"};
    char buff[1024 * 8] = {0};
    unsigned int fileSize = 0;
    struct dirent *ent = NULL;
    char Available[256] = {"\0"};
    char Filesystem[256] = {"\0"};
    char Mounted_on[256] = {"\0"};
    char data2Parse[1024 * 8] = {0};
    char dataBuffer[1024 * 8] = {0};
    unsigned int haveRecvedSize = 0;

    if (fd < 0) {
        spdlog::error("upgradeFile socket fd < 0");
        pthread_exit(NULL);
    }

    timeout.tv_sec = USER_TCPCOMTIMEOUT;
    timeout.tv_usec = 0;

    int result = setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, (char *)&timeout.tv_sec, sizeof(struct timeval));
    if (result < 0) {
        spdlog::error("setsockopt failed, errstr:[{1}]", strerror(errno));
    }

    pthread_setname_np(pthread_self(), "upgradefile_thr");

    while (1) {
        res = recv(fd, buff, sizeof(buff), 0);
        if (res < 0) {
            spdlog::error("upgrade recv failed, errstr:[{0}]", strerror(errno));
            close(fd);
            g_upgradingFlg = 0;
            pthread_exit(NULL);
        } else if (res == 0) {
            spdlog::error("upgrade recv == 0, errstr:[{0}]", strerror(errno));
            close(fd);
            g_upgradingFlg = 0;
            pthread_exit(NULL);
        }

        if ((buff[0] == 0x02) && (buff[1] == 0x0A)) {
            /* SD卡挂载成功? */
            system("cat /proc/mounts > /run/mountsFile");
            usleep(50000);

            fp = fopen("/run/mountsFile", "r+");
            if (fp == NULL) {
                failedFlg = 1;
                goto goto_upgradeFileFailed;
            }

            failedFlg = 1;
            while (fgets(dataBuffer, sizeof(dataBuffer), fp) != NULL) {
                spdlog::info("upgrade device:[{1}]", dataBuffer);
                if ((strstr(dataBuffer, "/dev/mmcblk0p1") != NULL) && (strstr(dataBuffer, "/run/media/mmcblk0p1") != NULL)) {
                    failedFlg = 0;
                    break;
                }
            }

            fclose(fp);
            if (failedFlg == 1) {
                failedFlg = 0;
                goto goto_upgradeFileFailed;
            }

            /* 正在升级? */
            if (g_upgradingFlg == 1) {
goto_upgradeFileFailed:
                memset(buff, '\0', sizeof(buff));
                buff[0] = 0x02;
                buff[1] = 0x8a;
                if (failedVal == 0) {
                    buff[2] = 0x01;
                } else if (failedVal == 4) {
                    buff[2] = 0x04;
                }

                unsigned short int crcVal;
                crcVal = gen_crc((unsigned char *)&buff[1], 2);
                buff[3] = crcVal >> 8;
                buff[4] = crcVal;
                buff[5] = 0x03;
                ret = send(fd, buff, 6, 0);
                if (ret < 0) {
                    close(fd);
                    pthread_exit(NULL);
                }

                close(fd);
                pthread_exit(NULL);
            }

            g_upgradingFlg = 1;
            wrIdx = 0;
            memcpy(data2Parse, buff, res);
            wrIdx = user_translateBuf(data2Parse, res, dataBuffer);

            // 校验成功?
            crcVal = gen_crc((unsigned char *)&dataBuffer[1], wrIdx - 4);
            if (((char)(crcVal >> 8 & 0xff) != dataBuffer[wrIdx - 3]) || ((char)(crcVal & 0xff) != dataBuffer[wrIdx - 2])) {
                goto goto_upgradeFileFailed;
            }

            fileSize = ((unsigned int)dataBuffer[2] << 24) | ((unsigned int)dataBuffer[3] << 16) | ((unsigned int)dataBuffer[4] << 8) | ((unsigned int)dataBuffer[5]);
            spdlog::info("upgrade fileSize:[{0}]", fileSize);
            if (fileSize <= USER_ONEPACKETSIZE) {
                lastPackFlg = 1;
            }

            // SD卡容量足够?
            system("df /dev/mmcblk0p1 -h > /run/df_h_mmcblk0p1");
            usleep(200000);

            fp = fopen("/run/df_h_mmcblk0p1", "r+");
            if (fp == NULL) {
                failedFlg = 1;
                goto goto_upgradeFileFailed;
            }

            failedFlg = 1;
            while (fgets(dataBuffer, sizeof(dataBuffer), fp) != NULL) {
                if (strstr(dataBuffer, "/dev/mmcblk0p1") != NULL) {
                    sscanf(dataBuffer, "%s %s %s %s %s %s", Filesystem, Size, Used, Available, UsePer, Mounted_on);
                    spdlog::info("Filesystem:[{0}], Size:[{1}], Used:[{2}], Available:[{3}], UsePer:[{4}], Mounted_on:[{5}]", Filesystem, Size, Used, Available, UsePer, Mounted_on);

                    if (Available[strlen(Available) - 1] == 'G') {
                        davailSize = strtod(Available, NULL);
                        davailSize = davailSize * 1024 * 1024 * 1024;
                    } else if (Available[strlen(Available) - 1] == 'M') {
                        davailSize = strtod(Available, NULL);
                        davailSize = davailSize * 1024 * 1024;
                    }

                    if ((int)(davailSize * 0.4) < fileSize) {
                        goto goto_upgradeFileFailed;
                    }

                    failedFlg = 0;
                    break;
                }
            }

            fclose(fp);
            if (failedFlg == 1) {
                failedFlg = 0;
                goto goto_upgradeFileFailed;
            }

            g_upgradingFlg = 1;
            wrIdx = 0;

            fileFd = open("/run/media/mmcblk0p1/upgradeFiles.zip", O_WRONLY | O_CREAT, 0777);
            if (fileFd < 0) {
                close(fd);
                g_upgradingFlg = 0;
                pthread_exit(NULL);
            }

            memset(buff, '\0', sizeof(buff));
            buff[0] = 0x02;
            buff[1] = 0x8a;
            buff[2] = 0x00;

            unsigned short int crcVal;
            crcVal = gen_crc((unsigned char *)&buff[1], 2);
            buff[3] = crcVal >> 8;
            buff[4] = crcVal;
            buff[5] = 0x03;

            ret = send(fd, buff, 6, 0);
            if (ret < 0) {
                close(fd);
                close(fileFd);
                g_upgradingFlg = 0;
                pthread_exit(NULL);
            }

            continue;
        }
    
        if ((res == 5)) {
            if ((buff[0] == 0x02) && (buff[1] == 0x0C)) {
                memset(buff, '\0', sizeof(buff));
                buff[0] = 0x02;
                buff[1] = 0x8c;
                (g_upgradingFlg == 1) ? (buff[2] = 0x00) : (buff[2] = 0x01, g_upgradingFlg = 0);

                unsigned short int crcVal;
                crcVal = gen_crc((unsigned char *)&buff[1], 2);
                buff[3] = crcVal >> 8;
                buff[4] = crcVal;
                buff[5] = 0x03;

                ret = send(fd, buff, 6, 0);
                if (ret < 0) {
                    close(fd);
                    close(fileFd);
                    g_upgradingFlg = 0;
                    pthread_exit(NULL);
                }

                close(fileFd);
                break;
            }
        }

        memcpy(data2Parse + wrIdx, buff, res);
        wrIdx += res;
        if ((data2Parse[wrIdx - 1] == 0x03) && (g_upgradingFlg == 1)) {
            wrIdxBackup = wrIdx;
            wrIdx = user_translateBuf(data2Parse, wrIdx, dataBuffer);
            if (lastPackFlg == 0) {             // 非最后一包（一包可能被分成多次recv接收）
                if (wrIdx < USER_ONEPACKETSIZE + 5) {
                    wrIdx = wrIdxBackup;
                    continue;
                }
            } else if (lastPackFlg == 1) {      // 最后一包（一包可能被分成多次recv接收）
                if (wrIdx < fileSize - haveRecvedSize + 5) {
                    wrIdx = wrIdxBackup;
                    continue;
                }
            }

            crcVal = gen_crc((unsigned char *)&dataBuffer[1], wrIdx - 4);
            if (((char)(crcVal >> 8 & 0xff) != dataBuffer[wrIdx - 3]) || ((char)(crcVal & 0xff) != dataBuffer[wrIdx - 2])) {
                memset(buff, '\0', sizeof(buff));
                buff[0] = 0x02;
                buff[1] = 0x8b;
                buff[2] = 0x01;

                unsigned short int crcVal;
                crcVal = gen_crc((unsigned char *)&buff[1], 2);
                buff[3] = crcVal >> 8;
                buff[4] = crcVal;
                buff[5] = 0x03;

                ret = send(fd, buff, 6, 0);
                if (ret < 0) {
                    spdlog::error("upgrade send failed");
                }

                close(fd);
                close(fileFd);
                g_upgradingFlg = 0;
                pthread_exit(NULL);
            }

            ret = write(fileFd, dataBuffer + 2, wrIdx - 5);
            if (ret < 0) {
                close(fd);
                close(fileFd);
                g_upgradingFlg = 0;
                pthread_exit(NULL);
            }

            memset(dataBuffer, '\0', sizeof(dataBuffer));
            memset(buff, '\0', sizeof(buff));
            buff[0] = 0x02;
            buff[1] = 0x8b;
            buff[2] = 0x00;

            unsigned short int crcVal;
            crcVal = gen_crc((unsigned char *)&buff[1], 2);
            buff[3] = crcVal >> 8;
            buff[4] = crcVal;
            buff[5] = 0x03;

            ret = send(fd, buff, 6, 0);
            if (ret < 0) {
                close(fd);
                close(fileFd);
                g_upgradingFlg = 0;
                pthread_exit(NULL);
            }

            wrIdx = 0;
            haveRecvedSize += USER_ONEPACKETSIZE;
            if (fileSize - haveRecvedSize <= USER_ONEPACKETSIZE) { // 接下来接收最后一包
                lastPackFlg = 1;
            }
        }
    }

    if (g_upgradingFlg == 1) {
        system("unzip -o /run/media/mmcblk0p1/upgradeFiles.zip -d /run/media/mmcblk0p1/");
        sleep(1);

        failedFlg = 1;
        DIR *pDir = opendir("/run/media/mmcblk0p1/upgradeFiles");
        if (pDir == NULL) {
            close(fd);
            close(fileFd);
            g_upgradingFlg = 0;
            pthread_exit(NULL);
        }

        while ((ent = readdir(pDir)) != NULL) {
            if ((strcmp(ent->d_name, ".") == 0) || (strcmp(ent->d_name, "..") == 0)) {
                continue;
            }

            if (strcmp(ent->d_name, "gotoUpgradeShellFile.sh") == 0) {
                system("/run/media/mmcblk0p1/upgradeFiles/gotoUpgradeShellFile.sh");
                failedFlg = 0;
                break;
            }
        }

        if (failedFlg == 1) {
            failedFlg = 0;
            closedir(pDir);
            g_upgradingFlg = 0;
            failedVal = 4;
            goto goto_upgradeFileFailed;
        }

        closedir(pDir);
        close(fd);
        close(fileFd);
        g_upgradingFlg = 0;
    }

    pthread_exit(NULL);
}

unsigned int gen_crc(unsigned char *frame, unsigned short frame_len)
{
    short i, j;
    unsigned int wcrc = 0;
    unsigned char c, treat, bcrc;

    for (i = 0; i < frame_len; i++) {
        c = frame[i];
        for (j = 0; j < 8; j++) {
            treat = c & 0x80;
            c <<= 1;
            bcrc = (wcrc >> 8) & 0x80;
            wcrc <<= 1;
            if (treat != bcrc) {
                wcrc ^= 0x1021;
            }
        }
    }

    return wcrc;
}

int user_translateBuf(char *in, int inSize, char *out)
{
    out[0] = in[0];
    out[1] = in[1];

    int wrIdx = 2;
    for (int i = 2; i < inSize;) {
        if ((in[i] == 0x02) && (in[i + 1] == 0xe7)) {
            out[wrIdx++] = 0x02;
            i = i + 2;
        } else if ((in[i] == 0x03) && (in[i + 1] == 0xe8)) {
            out[wrIdx++] = 0x03;
            i = i + 2;
        } else {
            out[wrIdx++] = in[i];
            i = i + 1;
        }
    }

    return wrIdx;
}

int user_sendTranslateBuf(char *in, int inSize, char *out)
{
    out[0] = in[0];

    int i, wrIdx = 1;

    for (i = 1; i < inSize - 1; i++) {
        if (in[i] == 0x02) {
            out[wrIdx++] = 0x02;
            out[wrIdx++] = 0xE7;
        } else if (in[i] == 0x03) {
            out[wrIdx++] = 0x03;
            out[wrIdx++] = 0xE8;
        } else {
            out[wrIdx++] = in[i];
        }
    }

    out[wrIdx++] = in[i];
    return wrIdx;
}
