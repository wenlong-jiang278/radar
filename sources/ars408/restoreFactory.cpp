/********************************************************************************
 * @File name:restoreFactory.c
 * @Author:李军
 * @Version: 1.0
 * @Date:2023.05.23
 * @Description:恢复出厂功能
 ********************************************************************************/

#include "restoreFactory.h"
#include "can_tcp.h"
#include "cluster.h"
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

int user_restoreFactory_func(void)
{
    int ret = access("/run/media/mmcblk0p1/fenghai_restoreFactory.txt", F_OK);
    if (ret != 0) {
        spdlog::error("/run/media/mmcblk0p1/fenghai_restoreFactory.txt not exist");
        return -1;
    }

    spdlog::info("going to restore factory ......");
    ret = restoreFactory((char *)"\"mode\":0\r\n");
    if (ret == 0) {
        system("sync");
        sleep(1);

        system("rm -rf /run/media/mmcblk0p1/fenghai_restoreFactory.txt");
        system("sync");
        sleep(1);

        spdlog::info("restore factory success and ready to reboot");

        system("ifconfig can0 down");
        system("reboot");

        return 0;
    }

    return -1;
}
