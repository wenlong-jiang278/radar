#include <fcntl.h>
#include <stdio.h>
#include <errno.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <signal.h>
#include <stdbool.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/ioctl.h>
#include <sys/epoll.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <netinet/tcp.h>
#include <spdlog/spdlog.h>
#include <linux/watchdog.h>

#define WDOG_DEV                        "/dev/watchdog"
#define IS_CONNECTION_CLOSED(events)    ((events) & (EPOLLRDHUP | EPOLLHUP | EPOLLERR))

static int feed = 5;                    /* 喂狗周期(s) */
static int timeout = 20;                /* 超时时间(s) */
static int epoll_fd = -1;
static uint8_t frame[256] = {0};
static int online_client_fd = -1;
static int online_client_count = 0;
static uint16_t listen_port = 65530;
static bool tcp_thread_quit = false;
static pthread_t cfg_server_threads;

static int watchdog_fd = -1;
static bool watchdog_running_enable = false;

/**
 * 客户端发送:
 *      帧头 55 AA 55 BB
 *      校验 数据的校验和(没有数据则为指令的校验和)
 *      指令 01: 重启设备；02: 查询看门狗喂狗周期和超时时间
 *      帧尾 55 CC 55 DD
 * 
 * 服务器响应:
 *      帧头 55 AA 55 BB
 *      校验 指令和数据校验和
 *      指令 01，02; FF: 错误指令
 *      数据 01 - 00/01，02 - xx yy；指令为FF，则没有数据
 *      帧尾 55 CC 55 DD
 */

/**
 * 关闭看门狗:  R: 55 AA 55 BB 03 03 00 55 CC 55 DD    S: 成功: 55 AA 55 BB 03 03 00 55 CC 55 DD 失败: 55 AA 55 BB 03 03 01 55 CC 55 DD
 * 开启看门狗:  R: 55 AA 55 BB 03 03 01 55 CC 55 DD    S: 成功: 55 AA 55 BB 03 03 00 55 CC 55 DD 失败: 55 AA 55 BB 03 03 01 55 CC 55 DD
 */

static int enable_watchdog(void)
{
    int option = WDIOS_ENABLECARD;
    if (ioctl(watchdog_fd, WDIOC_SETOPTIONS, &option) < 0) {
        spdlog::error("enable watchdog ioctl failed, error:[{0}]", strerror(errno));
        return -1;
    } else {
        watchdog_running_enable = true;
    }

    return 0;
}

static int disable_watchdog(void)
{
    int option = WDIOS_DISABLECARD;
    if (ioctl(watchdog_fd, WDIOC_SETOPTIONS, &option) < 0) {
        spdlog::error("disable watchdog ioctl failed, error:[{0}]", strerror(errno));
        return -1;
    } else {
        watchdog_running_enable = false;
    }

    return 0;
}

static uint8_t calculate_checksum(uint8_t *data, uint8_t bytes)
{
    uint8_t i = 0;
    uint32_t checksum = 0;

    for (i = 0; i < bytes; ++i) {
        checksum += data[i];
    }

    return checksum & 0xFF;
}

static void watchdog_handle_client_data(uint8_t *data, int bytes)
{
    if ((data == NULL) || (bytes == 0)) {
        return;
    }

    int send_bytes = 0;
    bool need_reboot = false;

    memset(frame, 0x00, sizeof(frame));
    frame[0] = 0x55;
    frame[1] = 0xAA;
    frame[2] = 0x55;
    frame[3] = 0xBB;
    send_bytes += 4;

    if (bytes == 10) {
        if ((data[0] == 0x55)
            && (data[1] == 0xAA)
            && (data[2] == 0x55)
            && (data[3] == 0xBB)
            && (data[bytes - 4] == 0x55)
            && (data[bytes - 3] == 0xCC)
            && (data[bytes - 2] == 0x55)
            && (data[bytes - 1] == 0xDD)
            && (data[4] == calculate_checksum(&data[5], bytes - 8 - 1)))
        {
            if (data[5] == 0x01) {
                need_reboot = true;

                frame[5]  = 0x01;       /* 指令 */
                frame[6]  = 0x00;       /* 数据 */
                frame[4]  = calculate_checksum(&frame[5], 2);
                frame[7]  = 0x55;
                frame[8]  = 0xCC;
                frame[9]  = 0x55;
                frame[10] = 0xDD;
                send_bytes += 7;
            } else if (data[5] == 0x02) {
                frame[5]  = 0x02;       /* 指令 */
                frame[6]  = feed;       /* 喂狗周期 */
                frame[7]  = timeout;    /* 超时时间 */
                frame[4]  = calculate_checksum(&frame[5], 3);
                frame[8]  = 0x55;
                frame[9]  = 0xCC;
                frame[10] = 0x55;
                frame[11] = 0xDD;
                send_bytes += 8;
            } else {
                frame[5]  = 0xFF;
                frame[6]  = 0xFD;       /* 不支持指令 */
                frame[4]  = calculate_checksum(&frame[5], 2);
                frame[7]  = 0x55;
                frame[8]  = 0xCC;
                frame[9]  = 0x55;
                frame[10] = 0xDD;
                send_bytes += 7;
            }
        } else {
            frame[5]  = 0xFF;
            frame[6]  = 0xFF;           /* 帧格式不正确 */
            frame[4]  = calculate_checksum(&frame[5], 2);
            frame[7]  = 0x55;
            frame[8]  = 0xCC;
            frame[9]  = 0x55;
            frame[10] = 0xDD;
            send_bytes += 7;
        }
    } else if (bytes == 11) {
        if ((data[0] == 0x55)
            && (data[1] == 0xAA)
            && (data[2] == 0x55)
            && (data[3] == 0xBB)
            && (data[bytes - 4] == 0x55)
            && (data[bytes - 3] == 0xCC)
            && (data[bytes - 2] == 0x55)
            && (data[bytes - 1] == 0xDD)
            && (data[4] == calculate_checksum(&data[5], bytes - 8 - 1)))
        {
            if (data[5] == 0x03) {
                if (data[6] == 0x00) {
                    int ret = disable_watchdog();
                    frame[5] = 0x03;
                    frame[6] = ret == 0 ? 0x00 : 0x01;
                    frame[4] = calculate_checksum(&frame[5], 2);
                    frame[7]  = 0x55;
                    frame[8]  = 0xCC;
                    frame[9]  = 0x55;
                    frame[10] = 0xDD;
                    send_bytes += 7;
                } else if (data[6] == 0x01) {
                    int ret = enable_watchdog();
                    frame[5] = 0x03;
                    frame[6] = ret == 0 ? 0x00 : 0x01;
                    frame[4] = calculate_checksum(&frame[5], 2);
                    frame[7]  = 0x55;
                    frame[8]  = 0xCC;
                    frame[9]  = 0x55;
                    frame[10] = 0xDD;
                    send_bytes += 7;
                } else {
                    frame[5]  = 0xFF;
                    frame[6]  = 0xFD;       /* 不支持指令 */
                    frame[4]  = calculate_checksum(&frame[5], 2);
                    frame[7]  = 0x55;
                    frame[8]  = 0xCC;
                    frame[9]  = 0x55;
                    frame[10] = 0xDD;
                    send_bytes += 7;
                }
            } else {
                frame[5]  = 0xFF;
                frame[6]  = 0xFD;       /* 不支持指令 */
                frame[4]  = calculate_checksum(&frame[5], 2);
                frame[7]  = 0x55;
                frame[8]  = 0xCC;
                frame[9]  = 0x55;
                frame[10] = 0xDD;
                send_bytes += 7;
            }
        } else {
            frame[5]  = 0xFF;
            frame[6]  = 0xFF;           /* 帧格式不正确 */
            frame[4]  = calculate_checksum(&frame[5], 2);
            frame[7]  = 0x55;
            frame[8]  = 0xCC;
            frame[9]  = 0x55;
            frame[10] = 0xDD;
            send_bytes += 7;
        }
    } else {
        frame[5]  = 0xFF;
        frame[6]  = 0xFE;               /* 帧长度不正确 */
        frame[4]  = calculate_checksum(&frame[5], 2);
        frame[7]  = 0x55;
        frame[8]  = 0xCC;
        frame[9]  = 0x55;
        frame[10] = 0xDD;
        send_bytes += 7;
    }

    if (send(online_client_fd, frame, send_bytes, MSG_NOSIGNAL) < 0) {
        spdlog::error("watchdog send failed, client:[{0}] for port:[{1}], errstr:[{2}]", online_client_fd, listen_port, strerror(errno));
        epoll_ctl(epoll_fd, EPOLL_CTL_DEL, online_client_fd, NULL);
        close(online_client_fd);
        online_client_fd = -1;
        online_client_count = 0;
    }

    sleep(1);

    if (need_reboot == true) {
        need_reboot = false;
        system("sync");
        system("reboot");
    }
}

static void *watchdog_listen_thread(void *param)
{
    int ret = -1;
    uint8_t buffer[256] = {0};
    int server_fd, client_fd, nfds, n;
    struct epoll_event event, events[1];
    struct sockaddr_in server_addr, client_addr;

    server_fd = socket(AF_INET, SOCK_STREAM, 0);
    if (server_fd < 0) {
        spdlog::error("failed to create socket for port:[{0}], errstr:[{1}]", listen_port, strerror(errno));
        exit(EXIT_FAILURE);
        return NULL;
    }

    server_addr.sin_family = AF_INET;
    server_addr.sin_addr.s_addr = INADDR_ANY;
    server_addr.sin_port = htons(listen_port);

    /* 设置keepalive */
    int keepalive = 1;
    setsockopt(server_fd, SOL_SOCKET, SO_KEEPALIVE, &keepalive, sizeof(keepalive));

    int keep_idle = 60;
    setsockopt(server_fd, SOL_TCP, TCP_KEEPIDLE, &keep_idle, sizeof(keep_idle));

    int keep_interval = 30;
    setsockopt(server_fd, SOL_TCP, TCP_KEEPINTVL, &keep_interval, sizeof(keep_interval));

    int keep_count = 3;
    setsockopt(server_fd, SOL_TCP, TCP_KEEPCNT, &keep_count, sizeof(keep_count));

    memset(&server_addr, 0x00, sizeof(server_addr));
    server_addr.sin_family = AF_INET;
    server_addr.sin_addr.s_addr = INADDR_ANY;
    server_addr.sin_port = htons(listen_port);

    ret = bind(server_fd, (struct sockaddr *)&server_addr, sizeof(server_addr));
    if (ret < 0) {
        close(server_fd);
        spdlog::error("failed to bind server for port:[{0}], errstr:[{1}]", listen_port, strerror(errno));
        exit(EXIT_FAILURE);
        return NULL;
    }

    ret = listen(server_fd, 1);
    if (ret < 0) {
        close(server_fd);
        spdlog::error("failed to listen server for port:[{0}], errstr:[{1}]", listen_port, strerror(errno));
        exit(EXIT_FAILURE);
        return NULL;
    }

    epoll_fd = epoll_create1(EPOLL_CLOEXEC);
    if (epoll_fd < 0) {
        close(server_fd);
        spdlog::error("epoll create failed for port:[{0}], errstr:[{1}]", listen_port, strerror(errno));
        exit(-EXIT_FAILURE);
    }

    event.events = EPOLLIN;
    event.data.fd = server_fd;
    if (epoll_ctl(epoll_fd, EPOLL_CTL_ADD, server_fd, &event) == -1) {
        close(epoll_fd);
        close(server_fd);
        spdlog::error("epoll control failed for port:[{0}], errstr:[{1}]", listen_port, strerror(errno));
        exit(EXIT_FAILURE);
    }

    char name[24] = {0};
    snprintf(name, sizeof(name), "tcp_%d_thr", listen_port);
    pthread_setname_np(pthread_self(), name);

    spdlog::info("watchdog for port:[{0}] is started", listen_port);

    while (1) {
        nfds = epoll_wait(epoll_fd, events, 1, -1);
        if (nfds < 0) {
            if (errno == EINTR) {
                spdlog::warn("epoll is interrupted by signal EINTR, port:[{0}]", listen_port);
                sleep(1);
                continue;
            } else {
                close(epoll_fd);
                close(server_fd);
                spdlog::error("epoll failed to wait for port:[{0}], errstr:[{1}]", listen_port, strerror(errno));
                exit(EXIT_FAILURE);
                return NULL;
            }
        }

        for (n = 0; n < nfds; ++n) {
            if (events[n].data.fd == server_fd) {
                socklen_t client_len = sizeof(client_addr);
                online_client_fd = accept(server_fd, (struct sockaddr *)&client_addr, &client_len);
                if (online_client_fd < 0) {
                    online_client_count = 0;
                    spdlog::warn("accept error, port:[{0}], errstr:[{1}]", listen_port, strerror(errno));
                    continue;
                }

                online_client_count = 1;
                event.events = EPOLLIN | EPOLLRDHUP;
                event.data.fd = online_client_fd;
                epoll_ctl(epoll_fd, EPOLL_CTL_ADD, online_client_fd, &event);

                spdlog::info("client:[{0}] for port:[{1}] is online", online_client_fd, listen_port);
            } else {
                if (IS_CONNECTION_CLOSED(events[n].events)) {
                    online_client_count = 0;
                    epoll_ctl(epoll_fd, EPOLL_CTL_DEL, events[n].data.fd, NULL);
                    close(events[n].data.fd);
                    spdlog::info("client:[{0}] for port:[{1}] is offline", events[n].data.fd, listen_port);
                    continue;
                } else {
                    memset(buffer, 0, sizeof(buffer));
                    int bytes_received = recv(online_client_fd, buffer, sizeof(buffer), 0);
                    if (bytes_received <= 0) {
                        spdlog::error("client:[{0}], port:[{1}], received data is illegal, return:[{2}]", online_client_fd, listen_port, bytes_received);
                        epoll_ctl(epoll_fd, EPOLL_CTL_DEL, online_client_fd, NULL);
                        close(online_client_fd);
                        online_client_fd = -1;
                        online_client_count = 0;
                        continue;
                    } else {
                        watchdog_handle_client_data(buffer, bytes_received);
                    }
                }
            }
        }
    }

    close(epoll_fd);
    close(server_fd);
    online_client_count = 0;

    return NULL;
}

int main(int argc, char *argv[])
{
    int op;

    spdlog::set_pattern("[%Y-%m-%d %H:%M:%S.%e] [%^%L%$] %v");
    spdlog::set_level(spdlog::level::info);

    if (argc >= 3) {
        timeout = atoi(argv[1]);
        if (timeout <= 2) {
            timeout = 20;
        }

        feed = atoi(argv[2]);
        if (feed <= 0) {
            feed = 5;
        }

        if (feed > timeout) {
            feed = timeout - 1;
        }
    }

    watchdog_fd = open(WDOG_DEV, O_RDWR);
    if (watchdog_fd < 0) {
        spdlog::error("open {0} failed, errstr:[{1}]", WDOG_DEV, strerror(errno));
        exit(EXIT_FAILURE);
    }

    /* 打开看门狗之后，看门狗计时器会开启，先停止它 */
    op = WDIOS_DISABLECARD;
    if (ioctl(watchdog_fd, WDIOC_SETOPTIONS, &op) < 0) {
        close(watchdog_fd);
        spdlog::error("ioctl WDIOC_SETOPTIONS failed, error:[{0}]", strerror(errno));
        exit(EXIT_FAILURE);
    }

    if (ioctl(watchdog_fd, WDIOC_SETTIMEOUT, &timeout) < 0) {
        close(watchdog_fd);
        spdlog::error("ioctl WDIOC_SETTIMEOUT failed, error:[{0}]", strerror(errno));
        exit(EXIT_FAILURE);
    }

    /* 开启看门狗计时器 */
    op = WDIOS_ENABLECARD;
    if (ioctl(watchdog_fd, WDIOC_SETOPTIONS, &op) < 0) {
        close(watchdog_fd);
        spdlog::error("ioctl WDIOC_SETOPTIONS failed, error:[{0}]", strerror(errno));
        exit(EXIT_FAILURE);
    }

    spdlog::info("system watchdog starting, feed period {0}s, timeout {1}s", feed, timeout);

    signal(SIGPIPE, SIG_IGN);
    signal(SIGTTOU, SIG_IGN);
    signal(SIGTTIN, SIG_IGN);
    pthread_create(&cfg_server_threads, NULL, watchdog_listen_thread, NULL);

    watchdog_running_enable = true;

    while (1) {
        sleep(feed);
        if (watchdog_running_enable) {
            ioctl(watchdog_fd, WDIOC_KEEPALIVE, NULL);
        }
    }

    close(watchdog_fd);
    watchdog_running_enable = false;
    pthread_join(cfg_server_threads, NULL);

    return 0;
}
