#ifndef _GNU_SOURCE
#define _GNU_SOURCE
#endif

#include <time.h>
#include <errno.h>
#include <limits.h>
#include <signal.h>
#include <string.h>
#include <stdlib.h>
#include <stdbool.h>
#include <sys/epoll.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <netinet/tcp.h>
#include <spdlog/spdlog.h>

#include "radar_cfg.h"

#define IS_CONNECTION_CLOSED(events)    ((events) & (EPOLLRDHUP | EPOLLHUP | EPOLLERR))

static int epoll_fd = -1;
static int online_client_fd = -1;
static int online_client_count = 0;
static uint16_t listen_port = 5678;
static bool tcp_thread_quit = false;
static pthread_t cfg_server_threads;

void __attribute__((weak)) radar_upper_computer_config(char *data, int bytes)
{

}

/**
 * 函数名称: tcp_listen_thread
 * 功能描述: TCP服务器监听线程
 * 输入参数: param --> 线程参数
 * 输出参数: 无
 * 返回说明: NULL
 */
static void *tcp_listen_thread(void *param)
{
    int ret = -1;
    char buffer[256] = {0};
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

    spdlog::info("radar configure for port:[{0}] is started", listen_port);

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
                        radar_upper_computer_config(buffer, bytes_received);
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

/**
 * 函数名称: radar_response_send
 * 功能描述: 向客户端响应配置数据
 * 输入参数: frame --> 数据帧
 *          bytes --> 数据帧字节数
 * 输出参数: 无
 * 返回说明: 无
 */
void radar_response_send(const void *frame, int bytes)
{
    if ((frame == NULL) || (bytes == 0) || (online_client_count == 0) || (online_client_fd == -1)) {
        return;
    }

    if (send(online_client_fd, frame, bytes, MSG_NOSIGNAL) < 0) {
        spdlog::error("send failed, client:[{0}] for port:[{1}], errstr:[{2}]", online_client_fd, listen_port, strerror(errno));
        epoll_ctl(epoll_fd, EPOLL_CTL_DEL, online_client_fd, NULL);
        close(online_client_fd);
        online_client_fd = -1;
        online_client_count = 0;
    }
}

static void handle_signal(int signo)
{
    if (signo == SIGINT) {
        if (epoll_fd > 0) {
            close(epoll_fd);
            epoll_fd = -1;
        }

        spdlog::warn("received Ctrl-C signal. tcp server cleaning up and exiting");
        exit(EXIT_SUCCESS);
    }
}

/**
 * 函数名称: radar_config_exit
 * 功能描述: 雷达配置退出
 * 输入参数: 无
 * 输出参数: 无
 * 返回说明: 无
 */
void radar_config_exit(void)
{
    pthread_join(cfg_server_threads, NULL);
}

/**
 * 函数名称: radar_config_init
 * 功能描述: 雷达配置初始化
 * 输入参数: port --> 需要监听的端口
 * 输出参数: 无
 * 返回说明: 无
 */
void radar_config_init(uint16_t port)
{
    signal(SIGPIPE, SIG_IGN);
    signal(SIGTTOU, SIG_IGN);
    signal(SIGTTIN, SIG_IGN);
    signal(SIGINT, handle_signal);

    listen_port = port;
    pthread_create(&cfg_server_threads, NULL, tcp_listen_thread, NULL);
}
