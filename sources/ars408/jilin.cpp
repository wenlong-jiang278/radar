#include <time.h>
#include <stdio.h>
#include <errno.h>
#include <unistd.h>
#include <stdint.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include <pthread.h>
#include <stdbool.h>
#include <arpa/inet.h>
#include <linux/tcp.h>
#include <sys/socket.h>
#include <spdlog/spdlog.h>

#include "jilin.h"
#include "circular_queue.h"
#include "epoll_tcp_server.h"

static bool queue_null_use_flag = false;
static circular_queue_t *jilin_queue = NULL;

void jilin_push_message(uint8_t data)
{
#if defined(CONFIG_JILIN)
    queue_null_use_flag = true;
    circular_queue_push(jilin_queue, &data, 1);
#endif
}

#if defined(CONFIG_JILIN)

/* 数据时间戳 */
#pragma pack(push, 1)
typedef struct {
    uint8_t  year;                      /* 年。数据0 - 255，对应2000 - 2255年 */
    uint8_t  month;                     /* 月。1 - 12 */
    uint8_t  day;                       /* 日。1 - 31 */
    uint8_t  hour;                      /* 时。0 - 23 */
    uint8_t  minute;                    /* 分。0 - 59 */
    uint8_t  second;                    /* 秒。0 - 59 */
    uint16_t millisecond;               /* 毫秒。0 - 999 */
} jilin_timestamp_t;
#pragma pack(pop)

/* 数据帧 */
#pragma pack(push, 1)
typedef struct {
    uint32_t          head;             /* 报文头，固定0x55AA55BB */
    uint16_t          length;           /* 报文长度。从length到data的数据总字节数 */
    uint8_t           type;             /* 数据帧类型 */
    uint8_t           checksum;         /* 校验和。从timestamp到data */
    jilin_timestamp_t timestamp;        /* 数据时间戳。数据发送的时间(精确到毫秒) */
    uint8_t           data;             /* 目标数据 */
    uint32_t          tail;             /* 报文尾，固定0x55CC55DD */
} jilin_frame_t;
#pragma pack(pop)

static pthread_t jilin_tid;
static uint16_t jilin_frames_max = 3;

/* 获取当前系统时间 */
static void fill_jilin_timestamp(jilin_timestamp_t *timestamp)
{
    struct timespec ts;
    struct tm *current_time;

    clock_gettime(CLOCK_REALTIME, &ts);
    current_time = localtime(&ts.tv_sec);

    timestamp->year        = current_time->tm_year - 100;   /* 年份偏移量，假设当前年份大于2000年 */
    timestamp->month       = current_time->tm_mon + 1;
    timestamp->day         = current_time->tm_mday;
    timestamp->hour        = current_time->tm_hour + 0;    /* 时区偏移量，中国北京(+8) */
    timestamp->minute      = current_time->tm_min;
    timestamp->second      = current_time->tm_sec;
    timestamp->millisecond = htons(ts.tv_nsec / 1000000);   /* 纳秒转换为毫秒 */
}

/* 填充数据帧 */
static size_t jilin_frame_encode(uint8_t data, jilin_frame_t *frame)
{
    frame->head   = htonl(0x55AA55BB);
    frame->length = htons(sizeof(jilin_frame_t) - 8);
    frame->type   = 0x01;
    frame->data   = data;
    frame->tail   = htonl(0x55CC55DD);
    fill_jilin_timestamp(&frame->timestamp);

    uint8_t *ptr = (uint8_t *)&frame->timestamp;
    size_t size = sizeof(frame->timestamp);

    uint32_t checksum = data;
    for (size_t i = 0; i < size; i++) {
        checksum += ptr[i];
    }

    frame->checksum = checksum & 0xFF;
    return sizeof(jilin_frame_t);
}

/* 数据推送线程 */
static void *jilin_push_thread(void *param)
{
    pthread_setname_np(pthread_self(), "jilin_thread");

    spdlog::info("jilin_push_thread starting......");

    while (1) {
        size_t bytes = 0;
        jilin_frame_t frame;
        uint8_t detect_flag = 0x00;

        if ((jilin_queue != NULL) && !circular_queue_is_empty(jilin_queue)) {
            uint16_t pop_bytes = 0;
            circular_queue_pop(jilin_queue, &detect_flag, &pop_bytes);
            detect_flag = 0x01;
        } else if (jilin_queue == NULL) {
            if (queue_null_use_flag == true) {
                detect_flag = 0x01;
                queue_null_use_flag = false;
            }
        }

        if (tcp_server_clients(1501) > 0) {
            memset(&frame, 0x00, sizeof(frame));
            bytes = jilin_frame_encode(detect_flag, &frame);
            tcp_server_send(1501, (const void *)&frame, bytes);
        }

        usleep(500 * 1000);
    }

    spdlog::info("jilin_push_thread finished......");
    pthread_exit(NULL);
}

/* 初始化 */
void jilin_push_init(void)
{
    jilin_queue = create_circular_queue(jilin_frames_max, 1);
    if (jilin_queue != NULL) {
        spdlog::info("create jilin circular queue {0} frames success", jilin_frames_max);
    } else {
        spdlog::error("create jilin circular queue {0} frames failed", jilin_frames_max);
    }

    pthread_create(&jilin_tid, NULL, jilin_push_thread, NULL);
}

/* 退出 */
void jilin_push_exit(void)
{
    release_circular_queue(jilin_queue);
    pthread_join(jilin_tid, NULL);
}

#endif
