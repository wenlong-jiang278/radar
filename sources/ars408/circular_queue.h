#ifndef CIRCULAR_QUEUE_H
#define CIRCULAR_QUEUE_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* 一帧数据 */
typedef struct {
    void     *data;             /* 数据缓冲区 */
    uint16_t bytes;             /* 帧中数据的实际字节数 */
} frame_t;

/* 环形队列 */
typedef struct {
    frame_t  *frames;           /* 帧数组 */
    uint16_t front;             /* 队头索引 */
    uint16_t rear;              /* 队尾索引 */
    uint16_t count;             /* 当前队列中的帧数 */
    uint16_t max_bytes;         /* 一帧允许的最大字节数 */
    uint16_t max_frames;        /* 队列可以容纳的最大帧数 */
} circular_queue_t;

/**
 * 函数名称: create_circular_queue
 * 功能描述: 创建环形队列
 * 输入参数: max_frames - 最大帧数
 *          max_bytes  - 一帧最大字节数
 * 输出参数: 无
 * 返回说明: 成功返回环形队列指针，失败返回NULL
 */
circular_queue_t *create_circular_queue(const uint16_t max_frames, const uint16_t max_bytes);

/**
 * 函数名称: release_circular_queue
 * 功能描述: 释放环形队列
 * 输入参数: queue - 环形队列指针
 * 输出参数: 无
 * 返回说明: 成功返回true，否则返回false
 */
bool release_circular_queue(circular_queue_t *queue);

/**
 * 函数名称: circular_queue_frames
 * 功能描述: 获取当前环形队列中的帧数
 * 输入参数: queue - 环形队列指针
 * 输出参数: 无
 * 返回说明: 返回当前环形队列中的帧数
 */
uint16_t circular_queue_frames(circular_queue_t *queue);

/**
 * 函数名称: circular_queue_is_full
 * 功能描述: 判断环形队列是否已满
 * 输入参数: queue - 环形队列指针
 * 输出参数: 无
 * 返回说明: 队列已满返回true，否则返回false
 */
bool circular_queue_is_full(circular_queue_t *queue);

/**
 * 函数名称: circular_queue_is_empty
 * 功能描述: 判断环形队列是否为空
 * 输入参数: queue - 环形队列指针
 * 输出参数: 无
 * 返回说明: 队列为空返回true，否则返回false
 */
bool circular_queue_is_empty(circular_queue_t *queue);

/**
 * 函数名称: circular_queue_pop
 * 功能描述: 从环形队列中弹出一帧数据
 * 输入参数: queue - 环形队列指针
 * 输出参数: data  - 数据缓冲区
 *          bytes - 数据字节数
 * 返回说明: 成功返回true，失败返回false
 */
bool circular_queue_pop(circular_queue_t *queue, uint8_t *data, uint16_t *bytes);

/**
 * 函数名称: circular_queue_push
 * 功能描述: 将数据压入环形队列
 * 输入参数: queue - 环形队列指针
 *          data  - 数据缓冲区
 *          bytes - 数据字节数
 * 输出参数: 无
 * 返回说明: 成功返回true，失败返回false
 */
bool circular_queue_push(circular_queue_t *queue, const uint8_t *data, const uint16_t bytes);

#ifdef __cplusplus
}
#endif

#endif
