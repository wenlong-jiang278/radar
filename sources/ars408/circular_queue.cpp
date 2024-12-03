#include <string.h>
#include <stdlib.h>
#include "circular_queue.h"

/**
 * 函数名称: create_circular_queue
 * 功能描述: 创建环形队列
 * 输入参数: max_frames - 最大帧数
 *          max_bytes  - 一帧最大字节数
 * 输出参数: 无
 * 返回说明: 成功返回环形队列指针，失败返回NULL
 */
circular_queue_t *create_circular_queue(const uint16_t max_frames, const uint16_t max_bytes)
{
    circular_queue_t *queue = (circular_queue_t *)malloc(sizeof(circular_queue_t));
    if (queue == NULL) {
        return NULL;
    }

    queue->frames = (frame_t *)malloc(max_frames * sizeof(frame_t));
    if (queue->frames == NULL) {
        free(queue);
        return NULL;
    }

    queue->rear = 0;
    queue->front = 0;
    queue->count = 0;
    queue->max_bytes = max_bytes;
    queue->max_frames = max_frames;
    for (uint16_t i = 0; i < max_frames; i++) {
        queue->frames[i].bytes = 0;
        queue->frames[i].data = malloc(max_bytes);
        if (queue->frames[i].data != NULL) {
            memset(queue->frames[i].data, 0x00, max_bytes);
        }
    }

    return queue;
}

/**
 * 函数名称: release_circular_queue
 * 功能描述: 释放环形队列
 * 输入参数: queue - 环形队列指针
 * 输出参数: 无
 * 返回说明: 成功返回true，否则返回false
 */
bool release_circular_queue(circular_queue_t *queue)
{
    if (queue == NULL) {
        return false;
    }

    if (queue->frames != NULL) {
         for (uint16_t i = 0; i < queue->max_frames; ++i) {
            if (queue->frames[i].data != NULL) {
                free(queue->frames[i].data);
                queue->frames[i].data = NULL;
            }
        }

        free(queue->frames);
        queue->frames = NULL;
    }

    free(queue);
    queue = NULL;

    return true;
}

/**
 * 函数名称: circular_queue_frames
 * 功能描述: 获取当前环形队列中的帧数
 * 输入参数: queue - 环形队列指针
 * 输出参数: 无
 * 返回说明: 返回当前环形队列中的帧数
 */
uint16_t circular_queue_frames(circular_queue_t *queue)
{
    if (queue == NULL) {
        return 0;
    }

    return queue->count;
}

/**
 * 函数名称: circular_queue_is_full
 * 功能描述: 判断环形队列是否已满
 * 输入参数: queue - 环形队列指针
 * 输出参数: 无
 * 返回说明: 队列已满返回true，否则返回false
 */
bool circular_queue_is_full(circular_queue_t *queue)
{
    if (queue == NULL) {
        return true;
    }

    return (queue->count == queue->max_frames);
}

/**
 * 函数名称: circular_queue_is_empty
 * 功能描述: 判断环形队列是否为空
 * 输入参数: queue - 环形队列指针
 * 输出参数: 无
 * 返回说明: 队列为空返回true，否则返回false
 */
bool circular_queue_is_empty(circular_queue_t *queue)
{
    if (queue == NULL) {
        return true;
    }

    return (queue->count == 0);
}

/**
 * 函数名称: circular_queue_pop
 * 功能描述: 从环形队列中弹出一帧数据
 * 输入参数: queue - 环形队列指针
 * 输出参数: data  - 数据缓冲区
 *          bytes - 数据字节数
 * 返回说明: 成功返回true，失败返回false
 */
bool circular_queue_pop(circular_queue_t *queue, uint8_t *data, uint16_t *bytes)
{
    if (circular_queue_is_empty(queue)) {
        return false;
    }

    uint16_t index = queue->front;
    if ((queue->frames == NULL) || (queue->frames[index].data == NULL)) {
        return false;
    }

    uint16_t bytes2Read = queue->frames[index].bytes;

    /* 接收数据缓冲区为空 */
    if ((data == NULL) || (bytes2Read == 0)) {
        return false;
    }

    /* 将数据复制到缓冲区 */
    memcpy(data, queue->frames[index].data, bytes2Read);

    if (bytes != NULL) {
        /* 当前数据帧的字节数 */
        *bytes = bytes2Read;
    }

    queue->frames[index].bytes = 0;                                         /* 清空当前帧字节数 */
    queue->front = (queue->front + 1) % queue->max_frames;                  /* 将队头索引移动到下一个位置 */
    queue->count--;                                                         /* 队列中的帧数减1 */

    return true;
}

/**
 * 函数名称: circular_queue_push
 * 功能描述: 将数据压入环形队列
 * 输入参数: queue - 环形队列指针
 *          data  - 数据缓冲区
 *          bytes - 数据字节数
 * 输出参数: 无
 * 返回说明: 成功返回true，失败返回false
 */
bool circular_queue_push(circular_queue_t *queue, const uint8_t *data, const uint16_t bytes)
{
    if (queue == NULL) {
        return false;
    }

    /* 数据超过最大帧大小 */
    if (bytes > queue->max_bytes) {
        return false;
    }

    if (circular_queue_is_full(queue)) {
        return false;

        /* 覆盖旧帧 */
        // queue->front = (queue->front + 1) % queue->max_frames;
        // queue->count--;
    }

    uint16_t index = queue->rear;
    if ((queue->frames == NULL) || (queue->frames[index].data == NULL)) {
        return false;
    }

    memcpy(queue->frames[index].data, data, bytes);             /* 将数据复制到帧中 */
    queue->frames[index].bytes = bytes;                         /* 设置当前帧字节数 */
    queue->rear = (queue->rear + 1) % queue->max_frames;        /* 将队尾索引移动到下一个位置 */
    queue->count++;

    return true;
}
