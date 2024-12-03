#ifndef __RADAR_CONFIG_H__
#define __RADAR_CONFIG_H__

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * 函数名称: radar_config_init
 * 功能描述: 雷达配置初始化
 * 输入参数: port --> 需要监听的端口
 * 输出参数: 无
 * 返回说明: 无
 */
void radar_config_init(uint16_t port);

/**
 * 函数名称: radar_config_exit
 * 功能描述: 雷达配置退出
 * 输入参数: 无
 * 输出参数: 无
 * 返回说明: 无
 */
void radar_config_exit(void);

/**
 * 函数名称: radar_response_send
 * 功能描述: 向客户端响应配置数据
 * 输入参数: frame --> 数据帧
 *          bytes --> 数据帧字节数
 * 输出参数: 无
 * 返回说明: 无
 */
void radar_response_send(const void *frame, int bytes);

#ifdef __cplusplus
}
#endif

#endif
