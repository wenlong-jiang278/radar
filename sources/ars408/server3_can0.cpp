#include <errno.h>
#include <unistd.h>
#include <signal.h>
#include <spdlog/spdlog.h>

#include "gps.h"
#include "ntp.h"
#include "http.h"
#include "Track.h"
#include "jilin.h"
#include "frame.h"
#include "upgrade.h"
#include "cluster.h"
#include "can_tcp.h"
#include "can_frame.h"
#include "radar_cfg.h"
#include "server3_can0.h"
#include "restoreFactory.h"
#include "searchBroadcast.h"
#include "epoll_tcp_server.h"

int server3_can0_main(int argc, char *argv[])
{
    errno = 0;//重置errno全局变量，用于错误处理。

    nice(-20);//降低程序的优先级，使其运行在较低的CPU优先级。
    signal(SIGPIPE, SIG_IGN);//忽略SIGPIPE信号，通常在写入已关闭的管道时产生。

    srand((int)time(NULL));//设置随机数种子，用于生成随机数。

    /*使用spdlog库设置日志模式和级别*/
    spdlog::set_pattern("[%Y-%m-%d %H:%M:%S.%e] [%^%L%$] %v");
    spdlog::set_level(spdlog::level::info);

    /* 初始化跟踪程序 */
    track_init(&track);
    /* 初始化跟踪器包 */
    track_pre_start(&track);

    event_params.car_head = (struct list *)List_Init(NULL);
    event_params.car_stop = (struct list *)List_Init(NULL);
    event_params.Lane_List_Length = 0;

    /*读取配置文件和GNSS配置文件*/
    user_readCfgFile();
    user_readGNSSCfgFile();

    if (user_restoreFactory_func() == 0) {
        return 0;
    }//恢复出厂设置，如果失败则退出程序。

    user_readClusterAndObjectSend2Rearend_func();   // 点云数据传输配置
    user_readDynamicOrStaticCalibrationMode_func(); // 标定配置
    user_readCalibrationPara();                     // 动态标定参数
    user_readStaticCalibrationPara();               // 静态标定参数
    user_readOffsetParameters();                    // 偏移参数
    user_readRoadLaneParam();                       // 读取道路参数

    // 定义返回值和线程属性变量
    int ret;
    pthread_attr_t attr;// 线程属性对象
    struct sched_param sp;// 调度参数结构体
    /*根据是否定义CONFIG_JILIN宏，设置监听端口数组*/
#if defined(CONFIG_JILIN)
    uint16_t listen_ports[] = {1500, 1501, 1502};// 如果定义了CONFIG_JILIN，监听1500, 1501, 1502端口
#else
    uint16_t listen_ports[] = {1500, 1502};//否则，只监听1500和1502端口
#endif
    // 定义多个线程ID变量
    pthread_t id1, id2, id3, id4, id5, id6, id7, id8, id9, id10;

    radar_config_init(5678);// 初始化雷达配置，端口号为5678
    tcp_server_probe(listen_ports, sizeof(listen_ports) / sizeof(listen_ports[0]));// 探测TCP服务器监听的端口


    ret = pthread_attr_init(&attr);// 初始化线程属性对象
    if (ret != 0) {
        spdlog::error("pthread_attr_init error"); // 如果初始化失败，记录错误日志
    }

    ret = pthread_attr_setinheritsched(&attr, PTHREAD_EXPLICIT_SCHED);// 设置线程属性为显式调度
    if (ret != 0) {
        spdlog::error("pthread_attr_setinheritsched error");// 如果设置失败，记录错误日志
    }

    ret = pthread_attr_setschedpolicy(&attr, SCHED_RR);// 设置线程调度策略为SCHED_RR（轮询调度）
    if (ret != 0) {
        spdlog::error("pthread_attr_setschedpolicy error");
    }

    sp.sched_priority = 99;// 设置线程的调度优先级
    ret = pthread_attr_setschedparam(&attr, &sp);
    if (ret != 0) {
        spdlog::error("pthread_attr_setschedparam error");
    }

    ret = pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_JOINABLE);// 设置线程的分离状态为可连接，以便可以等待线程结束
    if (ret != 0) {
        spdlog::error("pthread_attr_setdetachstate error");
    }

    ret = pthread_create(&id1, NULL, usr_readCanData, NULL);// 创建一个线程来读取CAN数据
    if (ret) {
        spdlog::error("create usr_readCanData thread error");
    }

#if defined(CONFIG_CANFRAME)// 如果定义了CONFIG_CANFRAME宏，则初始化CAN帧分析模块
    can_frame_analysis_init();
#endif

    ret = pthread_create(&id2, NULL, http_manager_thread, NULL);// 创建HTTP管理线程
    if (ret) {
        spdlog::error("create http_manager thread error");
    }

    ret = pthread_create(&id3, NULL, ntpGetDateTime, NULL);// 创建NTP时间获取线程
    if (ret) {
        spdlog::error("create ntpGetDateTime thread error");
    }

#if defined(CONFIG_UPGRADE)// 如果定义了CONFIG_UPGRADE宏，则创建升级线程
    ret = pthread_create(&id4, NULL, upgradeThred, NULL);
    if (ret) {
        spdlog::error("create upgrade thread error");
    }
#endif

#if defined(CONFIG_GPS)// 如果定义了CONFIG_GPS宏，则创建GPS相关线程
    ret = pthread_create(&id5, NULL, uart5_pthread, NULL);
    if (ret) {
        spdlog::error("create uart5 thread error");
    }

    ret = pthread_create(&id6, NULL, GNSSSetTime_pthread, NULL);
    if (ret) {
        spdlog::error("create GNSSSetTime thread error");
    }
#endif

#if defined(CONFIG_SEARCHBRDCAST)// 如果定义了CONFIG_SEARCHBRDCAST宏，则创建广播搜索和恢复线程
    ret = pthread_create(&id7, NULL, hdlSearchBroadcast_pthread, NULL);
    if (ret) {
        spdlog::error("create hdlSearchBroadcast thread error");
    }

    ret = pthread_create(&id8, NULL, hdlSearchAndRestoryBrdcast_pthread, NULL);
    if (ret) {
        spdlog::error("create hdlSearchAndRestoryBrdcast thread error");
    }
#endif

    sp.sched_priority = 98;// 将线程的调度优先级设置为98
    pthread_attr_setschedparam(&attr, &sp);

    ret = pthread_create(&id10, NULL, user_hdlCanFrameThread, NULL);// 创建用户CAN帧处理线程
    if (ret) {
        spdlog::error("create user_hdlCanFrame thread error");
    }

    frame_push_protocol_init();// 初始化帧推送协议

#if defined(CONFIG_JILIN)// 如果定义了CONFIG_JILIN宏，则初始化吉林推送模块
    jilin_push_init();
#endif

    pthread_join(id1, NULL);// 等待所有创建的线程结束
    pthread_join(id2, NULL);
    pthread_join(id3, NULL);
#if defined(CONFIG_UPGRADE)
    pthread_join(id4, NULL);
#endif
    pthread_join(id5, NULL);
    pthread_join(id6, NULL);
#if defined(CONFIG_SEARCHBRDCAST)
    pthread_join(id7, NULL);
    pthread_join(id8, NULL);
#endif
    pthread_join(id10, NULL);

    frame_push_protocol_exit();// 退出帧推送协议

#if defined(CONFIG_JILIN)// 如果定义了CONFIG_JILIN宏，则退出吉林推送模块
    jilin_push_exit();
#endif

#if defined(CONFIG_CANFRAME)// 如果定义了CONFIG_CANFRAME宏，则退出CAN帧分析模块
    can_frame_analysis_exit();
#endif

    radar_config_exit();// 退出雷达配置和TCP服务器
    tcp_server_remove();
    return 0;
}
