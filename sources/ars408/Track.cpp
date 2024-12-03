#include "Track.h"
#include "trafficEvent.h"
#include "ExtraJudge.h"
#include "filter_track.h"

#include <exception>
#include <spdlog/spdlog.h>

#include <fstream>

/* 定义目标跟踪全局变量 */
track_t track; //用于存储跟踪器的状态和参数。
AdjustParams adjust_Params; //用于存储调整参数。

extern CalbParams g_CalbPara;  //校准参数
extern StaticCalbParams g_StaticCalbPara;  //静态校准模式
extern char g_dynamicOrStaticCalibrationMode; //动态校准模式
extern ARS408RadarObjectInfo_Wr RadarObjectData_Wr[250];//雷达对象数据

KalmanFilter kf; //卡尔曼滤波
radar_sys_params_t RadarSysParams;
FUNC_DECOR track_trk_pkg_t trk_internal;

/*---------输出延时补偿用----------*/
Track_ListObj *ObjectList = NULL;

// std::ofstream ofs;
// bool file_open_flag = false;
// int file_close_count = 0;
/*-------------------------------*/

/********************************************************
 * Function name ：Track_PointCloud_Process
 * Description   ：点云数据跟踪分类算法主函数
 * Parameter     ：
 * @track           跟踪器参数
 * @RadarPointCloud 输入点云数据数组
 * @PointNumber     输入点云数目
 * @RadarObjectData 输出跟踪处理后的目标数据数组
 * @ObjectNumber    输出跟踪处理后的目标数目
 * Return          ：RCS基准值
 **********************************************************/
#if TEST_MODE

#else
/**
 * 处理雷达点云数据，进行目标跟踪和事件检测，并将结果用于进一步的分析。
 */
void Track_PointCloud_Process(track_t *track_i, RadarClusterInfoARS *RadarPointCloud, int PointNumber, ARS408RadarObjectInfo *RadarObjectData, int *ObjectNumber, Event_Params *event_params)
{
    double elapsed = 0.0;
    struct timeval starting, finished;

#if 0
    track_Params_update(&track);
    track_read(&track, RadarPointCloud, PointNumber);
    track_run(&track);
    track_stop(&track);
    track_OutObjectList(&track, RadarObjectData, ObjectNumber);
#else
    /*---------输出延时补偿用----------*/
    if (ObjectList == NULL) {
      track_listInit(&ObjectList);
    }
    /*-------------------------------*/

    // gettimeofday(&starting, NULL);
    track_Params_update(track_i);

    track_read(track_i, RadarPointCloud, PointNumber);
    track_run(track_i);
    track_stop(track_i);
    // gettimeofday(&finished, NULL);
    // elapsed = calc_elapsed_time(starting, finished);
    // printf("exec track prev took: %f ms\n", elapsed);

    // gettimeofday(&starting, NULL);
    track_OutObjectList(track_i, RadarObjectData, ObjectNumber);
    // gettimeofday(&finished, NULL);
    // elapsed = calc_elapsed_time(starting, finished);
    // printf("exec track_OutObjectList took: %f ms\n", elapsed);

    /*---------输出延时补偿用----------*/
    track_DelayCompensation(ObjectList, RadarObjectData, ObjectNumber);
    // if (!file_open_flag) {
    //     ofs.open("/home/root/20241016.csv", std::ios::out | std::ios::trunc);
    //     if (!ofs.is_open()) {
    //         std::cout << "file open failed" << std::endl;
    //     }
    //     ofs << "timestamp, ID, DistLong, DistLat, VrelLong, VrelLat" << std::endl;
    //     file_open_flag = true;
    // }

    // for (int i = 0; i < *ObjectNumber; i++) {
    //         ofs << RadarObjectData[i].Object_timestamp << "," << RadarObjectData[i].Object_ID << "," <<
    //             RadarObjectData[i].Object_DistLong << "," << RadarObjectData[i].Object_DistLat << "," <<
    //             RadarObjectData[i].Object_VrelLong << "," << RadarObjectData[i].Object_VrelLat << std::endl;
    //             file_close_count++;
    // }

    // if (file_close_count > 100000) {
    //     ofs.close();
    // }
    /*-------------------------------*/
#endif

    /* 事件检测 */
    if (adjust_Params.EnableExtra_Judge) {
        // gettimeofday(&starting, NULL);
        Extra_Judge(RadarObjectData, ObjectNumber, event_params);
        // gettimeofday(&finished, NULL);
        // elapsed = calc_elapsed_time(starting, finished);
        // printf("exec Extra_Judge took: %f ms\n", elapsed);
    }

    // gettimeofday(&starting, NULL);
    TrafficEventRecongition(RadarObjectData, ObjectNumber, event_params);
    // gettimeofday(&finished, NULL);
    // elapsed = calc_elapsed_time(starting, finished);
    // printf("exec TrafficEventRecongition took: %f ms\n", elapsed);

    // gettimeofday(&starting, NULL);
    trafficFlowDetection(RadarObjectData, *ObjectNumber);
    // gettimeofday(&finished, NULL);
    // elapsed = calc_elapsed_time(starting, finished);
    // printf("exec trafficFlowDetection took: %f ms\n", elapsed);
}
#endif


/******************
 * * 初始化跟踪器函数 
 * 该函数设置了跟踪器的初始状态，包括Kalman滤波器的初始化、用户参数的清空、雷达参数的设置、存储空间的清零以及交通流统计参数的初始化。
 * ***************/
void track_init(track_t *track)
{
    if (track == NULL) {
        return;
    }

    kalman_init(&kf, 0.5f, 0.1f);//初始化kalman滤波器 0.5f和0.1f是滤波器的噪声系数

    user_flushParams(NULL);//清空或重置用户参数

    Install_Ekf_Track(track);//安装扩展卡尔曼滤波器
    // track_ready = true;

    track->radar_params = &RadarSysParams; // 设置跟踪器的雷达参数指针为全局雷达系统参数。

    /* 雷达参数初始化 */
    track->radar_params->rng_delta = 0.50;    /* 距离精度   */
    track->radar_params->vel_delta = 0.25;    /* 速度精度   */
    track->radar_params->az_delta_deg = 0.50; /* 方位角精度 */

    track->radar_params->trk_capt_delay = 10;   /* 跟踪器捕获延迟*/
    track->radar_params->trk_drop_delay = 10;   /* 跟踪器丢弃延迟*/
    track->radar_params->trk_fov_az_left = -60; /* 左视场角度 */
    track->radar_params->trk_fov_az_right = 60; /* 右视场角度 */
    track->radar_params->trk_fps = 1;           /* 帧率 */
    track->radar_params->trk_nf_thres = 10; /* 雷达近场距离阈值*/

    /* 初始化所有存储点云空间*/
    memset(&track->f_last_time, 0, sizeof(track->f_last_time));
    memset(&track->f_now, 0, sizeof(track->f_now));
    memset(&track->cdi_pkg, 0, sizeof(track->cdi_pkg));
    memset(&track->output_obj, 0, sizeof(track->output_obj));
    memset(&track->output_hdr, 0, sizeof(track->output_hdr));
    memset(track->trk_data, 0, sizeof(track->trk_data));

    track_Params_update(track); //更新跟踪器参数

    /* 交通流统计参数初始化 */
    // had.headers.header1 = Init_LinkListV();
    // had.headers.header2 = Init_LinkListV();
    // had.headers.header3 = Init_LinkListV();
    // had.headers.header4 = Init_LinkListV();

    // had.headergap.header1 = Init_LinkListV();
    // had.headergap.header2 = Init_LinkListV();
    // had.headergap.header3 = Init_LinkListV();
    // had.headergap.header4 = Init_LinkListV();

    /* 交通流统计参数初始化 */
    for (int i = 0; i < adjust_Params.MaxLaneNum; i++) {
        had.headers[i] = Init_LinkListV();
        had.headergap[i] = Init_LinkListV();
    }

    enableTrafficDetrctionDef.sectionAverageSpeedCount = 0;
    enableTrafficDetrctionDef.laneDividedAverageSpeedCount = 0;
    enableTrafficDetrctionDef.laneDividedTimeOccupancyCount = 0;
    enableTrafficDetrctionDef.laneDividedAverageHeadwayCount = 0;
    enableTrafficDetrctionDef.laneDividedAverageHeadwayGapCount = 0;
}

/*******************
 * 该函数用于将EKF跟踪算法安装到跟踪器中的函数。它设置了跟踪器结构体中的处理函数指针，
 * 以便在跟踪器的生命周期中调用EKF跟踪算法的相应函数。这些函数包括初始化、预处理、处理、后处理、停止、设置帧间隔、更新目标信息、
 * 更新头信息、判断是否第一次运行以及目标分类和参数更新等。
 * *******************/
void Install_Ekf_Track(track_t *track)
{
    if (track == NULL) {
        spdlog::error("Install_Ekf_Track called with null track!");
        return;
    }

    track_trk_pkg_t *trk_pkg = &trk_internal;

    // 设置跟踪算法包的输入、输出目标和输出头指针为跟踪器结构体中的相应成员。
    trk_pkg->raw_input = &track->cdi_pkg; 
    trk_pkg->output_obj = &track->output_obj;
    trk_pkg->output_hdr = &track->output_hdr;

    track->trk_data = trk_pkg; //将跟踪算法包指针赋值给跟踪器的trk_data成员，以便在跟踪算法中使用。
    track->trk_init = func_track_init;// 算法初始化处理程序
    track->trk_pre = func_track_pre;// 算法预运行处理程序
    track->trk_run = func_track_run; // 算法运行处理程序
    track->trk_post = func_track_post;// 算法后运行处理程序
    track->trk_stop = func_track_stop;  // 算法停止处理程序
    track->trk_set_frame_int = func_set_frame_int;// 设置算法运行的帧间隔
    track->trk_update_obj_info = func_track_obj_info_update;// 更新输出目标信息的处理程序
    track->trk_update_header = func_track_header_update;  // 更新输出头信息的处理程序
    track->trk_has_run = func_has_run; // 判断是否是第一次运行算法
    track->trk_Classification = func_track_Object_Classification;// 跟踪目标分类
    track->trk_paramsUpdate = func_track_ParamsUpdate;  // 跟踪参数更新
}

/****************************
 * 跟踪器预启动函数，用于在跟踪开始前进行初始化
 * 确保在跟踪开始之前，跟踪器已经正确初始化。函数首先检查跟踪器指针和雷达参数指针是否有效，
 * 然后提取雷达系统参数，并调用跟踪器的初始化函数（如果存在）。 
 * ****************************/
void track_pre_start(track_t *track)
{
    if ((track == NULL) || (track->radar_params == NULL)) {
        spdlog::error("track_pre_start() called with invalid track pointer");
        return;
    }

    void *sys_params = track->radar_params;

    if (track->trk_init) {
        track->trk_init(track->trk_data, sys_params);// 检查跟踪器是否具有初始化函数，如果有，则调用该函数并传入跟踪器数据和系统参数。
    }
}



/****************************
 * 跟踪器参数更新 
 * 该函数负责更新跟踪器的参数，确保它们与调整参数保持一致。函数首先检查跟踪器指针和相关数据指针的有效性，
 * 然后更新跟踪类型阈值和帧间隔。如果跟踪器具备参数更新函数，该函数也会被调用，以允许跟踪算法根据当前的系统参数和调整参数执行更详细的参数更新。
 * **************************/
void track_Params_update(track_t *track)
{
    if ((track == NULL) || (track->trk_data == NULL) || (track->radar_params == NULL)) {
        spdlog::error("Track_params_update: invlaid pointer in call");
        return; // 检查传入的跟踪器指针、跟踪器数据指针和雷达参数指针是否为空，如果任何一个为空，则记录错误并返回。
    }

    track_trk_pkg_t *trk_pkg = (track_trk_pkg_t *)track->trk_data;// 将跟踪器数据转换为track_trk_pkg_t类型，以便访问特定于跟踪算法的数据。
    radar_sys_params_t *sys_params = (radar_sys_params_t *)track->radar_params;// 将雷达参数转换为radar_sys_params_t类型，以便访问雷达系统参数。

    // 如果跟踪类型阈值（预测）与调整参数中的值不匹配，则更新之。
    if (trk_pkg->track_type_thres_pre != adjust_Params.AdjustParams_EKFUpdatePreTypeThres) {
        trk_pkg->track_type_thres_pre = adjust_Params.AdjustParams_EKFUpdatePreTypeThres;
    }
     // 如果跟踪类型阈值（验证）与调整参数中的值不匹配，则更新之。
    if (trk_pkg->track_type_thres_val != adjust_Params.AdjustParams_EKFUpdateValTypeThres) {
        trk_pkg->track_type_thres_val = adjust_Params.AdjustParams_EKFUpdateValTypeThres;
    }
     // 如果帧间隔与调整参数中的值不匹配，则更新之。
    if (trk_pkg->frame_int != adjust_Params.AdjustParams_EKFFilterTimeInt) {
        trk_pkg->frame_int = adjust_Params.AdjustParams_EKFFilterTimeInt;
    }
    // 如果跟踪器具有参数更新函数，则调用该函数并传入跟踪器数据和系统参数。
    if (track->trk_paramsUpdate) {
        track->trk_paramsUpdate(track->trk_data, sys_params);
    }
}
/* 停止跟踪函数 */
void track_stop(track_t *track)
{
    if (track == NULL) {
        spdlog::error("track_stop(): passed a NULL track handle");
        return;
    }

    if (track->trk_stop) {
        track->trk_stop(track->trk_data, NULL);
    }
}

/// @brief                  叠加多帧数据，并重置原始点云
/// @param raw_input     ： 输入预处理后的点云数据
/// @param raw_timestamp ： 该帧点云数据的时间戳

static bool init_first = false;
static frame_buff_t cache_frame[FRAME_CACHE_MAX];//缓存帧


/***************************
 * 该函数用于将多帧点云数据叠加到原始点云数据中。它首先检查输入的点云数据包是否为空，然后检查是否是第一次初始化，
 * 如果是，则清空缓存帧。函数接着将新的帧数据加入到缓存帧的开始位置，并将旧的帧数据向后移动。然后，函数遍历缓存帧，
 * 将每个帧的点云数据叠加到原始点云数据中，并根据时间差更新点云数据的距离。最后，函数更新原始点云数据包中的点云数量。
 * **************************/
static void OverLopMultiFramePointEx(track_cdi_pkg_t *raw_input, track_float_t raw_timestamp)
{
    if (raw_input == NULL) {
        return;
    }

    if (!init_first) {
        init_first = true;
        memset(cache_frame, 0, sizeof(cache_frame));
    }// 如果这是第一次初始化，设置标志为真，并清空缓存帧

    int i = 0, j = 0;
    for (i = FRAME_CACHE_MAX - 1; i > 0; i--) {
        memcpy(&cache_frame[i], &cache_frame[i - 1], sizeof(frame_buff_t));// 将缓存帧向后移动一个位置，为新的帧数据腾出空间。
    }
    // 创建一个新的帧结构体，设置标签和时间戳，并复制当前帧的点云数据。
    frame_buff_t frame;
    frame.tag = 0x55AA55BB;
    frame.fts = raw_timestamp;
    memcpy(&(frame.data), raw_input, sizeof(track_cdi_pkg_t));
    memcpy(&(cache_frame[0]), &frame, sizeof(frame_buff_t));

    uint32_t total_num = 0;
     // 遍历缓存帧，叠加数据。
    for (i = 0; i < FRAME_CACHE_MAX; ++i) {
        if (cache_frame[i].tag != 0x55AA55BB) {// 如果帧标签不匹配，说明缓存帧已处理完，退出循环。
            break;
        }

        track_float_t timeInt = raw_timestamp - cache_frame[i].fts;//计算时间差
        // 复制点云数据，并根据时间差更新距离。
        for (j = 0; j < cache_frame[i].data.cdi_number; ++j) {
            memcpy(&(raw_input->cdi[total_num]), &(cache_frame[i].data.cdi[j]), sizeof(raw_input->cdi[total_num]));
            raw_input->cdi[total_num].raw_z.rng += timeInt * raw_input->cdi[total_num].raw_z.vel;
            total_num += 1;
            // 如果总数达到点云数据的最大数量，回绕到起点。
            if (total_num >= TRACK_NUM_CDI) {
                total_num--;
            }
        }
    }
     // 更新原始点云数据包中的点云数量。
    raw_input->cdi_number = total_num;
    raw_input->raw_number = total_num;
}

/// @brief OverLopMultiFramePoint 叠加多帧数据，并重置原始点云
/// @param raw_input  ： 输入预处理后的点云数据
/// @param raw_timestamp ： 该帧点云数据的时间戳

/**************************
 * 该函数用于将多帧点云数据叠加到原始点云数据中。它首先检查输入的点云数据包是否为空，然后检查是否是第一次初始化，
 * 如果是，则初始化数据缓冲区。函数接着将新的帧数据加入到数据缓冲区的开始位置，并将旧的帧数据向后移动。
 * 然后，函数遍历数据缓冲区，将每个帧的点云数据叠加到原始点云数据中，并根据时间差更新点云数据的距离。
 * 最后，函数更新原始点云数据包中的点云数量。
 * ****************************/
static void OverLopMultiFramePoint(track_cdi_pkg_t *raw_input, track_float_t raw_timestamp)
{
#define FRAME_BUF_NUM 3 // 缓存数据帧数
    struct FrameBuf_t {
        track_float_t   timestamp;
        track_cdi_pkg_t data;
    };

    // 存储最近三帧数据容器
    int i = 0, j = 0;
    static int inited_flag = false;
    static struct FrameBuf_t *dataBuffer[FRAME_BUF_NUM];

    if (!inited_flag) {
        for (i = 0; i < FRAME_BUF_NUM; ++i) {
            dataBuffer[i] = NULL;
        }

        inited_flag = true;
    }

    // 处理dataBuffer中的最后一帧数据
    if (dataBuffer[FRAME_BUF_NUM - 1] != NULL) {   
        free(dataBuffer[FRAME_BUF_NUM - 1]);
        dataBuffer[FRAME_BUF_NUM - 1] = NULL;
    }

    // 更新最近传入数据，从后面向前遍历处理
    for (i = FRAME_BUF_NUM - 2; i >= 0; --i) {
        // 若后面一个databuffer为空，则将当前的指针向后移动
        dataBuffer[i + 1] = dataBuffer[i];
        dataBuffer[i] = NULL;
    }

    // 拷贝传入新一帧数据到新的FrameBuf_t结构体，并分配内存。
    struct FrameBuf_t *ptr = (struct FrameBuf_t *)malloc(sizeof(struct FrameBuf_t));
    ptr->timestamp = raw_timestamp;
    memcpy(&(ptr->data), raw_input, sizeof(track_cdi_pkg_t));

    // 将最新数据插入到头部
    dataBuffer[0] = ptr;

    // 处理成功，将新一帧数据存放头部

    memset(raw_input, 0x0, sizeof(raw_input));// 清空原始点云数据包以便存放处理后的数据。
    uint32_t total_num = 0;// 用于记录处理后的点云数据总数。

    for (i = 0; i < FRAME_BUF_NUM; ++i) {
        if (dataBuffer[i] == NULL) {
            break;
        }

        track_float_t timeInt = raw_timestamp - dataBuffer[i]->timestamp;
        for (j = 0; j < dataBuffer[i]->data.cdi_number; ++j) {
            memcpy(&(raw_input->cdi[total_num]), &(dataBuffer[i]->data.cdi[j]), sizeof(raw_input->cdi[total_num]));
            raw_input->cdi[total_num].raw_z.rng += timeInt * raw_input->cdi[total_num].raw_z.vel;
            ++total_num;
        }
    }

    raw_input->cdi_number = total_num;
    raw_input->raw_number = total_num;
}

#if TEST_MODE

#else

/***************************
 * 该函数负责读取雷达点云数据，并进行一系列的预处理步骤，包括滤波、坐标转换和排序，以准备数据输入到跟踪算法中。
 * 函数首先检查输入参数的有效性，然后更新跟踪器的时间戳，遍历点云数据进行滤波和坐标转换，排除不符合条件的点，
 * 并最终更新跟踪器的点云包中的数据。
 * @param track 指向跟踪器结构体的指针。
 * @param RadarPointCloud 指向雷达点云数据的指针。
 * @param PointNumber 点云数据的数量。
 * **************************/
void track_read(track_t *track, RadarClusterInfoARS *RadarPointCloud, int PointNumber)
{
    // 检查传入的跟踪器指针、跟踪器数据指针和雷达点云数据指针是否为空，如果任何一个为空则记录错误并返回。
    if ((track == NULL) || (track->trk_data == NULL) || (RadarPointCloud == NULL)) {
        spdlog::error("track_read error: track or RadarPointCloud is NULL");
        return;
    }

    int i = 0, j = 0;
    int group_num = 1;
    int Point_num = PointNumber;
    // 点云数据的数量，如果超过最大值则限制在最大值。
    if (Point_num > TRACK_NUM_CDI) {
        Point_num = TRACK_NUM_CDI;
    }
    
    // 提取雷达系统参数。
    radar_sys_params_t *sys_params = track->radar_params;

    /* 更新跟踪器的时间戳 */
    track->f_last_time = track->f_now;
    track->f_now = RadarPointCloud[0].Cluster_timestamp;

    /* 测试所用帧号 */
    track_trk_pkg_t *trk_pkg = (track_trk_pkg_t *)track->trk_data;
    trk_pkg->f_numb = track->f_now;

    for (i = 0, j = 0; (i < Point_num) && (j < TRACK_NUM_CDI); i++) {
        // std::printf("object:[%02d], DYNP:[%02X], RCS:[%6.2f], X:[%8.3f], Y:[%8.3f], VX:[%8.3f]\n", i,
        //     RadarPointCloud[i].Cluster_DynProp, RadarPointCloud[i].Cluster_RCS,
        //     RadarPointCloud[i].Cluster_DistLong, RadarPointCloud[i].Cluster_DistLat,
        //     RadarPointCloud[i].Cluster_VrelLong);
        // 如果点云数据在距离、速度和RCS的预设范围内，则进行进一步处理。
        /* 加入点云数据滤波器 */
        if (RadarPointCloud[i].Cluster_DistLong    >= adjust_Params.AdjustParams_DisLongMin
            && RadarPointCloud[i].Cluster_DistLong <= adjust_Params.AdjustParams_DisLongMax
            && RadarPointCloud[i].Cluster_DistLat  >= adjust_Params.AdjustParams_DisLatMin
            && RadarPointCloud[i].Cluster_DistLat  <= adjust_Params.AdjustParams_DisLatMax
            && RadarPointCloud[i].Cluster_VrelLong >= adjust_Params.AdjustParams_VrelLongMin
            && RadarPointCloud[i].Cluster_VrelLong <= adjust_Params.AdjustParams_VrelLongMax
            && RadarPointCloud[i].Cluster_VrelLat  >= adjust_Params.AdjustParams_VrelLatMin
            && RadarPointCloud[i].Cluster_VrelLat  <= adjust_Params.AdjustParams_VrelLatMax
            && RadarPointCloud[i].Cluster_RCS      >= adjust_Params.AdjustParams_RCSMin
            && RadarPointCloud[i].Cluster_RCS      <= adjust_Params.AdjustParams_RCSMax)
        {
            /**
             * Cluster_DynProp:
             *      0x00   -->   移动的
             *      0x01   -->   静止的
             *      0x02   -->   来向的
             *      0x03   -->   可能静止的
             *      0x04   -->   未知的
             *      0x05   -->   横穿静止的
             *      0x06   -->   横穿的
             *      0x07   -->   停止的
             */

            /* 筛除速度为零的静止点 */  /* 筛除RCS低于X的点 */
            if (RadarPointCloud[i].Cluster_VrelLong         != 0
                && RadarPointCloud[i].Cluster_RCS           > -65
                && RadarPointCloud[i].Cluster_Pdh0          < 0x3
                && (RadarPointCloud[i].Cluster_InvalidState != 0x0A
                && RadarPointCloud[i].Cluster_InvalidState  != 0x0C)
                && (RadarPointCloud[i].Cluster_DynProp      != 0x1
                && RadarPointCloud[i].Cluster_DynProp       != 0x3
                && RadarPointCloud[i].Cluster_DynProp       != 0x4
                && RadarPointCloud[i].Cluster_DynProp       != 0x5
                && RadarPointCloud[i].Cluster_DynProp       != 0x6)
                && (RadarPointCloud[i].Cluster_AmbigState   != 0x1
                && RadarPointCloud[i].Cluster_AmbigState    != 0x2))
            {
                /* 根据雷达安装高度，修改距离 */
                // RadarPointCloud[i].Cluster_DistLong = sqrt(RadarPointCloud[i].Cluster_DistLong * RadarPointCloud[i].Cluster_DistLong - 7 * 7);

                /* 笛卡尔坐标系转换为极坐标系 */
                track_float_t Range_Point = sqrt(RadarPointCloud[i].Cluster_DistLong * RadarPointCloud[i].Cluster_DistLong + RadarPointCloud[i].Cluster_DistLat * RadarPointCloud[i].Cluster_DistLat);
                track_float_t Velocity_Point = RadarPointCloud[i].Cluster_VrelLong;
                track_float_t Azimuth_Point = atan2(RadarPointCloud[i].Cluster_DistLat, RadarPointCloud[i].Cluster_DistLong) * RAD2ANG;
                track_float_t RCS = RadarPointCloud[i].Cluster_RCS;

                /* 筛除方位角超出雷达范围的杂点 */
                if ((Azimuth_Point >= sys_params->trk_fov_az_left) && (Azimuth_Point <= sys_params->trk_fov_az_right)) {
                    /* 将符合条件的数据存储到跟踪器 */
                    track->cdi_pkg.cdi[j].raw_z.rng = Range_Point;
                    track->cdi_pkg.cdi[j].raw_z.vel = Velocity_Point;
                    track->cdi_pkg.cdi[j].raw_z.ang = Azimuth_Point;
                    track->cdi_pkg.cdi[j].raw_z.sig = RCS;
                    track->cdi_pkg.cdi[j].raw_z.noi = 1;

                    j++;
                }
            }
        }
    }

    /* 更新滤除后的候选点的数量 */
    track->cdi_pkg.cdi_number = track->cdi_pkg.raw_number = j;

    // 将多帧的数据进行叠加
    // OverLopMultiFramePoint(&(track->cdi_pkg), track->f_now);
    OverLopMultiFramePointEx(&(track->cdi_pkg), track->f_now);

    /* 将Point按照RCS进行排序 */
    track_SortRCSDiffe(&(track->cdi_pkg));
}
#endif

/**
 * 运行跟踪算法，执行跟踪处理。
 * @param track 指向跟踪器结构体的指针。
 * 该函数是跟踪器的核心函数，它负责调用跟踪算法的不同阶段函数，包括预处理、滤波、后处理、头信息更新和目标分类。
 * 函数首先检查跟踪器指针和跟踪器数据指针的有效性，然后提取雷达系统参数和跟踪算法包。在try块中，
 * 函数依次调用跟踪算法的不同阶段函数，如果在调用过程中发生异常，则在catch块中记录错误信息。
 */
void track_run(track_t *track)
{
    if ((track == NULL) || (track->trk_data == NULL)) {
        spdlog::error("track_run: Bad track handle");
        return;
    }

    void *sys_params = track->radar_params;
    track_trk_pkg_t *trk_pkg = (track_trk_pkg_t *)track->trk_data;

    // if (track->trk_set_frame_int) {
    //     track->trk_set_frame_int(track->trk_data, track->f_now - track->f_last_time);
    // }

    try {
        // 如果存在预处理函数，则调用该函数进行预处理。
        if (track->trk_pre) {
            // track->trk_pre(track->trk_data, sys_params, TrackingObjectData_fp);
            track->trk_pre(track->trk_data, sys_params);
        }

        /* filter run*/
        if (track->trk_run) {
            track->trk_run(track->trk_data, sys_params);
        }

        /* post run */
        if (track->trk_post) {
            track->trk_post(track->trk_data, sys_params);
        }

        /* Out header update run */
        if (track->trk_update_header) {
            track->trk_update_header(track->trk_data, sys_params);
        }

        /* Classification run */
        if (track->trk_Classification) {
            track->trk_Classification(track->trk_data, sys_params);
        }
    } catch (const std::exception &e) {
        // 如果在跟踪过程中发生异常，则记录错误信息。
        spdlog::error("track_run exception: {}", e.what());
    }
}

/**
 * 输出跟踪结果，将跟踪到的目标信息输出到目标列表。
 * @param track 指向跟踪器结构体的指针。
 * @param ObjectList 指向存储跟踪目标信息的数组的指针。
 * @param ObjectNum 指向存储跟踪到的目标数量的整数指针。
 * 该函数函数负责将跟踪器跟踪到的目标信息输出到目标列表中。函数首先检查输入参数的有效性，
 * 然后遍历所有跟踪到的目标，检查每个目标是否在区域兴趣（ROI）内，如果目标有效且在ROI内，
 * 则将其信息记录到目标列表中。这个过程包括记录目标的时间戳、ID、距离、速度、加速度、经纬度坐标、类别和RCS值等。
 * 最后，函数更新输出的目标数量。
 * */
void track_OutObjectList(track_t *track, ARS408RadarObjectInfo *ObjectList, int *ObjectNum)
{
    if ((track == NULL) || (track->trk_data == NULL) || (ObjectList == NULL)) {
        return;
    }

    Point ROILoc[4];// 定义区域（ROI）的四个顶点坐标。
    int isInROI = 0, k;// 标记目标是否在ROI区域内。
    int ObjectOutNum = 0;// 标记输出的目标数量。
    uint32_t i = 0, j = 0, lop;
    long double tmp_longlat[3] = {0};// 存储目标的经纬度坐标。
    Point ObjectLoc = {.x = 0, .y = 0}; // 存储目标的坐标。
    void *sys_params = track->radar_params; // 提取雷达系统参数。
    track_trk_pkg_t *trk_pkg = (track_trk_pkg_t *)track->trk_data;// 提取跟踪算法包。

    // memset(ObjectList, 0, sizeof(ARS408RadarObjectInfo) * TRACK_NUM_TRK);

    for (i = 0, j = 0; i < TRACK_NUM_TRK; i++) {
        /* 提取点信息 */
        // 如果存在目标信息更新函数，则调用该函数更新目标信息。
        if (track->trk_update_obj_info) {
            track->trk_update_obj_info(track->trk_data, sys_params, i);
        }

        /* 判断该点是否在ROI区域内 */
        isInROI = 0;
        ObjectLoc.x = trk_pkg->output_obj->DistLong;
        ObjectLoc.y = trk_pkg->output_obj->DistLat;

        for (k = 0; k < adjust_Params.AdjustParams_ROI.UsingNumbofROI; k++) {
            /* 注意： 雷达坐标系 */
            ROILoc[0].x = adjust_Params.AdjustParams_ROI.PositionofROI[k].x1;
            ROILoc[0].y = adjust_Params.AdjustParams_ROI.PositionofROI[k].y1;

            ROILoc[1].x = adjust_Params.AdjustParams_ROI.PositionofROI[k].x2;
            ROILoc[1].y = adjust_Params.AdjustParams_ROI.PositionofROI[k].y2;

            ROILoc[2].x = adjust_Params.AdjustParams_ROI.PositionofROI[k].x3;
            ROILoc[2].y = adjust_Params.AdjustParams_ROI.PositionofROI[k].y3;

            ROILoc[3].x = adjust_Params.AdjustParams_ROI.PositionofROI[k].x4;
            ROILoc[3].y = adjust_Params.AdjustParams_ROI.PositionofROI[k].y4;

            isInROI = isInROI || IsPointInPolygon(ROILoc, 4, ObjectLoc);// 检查目标是否在当前ROI区域内。
        }

        if (adjust_Params.AdjustParams_ROI.UsingNumbofROI == 0) {
            isInROI = 1;// 如果没有定义ROI，则默认所有目标都在ROI内。
        }

        /* 将符合条件的点输出到Objectlist*/
        if ((trk_pkg->output_obj->output == true) && (trk_pkg->output_obj->track_level != 0) && isInROI) {
            // ObjectList[j].Object_timestamp = trk_pkg->output_hdr->frame_id;
            ObjectList[j].Object_timestamp = track->f_now;
            ObjectList[j].Object_ID = trk_pkg->output_obj->UUID;

            ObjectList[j].Object_DistLong = trk_pkg->output_obj->DistLong;
            ObjectList[j].Object_DistLat = trk_pkg->output_obj->DistLat;

            /* 减少抖动 */
            // ObjectList[j].Object_DistLat = kalman_update(&kf, trk_pkg->output_obj->DistLat);

            ObjectList[j].Object_VrelLong = trk_pkg->output_obj->VelLong;
            ObjectList[j].Object_VrelLat = trk_pkg->output_obj->VelLat;

            ObjectList[j].Object_ArelLong = trk_pkg->output_obj->ArelLong;
            ObjectList[j].Object_ArelLat = trk_pkg->output_obj->ArelLat;

            /* 计算经纬度 */
            memset(&tmp_longlat, 0, sizeof(tmp_longlat));
            if (g_dynamicOrStaticCalibrationMode == 0) {
                track_coordinate2GPS(g_CalbPara, trk_pkg->output_obj->DistLong, trk_pkg->output_obj->DistLat, tmp_longlat);
            } else {
                Cal_latlon(g_StaticCalbPara.origin_latitude, g_StaticCalbPara.origin_longitude, g_StaticCalbPara.angle,
                    trk_pkg->output_obj->DistLong, trk_pkg->output_obj->DistLat, (double *)&(tmp_longlat[0]), (double *)&(tmp_longlat[1]));
            }

            ObjectList[j].Object_Latitude = tmp_longlat[0];
            ObjectList[j].Object_Longitude = tmp_longlat[1];
            ObjectList[j].Object_Altitude = tmp_longlat[2];

            RadarObjectData_Wr[j].Object_Latitude = tmp_longlat[0];
            RadarObjectData_Wr[j].Object_Longitude = tmp_longlat[1];
            RadarObjectData_Wr[j].Object_Altitude = tmp_longlat[2];
            
            // 记录目标的类别和RCS值。
            ObjectList[j].Object_Class = trk_pkg->output_obj->Class;
            ObjectList[j].Object_RCS = trk_pkg->output_obj->RCS;

// #if defined(CONFIG_ZZTK_PROTOCOL)
            // 根据中资泰克测试协议要求：
            // 方位角进行特殊处理，结合标定情况，生成基于正北的方位角
            long double angle_tmp = (360 - ((g_dynamicOrStaticCalibrationMode == 0 ? g_CalbPara.angle : g_StaticCalbPara.angle) + trk_pkg->output_obj->OrienAngle));

            int tmpAngle = (int)((360 - (trk_pkg->output_obj->OrienAngle < 360 ? angle_tmp : (angle_tmp - 360))) * 100);
            ObjectList[j].Object_OrientationAngel = (tmpAngle % 36000) / 100.0;  // 根据中资泰克协议，正北为零，顺时针
// #else
//             ObjectList[j].Object_OrientationAngel = trk_pkg->output_obj->OrienAngle;
// #endif
            ObjectList[j].Object_Width = trk_pkg->output_obj->Width;
            ObjectList[j].Object_Length = trk_pkg->output_obj->Length;
            j += 1;// 增加输出的目标数量。
        }
    }
    
    // 更新输出的目标数量
    ObjectOutNum = j;
    if (ObjectNum != NULL) {
        *ObjectNum = ObjectOutNum;
    }
}

/* 事件检测功能 */
void track_TrafficEventDetection()
{

}

/**
 * 根据RCS值对点云数据进行排序，使用选择排序算法。
 * @param raw_input 指向包含点云数据的结构体的指针。
 * 该函数根据雷达点云数据中的RCS（雷达散射截面）值对数据进行排序。
 * 函数使用选择排序算法，遍历点云数据，找到每一轮中最大的RCS值，并将其放到当前排序位置。
 * */
void track_SortRCS(track_cdi_pkg_t *raw_input)
{
    // 检查输入的点云数据包是否包含有效的点云数据。
    if (raw_input->raw_number > 0) {
        track_cdi_t cdi = {0};
        int i = 0, j = 0, k = 0;

        // 使用选择排序算法对点云数据按RCS值进行排序。
        for (i = 0; i < (raw_input->raw_number - 1); i++) {
            k = i;
            for (j = i + 1; j < (raw_input->raw_number); j++) {
                if (raw_input->cdi[j].raw_z.sig > raw_input->cdi[k].raw_z.sig) {
                    k = j;
                }
            }
            //交换 cdi临时变量
            cdi.raw_z = raw_input->cdi[k].raw_z;
            raw_input->cdi[k].raw_z = raw_input->cdi[i].raw_z;
            raw_input->cdi[i].raw_z = cdi.raw_z;
        }
    }
}

/* 按照RCS差进行排序
* 根据雷达点云数据中的RCS值与其基准值的差异对数据进行排序。函数首先计算每个点的RCS差异值，
* 然后使用选择排序算法对这些差异值进行排序，并相应地调整点云数据的顺序。
* */
void track_SortRCSDiffe(track_cdi_pkg_t *raw_input)
{
    if (raw_input->raw_number > 0) {
        int i, j, k;
        track_cdi_t cdi = {0};// 临时变量，用于交换数据。
        track_float_t RCStemp = 0;// 临时变量，用于交换RCS差异值。
        track_float_t RCSdiff[TRACK_NUM_CDI] = {0};// 存储每个点云数据的RCS差异值。

        // 如果点云数据的数量超过预定义的最大值，则限制它。
        if (raw_input->raw_number > TRACK_NUM_CDI) {
            raw_input->raw_number = TRACK_NUM_CDI;
        }

        /* 计算差值 */
        for (i = 0; i < (raw_input->raw_number - 1); i++) {
            RCSdiff[i] = raw_input->cdi[i].raw_z.sig - RCS_func(raw_input->cdi[i].raw_z.rng);
        }

        // 使用选择排序算法对点云数据按RCS差异值进行排序。
        for (i = 0; i < (raw_input->raw_number - 1); i++) {
            k = i;
            for (j = i + 1; j < raw_input->raw_number; j++) {
                if (RCSdiff[j] > RCSdiff[k]) {
                    k = j;
                }
            }

            cdi.raw_z = raw_input->cdi[k].raw_z;
            raw_input->cdi[k].raw_z = raw_input->cdi[i].raw_z;
            raw_input->cdi[i].raw_z = cdi.raw_z;
            
            // 同时交换对应的RCS差异值
            RCStemp = RCSdiff[k];
            RCSdiff[k] = RCSdiff[i];
            RCSdiff[i] = RCStemp;
        }
    }
}

/* 根据矩阵计算目标经纬度 */
void track_coordinate2GPS(CalbParams Calibration_Params, double range_X, double range_Y, long double *tmp)
{
    tmp[0] = g_CalbPara.matrix_1 * range_X + g_CalbPara.matrix_3 * range_Y + g_CalbPara.matrix_5;
    tmp[1] = g_CalbPara.matrix_2 * range_X + g_CalbPara.matrix_4 * range_Y + g_CalbPara.matrix_6;
    tmp[2] = g_CalbPara.p00 + g_CalbPara.p10 * tmp[0] + g_CalbPara.p01 * tmp[1] + g_CalbPara.p20 * tmp[0] * tmp[0] + g_CalbPara.p11 * tmp[0] * tmp[1] + g_CalbPara.p02 * tmp[1] * tmp[1];
}

/**
 * @brief 静态标定坐标转换函数
 * 雷达坐标系中的目标坐标转换为地理坐标系中的经纬度坐标
 * @param[in] lat1 : 雷达基准维度
 * @param[in] lon1 : 雷达基准经度
 * @param[in] angle_r : 雷达基准正北偏转角
 * @param[in] D_x : 输入目标雷达坐标系坐标 纵轴
 * @param[in] D_y : 输入目标雷达坐标系坐标 横轴
 * @param[out] lat2_out : 输出目标维度信息
 * @param[out] lon2_out : 输出目标经度信息
 * @author : jinhang
 * @date : 20240530
 */
void Cal_latlon(double lat1, double lon1, double angle_r, double D_x, double D_y, double *lat2_out, double *lon2_out)
{
    double S = sqrt((D_x * D_x) + (D_y * D_y));                 // 目标与雷达原点的径向距离
    double angle = (angle_r - (atan(D_y / D_x) * 180 / M_PI));  // 雷达原点与雷达目标连线的正北偏转角

    // MATLAB代码直译
    // 将经纬度转换为弧度。
    lat1 = lat1 * M_PI / 180.0;
    lon1 = lon1 * M_PI / 180.0;
    
    // 地球椭球体的参数。
    double f = 1.0 / 298.257223563;
    double a = 6378137.0;
    double b = 6356752.314245;
    
    // 将角度转换为弧度
    double angle1 = angle * M_PI / 180.0;

    double sinangle1 = sin(angle1);
    double cosangle1 = cos(angle1);
    double tanU1 = (1 - f) * tan(lat1);
    double cosU1 = 1 / sqrt((1 + tanU1 * tanU1));
    double sinU1 = tanU1 * cosU1;
    double delta1 = atan2(tanU1, cosangle1);
    double sinangle = cosU1 * sinangle1;
    double cosSqangle = 1 - sinangle * sinangle;
    double uSq = cosSqangle * (a * a - b * b) / (b * b);
    double A = 1 + uSq / 16384.0 * (4096.0 + uSq * (-768.0 + uSq * (320.0 - 175.0 * uSq)));
    double B = uSq / 1024.0 * (256.0 + uSq * (-128.0 + uSq * (74.0 - 47.0 * uSq)));

    double delta = S / (b * A);
    double sindelta = 0;
    double cosdelta = 0;
    double cos2deltam = 0;
    double delta_ = 0;

    // 设置迭代限制
    int iterationLimit = 100;
    while ((fabs(delta - delta_) > 1e-12) && (iterationLimit > 0)) {
        cos2deltam = cos(2.0 * delta1 + delta);
        sindelta = sin(delta);
        cosdelta = cos(delta);
        
        
        double delta_delta = B * sindelta * (cos2deltam + B / 4.0 * (cosdelta * (-1.0 + 2.0 * cos2deltam * cos2deltam) - B / 6.0 * cos2deltam * (-3.0 + 4.0 * sindelta * sindelta) * (-3.0 + 4.0 * cos2deltam * cos2deltam)));
        // 更新delta和迭代限制。
        delta_ = delta;
        delta = S / (b * A) + delta_delta;
    }

    double x = sinU1 * sindelta - cosU1 * cosdelta * cosangle1;
    double lat2 = atan2(sinU1 * cosdelta + cosU1 * sindelta * cosangle1, (1.0 - f) * sqrt(sinangle * sinangle + x * x));
    double lamda = atan2(sindelta * sinangle1, cosU1 * cosdelta - sinU1 * sindelta * cosangle1);
    double C = f / 16.0 * cosSqangle * (4 + f * (4.0 - 3.0 * cosSqangle));
    double L = lamda - (1.0 - C) * f * sinangle * (delta + C * sindelta * (cos2deltam + C * cosdelta * (-1.0 + 2.0 * cos2deltam * cos2deltam)));
    double lon2 = lon1 + L;
    double angle2 = atan2(sinangle, -x) * 180.0 / M_PI;
     // 将结果转换为度数。
    lat2 = lat2 * 180.0 / M_PI;
    lon2 = lon2 * 180.0 / M_PI;

    (*lat2_out) = lat2;
    (*lon2_out) = lon2;
}

/*添加时间：2024-07-24*/
/*----------输出延时补偿----------------*/
void track_listInit(Track_ListObj **pList)
{
    Track_ListObj *list;
    list = (Track_ListObj *)malloc(sizeof(Track_ListObj));
    if (list == NULL) {
        printf("Track_ListObj malloc failed\n");
        return;
    }

    list->count = 0;
    list->begin = NULL;
    list->end = NULL;
    list->timeList = NULL;
    *pList = list;
}

int32_t track_is_ListEmpty(Track_ListObj *list)
{
    if (list->count == 0) {
        return 1;
    }

    return 0;
}

int32_t track_listEnQueue(Track_ListObj *list, Track_ListElem *elem)
{
    if (elem == NULL) {
        return 0;
    }

    if (list->begin == NULL) {
        elem->prev = NULL;
        elem->next = NULL;

        list->begin = elem;
        list->end = elem;
    } else {
        list->end->next = elem;
        elem->next = NULL;
        elem->prev = list->end;

        list->end = elem;
    }

    list->count++;
    return 1;
}

int32_t track_listDeQueue(Track_ListObj *list, int32_t Elem_id)
{
    if (list->begin == NULL) {
        return 0;
    }

    Track_ListElem *elem = track_is_Exist(list, Elem_id);

    if (elem != NULL) {
        if (elem->prev == NULL && elem->next == NULL) {
            list->begin = NULL;
            list->end = NULL;
        } else if (elem->prev == NULL && elem->next != NULL) {
            list->begin = elem->next;
            elem->next->prev = NULL;
        } else if (elem->prev != NULL && elem->next == NULL) {
            list->end = elem->prev;
            elem->prev->next = NULL;
        } else {
            elem->prev->next = elem->next;
            elem->next->prev = elem->prev;
        }

        free(elem);
        list->count--;
        return 1;
    } else {
        return 0;
    }
}

Track_ListElem *track_is_Exist(Track_ListObj *list, int32_t Elem_id)
{
    Track_ListElem *tmp = list->begin;

    while (tmp != NULL) {
        if (tmp->Eid == Elem_id) {
            return tmp;
        }

        tmp = tmp->next;
    }
    return NULL;
}

Track_ListElem *track_listGetFirst(Track_ListObj *list)
{
    return list->begin;
}

uint32_t track_listGetCount(Track_ListObj *list)
{
    return list->count;
}

void track_DelayCompensation(Track_ListObj *list, ARS408RadarObjectInfo *out_RadarObjectData, int *out_ObjectNumber)
{
    if (track_is_ListEmpty(list)) {  // 首次加入目标信息，全都进行帧补偿
        /*-------------------------------2024.10.15-----------------------------------*/
        //初始化时间保存链表
        if (list->timeList == NULL) {
            Time_ListInit(&(list->timeList));
        } else {
            //已经初始化过，只是长时间没目标数据，Track_ListObj已经空了
            //将timeList存的时间清空，再重新存
            Time_ListClear(list->timeList);
        }

        /*----------------------------------------------------------------------------*/
        for (int i = 0; i < *out_ObjectNumber; i++) {
            Track_ListElem *elem = NULL;
            elem = (Track_ListElem *)malloc(sizeof(Track_ListElem));
            elem->Eid = out_RadarObjectData[i].Object_ID;

            elem->conpensationList = NULL;

            ConpensationListInit(&(elem->conpensationList));

            for (int j = 0; j < DELAY_FRAME + 1; j++) {
                ARS408RadarObjectInfoElem *tmp = (ARS408RadarObjectInfoElem *)malloc(sizeof(ARS408RadarObjectInfoElem));
                if (tmp == NULL) {
                    return;
                }

                tmp->data = out_RadarObjectData[i];

                tmp->data.Object_DistLong = tmp->data.Object_DistLong - (DELAY_FRAME - j) * 0.075 * tmp->data.Object_VrelLong;
                tmp->data.Object_DistLat = tmp->data.Object_DistLat - (DELAY_FRAME - j) * 0.075 * tmp->data.Object_VrelLat;
                tmp->data.Object_timestamp -= (DELAY_FRAME - j) * 75;

                ConpensationListEnQueue(elem->conpensationList, tmp);

                //首次加入目标时就存下补偿的时间
                if (i == *out_ObjectNumber - 1) {
                    Time_ListElem* ttmp = (Time_ListElem*)malloc(sizeof(Time_ListElem));
                    ttmp->timeStamp = tmp->data.Object_timestamp;
                    Time_ListEnQueue(list->timeList, ttmp);
                }
            }

            elem->flag = 1;
            track_listEnQueue(list, elem);
        }
    } else {
        Time_ListElem* ttmp = (Time_ListElem*)malloc(sizeof(Time_ListElem));
        if (*out_ObjectNumber == 0) {
            ttmp->timeStamp = list->timeList->end->timeStamp + 0.075;   //这个条件一般用不上
        } else {
            ttmp->timeStamp = out_RadarObjectData[0].Object_timestamp;
        }
        Time_ListEnQueue(list->timeList, ttmp);

        for (int i = 0; i < *out_ObjectNumber; i++) {
            Track_ListElem *elem = track_is_Exist(list, out_RadarObjectData[i].Object_ID);
            if (elem == NULL) {
                elem = (Track_ListElem *)malloc(sizeof(Track_ListElem));
                elem->Eid = out_RadarObjectData[i].Object_ID;

                elem->conpensationList = NULL;
                ConpensationListInit(&(elem->conpensationList));

                Time_ListElem* tle = Time_ListSearch(list->timeList);
                for (int j = 0; j < DELAY_FRAME + 1; j++) {
                    ARS408RadarObjectInfoElem *tmp = (ARS408RadarObjectInfoElem *)malloc(sizeof(ARS408RadarObjectInfoElem));
                    if (tmp == NULL) {
                        return;
                    }
                    tmp->data = out_RadarObjectData[i];
                    double nowTime = out_RadarObjectData[i].Object_timestamp;
                    double conpenTime = tle->timeStamp;
                    float deltaT = fabsf(nowTime - conpenTime) / 1000.0;
                    tmp->data.Object_DistLong = tmp->data.Object_DistLong - deltaT * tmp->data.Object_VrelLong;
                    
                    tmp->data.Object_DistLat = tmp->data.Object_DistLat - deltaT * tmp->data.Object_VrelLat;
                    tmp->data.Object_timestamp = conpenTime;

                    ConpensationListEnQueue(elem->conpensationList, tmp);
                    tle = tle->next;
                }

                elem->flag = 1;
                
                track_listEnQueue(list, elem);
            } else {
                // elem->radarObjectData[elem->length++] = out_RadarObjectData[i];
                ARS408RadarObjectInfoElem *c_elem = NULL;
                c_elem = (ARS408RadarObjectInfoElem *)malloc(sizeof(ARS408RadarObjectInfoElem));
                c_elem->data = out_RadarObjectData[i];
                ConpensationListEnQueue(elem->conpensationList, c_elem);
            }
        }
    }

    // 取数据
    int k = 0;
    int count = 0;
    Track_ListElem *tmp = list->begin;
    int32_t delete_id_array[250] = {0};

    while (tmp != NULL) {
        if (tmp->conpensationList->length > 0) {
            out_RadarObjectData[count++] = ConpensationListDeQueue(tmp->conpensationList);
            if (tmp->conpensationList->length == 0) {
                delete_id_array[k++] = tmp->Eid;
            }
        }
        tmp = tmp->next;
    }
    *out_ObjectNumber = count;

    if (list->timeList->count > 40) {
        Time_ListDeQueue(list->timeList);
    }
    // 删除航迹取消的目标信息
    for (int j = 0; j < k; j++) {
        track_listDeQueue(list, delete_id_array[j]);
    }
}

void ConpensationListInit(ConpensationList **pc_list)
{
    ConpensationList *c_list = (ConpensationList *)malloc(sizeof(ConpensationList));
    if (c_list != NULL) {
        c_list->length = 0;
        c_list->begin = NULL;
        c_list->end = NULL;
        *pc_list = c_list;
    }
}

int32_t ConpensationListIsEmpty(ConpensationList *c_list)
{
    if (c_list->length == 0) {
        return 1;
    }

    return 0;
}

int32_t ConpensationListEnQueue(ConpensationList *c_list, ARS408RadarObjectInfoElem *elem)
{
    if (elem == NULL) {
        return 0;
    }

    if (c_list->begin == NULL) {
        elem->prev = NULL;
        elem->next = NULL;

        c_list->begin = elem;
        c_list->end = elem;
    } else {
        c_list->end->next = elem;
        elem->next = NULL;
        elem->prev = c_list->end;
        c_list->end = elem;
    }

    c_list->length++;
    return 1;
}

ARS408RadarObjectInfo ConpensationListDeQueue(ConpensationList *c_list)
{
    ARS408RadarObjectInfo res;
    ARS408RadarObjectInfoElem *tmp;

    if (c_list->begin == NULL) {
        return res;
    }

    tmp = c_list->begin;
    res = tmp->data;

    if (c_list->length == 1) {
        c_list->begin = NULL;
        c_list->end = NULL;
    } else {
        c_list->begin = tmp->next;
    }

    free(tmp);
    c_list->length--;
    return res;
}

void Time_ListInit(Time_List **ptList) {
    Time_List *list = (Time_List*)malloc(sizeof(Time_List));
    if (list == NULL) return;

    list->count = 0;
    list->begin = NULL;
    list->end = NULL;
    *ptList = list;
}

int32_t Time_ListIsEmpty(Time_List* list) {
    if (list->count == 0) return 1;
    return 0;
}

int32_t Time_ListEnQueue(Time_List* list, Time_ListElem* elem) {
    if (elem == NULL) {
        return 0;
    }

    if (list->begin == NULL) {
        elem->prev = NULL;
        elem->next = NULL;

        list->begin = elem;
        list->end = elem;
    } else {
        list->end->next = elem;
        elem->next = NULL;
        elem->prev = list->end;
        list->end = elem;
    }

    list->count++;
    return 1;
}

void Time_ListDeQueue(Time_List* list) {
    if (list->begin == NULL) return;

    Time_ListElem* tmp;
    tmp = list->begin;

    if (list->count == 1) {
        list->begin = NULL;
        list->end = NULL;
    } else {
        list->begin = tmp->next;
    }

    free(tmp);
    list->count--;
}

void Time_ListClear(Time_List* list) {
    while (!Time_ListIsEmpty(list)) {
        Time_ListDeQueue(list);
    }
}

Time_ListElem* Time_ListSearch(Time_List* list) {
    Time_ListElem* tmp = list->end;
    int count = DELAY_FRAME;
    while (count) {
        tmp = tmp->prev;
        count--;
    }

    return tmp;
}