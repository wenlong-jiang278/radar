#ifndef __FRAME_H__
#define __FRAME_H__

#include <stdint.h>

#include <mutex>
#include <deque>
#include <tuple>
#include <chrono>
#include <condition_variable>

#include "any.hpp"

/* 数据帧类型 */
typedef enum {
    TRAJECTORY_DATA = 0x01,             /* 轨迹数据 */
    TRAFFIC_FLOW    = 0x02,             /* 交通量 */
    TRAFFIC_EVENT   = 0x03,             /* 交通事件 */
    RADAR_CLUSTER   = 0x04,             /* 雷达点云 */
    HEART_BEAT      = 0xFF,             /* 心跳 */
} frame_type_e;

/* 事件类型 */
typedef enum {
    OVERSPEED       = 0x01,             /* 超速 */
    RETROGRADE      = 0x02,             /* 逆行 */
    LANE_CHANGE     = 0x03,             /* 变道 */
    ILLEGAL_PARKING = 0x04,             /* 停车 */
    CONGESTION      = 0x05,             /* 拥堵 */
} event_type_e;

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
} timestamp_t;
#pragma pack(pop)

/* 轨迹数据结构 */
#pragma pack(push, 1)
typedef struct {
    uint32_t detect_sec;                /* 检测时间戳。目标检测时间，单位：秒 */
    uint16_t detect_msec;               /* 检测时间。目标检测时间，单位：毫秒(0-999) */
    uint16_t object_id;                 /* 目标ID。雷达识别编号，取值0x0000 - 0xFFFF */
    uint32_t event_type;                /* 事件类型 */
    uint8_t  object_lane;               /* 目标所属车道。最左侧超车道为0x01，依次向右递增。目标不在车道内0xFF，无效值0x00 */
    uint8_t  object_type;               /* 目标类型。厂商自行定义 */
    uint8_t  direction;                 /* 行进方向。0x01来向，0x02去向，0x00无效 */
    uint8_t  object_length;             /* 目标长度。车俩车身长度，单位m。表示范围(0 - 25.5m)。数值0表示无效数据，保留1位小数，分辨率为0.1m */
    uint8_t  object_width;              /* 目标宽度。车俩车身宽度，单位m。表示范围(0 - 25.5m)。数值0表示无效数据，保留1位小数，分辨率为0.1m */
    uint8_t  object_height;             /* 目标高度。车俩车身高度，单位m。表示范围(0 - 25.5m)。数值0表示无效数据，保留1位小数，分辨率为0.1m */
    int16_t  object_rcs;                /* 目标RCS。目标雷达闪射截面强度，单位dbm。表示范围(-128 - 20dbm)。数值0表示无效数据，保留1位小数，分辨率为0.1dbm */
    uint16_t object_yaw_angle;          /* 目标偏航角。目标当前在地理上的行驶方向。取值范围0 - 3599，表示0 - 359.9°。正北为0°，沿着顺时针方向计算。分辨率为0.1° */
    int32_t  object_x_coord;            /* 目标X坐标。目标检测物相对于雷达坐标系的坐标X，单位为m，分辨率为0.01m */
    int32_t  object_y_coord;            /* 目标Y坐标。目标检测物相对于雷达坐标系的坐标Y，单位为m，分辨率为0.1m */
    int16_t  object_x_speed;            /* X轴速度。目标检测物相对于雷达坐标系的X轴速度，单位为m/s，分辨率为0.01m/s */
    int16_t  object_y_speed;            /* Y轴速度。目标检测物相对于雷达坐标系的Y轴速度，单位为m/s，分辨率为0.01m/s */
    uint16_t object_speed;              /* 目标速度。目标检测物的换算速度，单位km/h，分辨率为0.01km/h */
    int16_t  object_accel;              /* 目标加速度。目标检测物运动方向的加速度，单位m/s^2，分辨率为0.1m/s^2。取值范围(-15, 15) */
    int16_t  object_x_accel;            /* X轴加速度。目标检测物相对于雷达坐标系的X轴加速度，单位m/s^2，分辨率为0.1m/s^2。取值范围(-15, 15) */
    int16_t  object_y_accel;            /* Y轴加速度。目标检测物相对于雷达坐标系的Y轴加速度，单位m/s^2，分辨率为0.1m/s^2。取值范围(-15, 15) */
    int32_t  object_coord_lon;          /* 目标经度。目标的坐标点经度定义经度数值，东经为正，单位: 1/10微度的单位，分辨率为0.0000001° */
    int32_t  object_coord_lat;          /* 目标纬度。目标的坐标点纬度定义纬度数值，北纬为正，单位: 1/10微度的单位，分辨率为0.0000001° */
} trajectory_data_t;
#pragma pack(pop)

/* 交通事件结构 */
#pragma pack(push, 1)
typedef struct {
    uint16_t event_id;                  /* 事件ID。雷达识别编号，取值0x0000 - 0xFFFF */
    uint32_t event_type;                /* 事件类型。参考事件类型定义 */
    uint8_t  direction;                 /* 方向。0x01上行，0x02下行 */
    uint8_t  object_type;               /* 目标类型。厂商自行定义 */
    uint8_t  lane_id;                   /* 车道编号。最左侧超车道为0x01，依次向右递增 */
    int32_t  longitude;                 /* 经度。目标的坐标点经度定义经度数值，东经为正。单位: 1/10微度的单位，分辨率为0.0000001° */
    int32_t  latitude;                  /* 纬度。目标的坐标点纬度定义纬度数值，北纬为正。单位: 1/10微度的单位，分辨率为0.0000001° */
} traffic_event_t;
#pragma pack(pop)

/* 交通量统计结构 */
#pragma pack(push, 1)
typedef struct {
    uint16_t period;                    /* 统计周期。0x05表示5分钟发送一次数据*/
    uint16_t small_car_flow;            /* 小车流量。单位：辆，车长小于6米为小车 */
    uint16_t big_car_flow;              /* 大车流量。单位：辆，车长大于等于6米且小于10米为大车 */
    uint16_t super_big_car_flow;        /* 超大车流量。单位：辆，车长大于等于10米为超大车 */
    uint16_t total_flow;                /* 总流量。单位：辆。总流量 = 小车流量 + 大车流量 + 超大车流量。统计周期内通过检测位置所有车辆总数 */
    uint16_t average_speed;             /* 平均车速。分辨率为0.1km/h，统计周期内通过检测断面所有车辆的速度的平均值。统计周期内没有车辆通过，上报0x00 */
    uint16_t head_time;                 /* 车头时距。单位：0.1秒/辆。在同一车道中，连续行驶的两辆车头通过检测断面的时间间隔。统计周期内检测断面车头时距的平均值 */
    uint16_t vir_coil_occ_time;         /* 虚拟线圈时间占有率。单位：0.1。不测，保留，百分比 */
    uint16_t max_queue_length;          /* 最大排队长度。不测，保留，分辨率0.1米。排队长度：从停车线到排队车辆队尾的长度；最大排队长度：统计周期内该车道最大的排队长度 */
    uint16_t average_body_space;        /* 平均车身间距。不测，保留，单位：米 */
    uint16_t lane1_flow;                /* 车道1流量。单位：辆 */
    uint16_t lane2_flow;                /* 车道2流量。单位：辆 */
    uint16_t lane3_flow;                /* 车道3流量。单位：辆 */
    uint16_t lane4_flow;                /* 保留。车道4流量。单位：辆 */
    uint16_t lane5_flow;                /* 保留。车道5流量。单位：辆 */
    uint16_t lane6_flow;                /* 保留。车道6流量。单位：辆 */
    uint16_t lane7_flow;                /* 保留。车道7流量。单位：辆 */
    uint16_t lane8_flow;                /* 保留。车道8流量。单位：辆 */
} traffic_flow_t;
#pragma pack(pop)

/* 雷达点云信息结构 */
#pragma pack(push, 1)
typedef struct {
    uint32_t detect_sec;                /* 检测时间戳。目标检测时间，单位：秒 */
    uint16_t detect_msec;               /* 检测时间。目标检测时间，单位：毫秒(0-999) */
    uint16_t object_id;                 /* 目标ID。雷达识别编号，取值0x0000 - 0xFFFF */
    int32_t  object_x_coord;            /* 目标X坐标。目标检测物相对于雷达坐标系的坐标X，单位为m，分辨率为0.01m */
    int32_t  object_y_coord;            /* 目标Y坐标。目标检测物相对于雷达坐标系的坐标Y，单位为m，分辨率为0.1m */
    int16_t  object_x_speed;            /* X轴速度。目标检测物相对于雷达坐标系的X轴速度，单位为m/s，分辨率为0.01m/s */
    int16_t  object_y_speed;            /* Y轴速度。目标检测物相对于雷达坐标系的Y轴速度，单位为m/s，分辨率为0.01m/s */
    uint16_t cluster_dynprop;           /* 动态属性 */
    int16_t  object_rcs;                /* 目标RCS。目标雷达闪射截面强度，单位dbm。表示范围(-128 - 20dbm)。数值0表示无效数据，保留1位小数，分辨率为0.1dbm */
    uint16_t x_rms;                     /* 纵向距离标准差 */
    uint16_t y_rms;                     /* 横向距离标准差 */
    uint16_t vx_rms;                    /* 纵向相对速度标准差 */
    uint16_t vy_rms;                    /* 横向相对速度标准差 */
    uint16_t pdh0;                      /* 点云的误报概率(即成为集群的概率由多路径或类似原因造成的伪影) */
    uint16_t ambig_state;               /* Doppler(轴向速度)模糊度解的状态 */
    uint16_t invalid_state;             /* 点云有效性状态的状态 */
} radar_cluster_t;
#pragma pack(pop)

/* 数据报文帧 */
#pragma pack(push, 1)
typedef struct {
    uint32_t      head;                 /* 报文头，固定0x55AA55BB */
    uint16_t      length;               /* 报文长度。从length到data的数据总字节数 */
    uint8_t       type;                 /* 数据帧类型 */
    uint8_t       checksum;             /* 校验和。从deviceSn到data，测试时为00 */
    uint8_t       deviceSn[20];         /* 设备编号*/
    timestamp_t   timestamp;            /* 数据时间戳。数据发送的时间(精确到毫秒) */
    // uint8_t       data[0];              /* 目标数据 */
    uint32_t      tail;                 /* 报文尾，固定0x55CC55DD */
} push_frame_t;
#pragma pack(pop)

extern std::deque<std::tuple<uint8_t, std::shared_ptr<linb::any>>> frameQueue;

void frame_push_protocol_init(void);
void frame_push_protocol_exit(void);

#endif
