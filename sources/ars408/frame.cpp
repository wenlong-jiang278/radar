#ifndef _GNU_SOURCE
#define _GNU_SOURCE
#endif

#include <unistd.h>
#include <stdlib.h>
#include <string.h>
#include <pthread.h>
#include <stdbool.h>
#include <arpa/inet.h>

#include <future>
#include <vector>
#include <spdlog/spdlog.h>

#include "frame.h"
#include "any.hpp"
#include "serial.h"
#include "epoll_tcp_server.h"
#include "TrafficFlowDetection.h"

static bool get_serial = false;
static std::mutex mTcpSendMutex;
static char serial_buffer[32] = {0};
static pthread_t pushThreadId, flowThreadId;
static const uint32_t TCP_FRAME_HEAD = 0x55AA55BB;
static const uint32_t TCP_FRAME_TAIL = 0x55CC55DD;

std::deque<std::tuple<uint8_t, std::shared_ptr<linb::any>>> frameQueue;

static void get_device_code(uint8_t *deviceSn)
{
    if (!get_serial) {
        if (get_serial_number(serial_buffer, sizeof(serial_buffer)) == 0) {
            get_serial = true;
        }
    }

    if (get_serial && deviceSn) {
        memcpy(deviceSn, serial_buffer, 20);
    }
}

static uint64_t currentTimestamp(int time_level)
{
    uint64_t current_time_stamp;
    auto timeinfo = std::chrono::system_clock::now().time_since_epoch();

    switch (time_level) {
        case 1:
            current_time_stamp = std::chrono::duration_cast<std::chrono::milliseconds>(timeinfo).count();
            break;

        case 2:
            current_time_stamp = std::chrono::duration_cast<std::chrono::microseconds>(timeinfo).count();
            break;

        case 3:
            current_time_stamp = std::chrono::duration_cast<std::chrono::nanoseconds>(timeinfo).count();
            break;

        case 0:
        default:
            current_time_stamp = std::chrono::duration_cast<std::chrono::seconds>(timeinfo).count();
            break;
    }

    return current_time_stamp;
}

static timestamp_t get_current_timestamp()
{
#if 0
    /* 获取当前系统时间 */
    auto now = std::chrono::system_clock::now();
    auto now_ms = std::chrono::time_point_cast<std::chrono::milliseconds>(now);
    auto epoch = now_ms.time_since_epoch();
    auto value = std::chrono::duration_cast<std::chrono::milliseconds>(epoch);
    std::time_t now_c = std::chrono::system_clock::to_time_t(now);
    std::tm *now_tm = std::localtime(&now_c);

    uint64_t milliseconds = value.count();
#else
    // 获取当前系统时间的毫秒级时间戳
    uint64_t milliseconds = currentTimestamp(1);

    // 将毫秒级时间戳转换为时间结构
    time_t seconds = milliseconds / 1000;
    // std::tm *now_tm = std::localtime(&seconds);
    std::tm *now_tm = std::gmtime(&seconds);        /* 使用gmtime获取UTC时间 */
#endif

    timestamp_t timestamp;
    timestamp.year   = now_tm->tm_year + 1900 - 2000;
    timestamp.month  = now_tm->tm_mon + 1;
    timestamp.day    = now_tm->tm_mday;
    timestamp.hour   = now_tm->tm_hour;
    timestamp.minute = now_tm->tm_min;
    timestamp.second = now_tm->tm_sec;
    timestamp.millisecond = htons(milliseconds % 1000);

    return timestamp;
}

static uint8_t calculate_checksum(std::vector<uint8_t> &data)
{
    uint32_t checksum = 0;
    for (const auto &item : data) {
        checksum += item;
    }

    return checksum & 0xFF;
}

static void SendTcpProtocolPacket(const size_t bytes, push_frame_t *pushFrame, std::vector<uint8_t> data)
{
    std::lock_guard<std::mutex> lock(mTcpSendMutex);

    size_t index = 0;
    std::vector<uint8_t> buff(bytes);

    try {
        memcpy(buff.data() + index, &pushFrame->head, sizeof(pushFrame->head));
        index += sizeof(pushFrame->head);
        memcpy(buff.data() + index, &pushFrame->length, sizeof(pushFrame->length));
        index += sizeof(pushFrame->length);
        memcpy(buff.data() + index, &pushFrame->type, sizeof(pushFrame->type));
        index += sizeof(pushFrame->type);
        memcpy(buff.data() + index, &pushFrame->checksum, sizeof(pushFrame->checksum));
        index += sizeof(pushFrame->checksum);
        memcpy(buff.data() + index, &pushFrame->deviceSn, sizeof(pushFrame->deviceSn));
        index += sizeof(pushFrame->deviceSn);
        memcpy(buff.data() + index, &pushFrame->timestamp, sizeof(pushFrame->timestamp));
        index += sizeof(pushFrame->timestamp);

        if (data.size() > 0) {
            memcpy(buff.data() + index, data.data(), data.size());
            index += data.size();
        }

        memcpy(buff.data() + index, &pushFrame->tail, sizeof(pushFrame->tail));
    } catch (const std::exception &e) {
        return;
    }

    try {
        tcp_server_send(1502, (const void *)buff.data(), buff.size());
    } catch (const std::exception &e) {
        spdlog::error("tcp send data exception:[{0}]", e.what());
    }
}

static void empty_trajectory_push(void)
{
    size_t totalBytes = sizeof(push_frame_t);

    std::vector<uint8_t> temp(totalBytes);
    push_frame_t *pushFrame = reinterpret_cast<push_frame_t *>(temp.data());

    pushFrame->head = htonl(TCP_FRAME_HEAD);
    pushFrame->tail = htonl(TCP_FRAME_TAIL);
    pushFrame->type = HEART_BEAT;
    pushFrame->length = htons(totalBytes - (sizeof(pushFrame->head) + sizeof(pushFrame->tail)));
    get_device_code(pushFrame->deviceSn);
    pushFrame->timestamp = get_current_timestamp();

    size_t checkSumDataSize = sizeof(pushFrame->deviceSn) + sizeof(pushFrame->timestamp);
    std::vector<uint8_t> checkSumData(checkSumDataSize);
    memcpy(checkSumData.data(), pushFrame->deviceSn, sizeof(pushFrame->deviceSn));
    memcpy(checkSumData.data() + sizeof(pushFrame->deviceSn), &pushFrame->timestamp, sizeof(pushFrame->timestamp));
    pushFrame->checksum = calculate_checksum(checkSumData);

    std::vector<uint8_t> data{};
    SendTcpProtocolPacket(totalBytes, pushFrame, data);
}

std::future<void> ProcessTcpPushFrameAsync(std::tuple<uint8_t, std::shared_ptr<linb::any>> frame)
{
    return std::async(std::launch::async, [frame = std::move(frame)]() {
        uint8_t type = std::get<0>(frame);

        if (type == TRAJECTORY_DATA) {      /* 轨迹数据 */
            std::vector<trajectory_data_t> radarObject = linb::any_cast<std::vector<trajectory_data_t>>(*std::get<1>(frame));

            size_t dataSize = sizeof(trajectory_data_t) * radarObject.size();
            size_t totalBytes = sizeof(push_frame_t) + dataSize;

            std::vector<uint8_t> temp(totalBytes);
            push_frame_t *pushFrame = reinterpret_cast<push_frame_t *>(temp.data());

            pushFrame->head = htonl(TCP_FRAME_HEAD);
            pushFrame->tail = htonl(TCP_FRAME_TAIL);
            pushFrame->type = type;
            pushFrame->length = htons(totalBytes - (sizeof(pushFrame->head) + sizeof(pushFrame->tail)));
            get_device_code(pushFrame->deviceSn);
            pushFrame->timestamp = get_current_timestamp();

            size_t checkSumDataSize = sizeof(pushFrame->deviceSn) + sizeof(pushFrame->timestamp) + dataSize;
            std::vector<uint8_t> checkSumData(checkSumDataSize);
            memcpy(checkSumData.data(), pushFrame->deviceSn, sizeof(pushFrame->deviceSn));
            memcpy(checkSumData.data() + sizeof(pushFrame->deviceSn), &pushFrame->timestamp, sizeof(pushFrame->timestamp));

            size_t dataIndex = 0;
            std::vector<uint8_t> data(dataSize);

            for (const auto &item : radarObject) {
                memcpy(data.data() + dataIndex, &item, sizeof(trajectory_data_t));
                dataIndex += sizeof(trajectory_data_t);
            }

            if (dataSize > 0) {
                memcpy(checkSumData.data() + sizeof(pushFrame->deviceSn) + sizeof(pushFrame->timestamp), data.data(), dataSize);
            }
            pushFrame->checksum = calculate_checksum(checkSumData);

            SendTcpProtocolPacket(totalBytes, pushFrame, data);
        } else if (type == TRAFFIC_FLOW) {  /* 交通流量 */
            std::vector<traffic_flow_t> trafficFlow = linb::any_cast<std::vector<traffic_flow_t>>(*std::get<1>(frame));

            size_t dataSize = sizeof(traffic_flow_t) * trafficFlow.size();
            size_t totalBytes = sizeof(push_frame_t) + dataSize;

            std::vector<uint8_t> temp(totalBytes);
            push_frame_t *pushFrame = reinterpret_cast<push_frame_t *>(temp.data());

            pushFrame->head = htonl(TCP_FRAME_HEAD);
            pushFrame->tail = htonl(TCP_FRAME_TAIL);
            pushFrame->type = type;
            pushFrame->length = htons(totalBytes - (sizeof(pushFrame->head) + sizeof(pushFrame->tail)));
            get_device_code(pushFrame->deviceSn);
            pushFrame->timestamp = get_current_timestamp();

            size_t checkSumDataSize = sizeof(pushFrame->deviceSn) + sizeof(pushFrame->timestamp) + dataSize;
            std::vector<uint8_t> checkSumData(checkSumDataSize);
            memcpy(checkSumData.data(), pushFrame->deviceSn, sizeof(pushFrame->deviceSn));
            memcpy(checkSumData.data() + sizeof(pushFrame->deviceSn), &pushFrame->timestamp, sizeof(pushFrame->timestamp));

            size_t dataIndex = 0;
            std::vector<uint8_t> data(dataSize);

            for (const auto &item : trafficFlow) {
                memcpy(data.data() + dataIndex, &item, sizeof(traffic_flow_t));
                dataIndex += sizeof(traffic_flow_t);
            }

            if (dataSize > 0) {
                memcpy(checkSumData.data() + sizeof(pushFrame->deviceSn) + sizeof(pushFrame->timestamp), data.data(), dataSize);
            }
            pushFrame->checksum = calculate_checksum(checkSumData);

            SendTcpProtocolPacket(totalBytes, pushFrame, data);
        } else if (type == TRAFFIC_EVENT) { /* 交通事件 */
            std::vector<traffic_event_t> trafficEvent = linb::any_cast<std::vector<traffic_event_t>>(*std::get<1>(frame));

            size_t dataSize = sizeof(traffic_event_t) * trafficEvent.size();
            size_t totalBytes = sizeof(push_frame_t) + dataSize;

            std::vector<uint8_t> temp(totalBytes);
            push_frame_t *pushFrame = reinterpret_cast<push_frame_t *>(temp.data());

            pushFrame->head = htonl(TCP_FRAME_HEAD);
            pushFrame->tail = htonl(TCP_FRAME_TAIL);
            pushFrame->type = type;
            pushFrame->length = htons(totalBytes - (sizeof(pushFrame->head) + sizeof(pushFrame->tail)));
            get_device_code(pushFrame->deviceSn);
            pushFrame->timestamp = get_current_timestamp();

            size_t checkSumDataSize = sizeof(pushFrame->deviceSn) + sizeof(pushFrame->timestamp) + dataSize;
            std::vector<uint8_t> checkSumData(checkSumDataSize);
            memcpy(checkSumData.data(), pushFrame->deviceSn, sizeof(pushFrame->deviceSn));
            memcpy(checkSumData.data() + sizeof(pushFrame->deviceSn), &pushFrame->timestamp, sizeof(pushFrame->timestamp));

            size_t dataIndex = 0;
            std::vector<uint8_t> data(dataSize);

            for (const auto &item : trafficEvent) {
                memcpy(data.data() + dataIndex, &item, sizeof(traffic_event_t));
                dataIndex += sizeof(traffic_event_t);
            }

            if (dataSize > 0) {
                memcpy(checkSumData.data() + sizeof(pushFrame->deviceSn) + sizeof(pushFrame->timestamp), data.data(), dataSize);
            }
            pushFrame->checksum = calculate_checksum(checkSumData);

            SendTcpProtocolPacket(totalBytes, pushFrame, data);
        } else if (type == RADAR_CLUSTER) { /* 目标点云 */
            std::vector<radar_cluster_t> cluster = linb::any_cast<std::vector<radar_cluster_t>>(*std::get<1>(frame));

            size_t dataSize = sizeof(radar_cluster_t) * cluster.size();
            size_t totalBytes = sizeof(push_frame_t) + dataSize;

            std::vector<uint8_t> temp(totalBytes);
            push_frame_t *pushFrame = reinterpret_cast<push_frame_t *>(temp.data());

            pushFrame->head = htonl(TCP_FRAME_HEAD);
            pushFrame->tail = htonl(TCP_FRAME_TAIL);
            pushFrame->type = type;
            pushFrame->length = htons(totalBytes - (sizeof(pushFrame->head) + sizeof(pushFrame->tail)));
            get_device_code(pushFrame->deviceSn);
            pushFrame->timestamp = get_current_timestamp();

            size_t checkSumDataSize = sizeof(pushFrame->deviceSn) + sizeof(pushFrame->timestamp) + dataSize;
            std::vector<uint8_t> checkSumData(checkSumDataSize);
            memcpy(checkSumData.data(), pushFrame->deviceSn, sizeof(pushFrame->deviceSn));
            memcpy(checkSumData.data() + sizeof(pushFrame->deviceSn), &pushFrame->timestamp, sizeof(pushFrame->timestamp));

            size_t dataIndex = 0;
            std::vector<uint8_t> data(dataSize);

            for (const auto &item : cluster) {
                memcpy(data.data() + dataIndex, &item, sizeof(radar_cluster_t));
                dataIndex += sizeof(radar_cluster_t);
            }

            if (dataSize > 0) {
                memcpy(checkSumData.data() + sizeof(pushFrame->deviceSn) + sizeof(pushFrame->timestamp), data.data(), dataSize);
            }
            pushFrame->checksum = calculate_checksum(checkSumData);

            SendTcpProtocolPacket(totalBytes, pushFrame, data);
        }
    });
}

static void *tcp_push_frame_thread(void *arg)
{
    pthread_setname_np(pthread_self(), "frame_pthread");

    while (1) {
        usleep(85000);

        if (!frameQueue.empty()) {
            auto data = std::move(frameQueue.front());
            frameQueue.pop_front();

            if (tcp_server_clients(1502) > 0) {
                auto future = ProcessTcpPushFrameAsync(std::move(data));
            }

            if (frameQueue.size() > 100) {
                /* 删除前面多余的帧 */
                int numFramesToRemove = frameQueue.size() - 100;
                for (int i = 0; i < numFramesToRemove; ++i) {
                    frameQueue.pop_front();
                }
            }
        } else {
            empty_trajectory_push();
        }
    }

    pthread_exit(NULL);
}

static void *trajectory_flow_statistics(void *arg)
{
    std::vector<traffic_flow_t> trafficFlow{};

    pthread_setname_np(pthread_self(), "flow_statistics");

    while (1) {
        /* 1分钟统计一次交通流量 */
        sleep(60);

        traffic_flow_t flow;
        flow.period             = htons(0x0001);
        flow.small_car_flow     = htons(trafficFlowDetectionStatisticsDef.smallVehicleFlowData);
        flow.big_car_flow       = htons(trafficFlowDetectionStatisticsDef.middleVehicleFlowData);
        flow.super_big_car_flow = htons(trafficFlowDetectionStatisticsDef.bigVehicleFlowData);
        flow.total_flow         = htons(trafficFlowDetectionStatisticsDef.trafficFlowData);
        flow.average_speed      = htons(trafficFlowDetectionStatisticsDef.sectionAverageSpeedData * 10);
        flow.head_time          = htons(0x0000 * 10);
        flow.vir_coil_occ_time  = htons(0x0000 * 10);
        flow.max_queue_length   = htons(0x0000 * 10);
        flow.average_body_space = htons(0x0000 * 10);
        flow.lane1_flow         = htons(trafficFlowDetectionStatisticsDef.laneDividedTrafficFlowData[0]);
        flow.lane2_flow         = htons(trafficFlowDetectionStatisticsDef.laneDividedTrafficFlowData[1]);
        flow.lane3_flow         = htons(trafficFlowDetectionStatisticsDef.laneDividedTrafficFlowData[2]);
        flow.lane4_flow         = htons(trafficFlowDetectionStatisticsDef.laneDividedTrafficFlowData[3]);
        flow.lane5_flow         = htons(trafficFlowDetectionStatisticsDef.laneDividedTrafficFlowData[4]);
        flow.lane6_flow         = htons(trafficFlowDetectionStatisticsDef.laneDividedTrafficFlowData[5]);
        flow.lane7_flow         = htons(trafficFlowDetectionStatisticsDef.laneDividedTrafficFlowData[6]);
        flow.lane8_flow         = htons(trafficFlowDetectionStatisticsDef.laneDividedTrafficFlowData[7]);
        trafficFlow.emplace_back(std::move(flow));

        if (trafficFlow.size() > 0) {
            frameQueue.emplace_back(std::make_tuple(TRAFFIC_FLOW, std::make_shared<linb::any>(std::move(trafficFlow))));
        }
    }

    pthread_exit(NULL);
}

void frame_push_protocol_init(void)
{
    pthread_create(&pushThreadId, NULL, tcp_push_frame_thread, NULL);
    pthread_create(&flowThreadId, NULL, trajectory_flow_statistics, NULL);
}

void frame_push_protocol_exit(void)
{
    pthread_cancel(pushThreadId);
    pthread_cancel(flowThreadId);
    pthread_join(pushThreadId, NULL);
    pthread_join(flowThreadId, NULL);
}
