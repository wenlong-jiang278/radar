#include <algorithm>
#include "filter_track.h"

void kalman_init(KalmanFilter *kf, float process_noise, float measurement_noise)
{
    kf->q = process_noise;
    kf->r = measurement_noise;
    kf->x = 0; // 初始估计值
    kf->p = 1; // 初始估计误差
}

float kalman_update(KalmanFilter *kf, float measurement)
{
    // 预测步骤
    kf->p += kf->q;

    // 更新步骤
    kf->k = kf->p / (kf->p + kf->r);
    kf->x += kf->k * (measurement - kf->x);
    kf->p *= (1 - kf->k);

    return kf->x;
}

float ewma_latitude(float new_value)
{
    static float alpha = 0.1f;      // 平滑因子，0 < alpha < 1
    static float smoothed_value = 0;

    smoothed_value = alpha * new_value + (1 - alpha) * smoothed_value;
    return smoothed_value;
}

float smooth_latitude(float new_value)
{
    static int index = 0;
    static int count = 0;
    static float window[WINDOW_SIZE] = {0};

    window[index] = new_value;
    index = (index + 1) % WINDOW_SIZE;

    if (count < WINDOW_SIZE) {
        count++;
    }

    float sum = 0;
    for (int i = 0; i < count; ++i) {
        sum += window[i];
    }

    return sum / count;
}

float median_filter(float new_value)
{
    static int index = 0;
    static int count = 0;
    static float window[WINDOW_SIZE] = {0};

    window[index] = new_value;
    index = (index + 1) % WINDOW_SIZE;

    if (count < WINDOW_SIZE) {
        count++;
    }

    float sorted_window[WINDOW_SIZE];
    std::copy(window, window + count, sorted_window);
    std::sort(sorted_window, sorted_window + count);

    return sorted_window[count / 2];
}
