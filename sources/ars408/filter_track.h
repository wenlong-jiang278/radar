#ifndef FILTER_TRACK_H
#define FILTER_TRACK_H

#ifdef __cplusplus
extern "C" {
#endif

#define WINDOW_SIZE                 5   /* 可以根据需要调整窗口大小 */

typedef struct {
    float q;                            /* 过程噪声 */
    float r;                            /* 测量噪声 */
    float x;                            /* 估计值 */
    float p;                            /* 估计误差 */
    float k;                            /* 卡尔曼增益 */
} KalmanFilter;

float kalman_update(KalmanFilter *kf, float measurement);
void kalman_init(KalmanFilter *kf, float process_noise, float measurement_noise);

float ewma_latitude(float new_value);
float median_filter(float new_value);
float smooth_latitude(float new_value);

#ifdef __cplusplus
}
#endif

#endif
