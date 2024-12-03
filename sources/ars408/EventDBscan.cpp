/********************************************************************************
* @File name: EventDBscan.c
* @Author: 石海新
* @Version: 1.0
* @Date: 2023.6.8
* @Description: EventDBscan聚类功能文件
********************************************************************************/
#if defined(__ARM_NEON)
#include <arm_neon.h>
#endif

#include "EventDBscan.h"
#include "trafficEvent.h"

/********************************************************
* Function name ：Event_Distance
* Description   ：实现对车辆距离矩阵的计算
* Parameter     ：
* @Outdata        滤波生成的目标数据
* @distance       距离矩阵
* Return        ：distance
**********************************************************/
#if !defined(__ARM_NEON)
void Event_Distance(ARS408RadarObjectInfo *Outdata, float *distance)
{
    int i, j;

    if ((Outdata == NULL) || (distance == NULL)) {
        return;
    }

    for (i = 0; i < TRACK_NUM_TRK; i++) {
        float yi = Outdata[i].Object_DistLat;
        float xi = Outdata[i].Object_DistLong;

        for (j = 0; j < TRACK_NUM_TRK; j++) {
            float yj = Outdata[j].Object_DistLat;
            float xj = Outdata[j].Object_DistLong;

            double sum = (yi - yj) * (yi - yj) + (xi - xj) * (xi - xj);
            distance[i + j * TRACK_NUM_TRK] = (float)sqrt(sum);
        }
    }
}
#else
void Event_Distance(ARS408RadarObjectInfo *Outdata, float *distance)
{
    int i, j;
    //double elapsed = 0.0;
    //struct timeval starting, finished;

    if ((Outdata == NULL) || (distance == NULL)) {
        return;
    }

    //gettimeofday(&starting, NULL);
    for (i = 0; i < TRACK_NUM_TRK; i++) {
        /* 使用 NEON 指令集加载对象的横向坐标和纵向坐标 */
        float32x4_t yi = vdupq_n_f32(Outdata[i].Object_DistLat);
        float32x4_t xi = vdupq_n_f32(Outdata[i].Object_DistLong);

       /**
        * 在ARM NEON优化中，通常会利用向量化指令来同时处理多个数据，以提高计算效率。在这段代码中，采用了NEON指令集来对四个对象进行并行处理，
        * 因此内层循环的步长为4。这样做的好处是可以利用NEON的并行性，一次处理多个数据，从而提高计算效率。
        * 具体来说，内层循环的步长为4的原因是因为在NEON指令集中，通常可以同时处理多个数据，比如一次性处理4个单精度浮点数(float32x4_t)。
        * 因此，通过内层循环的步长设置为4，可以充分利用NEON的并行计算能力，将多个数据打包成一个向量进行计算，从而提高计算效率。
        * 如果内层循环的步长为1(即for (j = 0; j < TRACK_NUM_TRK; j++))，那么在每次循环中只处理一个数据，无法充分利用NEON的并行性，效率会较低。
        * 因此，为了最大程度地发挥NEON指令集的优势，内层循环的步长通常设置为能够同时处理多个数据的值，比如4
        * 
        * 至于为什么内层循环的起始值是j = i + 1，而不是j = 0，这是因为在计算两个对象之间的距离时，我们只需要计算一次即可，因为距离是对称的。
        * 例如，计算对象1和对象2之间的距离后，计算对象2和对象1之间的距离是多余的，因为它们的距离是相同的。
        * 因此，通过将内层循环的起始值设置为j = i + 1，我们避免了重复计算对象之间的距离，减少了计算量。
        * 同时，内层循环的步长为4可以充分利用NEON指令集的并行性，提高计算效率
        */
        for (j = i + 1; j < TRACK_NUM_TRK; j += 4) {
            /* 使用 NEON 指令集进行手动向量化计算横向坐标和纵向坐标的差值 */
            float32x4_t y_diff = vsubq_f32(yi, vld1q_f32(&Outdata[j].Object_DistLat));
            float32x4_t x_diff = vsubq_f32(xi, vld1q_f32(&Outdata[j].Object_DistLong));

            /* 计算距离的平方 */
            float32x4_t sum = vaddq_f32(vmulq_f32(y_diff, y_diff), vmulq_f32(x_diff, x_diff));

            /**
             * 为了避免重复计算平方根，从而提高计算效率。在距离计算中，通常我们只需要比较距离的大小，而不需要准确的距离值。
             * 因此，可以先计算距离的平方，然后对平方进行一系列操作来近似计算平方根的倒数，从而避免使用较慢的平方根函数
             */

            /* 初始估计。这一步使用 vrsqrteq_f32 指令计算平方根的倒数的初始估计。这个指令是一个快速的估计，它的结果可能并不非常准确，但计算速度非常快 */
            float32x4_t sqrt_sum = vrsqrteq_f32(sum);

            /* 第一次修正。这一步使用 vrsqrtsq_f32 指令进行修正。首先，它计算 sum * sqrt_sum，然后将结果与 sqrt_sum 相乘，得到一个更准确的平方根倒数。这个修正的目的是提高初始估计的精度 */
            sqrt_sum = vmulq_f32(vrsqrtsq_f32(vmulq_f32(sum, sqrt_sum), sqrt_sum), sqrt_sum);

            /* 第二次修正：这一步与上一步类似，再次进行修正以进一步提高精度。通过再次使用 vrsqrtsq_f32，可以更好地逼近真实的平方根倒数 */
            sqrt_sum = vmulq_f32(vrsqrtsq_f32(vmulq_f32(sum, sqrt_sum), sqrt_sum), sqrt_sum);

            /* 最终计算：最后一步将平方和与平方根倒数相乘，得到最终的结果。此时 sqrt_sum 实际上存储的是平方根的值 */
            sqrt_sum = vmulq_f32(sum, sqrt_sum);

            /* 将计算结果存储到距离数组中 */
            vst1q_f32(&distance[i * TRACK_NUM_TRK + j], sqrt_sum);
            vst1q_f32(&distance[j * TRACK_NUM_TRK + i], sqrt_sum);  /* 复制对称值 */
        }
    }

    //gettimeofday(&finished, NULL);
    //elapsed = calc_elapsed_time(starting, finished);
    //printf("exec %d x %d loop took: %f ms\n", TRACK_NUM_TRK, TRACK_NUM_TRK, elapsed);
}
#endif

/********************************************************
* Function name ：Event_RegionQuery
* Description   ：实现按照距离阈值寻找在阈值之内的点数
* Parameter     ：
* @i              以第i个点为基准
* @neighbors      相邻点矩阵
* @distance       距离矩阵
* @Outlength      数据长度指针
* Return        ：neighbors
**********************************************************/
void Event_Regionquery(int i, float *neighbors, float *distance, int *Outlength)
{
    int j, num = 0;

    if ((neighbors == NULL) || (distance == NULL) || (Outlength == NULL)) {
        return;
    }

    for (j = 0; j < *Outlength; j++) {
        if (distance[i * TRACK_NUM_TRK + j] <= adjust_Params.AdjustParams_LIMIT_DIST) {
            neighbors[num++] = (float)(j + 1);
        }
    }
}

/********************************************************
* Function name ：Event_Numel
* Description   ：计算相邻点数
* Parameter     ：
* @neighbors      相邻点矩阵
* Return        ：相邻点数
**********************************************************/
int Event_Numel(float *neighbors)
{
    int i, num = 0;

    if (neighbors == NULL) {
        return 0;
    }

    for (i = 0; i < TRACK_NUM_TRK * TRACK_NUM_TRK; i++) {
        if (neighbors[i] > 0) {
            ++num;
        } else {
            break;
        }
    }

    return num;
}

/********************************************************
* Function name ：Event_Expand_Cluster
* Description   ：扩展簇代码
* Parameter     ：
* @Cluster        簇ID数组
* @i              以第i个点为基准
* @neighbors      相邻点矩阵
* @Cluster_Id      当前簇ID
* @visited        访问数组
* @distance       距离矩阵
* @Outlength      数据长度指针
* Return        ：neighbors
**********************************************************/
void Event_Expand_Cluster(int *Cluster, int i, float *neighbors, int Cluster_Id, int *visited, float *distance, int *Outlength)
{
    int k = 0, n;

    if ((Cluster == NULL) || (neighbors == NULL) || (visited == NULL) || (distance == NULL) || (Outlength == NULL)) {
        return;
    }

    Cluster[i] = Cluster_Id;

    while (1) {
        int j = (int)(neighbors[k] - 1);
        if (!visited[j]) {
            visited[j] = 1;

            // 从第一个邻居开始扩展
            static float neighbors2[TRACK_NUM_TRK];

            memset(neighbors2, 0, sizeof(float) * TRACK_NUM_TRK);
            Event_Regionquery(j, neighbors2, distance, Outlength);

            if (Event_Numel(neighbors2) >= MIN_POINTS) {
                int flag = 0, num2, ii;
                for (num2 = 0; num2 < Event_Numel(neighbors2); ++num2) {
                    flag = 0;
                    for (ii = 0; ii < Event_Numel(neighbors); ++ii) {
                        if (neighbors[ii] == neighbors2[num2]) {
                            flag = 1;
                            break;
                        }
                    }

                    if (flag == 1) {
                        continue;
                    } else {
                        neighbors[Event_Numel(neighbors)] = neighbors2[num2];
                    }
                }
            }
        }

        // 如果没分配cluster_id,则进行分配
        if (Cluster[j] == 0) {
            Cluster[j] = Cluster_Id;
        }

        // 如果没有则跳出
        k++;
        if (k >= Event_Numel(neighbors)) {
            break;
        }
    }
}

/********************************************************
* Function name ：Event_dbscan
* Description   ：实现DBSCAN主函数
* Parameter     ：
* @Cluster        簇ID数组
* @i              以第i个点为基准
* @neighbors      相邻点矩阵
* @Cluster_Id     当前簇ID
* @visited        访问数组
* @distance       距离矩阵
* @Outlength      数据长度指针
* Return        ：聚类结果
**********************************************************/
void Event_dbscan(ARS408RadarObjectInfo *Outdata, int *Outlength, int *Cluster)
{
    int Cluster_Id = 0;
    int visited[TRACK_NUM_TRK] = { 0 };
    int isnoise[TRACK_NUM_TRK] = { 0 };
    static float neighbors[TRACK_NUM_TRK] = { 0 };
    static float distance[TRACK_NUM_TRK * TRACK_NUM_TRK] = { 0 };

    if ((Outdata == NULL) || (Outlength == NULL) || (Cluster == NULL)) {
        return;
    }

    // 计算距离矩阵
    Event_Distance(Outdata, distance);

    if (*Outlength > TRACK_NUM_TRK) {
        *Outlength = TRACK_NUM_TRK;
    }

    for (int i = 0; i < *Outlength; i++) {
        visited[i] = 0;     // 初始化访问点集合
        isnoise[i] = 0;     // 初始化噪声点集合
    }

    for (int i = 0; i < *Outlength; i++) {
        if (!visited[i]) {
            /* 将该点置为访问过 */
            visited[i] = 1;

            /* 初始化相邻点 */
            memset(neighbors, 0, sizeof(float) * TRACK_NUM_TRK);

            /* 按照距离阈值寻找在阈值之内的点数 */
            Event_Regionquery(i, neighbors, distance, Outlength);

            if (Event_Numel(neighbors) < MIN_POINTS) {
                /* 若相邻点数目小于阈值，则认为是噪声 */
                isnoise[i] = 1;
            } else {
                /* 若相邻点数目大于阈值，则认为是可进行聚类的点 */
                /* 聚类簇数量 */
                Cluster_Id++;
                /* 将点迹数目进行聚类 */
                Event_Expand_Cluster(Cluster, i, neighbors, Cluster_Id, visited, distance, Outlength);
            }
        }
    }
}
