/********************************************************************************
* @File name: TrackDbscan.c
* @Author: 石海新
* @Version: 3.0
* @Date: 2023.5.9
* @Description: DBSCAN聚类功能文件
********************************************************************************/
#if defined(__ARM_NEON)
#include <arm_neon.h>
#endif

#include "TrackDbscan.h"
#include <exception>

/********************************************************
* Function name ：Distance_differ
* Description   ：实现对距离矩阵的计算
* Parameter     ：
* @Range_x               点云的x数据
* @Range_y               点云的y数据
* @NumberofCdi           点云数量
* @TableofDistanceDiffe  距离矩阵
* @lamada_r              半径因子
* Return        ：TableofDistanceDiffe
**********************************************************/
void Distance_differ(int i, track_float_t *Range_x, track_float_t *Range_y, uint32_t NumberofCdi, track_float_t *TableofDistanceDiffe, track_float_t lamada_r)
{
    float ellipse_a;
    float ellipse_b;

    if (lamada_r > 15) {
        ellipse_a = lamada_r * adjust_Params.AdjustParams_DBSCANMEDIUMCarEps_a + TrackEllipse_a;
        ellipse_b = adjust_Params.AdjustParams_DBSCANBIGCarEps_b;
    } else if (lamada_r < 0) {
        ellipse_a = TrackEllipse_a;
        ellipse_b = TrackEllipse_b;
    } else {
        ellipse_a = lamada_r * adjust_Params.AdjustParams_DBSCANSMALLCarEps_a + TrackEllipse_a;
        ellipse_b = lamada_r * adjust_Params.AdjustParams_DBSCANSMALLCarEps_b + TrackEllipse_b;
    }

    if (ellipse_b > 6) {
        ellipse_b = 6;
    }

    for (int j = 0; j < NumberofCdi; j++) {
        TableofDistanceDiffe[j + i * TRACK_NUM_CDI] = (track_float_t)0.0;
        track_float_t DistanceDiffe = 0;

        // 修改为椭圆阈值
        DistanceDiffe = ((Range_x[i] - Range_x[j]) * (Range_x[i] - Range_x[j]) / (ellipse_a * ellipse_a)) + ((Range_y[i] - Range_y[j]) * (Range_y[i] - Range_y[j]) / (ellipse_b * ellipse_b));
        TableofDistanceDiffe[j + i * TRACK_NUM_CDI] = fabs(DistanceDiffe);
    }
}

/********************************************************
* Function name ：Velocity_differ
* Description   ：实现对速度矩阵的计算
* Parameter     ：
* @Velocity              点云的速度
* @NumberofCdi           点云数量
* @TableofVelocityDiffe  速度矩阵
* Return        ：TableofVelocityDiffe
**********************************************************/
void Velocity_differ(track_float_t *Velocity, uint32_t NumberofCdi, track_float_t *TableofVelocityDiffe)
{
    for (int i = 0; i < NumberofCdi; i++) {
        for (int j = 0; j < NumberofCdi; j++) {
            float VelocityDiffe = 0;

            TableofVelocityDiffe[i + j * TRACK_NUM_CDI] = (track_float_t)0.0;
            VelocityDiffe = Velocity[i] - Velocity[j];
            TableofVelocityDiffe[i + j * TRACK_NUM_CDI] = fabs(VelocityDiffe);
        }
    }
}

/********************************************************
* Function name ：RegionQuery
* Description   ：实现按照二维阈值寻找在阈值之内的点数
* Parameter     ：
* @i                     以第i个点为基准
* @neighbors             相邻点矩阵
* @TableofDistanceDiffe  距离矩阵
* @TableofVelocityDiffe  速度矩阵
* @NumberofCdi           点云数量
* Return        ：neighbors
**********************************************************/
void Regionquery(int i, track_float_t *neighbors, track_float_t *TableofDistanceDiffe, track_float_t *TableofVelocityDiffe, uint32_t NumberofCdi)
{
    int j, num = 0;

    for (j = 0; j < NumberofCdi; j++) {
        if ((TableofDistanceDiffe[i * TRACK_NUM_CDI + j] <= DBscanEps_Dist) && (TableofVelocityDiffe[i * TRACK_NUM_CDI + j] <= DBscanEps_Velo)) {
            neighbors[num++] = j + 1;
        }
    }
}

/********************************************************
* Function name ：Numel
* Description   ：计算相邻点数
* Parameter     ：
* @neighbors      相邻点矩阵
* Return        ：相邻点数
**********************************************************/
int Numel(track_float_t *neighbors)
{
    int i, num = 0;

    for (i = 0; i < TRACK_NUM_CDI; i++) {
        if (neighbors[i] > 0) {
            ++num;
        } else {
            break;
        }
    }

    return num;
}

/********************************************************
* Function name ：Expand_Cluster
* Description   ：扩展簇代码
* Parameter     ：
* @IDX                   簇ID数组
* @i                     以第i个点为基准
* @neighbors             相邻点矩阵
* @NumberofCluster       当前簇ID指针
* @visited               访问数组
* @TableofDistanceDiffe  距离矩阵
* @TableofVelocityDiffe  速度矩阵
* @NumberofCdi           点云数量
* @Range_x               点云的x数据
* @Range_y               点云的y数据
* @lamada_r              半径因子
* Return        ：neighbors
**********************************************************/
void Expand_Cluster(int *IDX, int i, track_float_t *neighbors, uint32_t *NumberofCluster, track_float_t *visited, track_float_t *TableofDistanceDiffe,
    track_float_t *TableofVelocityDiffe, uint32_t NumberofCdi, track_float_t *Range_x, track_float_t *Range_y, track_float_t lamada_r)
{
    int k = 0, n;
    int Pots = 0;

    IDX[i] = *NumberofCluster;

    while (1) {
        int j = (int)(neighbors[k] - 1);
        if (!visited[j]) {
            visited[j] = 1;

            /* 从第一个邻居开始扩展 */
            static track_float_t neighbors2[TRACK_NUM_CDI];
            for (n = 0; n < TRACK_NUM_CDI; n++) {
                neighbors2[n] = 0;
            }

            /* 计算调整半径后第j+1个点到每个点的距离 */
            Distance_differ(j, Range_x, Range_y, NumberofCdi, TableofDistanceDiffe, lamada_r);
            Regionquery(j, neighbors2, TableofDistanceDiffe, TableofVelocityDiffe, NumberofCdi);

            /* 根据距离动态修改聚类最小点 */
            Pots = 0;
            if (sqrt(Range_x[j] * Range_x[j] + Range_y[j] * Range_y[j]) < 50) {
                Pots = DBscanPotsNumb;
            } else {
                Pots = DBscanPotsNumb - 1;
            }

            if (Numel(neighbors2) >= Pots) {
                int flag = 0, num2, ii;
                for (num2 = 0; num2 < Numel(neighbors2); ++num2) {
                    flag = 0;
                    for (ii = 0; ii < Numel(neighbors); ++ii) {
                        if (neighbors[ii] == neighbors2[num2]) {
                            flag = 1;
                            break;
                        }
                    }

                    if (flag == 1) {
                        continue;
                    } else {
                        neighbors[Numel(neighbors)] = neighbors2[num2];
                    }
                }
            }
        }

        /* 如果没分配cluster_id,则进行分配 */
        if (IDX[j] == 0) {
            IDX[j] = *NumberofCluster;
        }

        k++;
        if (k >= Numel(neighbors)) {
            break;
        }
    }
}

/********************************************************
* Function name ：Secondary_Cluster
* Description   ：进行二次聚类
* Parameter     ：
* @IDX                   簇ID数组
* @Range                 径向距离
* @Range_x               点云的x数据
* @Range_y               点云的y数据
* @Azimuth               方位角
* @NumberofCdi           点云数量
* @NumberofCluster       当前簇ID指针
* Return        ：IDX聚类结果
**********************************************************/
void Secondary_Cluster(int32_t *IDX, track_float_t *Range, track_float_t *Range_x, track_float_t *Range_y, track_float_t *Azimuth, uint32_t NumberofCdi, uint32_t *NumberofCluster)
{
    int i, j, k;
    int flag = *NumberofCluster;
    track_float_t differ_Azimuth = 999;

    // 进行簇之间的比较
    for (i = 1; i < flag; i++) {
        for (j = i + 1; j <= flag; j++) {
            // 计算簇中距离最短两点方位角的差值
            differ_Azimuth = Compare_distance(IDX, i, j, Range_x, Range_y, NumberofCdi, Azimuth);
            // 如果方位角的差值小于2将两簇合并
            if (differ_Azimuth < 1) {
                for (k = 0; k < NumberofCdi; k++) {
                    if (IDX[k] == j) {
                        IDX[k] = i;
                    }
                }

                (*NumberofCluster)--;
            }
        }
    }
}

/********************************************************
* Function name ：Compare_distance
* Description   ：比较两簇中最短点的距离并计算方位角差值
* Parameter     ：
* @IDX                   簇ID数组
* @IDX1                  用于比较的第二个簇号
* @IDX2                  用于比较的第二个簇号
* @Range_x               点云的x数据
* @Range_y               点云的y数据
* @NumberofCdi           点云数量
* @Azimuth               方位角
* Return        ：距离最短时方位角的差值
**********************************************************/
track_float_t Compare_distance(int32_t *IDX, int32_t IDX1, int32_t IDX2, track_float_t *Range_x, track_float_t *Range_y, uint32_t NumberofCdi, track_float_t *Azimuth)
{
#if 1
    int i, j;
    int count1 = 0;
    int count2 = 0;
    int a1[TRACK_NUM_CDI] = { 0 };    // 存放簇号为IDX1的数据
    int a2[TRACK_NUM_CDI] = { 0 };    // 存放簇号为IDX2的数据
    track_float_t distance = 0;
    track_float_t min_distance = 999;
    track_float_t differ_Azimuth = 999;

    if (NumberofCdi > TRACK_NUM_CDI) {
        NumberofCdi = TRACK_NUM_CDI;
    }

    for (i = 0; i < NumberofCdi; i++) {
        if (IDX[i] == IDX1) {
            a1[count1] = i;
            count1++;
        } else if (IDX[i] == IDX2) {
            a2[count2] = i;
            count2++;
        }
    }

    try {
        // 比较簇号为IDX1和IDX2的点的距离
        for (i = 0; i < count1; i++) {
            for (j = 0; j < count2; j++) {
                distance = sqrt((Range_x[a1[i]] - Range_x[a2[j]]) * (Range_x[a1[i]] - Range_x[a2[j]]) + (Range_y[a1[i]] - Range_y[a2[j]]) * (Range_y[a1[i]] - Range_y[a2[j]]));
                if ((distance < min_distance) && (fabs(Range_y[a1[i]] - Range_y[a2[j]]) < 1) && (fabs(Range_x[a1[i]] - Range_x[a2[j]]) < 5)) {
                    // 计算当距离最短时方位角的差值
                    min_distance = distance;
                    differ_Azimuth = fabs(Azimuth[a1[i]] - Azimuth[a2[j]]);
                }
            }
        }
    } catch (const std::exception &e) {
        printf("Compare_distance exception %s\n", e.what());
        differ_Azimuth = 999;
    }

    return differ_Azimuth;
#else
    int i, j;
    int count1 = 0;
    int count2 = 0;
    int a1[TRACK_NUM_CDI] = { 0 };    // 存放簇号为IDX1的数据
    int a2[TRACK_NUM_CDI] = { 0 };    // 存放簇号为IDX2的数据
    track_float_t distance = 0;
    track_float_t min_distance = 999;
    track_float_t differ_Azimuth = 999;

    if (NumberofCdi > TRACK_NUM_CDI) {
        NumberofCdi = TRACK_NUM_CDI;
    }

    for (i = 0; i < NumberofCdi; i++) {
        if (IDX[i] == IDX1) {
            a1[count1] = i;
            count1++;
        } else if (IDX[i] == IDX2) {
            a2[count2] = i;
            count2++;
        }
    }

    float32x4_t min_distance_vec = vdupq_n_f32(999.0f);
    float32x4_t differ_Azimuth_vec = vdupq_n_f32(999.0f);

    for (i = 0; i < count1; i+= 4) {
        float32x4_t range_x_a1 = vld1q_f32(&Range_x[a1[i]]);
        float32x4_t range_y_a1 = vld1q_f32(&Range_y[a1[i]]);

        for (j = 0; j < count2; j++) {
            float32x4_t range_x_diff = vsubq_f32(vld1q_f32(&Range_x[a2[j]]), range_x_a1);
            float32x4_t range_y_diff = vsubq_f32(vld1q_f32(&Range_y[a2[j]]), range_y_a1);

            float32x4_t distance_squared = vaddq_f32(vmulq_f32(range_x_diff, range_x_diff), vmulq_f32(range_y_diff, range_y_diff));
            float32x4_t distance = vrsqrteq_f32(distance_squared);

            // 条件判断
            uint32x4_t mask = vcltq_f32(distance, min_distance_vec);
            mask = vandq_u32(mask, vcltq_f32(vabsq_f32(vsubq_f32(vld1q_f32(&Range_y[a1[i]]), vld1q_f32(&Range_y[a2[j]]))), vdupq_n_f32(1.0f)));
            mask = vandq_u32(mask, vcltq_f32(vabsq_f32(vsubq_f32(vld1q_f32(&Range_x[a1[i]]), vld1q_f32(&Range_x[a2[j]])), vdupq_n_f32(5.0f)));

            min_distance_vec = vbslq_f32(mask, distance, min_distance_vec);
            differ_Azimuth_vec = vbslq_f32(mask, vabsq_f32(vsubq_f32(vld1q_f32(&Azimuth[a1[i]]), vld1q_f32(&Azimuth[a2[j]])), differ_Azimuth_vec);
        }
    }

    float min_distance_result = 999.0f;
    float differ_Azimuth_result = 999.0f;
    min_distance_result = fminf(fminf(fminf(min_distance_vec[0], min_distance_vec[1]), min_distance_vec[2]), min_distance_vec[3]);
    differ_Azimuth_result = fminf(fminf(fminf(differ_Azimuth_vec[0], differ_Azimuth_vec[1]), differ_Azimuth_vec[2]), differ_Azimuth_vec[3]);

    return differ_Azimuth_result;
#endif
}

/********************************************************
* Function name ：Track_DBSCAN
* Description   ：实现DBSCAN主函数
* Parameter     ：
* @IDX                   簇ID数组
* @Range                 径向距离
* @Range_x               点云的x数据
* @Range_y               点云的y数据
* @Velocity              点云的速度
* @Azimuth               方位角
* @Power                 RCS
* @NumberofCdi           点云数量
* @neighbors             相邻点矩阵
* @NumberofCluster       当前簇ID指针
* @visited               访问数组
* @isnoise               噪声矩阵
* @TableofDistanceDiffe  距离矩阵
* @TableofVelocityDiffe  速度矩阵
* Return        ：IDX聚类结果
**********************************************************/
void Track_DBSCAN(int32_t *IDX, track_float_t *Range, track_float_t *Range_x, track_float_t *Range_y, track_float_t *Velocity, track_float_t *Azimuth, track_float_t *Power, uint32_t NumberofCdi, uint32_t *NumberofCluster)
{
    int i, j, Pots = 0;
    track_float_t visited[TRACK_NUM_CDI] = { 0 };
    track_float_t isnoise[TRACK_NUM_CDI] = { 0 };
    static track_float_t neighbors[TRACK_NUM_CDI] = { 0 };
    static track_float_t TableofDistanceDiffe[TRACK_NUM_CDI * TRACK_NUM_CDI] = { 0 };
    static track_float_t TableofVelocityDiffe[TRACK_NUM_CDI * TRACK_NUM_CDI] = { 0 };

    *NumberofCluster = 0;

    for (i = 0; i < TRACK_NUM_CDI * TRACK_NUM_CDI; i++) {
        TableofDistanceDiffe[i] = 0;
        TableofVelocityDiffe[i] = 0;
    }

    /* 计算候选点之间的距离差表 */
    // Distance_differ(Range_x, Range_y, NumberofCdi, TableofDistanceDiffe);

    /* 计算候选点之间的速度差表 */
    Velocity_differ(Velocity, NumberofCdi, TableofVelocityDiffe);

    if (NumberofCdi > TRACK_NUM_CDI) {
        NumberofCdi = TRACK_NUM_CDI;
    }

    /* 查找点云邻居 */
    for (i = 0; i < NumberofCdi; i++) {
        /* 如果该点没被访问过 */
        if (!visited[i]) {
            /* 将该点置为访问过 */
            visited[i] = 1;

            /* 初始化相邻点 */
            for (j = 0; j < TRACK_NUM_CDI; j++) {
                neighbors[j] = 0;
            }

            /* 计算RCS标准值 */
            track_float_t rcs_base = 0;
            track_float_t lamada_r = 0;

            rcs_base = RCS_func(Range_x[i]);

            /* 计算比值 半径因子 */
            track_float_t rcs_present = 0;
            rcs_present = Power[i];
            lamada_r = rcs_present - rcs_base;

            /* 计算调整半径后第i+1个点到每个点的距离 */
            Distance_differ(i, Range_x, Range_y, NumberofCdi, TableofDistanceDiffe, lamada_r);
            /* 按照二维阈值寻找在阈值之内的点数 */
            Regionquery(i, neighbors, TableofDistanceDiffe, TableofVelocityDiffe, NumberofCdi);

            /* 根据距离动态修改聚类最小点 */
            Pots = 0;
            if (Range_x[i] < 50) {
                Pots = DBscanPotsNumb;
            } else {
                Pots = DBscanPotsNumb - 1;
            }

            /* 判断是否为噪声点 */
            if (Numel(neighbors) < Pots) {
                /* 若相邻点数目小于阈值，则认为是噪声 */
                isnoise[i] = 1;
            } else {
                /* 若相邻点数目大于阈值，则认为是可进行聚类的点 */
                /* 聚类簇数量++ */
                (*NumberofCluster)++;

                /* 将点迹数目进行聚类 */
                Expand_Cluster(IDX, i, neighbors, NumberofCluster, visited, TableofDistanceDiffe, TableofVelocityDiffe, NumberofCdi, Range_x, Range_y, lamada_r);
            }
        }
    }
}
