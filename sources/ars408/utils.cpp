#include "utils.h"
#include <exception>

/********************************************************
* Function name ：RCS_func
* Description   ：实现RCS拟合函数
* Parameter     ：
* @range          距离雷达的径向距离
* Return        ：RCS基准值
**********************************************************/
track_float_t RCS_func(track_float_t range)
{
    double w = 0.003153;
    track_float_t rcs = 0;
    double wrange = range * w;
    double b1 = 5507.0, b2 = -3597.0, b3 = 640.4;
    double a0 = -6338.0, a1 = 7799.0, a2 = -1352.0, a3 = -124.0;
    double cos_wrange = cos(wrange);
    double sin_wrange = sin(wrange);
    double cos2_wrange = cos(2 * wrange);
    double sin2_wrange = sin(2 * wrange);
    double cos3_wrange = cos(3 * wrange);
    double sin3_wrange = sin(3 * wrange);

    try {
        rcs = a0 + a1 * cos_wrange + b1 * sin_wrange + a2 * cos2_wrange + b2 * sin2_wrange + a3 * cos3_wrange + b3 * sin3_wrange;
    } catch (const std::exception &e) {
        rcs = 0;
    }

    return rcs;
}

/********************************************************
* Function name ：Random_Num
* Description   ：对随机UUID进行编码的随机函数
* Parameter
* Return
**********************************************************/
int Random_Num()
{
    return ((rand() << 16) + (rand() << 1) + rand() % 2);
}

/********************************************************
* Function name ：ROI功能函数
* Description   ：判断一个点是否存在一个多边形内
* Parameter
* Return
**********************************************************/
float max_roi(float a, float b)
{
    return a > b ? a : b;
}

float min_roi(float a, float b)
{
    return a < b ? a : b;
}

void GetPntsExtremeXYs(Point *Pnts, int PntNum, float *MinX, float *MaxX, float *MinY, float *MaxY)
{
    if ((Pnts == NULL) || (PntNum <= 0) || (MinX == NULL) || (MaxX == NULL) || (MinY == NULL) || (MaxY == NULL)) {
        return;
    }

    *MinX = Pnts[0].x;
    *MaxX = Pnts[0].x;
    *MinY = Pnts[0].y;
    *MaxY = Pnts[0].y;

    Point P;
    for (int i = 1; i < PntNum; i++) {
        P = Pnts[i];
        if (P.x < *MinX) {
            *MinX = P.x;
        }

        if (P.x > *MaxX) {
            *MaxX = P.x;
        }

        if (P.y < *MinY) {
            *MinY = P.y;
        }

        if (P.y > *MaxY) {
            *MaxY = P.y;
        }
    }
}

int PointInLineSegment(Point P1, Point P2, Point Q)
{
    // P1, P2, Q共线
    if ((Q.x - P1.x) * (P2.y - P1.y) == (P2.x - P1.x) * (Q.y - P1.y)
        && min_roi(P1.x, P2.x) <= Q.x && Q.x <= max_roi(P1.x, P2.x)
        && min_roi(P1.y, P2.y) <= Q.y && Q.y <= max_roi(P1.y, P2.y))
    {
        return 1;
    }

    return 0;
}

int IsPointInPolygon(Point *Pnts, int PntNum, Point Q)
{
    float MinX, MaxX, MinY, MaxY;

    if ((Pnts == NULL) || (PntNum <= 0)) {
        return 0;
    }

    GetPntsExtremeXYs(Pnts, PntNum, &MinX, &MaxX, &MinY, &MaxY);

    // 判别点在多边形正最小外接矩形外
    if ((Q.x < MinX) || (Q.y < MinY) || (Q.x > MaxX) || (Q.y > MaxY)) {
        return 0;
    }

    Point P1, P2;
    int i, j, c = 0;

    for (i = 0, j = PntNum - 1; i < PntNum; j = i++) {
        P1 = Pnts[i];
        P2 = Pnts[j];

        // 点在多边形的边上
        if (PointInLineSegment(P1, P2, Q)) {
            return 1;
        }

        if (((P1.y > Q.y) != (P2.y > Q.y)) &&                               // 这行代码既表达了Y轴在起始结束点之间，又同时解决了射线点如果和边框线顶点重合的问题
            (Q.x < (P2.x - P1.x) * (Q.y - P1.y) / (P2.y - P1.y) + P1.x))    // 在此假设从Q点水平向右引出一条射线，因此可以由Q.y带入直线方程计算出x
        {
            c = !c; // 奇数次为真
        }
    }

    return c;
}

/********************************************************
* Function name ：Encode_Event
* Description   ：实现对事件标志位的编码
* Parameter     ：
* @event          事件标志位
* @EventType      事件类型
* Return        ：event
**********************************************************/
int Encode_Event(int event, int EventType)
{
    return event | EventType;
}

/********************************************************
* Function name ：Decode_Event
* Description   ：实现对事件标志位的解码
* Parameter     ：
* @event          事件标志位
* @EventType      事件类型
* Return        ：事件是否发生的标志位
**********************************************************/
int Decode_Event(int event, int EventType)
{
    return event & EventType;
}
