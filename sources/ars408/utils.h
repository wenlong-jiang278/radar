#ifndef USER_UTILS_H
#define USER_UTILS_H

#include "track_common.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct _Point {
    float x;
    float y;
} Point;

track_float_t RCS_func(track_float_t range);

int Random_Num();
int Encode_Event(int event, int EventType);
int Decode_Event(int event, int EventType);
int IsPointInPolygon(Point *Pnts, int PntNum, Point Q);

#ifdef __cplusplus
}
#endif

#endif