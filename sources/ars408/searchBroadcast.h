#ifndef USER_SEARCHBROADCAST_H
#define USER_SEARCHBROADCAST_H

#ifdef __cplusplus
extern "C" {
#endif

#define USER_BROADCAST_PORT     6601
#define USER_SEARCHANDRST_PORT  7601

extern void *hdlSearchBroadcast_pthread(void *arg);
extern void *hdlSearchAndRestoryBrdcast_pthread(void *arg);

#ifdef __cplusplus
}
#endif

#endif
