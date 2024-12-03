#ifndef USER_UPGRADE_H
#define USER_UPGRADE_H

#ifdef __cplusplus
extern "C" {
#endif

extern int errno;
extern int acceptIdx;
extern int acceptId[10];
extern int g_upgradingFlg;

#define USER_ONEPACKETSIZE      4096
#define USER_TCPCOMTIMEOUT      60

extern void *upgradeFile(void *arg);
extern void *upgradeThred(void *arg);

extern int user_translateBuf(char *in, int inSize, char *out);
extern int user_sendTranslateBuf(char *in, int inSize, char *out);
extern unsigned int gen_crc(unsigned char *frame, unsigned short  frame_len);

#ifdef __cplusplus
}
#endif

#endif
