#ifndef JILIN_TCP_H
#define JILIN_TCP_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

void jilin_push_init(void);
void jilin_push_exit(void);
void jilin_push_message(uint8_t data);

#ifdef __cplusplus
}
#endif

#endif
