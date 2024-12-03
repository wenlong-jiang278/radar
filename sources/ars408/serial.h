#ifndef __SERIAL_H__
#define __SERIAL_H__

#include <sys/types.h>

#ifdef __cplusplus
extern "C" {
#endif

int get_serial_number(char *serial, size_t size);

#ifdef __cplusplus
}
#endif

#endif
