#include <ctype.h>
#include <stdio.h>
#include <string.h>
#include <spdlog/spdlog.h>

#include "serial.h"

/* 删除空白符 */
static int trim(char *str)
{
    char *end;

    while (isspace((unsigned char)*str)) {
        str++;
    }

    if (*str == 0) {
        return -1;
    }

    end = str + strlen(str) - 1;
    while ((end > str) && isspace((unsigned char)*end)) {
        end--;
    }
    end[1] = '\0';

    return 0;
}

/* 移除非ASCI字符 */
static void removeNonAlnum(char *str)
{
    char *src = str, *dst = str;
    while (*src) {
        if (isalnum((unsigned char)*src)) {
            *dst++ = *src;
        }

        src++;
    }
    *dst = '\0';
}

int get_serial_number(char *serial, size_t size)
{
    FILE *file = fopen("/proc/device-tree/serial-number", "r");
    if (file != NULL) {
        fgets(serial, size, file);
        fclose(file);

        trim(serial);
        removeNonAlnum(serial);
    }

    if (strlen(serial) == 0) {
        char line[256];
        FILE *file1 = fopen("/proc/cpuinfo", "r");

        if (file1 != NULL) {
            while (fgets(line, sizeof(line), file1)) {
                if (strstr(line, "Serial") != NULL) {
                    char *colon = strchr(line, ':');
                    if (colon != NULL) {
                        strncpy(serial, colon + 1, size);
                        trim(serial);
                        removeNonAlnum(serial);
                        break;
                    }
                }
            }

            fclose(file1);
        }
    }

    if (strlen(serial) > 0) {
        spdlog::info("CPU Serial:[{0}]", serial);
    }

    return strlen(serial) > 0 ? 0 : -1;
}
