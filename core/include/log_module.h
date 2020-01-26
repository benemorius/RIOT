#ifndef LOG_MODULE_H
#define LOG_MODULE_H

#include "log_dmesg.h"

#ifdef __cplusplus
extern "C" {
#endif

#define log_write(...) log_dmesg(__VA_ARGS__)

#ifdef __cplusplus
}
#endif
/**@}*/
#endif /* LOG_MODULE_H */
