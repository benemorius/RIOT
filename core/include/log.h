/*
 * Copyright (C) 2015 Kaspar Schleiser <kaspar@schleiser.de>
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup     core_util
 * @{
 *
 * @file
 * @brief       System logging header
 *
 * This header offers a bunch of "LOG_*" functions that, with the default
 * implementation, just use printf, but honour a verbosity level.
 *
 * If desired, it is possible to implement a log module which then will be used
 * instead the default printf-based implementation.  In order to do so, the log
 * module has to
 *
 * 1. provide "log_module.h"
 * 2. have a name starting with "log_" *or* depend on the pseudo-module LOG,
 * 3. implement log_write()
 *
 * See "sys/log/log_printfnoformat" for an example.
 *
 * @author      Kaspar Schleiser <kaspar@schleiser.de>
 */

#ifndef LOG_H
#define LOG_H

#ifdef __cplusplus
extern "C" {
#endif

#include <string.h>

/**
 * @brief defined log levels
 *
 * These are the logging levels a user can choose.
 * The idea is to set LOG_LEVEL to one of these values in the application's Makefile.
 * That will restrict output of log statements to those with equal or lower log level.
 *
 * The default log level is LOG_INFO, which will print every message.
 *
 * The log function calls of filtered messages will be optimized out at compile
 * time, so a lower log level might result in smaller code size.
 */
enum {
    LOG_NONE,       /**< Lowest log level, will output nothing */
    LOG_ERROR,      /**< Error log level, will print only critical,
                         non-recoverable errors like hardware initialization
                         failures */
    LOG_WARNING,    /**< Warning log level, will print warning messages for
                         temporary errors */
    LOG_INFO,       /**< Informational log level, will print purely
                         informational messages like successful system bootup,
                         network link state, ...*/
    LOG_DEBUG,      /**< Debug log level, printing developer stuff considered
                         too verbose for production use */
    LOG_ALL         /**< print everything */
};

#ifndef LOG_LEVEL
/**
 * @brief Default log level define
 */
// #undef LOG_LEVEL
#define LOG_LEVEL LOG_INFO
#endif

/**
 * @brief Log message if level <= LOG_LEVEL
 */
#ifdef __clang__    /* following pragmas required for clang 3.8.0 */
#define _LOG(level, file, function, line, ...) do { \
        _Pragma("clang diagnostic push") \
        _Pragma("clang diagnostic ignored \"-Wtautological-compare\"") \
        if ((level) <= LOG_LEVEL) log_write((level), file, function, line, __VA_ARGS__); } while (0U) \
        _Pragma("clang diagnostic pop")
#else
#define _LOG(level, file, function, line, ...) do { \
        if ((level) <= LOG_LEVEL) log_write((level), file, function, line, __VA_ARGS__); } while (0U)
#endif /* __clang__ */

/**
 * @name Logging convenience defines
 * @{
 */
#define __FILENAME__ (strrchr(__FILE__, '/') ? strrchr(__FILE__, '/') + 1 : __FILE__)
#define LOG_ERROR(...) _LOG(LOG_ERROR, __FILENAME__, __func__, __LINE__, __VA_ARGS__)
#define LOG_WARNING(...) _LOG(LOG_WARNING, __FILENAME__, __func__, __LINE__, __VA_ARGS__)
#define LOG_INFO(...) _LOG(LOG_INFO, __FILENAME__, __func__, __LINE__, __VA_ARGS__)
#define LOG_DEBUG(...) _LOG(LOG_DEBUG, __FILENAME__, __func__, __LINE__, __VA_ARGS__)
#define LOG(level, ...) _LOG(level, __FILENAME__, __func__, __LINE__, __VA_ARGS__)
/** @} */

#ifdef MODULE_LOG
#include "log_module.h"
#else
#include <stdio.h>

/**
 * @brief Default log_write function, just maps to printf
 */
#define log_write(level, file, function, line, ...) printf(__VA_ARGS__)
#endif

#ifdef __cplusplus
}
#endif

#endif /* LOG_H */
/** @} */
