/*
 * Copyright (C) 2019 Thomas Stilwell <stilwellt@openlabs.co>
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup     sys_shell_commands
 * @{
 *
 * @file
 * @brief       Shell command to interact with system date and time
 *
 * @author      Thomas Stilwell <stilwellt@openlabs.co>
 * @}
 */

#include <sys/time.h>
#include <stdlib.h>

/* for US_PER_SEC */
#include "xtimer.h"

#ifdef MODULE_PERIPH_RTC
#include "periph/rtc.h"
#endif

/* in sys/newlib_syscalls_default/syscalls.c */
extern int settimeofday(const struct timeval *restrict tp, const void *restrict tzp);
int gettimeofday(struct timeval *tp, void *tzp);

#ifdef MODULE_PERIPH_RTC
int _get_rtc_time(struct tm *tm)
{
    if (rtc_get_time(tm) == 0) {
        /* wait for rtc tick */
        int current_tm_sec = tm->tm_sec;
        while (tm->tm_sec == current_tm_sec) {
            if (rtc_get_time(tm) != 0) {
                fprintf(stderr, "error getting time from RTC\n");
                return 1;
            }
        }
    }
    else {
        fprintf(stderr, "error getting time from RTC\n");
        return 1;
    }
    return 0;
}
#endif

int _date(int argc, char **argv)
{
    argc = argc;
    argv = argv;

    // time_t t = time(NULL);
    // printf("time_t: %lu\n", t);

    struct timeval tv;
    gettimeofday(&tv, NULL);
    // printf("tv.tv_sec: %li\n", tv.tv_sec);
    // printf("tv.tv_usec: %li\n", tv.tv_usec);

    tv.tv_sec += (-7 * 3600);

    struct tm tm = *gmtime(&tv.tv_sec);

    printf("%04i-%02i-%02i %02i:%02i:%02i.%06lu PDT\n",
           tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday,
           tm.tm_hour, tm.tm_min, tm.tm_sec, tv.tv_usec);

    if (argv[1]) {
        tv.tv_sec = atoi(argv[1]);
        tv.tv_usec = 0;
        settimeofday(&tv, NULL);
    }

#ifdef MODULE_PERIPH_RTC
    /* compare rtc to system time */
    struct tm rtc_tm;
    _get_rtc_time(&rtc_tm);
    struct timeval sys_tv;
    gettimeofday(&sys_tv, NULL);
    struct timeval rtc_tv = {mktime(&rtc_tm), 0};
    int64_t rtc_usec = rtc_tv.tv_sec * US_PER_SEC + rtc_tv.tv_usec;
    int64_t sys_usec = sys_tv.tv_sec * US_PER_SEC + sys_tv.tv_usec;
    int32_t difference_usec = rtc_usec - sys_usec;
    printf("RTC differs from system time by %s%lu.%06lu seconds\n",
           difference_usec > 0 ? "+" : "-",
           labs(difference_usec) / US_PER_SEC,
           labs(difference_usec) % US_PER_SEC);
#endif

    return 0;
}
