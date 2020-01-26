/*
 * Copyright (C) 2019 Thomas Stilwell <stilwellt@openlabs.co>
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

#include <sys/time.h>
#include <stdlib.h>

#include "thread.h"
#include "xtimer.h"
#include "board.h"
#include "mutex.h"
#include "util.h"
#include "periph/adc.h"

#ifdef MODULE_PERIPH_RTC
#include "periph/rtc.h"
#endif

#ifdef MODULE_TELNET_SERVER
#include "net/telnet_server.h"
static telnet_server_t telnet_server = {.port = 23};
#endif

#ifdef MODULE_SNTP
static char _ntp_thread_stack [THREAD_STACKSIZE_DEFAULT + 512];
static void *_ntp_thread(void *arg)
{
    /* TODO maybe do RTC temperature compensation here */

    (void)arg;

    /* wait for RA then try NTP on the router */
    xtimer_sleep(5);
    uint32_t delay_sec = 10;
    while (1) {
        if (ntpdate() == 0) {
            delay_sec = SEC_PER_MIN * 60;
        }
        xtimer_sleep(delay_sec);
    }
    return NULL;
}

void ntp_start(void)
{
    thread_create(_ntp_thread_stack, sizeof(_ntp_thread_stack), 7,
                  THREAD_CREATE_STACKTEST | THREAD_CREATE_WOUT_YIELD,
                  _ntp_thread, NULL, "ntp client");
    return;
}
#endif

#if defined(MODULE_PERIPH_RTC) && !defined(BOARD_NATIVE)
void rtc_cb(void *arg)
{
    int32_t old_difference_usec = (int32_t)arg;
    static uint32_t counter = 0;

    /* compare rtc to system time */
    struct tm rtc_tm;
    rtc_get_time(&rtc_tm);

    struct timeval sys_tv;
    gettimeofday(&sys_tv, NULL);

    struct timeval rtc_tv = {mktime(&rtc_tm), 0};
    int64_t rtc_usec = rtc_tv.tv_sec * US_PER_SEC + rtc_tv.tv_usec;
    int64_t sys_usec = sys_tv.tv_sec * US_PER_SEC + sys_tv.tv_usec;
    int32_t difference_usec = sys_usec - rtc_usec - old_difference_usec;
    int32_t total_difference_usec = old_difference_usec + difference_usec;

    if (labs(difference_usec) > 70) {
        counter++;
        printf("%" PRIu32 " xtimer error: %s%" PRIu32 ".%06" PRIu32 " seconds (total %s%" PRIu32 ".%06" PRIu32 ")\n",
               counter,
               difference_usec > 0 ? "+" : "-",
               labs(difference_usec) / US_PER_SEC,
               labs(difference_usec) % US_PER_SEC,
               total_difference_usec > 0 ? "+" : "-",
               labs(total_difference_usec) / US_PER_SEC,
               labs(total_difference_usec) % US_PER_SEC
        );
        old_difference_usec = total_difference_usec;
    }

    rtc_get_time(&rtc_tm);
    time_t time = mktime(&rtc_tm);
    time += 10;
    rtc_set_alarm(gmtime(&time), rtc_cb, (void *)old_difference_usec);
}

void heartbeat_start(void)
{
    struct tm rtc_tm;
    rtc_get_time(&rtc_tm);
    time_t time = mktime(&rtc_tm);
    time += 3; /* wait for kernel to set system time from rtc */
    rtc_set_alarm(gmtime(&time), rtc_cb, (void *)0);

    return;
}

#endif


void auto_init_util(void)
{

#ifdef MODULE_UTIL_SHELL
#ifdef UTIL_EXTRA_SHELL_COMMANDS
    extern shell_command_t UTIL_EXTRA_SHELL_COMMANDS[];
    shell_start(UTIL_EXTRA_SHELL_COMMANDS);
#else
    shell_start(NULL);
#endif
#endif

#ifdef MODULE_TELNET_SERVER
    telnet_start_server(&telnet_server);
#endif

#ifdef MODULE_UTIL_TICK_MONITOR
    heartbeat_start();
#endif

#ifdef MODULE_UTIL_NTP
    ntp_start();
#endif

#ifdef MODULE_UTIL_OTA_SERVER
    ota_start_server();
#endif

#if defined(MODULE_PERIPH_ADC) && !defined(BOARD_NATIVE)
    adc_init(ADC_TEMPERATURE_CHANNEL);
#endif

}
