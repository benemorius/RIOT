/*
 * Copyright (C) 2016 Kaspar Schleiser <kaspar@schleiser.de>
 *               2013 Freie Universität Berlin
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup     core_internal
 * @{
 *
 * @file
 * @brief       Platform-independent kernel initialization
 *
 * @author      Kaspar Schleiser <kaspar@schleiser.de>
 *
 * @}
 */

#include <stdint.h>
#include <stdbool.h>
#include <errno.h>
#include "kernel_init.h"
#include "thread.h"
#include "irq.h"
#include "log.h"

#include "periph/pm.h"
#include "periph/rtc.h"

#include <time.h>
#include <sys/time.h>
#include "xtimer.h"

#define ENABLE_DEBUG (0)
#include "debug.h"

#ifdef MODULE_AUTO_INIT
#include <auto_init.h>
#endif

extern int main(void);

#ifdef MODULE_PERIPH_RTC
extern int settimeofday(const struct timeval *restrict tp, const void *restrict tzp);

static void set_system_time_from_rtc(void)
{
    struct tm tm;
    if (rtc_get_time(&tm) == 0) {
        /* wait for rtc tick */
        int current_tm_sec = tm.tm_sec;
        while (tm.tm_sec == current_tm_sec) {
            if (rtc_get_time(&tm) != 0) {
                LOG_ERROR("[kernel_init] error getting time from RTC\n");
                return;
            }
        }

        time_t time = mktime(&tm);
        const struct timeval tv = {time, XTIMER_BACKOFF * 6};
        settimeofday(&tv, NULL);
        LOG_INFO("[kernel_init] system time set from RTC\n");
    }
    else {
        LOG_ERROR("[kernel_init] error getting time from RTC\n");
    }
}
#endif

static void *main_trampoline(void *arg)
{
    (void) arg;

#ifdef MODULE_AUTO_INIT
    auto_init();
#endif

#ifdef MODULE_PERIPH_RTC
    set_system_time_from_rtc();
#endif

    LOG_INFO("main(): This is RIOT! (Version: " RIOT_VERSION ")\n");

    main();
    return NULL;
}

static void *idle_thread(void *arg)
{
    (void) arg;

    while (1) {
        pm_set_lowest();
    }

    return NULL;
}

const char *main_name = "main";
const char *idle_name = "idle";

static char main_stack[THREAD_STACKSIZE_MAIN];
static char idle_stack[THREAD_STACKSIZE_IDLE];

void kernel_init(void)
{
    (void) irq_disable();

    thread_create(idle_stack, sizeof(idle_stack),
            THREAD_PRIORITY_IDLE,
            THREAD_CREATE_WOUT_YIELD | THREAD_CREATE_STACKTEST,
            idle_thread, NULL, idle_name);

    thread_create(main_stack, sizeof(main_stack),
            THREAD_PRIORITY_MAIN,
            THREAD_CREATE_WOUT_YIELD | THREAD_CREATE_STACKTEST,
            main_trampoline, NULL, main_name);

    cpu_switch_context_exit();
}
