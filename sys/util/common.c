/*
 * Copyright (C) 2019 Thomas Stilwell <stilwellt@openlabs.co>
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

#include "thread.h"
#include "xtimer.h"
#include "board.h"
#include "mutex.h"
#include "util.h"
// #include "wdog.h"
#include "periph/adc.h"

static char _heartbeat_thread_stack [THREAD_STACKSIZE_SMALL];
static mutex_t _pause = MUTEX_INIT;

#ifdef MODULE_TELNET_SERVER
#include "net/telnet_server.h"
static telnet_server_t telnet_server = {.port = 23};
#endif

#if defined MODULE_SNTP || defined DOXYGEN
static char _ntp_thread_stack [THREAD_STACKSIZE_SMALL * 2];
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

static void *_heartbeat_thread(void *arg)
{
    (void) arg;
    while(1) {
        mutex_lock(&_pause);
        mutex_unlock(&_pause);

        // wdog_feed();

        LED0_ON;
        xtimer_usleep(1000*80);
        LED0_OFF;
        xtimer_usleep(1000*50);

        LED0_ON;
        xtimer_usleep(1000*80);
        LED0_OFF;
        xtimer_usleep(1000*50);

        xtimer_usleep(1000*1740);
    }
    return NULL;
}

void heartbeat_start(void)
{
    thread_create(_heartbeat_thread_stack, sizeof(_heartbeat_thread_stack), 7,
                  THREAD_CREATE_STACKTEST | THREAD_CREATE_WOUT_YIELD,
                  _heartbeat_thread, NULL, "heartbeat");
    return;
}

void heartbeat_pause(void)
{
    mutex_lock(&_pause);

}

void heartbeat_resume(void)
{
    mutex_unlock(&_pause);

}



void auto_init_util(void)
{

#ifdef MODULE_SHELL
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

#ifdef MODULE_SNTP
    ntp_start();
#endif

#ifdef UTIL_OTA_SERVER
    ota_start_server();
#endif

#ifdef MODULE_PERIPH_ADC
    adc_init(ADC_TEMPERATURE_CHANNEL);
#endif

}
