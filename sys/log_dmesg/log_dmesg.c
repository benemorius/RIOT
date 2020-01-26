#include <stdio.h>
#include <stdarg.h>
#include <string.h>
#include <stdlib.h>
#include <sys/time.h>

#include "ringbuffer.h"
#include "mutex.h"
#include "xtimer.h"
#include "log.h"

int gettimeofday(struct timeval *tp, void *tzp);

static char dmesg_buf_data[1024 * 32];
static ringbuffer_t dmesg_buf = RINGBUFFER_INIT(dmesg_buf_data);
static mutex_t dmesg_buf_lock = MUTEX_INIT;
FILE *dmesg_out;
static bool log_enabled = false;
static char _filter_string[1024] = {" \
    4-kw41zrf_xcvr.c \
    4-kw41zrf_getset.c \
    4-kw41zrf.c \
    4-gnrc_tcp_rcvbuf.c \
    4-auto_init_kw41zrf.c \
    4-auto_init.c \
    4-fib.c \
    4-nib.c \
    4-gnrc_ndp.c \
    4-_nib-internal.c \
    4-_nib-slaac.c \
    4-_nib-6ln.c \
    4-gnrc_netif.c \
    4-gnrc_icmpv6.c \
    4-gnrc_ipv6_hdr.c \
    4-gnrc_uhcpc.c \
    4-auto_init_adc.c \
    4-auto_init_gpio.c \
    4-gnrc_tcp_fsm.c \
    4-memarray.c \
    4-mbox.c \
    4-vfs.c \
    4-rbuf.c \
    4-inet_csum.c \
    4-gnrc_sixlowpan.c \
    4-gnrc_pktbuf_static.c \
    4-gnrc_netif_ethernet.c \
    4-gnrc_sixlowpan_iphc.c \
    4-gnrc_netif_ieee802154.c \
    4-uhcp.c \
    4-gnrc_udp.c \
    4-_nib-arsm.c \
    4-zb_bufpool.c \
    4-zb_uz2400.c \
    4-zb_debug.c \
    4-mac_common.c \
"};
//     3-kw41zrf_getset.c
// 4-kw41zrf_netdev.c

void log_dmesg_enable(void) {
    log_enabled = true;
}

void log_dmesg_disable(void) {
    log_enabled = false;
}

bool _filter_message(int level, const char *file, const char *function, unsigned line)
{
    (void)function;
    (void)line;
    char *_filter_string_p = _filter_string;

    if (level >= LOG_DEBUG) {
//         return true;
    }

    while (*_filter_string_p) {
        int filter_level = atoi(_filter_string_p);
        if (filter_level == 0) { /* didn't find an int */
            break;
        }
        char *plus_or_minus = strchr(_filter_string_p, '-');
        if (plus_or_minus != _filter_string_p) {
            _filter_string_p = plus_or_minus + 1;
        }
        else if ((plus_or_minus = strchr(_filter_string_p, '+')) != _filter_string_p) {
            _filter_string_p = plus_or_minus + 1;
        }
        else { /* didn't find '+' or '-' */
            break;
        }

        if ((strncmp(file, _filter_string_p, strlen(file)) == 0)
            && ( _filter_string_p[strlen(file)] == ' '
            || _filter_string_p[strlen(file)] == '\0'))
        { /* filename matches */
            if (*plus_or_minus == '-') {
                if (filter_level == level) {
                }
                    return true;
            }
        }

        char *next_entry = strchr(_filter_string_p, ' ');
        if (next_entry == NULL) {
            return false;
        }

        _filter_string_p = next_entry + 1;
    }
    return false;
}

const char *dmesg_get_filter_string(void)
{
    return _filter_string;
}

void dmesg_set_filter_string(const char *filter_string)
{
    strncpy(_filter_string, filter_string, sizeof(_filter_string));
    printf("set filter string to \"%s\"\n", filter_string);
}

void vlog_dmesg(unsigned level, const char *file, const char *function, unsigned line, const char *format, va_list args)
{
//     return;
//     uint32_t start = 0;
    if (dmesg_out) {
//         start = xtimer_now_usec();
    }

    if (!log_enabled  || irq_is_in()) {
        return;
    }
    // if (irq_is_in()) {
    //     return;
    // }

    if (level > LOG_INFO) {
//         return;
    }

    if (_filter_message(level, file, function, line)) {
        if (dmesg_out) {
//             printf("return after %lu\n", xtimer_now_usec() - start);
        }
        return;
    }

    mutex_lock(&dmesg_buf_lock);
    static char buf[512];
    // vprintf(format, args);
    vsnprintf(buf, sizeof(buf), format, args);

    struct timeval tv;
    gettimeofday(&tv, NULL);
    tv.tv_sec += (-7 * 3600); /* timezone offset */
    struct tm tm = *gmtime(&tv.tv_sec);
    if (0) {
        (void) tm.tm_sec;
    }
    char time[32];
    // strftime(time, sizeof(time), "%Y-%m-%d %H:%M:%S", &tm);
    snprintf(time, sizeof(time), "%04i-%02i-%02i %02i:%02i:%02i.%06lu",
           tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday,
           tm.tm_hour, tm.tm_min, tm.tm_sec, tv.tv_usec);

    char prefix[128];
    timex_t now;
    xtimer_now_timex(&now);
    snprintf(prefix, sizeof(prefix), "<%u><%u>[%lu.%06lu][%s] %s:%u %s() ",
             level, thread_getpid(), now.seconds, now.microseconds, time,
             file, line, function
    );

    char *c = prefix;
    while (*c) {
        ringbuffer_add_one(&dmesg_buf, *c++);
    }

    c = buf;
    while (*c) {
        ringbuffer_add_one(&dmesg_buf, *c++);
    }

    if (dmesg_out && !irq_is_in()) {
        fputs(prefix, dmesg_out);
        fputs(buf, dmesg_out);
        fflush(dmesg_out);
    }

    if (*(c - 1) != '\n') {
        ringbuffer_add_one(&dmesg_buf, '\n');
        if (dmesg_out) {
            fputc('\n', dmesg_out);
        }
    }

    mutex_unlock(&dmesg_buf_lock);
}

void log_dmesg(unsigned level, const char *file, const char *function, unsigned line, const char *format, ...)
{
    va_list args;
    va_start(args, format);
    vlog_dmesg(level, file, function, line, format, args);
    va_end(args);
}

size_t dmesg_peek(char *buffer, size_t buffer_length, size_t offset)
{
    mutex_lock(&dmesg_buf_lock);
    ringbuffer_t rb = dmesg_buf;
    ringbuffer_remove(&rb, offset);
    size_t bytes_read = ringbuffer_get(&rb, buffer, buffer_length);
    mutex_unlock(&dmesg_buf_lock);
    return bytes_read;
}

size_t dmesg_read(char *buffer, size_t buffer_length)
{
    mutex_lock(&dmesg_buf_lock);
    size_t bytes_read = ringbuffer_get(&dmesg_buf, buffer, buffer_length);
    mutex_unlock(&dmesg_buf_lock);
    return bytes_read;
}

size_t dmesg_free(void)
{
    mutex_lock(&dmesg_buf_lock);
	size_t bytes_free = ringbuffer_get_free(&dmesg_buf);
    mutex_unlock(&dmesg_buf_lock);
    return bytes_free;
}

size_t dmesg_total(void)
{
    return sizeof(dmesg_buf_data);
}

size_t dmesg_used(void)
{
    return dmesg_total() - dmesg_free();
}

void dmesg_clear(void)
{
    mutex_lock(&dmesg_buf_lock);
    ringbuffer_init(&dmesg_buf, dmesg_buf_data, sizeof(dmesg_buf_data));
    mutex_unlock(&dmesg_buf_lock);
}

void dmesg_attach(FILE *file)
{
    dmesg_out = file;
}
