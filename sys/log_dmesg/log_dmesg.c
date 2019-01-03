#include <stdio.h>
#include <stdarg.h>
#include <string.h>

#include "ringbuffer.h"
#include "mutex.h"
#include "xtimer.h"

static char dmesg_buf_data[8192];
static ringbuffer_t dmesg_buf = RINGBUFFER_INIT(dmesg_buf_data);
static mutex_t dmesg_buf_lock = MUTEX_INIT;

void log_dmesg(unsigned level, const char *format, ...)
{
    mutex_lock(&dmesg_buf_lock);
    static char buf[512];
    va_list args;
    va_start(args, format);
    // vprintf(format, args);
    vsnprintf(buf, sizeof(buf), format, args);
    va_end(args);
    char prefix[16];
    timex_t now;
    xtimer_now_timex(&now);
    snprintf(prefix, sizeof(prefix), "<%u>[%lu.%06lu] ", level, now.seconds, now.microseconds);
    ringbuffer_add(&dmesg_buf, prefix, strlen(prefix));
    ringbuffer_add(&dmesg_buf, buf, strlen(buf));
    mutex_unlock(&dmesg_buf_lock);
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
