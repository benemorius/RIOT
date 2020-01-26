#include <sys/types.h>
#include <stdarg.h>
#include <stdio.h>

void log_dmesg(unsigned level, const char *file, const char *function, unsigned line, const char *format, ...);
void vlog_dmesg(unsigned level, const char *file, const char *function, unsigned line, const char *format, va_list args);
size_t dmesg_peek(char *buffer, size_t buffer_length, size_t offset);
size_t dmesg_read(char *buffer, size_t buffer_length);
size_t dmesg_free(void);
size_t dmesg_total(void);
size_t dmesg_used(void);
void dmesg_clear(void);
void dmesg_attach(FILE *file);
void log_dmesg_enable(void);
void log_dmesg_disable(void);
void dmesg_set_filter_string(const char *filter_string);
const char *dmesg_get_filter_string(void);
