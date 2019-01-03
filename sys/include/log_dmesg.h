#include <sys/types.h>

void log_dmesg(unsigned level, const char *format, ...);
size_t dmesg_peek(char *buffer, size_t buffer_length, size_t offset);
size_t dmesg_read(char *buffer, size_t buffer_length);
size_t dmesg_free(void);
size_t dmesg_total(void);
size_t dmesg_used(void);
void dmesg_clear(void);
