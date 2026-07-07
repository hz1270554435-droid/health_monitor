#include <stddef.h>
#include <sys/reent.h>

#if defined(APP_CM55_STDOUT_NULL_ENABLE) && (APP_CM55_STDOUT_NULL_ENABLE == 1)

int __wrap__write(int fd, const void *buf, size_t len)
{
    (void)fd;
    (void)buf;
    return (int)len;
}

int __wrap__write_r(struct _reent *r, int fd, const void *buf, size_t len)
{
    (void)r;
    return __wrap__write(fd, buf, len);
}

#endif
