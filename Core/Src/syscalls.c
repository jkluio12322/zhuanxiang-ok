/* Minimal syscall implementations for newlib-nano on STM32F103 */

#include <sys/stat.h>
#include <sys/types.h>
#include <errno.h>

int _close(int fd) {
    (void)fd;
    errno = EBADF;
    return -1;
}

int _fstat(int fd, struct stat *buf) {
    (void)fd;
    (void)buf;
    errno = EBADF;
    return -1;
}

int _isatty(int fd) {
    (void)fd;
    return 1;
}

int _lseek(int fd, int ptr, int dir) {
    (void)fd;
    (void)ptr;
    (void)dir;
    errno = EBADF;
    return -1;
}

int _read(int fd, char *ptr, int len) {
    (void)fd;
    (void)ptr;
    (void)len;
    errno = EBADF;
    return -1;
}

int _write(int fd, char *ptr, int len) {
    (void)fd;
    (void)ptr;
    return len;
}

void *_sbrk(int incr) {
    extern char _end;
    static char *heap_end = (char *)&_end;
    char *prev = heap_end;
    heap_end += incr;
    return (void *)prev;
}

int _getpid(void) {
    return 1;
}

int _kill(int pid, int sig) {
    (void)pid;
    (void)sig;
    errno = EINVAL;
    return -1;
}
