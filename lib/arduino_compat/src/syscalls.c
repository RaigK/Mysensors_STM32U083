/*
 * Minimal newlib syscalls. nosys.specs should cover these via weak stubs,
 * but our bundled toolchain (7.2.1) doesn't always pick them up under
 * -nostartfiles. Provide strong no-ops.
 *
 * We do not use malloc / printf-to-file / file I/O. All these return failure.
 */

#include <errno.h>
#include <sys/stat.h>
#include <sys/types.h>

#undef errno
extern int errno;

/* Heap grows up from end-of-bss. Linker script defines _end. We give malloc
 * a small heap so libc bring-up doesn't explode; real code shouldn't use it. */
extern char _end;
extern char _estack;

void* _sbrk(ptrdiff_t incr)
{
    static char* heap = 0;
    if (heap == 0) heap = &_end;
    /* Cap heap at stack top minus 1 KB to leave room for stack. */
    char* limit = &_estack - 1024;
    if (heap + incr > limit) { errno = ENOMEM; return (void*)-1; }
    char* prev = heap;
    heap += incr;
    return prev;
}

int _write(int fd, const void* buf, size_t len) { (void)fd; (void)buf; return len; }
int _read (int fd, void* buf, size_t len)       { (void)fd; (void)buf; (void)len; return 0; }
int _close(int fd)                              { (void)fd; return -1; }
int _lseek(int fd, off_t pos, int whence)       { (void)fd; (void)pos; (void)whence; return 0; }
int _fstat(int fd, struct stat* st)             { (void)fd; st->st_mode = S_IFCHR; return 0; }
int _isatty(int fd)                             { (void)fd; return 1; }
int _getpid(void)                               { return 1; }
int _kill(int pid, int sig)                     { (void)pid; (void)sig; errno = EINVAL; return -1; }
void _exit(int status)                          { (void)status; while (1) { } }
