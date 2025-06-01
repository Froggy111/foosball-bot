#include <stdio.h>
#include <sys/errno.h>
#include <sys/fcntl.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <sys/times.h>
#include <sys/types.h>
#include <unistd.h>

#include "config.hpp"
#include "usb.hpp"

extern "C" {

#if STDIO_TARGET == STDIO_USB

#ifdef __GNUC__

int _write(int file, char *ptr, int len) {
    if (file != 1 && file != 2) {
        return -1;
    }

    usb::write((uint8_t *)ptr, len);
    return len;
}

#else

int fputc(int ch, FILE *f) {
    usb::write((uint8_t *)&ch, 1);
    return ch;
}

#endif
#else

int _write([[maybe_unused]] int file, [[maybe_unused]] char *ptr,
           [[maybe_unused]] int len) {
    return -1;
}

#endif

int _close([[maybe_unused]] int file) { return -1; }

int _fstat([[maybe_unused]] int file, [[maybe_unused]] struct stat *st) {
    if (file == STDOUT_FILENO || file == STDERR_FILENO ||
        file == STDIN_FILENO) {
        st->st_mode = S_IFCHR;  // Character device
        st->st_blksize = 0;
        return 0;
    }
    return -1;
}
int _getpid(void) { return 1; }

int _isatty([[maybe_unused]] int file) {
    if (file == STDOUT_FILENO || file == STDERR_FILENO ||
        file == STDIN_FILENO) {
        return 1;
    }
    errno = ENOSYS;
    return 0;
}

int _kill([[maybe_unused]] int pid, [[maybe_unused]] int sig) { return -1; }

off_t _lseek([[maybe_unused]] int file, [[maybe_unused]] off_t ptr,
             [[maybe_unused]] int dir) {
    return (off_t)-1;
}

int _read([[maybe_unused]] int file, [[maybe_unused]] char *ptr,
          [[maybe_unused]] int len) {
    return -1;
}
}
