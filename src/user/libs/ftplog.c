#include <stdio.h>
#include <stdarg.h>
#include <unistd.h>
#include <ulib.h>

#define FTP_SESSION "SESSION"
#define FTP_DAEMON  "DAEMON "

// this file contains 3 simple log tool for the ftp

/**
 * ftperr, output error messages
 * @param(int) type, 0 for the daemon and 1 for the connection client
 * @param(char *) fmt, the same to printf
 */
void ftperr(int type, const char *fmt, ...) {
    cprintf("[ EE ] [ %s %d ]: ", type ? FTP_SESSION : FTP_DAEMON, getpid());
    va_list args;
    va_start(args, fmt);
    vcprintf(fmt, args);
    va_end(args);
    cprintf("\n");
}

/**
 * ftpwarn, output warning messages 
 * @param(int) type, 0 for the daemon and 1 for the connection client
 * @param(char *) fmt, the same to printf
 */
void ftpwarn(int type, const char *fmt, ...) {
    cprintf("[ WW ] [ %s %d ]: ", type ? FTP_SESSION : FTP_DAEMON, getpid());
    va_list args;
    va_start(args, fmt);
    vcprintf(fmt, args);
    va_end(args);
    cprintf("\n");
}

/**
 * ftpinfo, output information messages 
 * @param(int) type, 0 for the daemon and 1 for the connection client
 * @param(char *) fmt, the same to printf
 */
void ftpinfo(int type, const char *fmt, ...) {
    cprintf("[ II ] [ %s %d ]: ", type ? FTP_SESSION : FTP_DAEMON, getpid());
    va_list args;
    va_start(args, fmt);
    vcprintf(fmt, args);
    va_end(args);
    cprintf("\n");
}

