#include <sockets.h>
#include <stdio.h>
#include <stdarg.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <malloc.h>
#include <file.h>
#include <stat.h>
#include <dirent.h>
#include <dir.h>
#include <syscall.h>

#include <ftpvars.h>
#include <ftputils.h>
#include <ftplog.h>

// this file contains some helper  functions for the ftp client and server

/**
 * new_addr, construct a sockaddr struct according to addr and port
 * @param(int) inaddr, address number
 * @param(short) port, the port number
 * @note the inaddr and port should be host-endian rather than network endian
 */
struct sockaddr new_addr(uint32_t inaddr, unsigned short port) {
    struct sockaddr_in addr;
    memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = htonl(inaddr);
    addr.sin_port = htons(port);
    return *(struct sockaddr *)&addr;
}

/**
 * new_server, construt a server socket from the addr, port and backlog
 * @param(int) inaddr, the address server will be bound to
 * @param(short) port , the port server will be bound to 
 * @return(int) the socket fd on success, ERROR number on error
 */
int new_server(uint32_t inaddr, uint16_t port, int backlog) {
    int server;
    server = socket(AF_INET, SOCK_STREAM, 0);
    struct sockaddr addr = new_addr(inaddr, port);
    if (bind(server, &addr, sizeof(addr)) < 0) {
        return -2;
    }
    if (listen(server, backlog) < 0) {
        return -3;
    }
    return server;
}

/**
 * new client, construct a client socket connected to the server
 * @param(int), srv_addr, server address
 * @param(short), server port
 * @return {int} status, -2 create socket ftperror, -1 connect ftperror
 */
int new_client(uint32_t srv_addr, unsigned short port) {
    int client = socket(AF_INET, SOCK_STREAM, 0);
    if (client < 0) return -2;
    struct sockaddr server = new_addr(srv_addr, port);
    int st = connect(client, &server, sizeof(server));
    if (st < 0) return -1;
    return client;
}

/**
 * send string to the socket peer. It supports fmt print like printf
 * this is very convenient for use
 * @param(int) peer socket
 * @fmt, fmt is the same to printf
 */
int send_str(int peer, const char* fmt, ...) {
    va_list args;
    char msgbuf[BUF_SIZE];
    va_start(args, fmt);
    vsnprintf(msgbuf, sizeof(msgbuf), fmt, args);
    va_end(args);
    return send(peer, msgbuf, strlen(msgbuf), 0);
}

/**
 * send a file to the socket peer by a file fd
 * @param(int) peer, the peer socket
 * @param(int) f, the file fd
 * @return -1 ftperror, 0 ok
 */
int send_file(int peer, int f) {
    char filebuf[BUF_SIZE+1];
    int n, ret = 0;
    while ((n = read(f, filebuf, BUF_SIZE)) > 0) {
        int st = send(peer, filebuf, n, 0);
        if (st < 0) {
            ret = -1;
            break;
        } else {
            filebuf[n] = 0;
            ftpinfo(1, " %d bytes sent", st);
        }
    }
    return ret;
}

/**
 * send a file by the file path to the socket peer
 * @param(int) peer, the socket peer
 * @param(char *) the file path
 * @param(int) offset, the file offset
 *  -1 ftperror opening file, -2 send file ftperror, -3 close file ftperror
 */
int send_path(int peer, char *file, uint32_t offset) {
    int f = open(file, O_RDONLY);
    if (f) {
        seek(f, offset, LSEEK_SET);
        int st = send_file(peer, f);
        if (st < 0) {
            return -2;
        }
    } else {
        return -1;
    }
    int ret = close(f);
    return ret == 0 ? 0 : -3;
}

/**
 *  recv a file by fd from the peer
 * @param(int) peer, the socket peer
 * @param(int) f, the file fd
 */
int recv_file(int peer, int f) {
    char filebuf[BUF_SIZE];
    int n;
    while ((n=recv(peer, filebuf, BUF_SIZE, 0)) > 0) {
        write(f, filebuf, n);
    }
    return n;
}

/**
 * recv file by file path
 * @param {int} peer, peer socket
 * @param {char *} file path
 * @param {int} offset
 * @return {int} status, 
 *              -1 means recv_file ftperror, 
 *              -2 means file open failure, 
 *              EOF means close file ftperror
 * 
 */
int recv_path(int peer, char *file, uint32_t offset) {
    int f = open(file, O_WRONLY | O_CREAT);
    if (f < 0) {
        ftperr(1, "recv path open file error: %d", f);
        return -2;
    }
    seek(f, offset, LSEEK_SET);
    int st = recv_file(peer, f);
    int cl = close(f);
    return st < 0 ? st : cl;
}

/**
 * parse number from the string sent to the ftp server/client
 * @param(char *) buf, the string buffer
 * @param(int *) the number to be parsed and stored
 */
int parse_number(const char *buf, uint32_t *number) {
    int f = -1, i;
    char tmp[BUF_SIZE] = {0};
    int ret = -1;
    for (i=0; buf[i]!=0 && i<BUF_SIZE; i++) {
        if (!(buf[i] >= '0' && buf[i] <= '9')) {
            if (f >= 0) {
                memcpy(tmp, &buf[f], i-f);
                tmp[i-f] = 0;
                *number = atoi(tmp);
                ret = 0;
                f = -1;
                break;
            }
        } else {
            if (f < 0) {
                f = i;
            }
        }
    }
    return ret;
}

/**
 * parse address and port from the string sent to the ftp server/client
 * this function is for FTP cmd like PASV
 * @param(char *) buf, the string buffer
 * @param(int *) the number to be parsed and stored
 */
int parse_addr_port(const char *buf, uint32_t *addr, uint16_t *port) {
    int i;
    *addr = *port = 0;
    int f = -1;
    char tmp[BUF_SIZE] = {0};
    int cnt = 0;
    int portcnt = 0;
    for(i=0; buf[i]!=0 && i<BUF_SIZE; i++) {
        if(!(buf[i] >= '0' && buf[i] <= '9')) {
            if (f >= 0) {
                memcpy(tmp, &buf[f], i-f);
                tmp[i-f] = 0;
                if (cnt < 4) {
                    *addr = (*addr << 8) + (0xff & atoi(tmp));
                    cnt++;
                } else if (portcnt < 2) {
                    *port = (*port << 8) + (0xff & atoi(tmp));
                    portcnt++;
                } else {
                    break;
                }
                f = -1;
            }
        } else {
            if (f < 0) {
                f = i;
            }
        }
    }
    return cnt == 4 && portcnt == 2;
}


/**
 * parse the file path from the string sent to the ftp server/client
 * this function is for FTP cmd like STOR
 * @param(char *) buf, the string buffer
 * @return(char *) the path to be parsed and stored
 */
char * parse_path(const char *buf) {
    char * path = (char *)malloc(BUF_SIZE);
    int i, j;
    for (i=0; buf[i]!=' ' && i < BUF_SIZE; i++);
    if (i == BUF_SIZE) return NULL;
    i++;
    for (j=i; buf[j]!='\r' && buf[j]!= '\n' && j < BUF_SIZE; j++);
    memcpy(path, &buf[i], j-i);
    path[j-i] = 0;
    return path;
}

/**
 * combination of htonl and inet_ntoa
 * it's just a shortcut function
 */
char * n2a(uint32_t addr) {
    uint32_t t = htonl(addr);
    return inet_ntoa(*(struct in_addr *)&t);
}

// for ls
// the 3 functions below are used for FTP LS/LIST command 
// the correct way to use it is to write the result to a file and 
// then read it 
// the code is borrowed from ls.c
static char
ftp_getmode(uint32_t st_mode) {
    char mode = '?';
    if (S_ISREG(st_mode)) mode = '-';
    if (S_ISDIR(st_mode)) mode = 'd';
    if (S_ISLNK(st_mode)) mode = 'l';
    if (S_ISCHR(st_mode)) mode = 'c';
    if (S_ISBLK(st_mode)) mode = 'b';
    return mode;
}

static int
ftp_getstat(const char *name, struct stat *stat) {
    int fd, ret;
    if ((fd = open(name, O_RDONLY)) < 0) {
        return fd;
    }
    ret = fstat(fd, stat);
    close(fd);
    return ret;
}

void
ftp_lsstat(struct stat *stat, const char *filename, int fd) {
    fprintf(fd, "   [%c]", ftp_getmode(stat->st_mode));
    fprintf(fd, " %3d(h)", stat->st_nlinks);
    fprintf(fd, " %8d(b)", stat->st_blocks);
    fprintf(fd, " %8d(s)", stat->st_size);
    fprintf(fd, "   %s\n", filename);
}

int
ftp_lsdir(const char *path, int fd) {
    struct stat __stat, *stat = &__stat;
    static char cwdbuf[BUF_SIZE];
    int ret;
    if ((ret = getcwd(cwdbuf, BUF_SIZE)) != 0) {
        return ret;
    }
    if ((ret = chdir(path)) != 0) {
        return ret;
    }
    DIR *dirp = opendir(".");
    if (dirp == NULL) {
        return -1;
    }
    struct dirent *direntp;
    while ((direntp = readdir(dirp)) != NULL) {
        if ((ret = ftp_getstat(direntp->name, stat)) != 0) {
            goto failed;
        }
        ftp_lsstat(stat, direntp->name, fd);
    }
    closedir(dirp);
    return chdir(cwdbuf);

failed:
    closedir(dirp);
    chdir(cwdbuf);
    return ret;
}

int
ftp_ls(const char *path, int fd) {
    struct stat __stat, *stat = &__stat;
    int ret, type;
    if ((ret = ftp_getstat(path, stat)) != 0) {
        return ret;
    }

    static const char *filetype[] = {
        " [  file   ]",
        " [directory]",
        " [ symlink ]",
        " [character]",
        " [  block  ]",
        " [  ?????  ]",
    };
    switch (ftp_getmode(stat->st_mode)) {
    case '0': type = 0; break;
    case 'd': type = 1; break;
    case 'l': type = 2; break;
    case 'c': type = 3; break;
    case 'b': type = 4; break;
    default:  type = 5; break;
    }

    fprintf(fd, " @ is %s", filetype[type]);
    fprintf(fd, " %d(hlinks)", stat->st_nlinks);
    fprintf(fd, " %d(blocks)", stat->st_blocks);
    fprintf(fd, " %d(bytes) : @'%s'\n", stat->st_size, path);
    if (S_ISDIR(stat->st_mode)) {
        return ftp_lsdir(path, fd);
    }
    return 0;
}

size_t time() {
    return sys_gettime();
}

