/**
 * print server by zTrix, create server socket and print all of what it heard
 * for ftp debug
 */

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
// #include <signal.h>
#include <types.h>
#include <sockets.h>
#include <ulib.h>
#include <malloc.h>

#include <ftputils.h>

#define BUF_SIZE1 10
#define BUF_SIZE2 1024
int server = -1;
int client = -1;

int main(int argc, char *argv[]) {
    if (argc < 2) {
        cprintf("usage: ./print_server <port>\n");
        exit(0);
    }
    int port = atoi(argv[1]);
    struct sockaddr_in client_addr;
    uint32_t len = sizeof(client_addr);
    server = new_server(INADDR_ANY, port, 1);
    int n;
    client = accept(server, (struct sockaddr *)&client_addr, &len);
    cprintf("[ II ] client accpeted, addr is %s: %u \n", inet_ntoa(*(struct in_addr *)&client_addr.sin_addr.s_addr), ntohs(client_addr.sin_port));

    struct msghdr msg;
    msg.msg_name = NULL;
    msg.msg_namelen = 0;
    struct iovec iov[2];
    iov[0].iov_base = (char *)malloc(BUF_SIZE1);
    iov[0].iov_len = BUF_SIZE1 - 1;
    iov[1].iov_base = (char *)malloc(BUF_SIZE2);
    iov[1].iov_len = BUF_SIZE2 - 1;
    msg.msg_iov = iov;
    msg.msg_iovlen = 2;

    while ((n = recvmsg(client, &msg, 0)) > 0) {
        if (n > BUF_SIZE1 - 1) {
            msg.msg_iov[0].iov_base[BUF_SIZE1-1] = 0;
            cprintf("%s", msg.msg_iov[0].iov_base);
            msg.msg_iov[1].iov_base[n - BUF_SIZE1 + 1] = 0;
            cprintf("%s", msg.msg_iov[1].iov_base);
        } else {
            msg.msg_iov[0].iov_base[n] = 0;
            cprintf("%s", msg.msg_iov[0].iov_base);
        }
    }
    cprintf("[ II ] connection closed\n");
    return 0;
}
