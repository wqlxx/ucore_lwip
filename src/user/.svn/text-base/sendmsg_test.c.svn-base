#include <stdlib.h>
#include <ulib.h>
#include <sockets.h>
#include <stdio.h>

#include "ftputils.h"
#include "ftplog.h"

int main(int argc, char * argv[]) {
    int server_port = 2121;
    if (argc < 2) {
        cprintf("usage: %s <addr> [2121]\n", argv[0]);
        exit(0);
    }
    if (argc >= 3) {
        server_port = atoi(argv[2]);
    }
    int client = new_client(ntohl(inet_addr(argv[1])), server_port);
    if (client < 0) {
        ftperr(1, "cannot connect to %s %d, ret code is %d", argv[1], server_port, client);
        ftpinfo(1, "exiting ...");
        exit(1);
    }
    char send_buf[]  = "!!!! from ucore sendmsg test buf 1\n";
    char send_buf2[] = "!!!! from ucore sendmsg test buf 2\n";
    struct msghdr msg;
    struct iovec iov[2];
    msg.msg_name = NULL;
    msg.msg_namelen = 0;
    iov[0].iov_base = send_buf;
    iov[0].iov_len = sizeof(send_buf)-1;
    iov[1].iov_base = send_buf2;
    iov[1].iov_len = sizeof(send_buf2)-1;
    msg.msg_iov = iov;
    msg.msg_iovlen = 2;
    int err = sendmsg(client, &msg, 0);
    cprintf("client msg sent, status code is %d \n", err);
    int st = sockclose(client);
    ftpinfo(1, "close socket ... %d", st);
    return 0;
}
