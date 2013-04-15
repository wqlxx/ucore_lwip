#include <stdlib.h>
#include <ulib.h>
#include <sockets.h>
#include <stdio.h>

#include "ftputils.h"
#include "ftplog.h"

int main(int argc, char * argv[]) {
    int server_port = 2121;
    if (argc < 2) {
        cprintf("usage: %s [port]\n", argv[0]);
        exit(0);
    } else {
        server_port = atoi(argv[1]);
    }
    int server = socket(AF_INET, SOCK_DGRAM, 0);
    if (server < 0) {
        ftperr(1, "cannot connect to %s %d", argv[1], server_port);
        ftpinfo(1, "exiting ...");
        exit(1);
    } else {
        ftpinfo(1, "UDP server at port %d", server_port);
    }
    struct sockaddr addr = new_addr(INADDR_ANY, server_port);
    if (bind(server, &addr, sizeof(struct sockaddr_in)) < 0) {
        ftperr(1, "bind error");
        return -1;
    }
    char buf[1024];
    struct sockaddr clientaddr;
    uint32_t size = sizeof(struct sockaddr_in);
    int count = recvfrom(server, buf, sizeof buf, 0, &clientaddr, &size);
    buf[count] = 0;
    cprintf("%s\n", buf);
    int st = sockclose(server);
    ftpinfo(1, "close socket ... %d", st);
    return 0;
}
