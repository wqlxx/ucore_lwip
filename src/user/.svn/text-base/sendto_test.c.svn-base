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
    int client = socket(AF_INET, SOCK_DGRAM, 0);
    if (client < 0) {
        ftperr(1, "cannot connect to %s %d", argv[1], server_port);
        ftpinfo(1, "exiting ...");
        exit(1);
    }
    struct sockaddr addr = new_addr(ntohl(inet_addr(argv[1])), server_port);
    char buf[] = "sendto test\n";
    uint32_t size = sizeof(struct sockaddr_in);
    cprintf("sendto addr size %u\n", size);
    sendto(client, buf, sizeof(buf)-1, 0, &addr, size);
    int st = sockclose(client);
    ftpinfo(1, "close socket ... %d", st);
    return 0;
}
