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

#define BUF_SIZE 1024
int server = -1;
int client = -1;

void ouch(int i) {
    int st;
    if (server >= 0) {
        st = close(server);
        cprintf("server closed ... %d\n", st);
    }
    if (client >= 0) {
        st = close(client);
        cprintf("client closed ... %d\n", st);
    }
}

int main(int argc, char *argv[]) {
    if (argc < 2) {
        cprintf("usage: ./print_server <port>\n");
        exit(0);
    }
    //signal(SIGINT, ouch);
    //signal(SIGTERM, ouch);
    int port = atoi(argv[1]);
    struct sockaddr_in client_addr;
    uint32_t len = sizeof(client_addr);
    server = new_server(INADDR_ANY, port, 1);
    char buf[1024];
    int n;
    client = accept(server, (struct sockaddr *)&client_addr, &len);
    cprintf("[ II ] client accpeted, addr is %s: %u \n", inet_ntoa(*(struct in_addr *)&client_addr.sin_addr.s_addr), ntohs(client_addr.sin_port));
    cprintf("[ II ] test getsockname ....\n");
    uint32_t size = sizeof(struct sockaddr_in) + 2;
    struct sockaddr_in * test_server_addr = (struct sockaddr_in *)malloc(size);
    getsockname(server, (struct sockaddr *)test_server_addr, &size);
    cprintf("result: server sock len = %u, family = %u, port = %u, addr = %u\n",
            test_server_addr->sin_len,
            test_server_addr->sin_family,
            ntohs(test_server_addr->sin_port),
            ntohl(test_server_addr->sin_addr.s_addr));

    struct sockaddr_in * test_client_addr = (struct sockaddr_in *)malloc(size);
    cprintf("\n[ II ] test getpeername ....\n");
    getpeername(client, (struct sockaddr *)test_client_addr, &size);
    cprintf("result: client sock len = %u, family = %u, port = %u, addr = %s\n",
            test_client_addr->sin_len,
            test_client_addr->sin_family,
            ntohs(test_client_addr->sin_port),
            inet_ntoa(test_client_addr->sin_addr));
    while ((n = recv(client, buf, BUF_SIZE, 0)) > 0) {
        buf[n] = 0;
        cprintf("%s",buf);
    }
    cprintf("[ II ] connection closed\n");
    return 0;
}
