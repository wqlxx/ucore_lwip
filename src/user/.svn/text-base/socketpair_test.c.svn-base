#include <stdio.h>
#include <sockets.h>
#include <ulib.h>

int main() {
	int sockets[2], child;
	char buf[1024];
    
    int err = socketpair(AF_INET, SOCK_STREAM, 0, sockets);
	if (err < 0) {
		cprintf("socketpair error, error code %d \n", err);
		return -1;
	} else {
        cprintf("socketpair created\n");
        cprintf("pair[0] = %d, pair[1] = %d\n", sockets[0], sockets[1]);
    }

    char DATA1[] = "msg from the child\n";
    char DATA2[] = "msg from the parent\n";

	if ((child = fork()) == -1) {
		cprintf("fork error");
	} else if (child) {	/* This is the parent. */
		if (recv(sockets[1], buf, 1024, O_NONBLOCK) < 0)
			cprintf("reading stream message\n");
		cprintf("-->%s\n", buf);
		if (send(sockets[1], DATA2, sizeof(DATA2)-1, O_NONBLOCK) < 0)
			cprintf("writing stream message\n");
		sockclose(sockets[1]);
	} else {		/* This is the child. */
		if (send(sockets[0], DATA1, sizeof(DATA1)-1, 0) < O_NONBLOCK)
			cprintf("writing stream message\n");
		if (recv(sockets[0], buf, 1024, O_NONBLOCK) < 0)
			cprintf("reading stream message\n");
		cprintf("-->%s\n", buf);
		sockclose(sockets[0]);
	}
    return 0;
}
