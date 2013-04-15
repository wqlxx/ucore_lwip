#include <ulib.h>
#include <sockets.h>
#include <stdio.h>

//#define DEBUG_OP

int main()
{
	unsigned char data[1024];
	int s;
	int len;
#ifdef DEBUG_OP
	cprintf("will socket()\n");
#endif
	s = socket(PF_INET, SOCK_STREAM, 0);
	struct sockaddr_in sa;
	sa.sin_family = AF_INET;
	sa.sin_port = htons(80);
	sa.sin_addr.s_addr = inet_addr("192.168.1.13");
	len = 1;
	cprintf("will setsockopt\n");
	setsockopt(s, SOL_SOCKET, SO_REUSEPORT, &len, sizeof(int));
	cprintf("will bind()\n");
	bind(s, (struct sockaddr *)&sa, sizeof(sa));
	int addrlen = sizeof(sa);
	//  len = lwip_recvfrom(s, data, sizeof(data), 0, &sa, &addrlen);
	//  lwip_sendto(s, data, len, 0, &sa, addrlen);
	int client;
	cprintf("will listen()\n");
	listen(s, 1);
	cprintf("will accept()\n");
	while ((client = accept(s, (struct sockaddr *)&sa, (socklen_t *)&addrlen)) > 0)
	{
		do {
#ifdef DEBUG_OP
			cprintf("will recv()\n");
#endif
			len = recv(client, data, sizeof(data), 0);
#ifdef DEBUG_OP
			cprintf("received %d bytes\n", len);
			cprintf("will send()\n");
#endif
			send(client, data, len, 0);
//			if (data[0] == '!')
//			   len = -1;
		} while (len > 0);
#ifdef DEBUG_OP
		cprintf("will sockclose()\n");
#endif
		sockclose(client);
#ifdef DEBUG_OP
		cprintf("will accept()\n");
#endif
	}
	sockclose(s);
	return 0;
}
