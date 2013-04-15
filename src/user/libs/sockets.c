#include <sockets.h>
#include <syscall.h>

/**
 * accept a new socket connection
 * @param(int) the server socket
 * @param(sockaddr*) the sockaddr to store the peer socket addr in
 * @param(int) the sockaddr length
 */
int accept(int s, struct sockaddr *addr, socklen_t *addrlen)
{
	return sys_accept(s, addr, addrlen);
}

/**
 * bind a name to the socket
 * @param(int) socket to be bound
 * @param(sockaddr*) the sockaddr to store the peer socket addr in
 * @param(int) the sockaddr length
 */
int bind(int s, const struct sockaddr *name, socklen_t namelen)
{
	return sys_bind(s, name, namelen);
}

/**
 * it's the same to sockclose
 * because lwip does not support half-open socket
 * @param(int) the socket to be closed
 */
int shutdown(int s, int how)
{
	return sys_shutdown(s, how);
}

/**
 * getpeername, get the peer socket information
 * @param(int) my socket
 * @param(sockaddr *) the sockaddr struct to store the info in
 * @param(int) sockaddr struct length
 */
int getpeername(int s, struct sockaddr *name, socklen_t *namelen)
{
	return sys_getpeername(s, name, namelen);
}

/**
 * getsockname, get the my socket information
 * @param(int) my socket
 * @param(sockaddr *) the sockaddr struct to store the info in
 * @param(int) sockaddr struct length
 */
int getsockname(int s, struct sockaddr *name, socklen_t *namelen)
{
	return sys_getsockname(s, name, namelen);
}

// not supported
int getsockopt(int s, int level, int optname, void *optval, socklen_t *optlen)
{
	return sys_getsockopt(s, level, optname, optval, optlen);
}

/**
 * set socket option
 * @param(int) socket
 * other param is just like linux 
 */
int setsockopt(int s, int level, int optname, const void *optval, socklen_t optlen)
{
	return sys_setsockopt(s, level, optname, optval, optlen);
}

/**
 * close the socket
 * @param(int) the socket to be closed
 */
int sockclose(int s)
{
	return sys_sockclose(s);
}

/**
 * connect, connect to the peer socket
 * @param(int) my socket
 * @param(sockaddr*) peer socket address
 * @param(int) address length
 */
int connect(int s, const struct sockaddr *name, socklen_t namelen)
{
	return sys_connect(s, name, namelen);
}

/**
 * listen, listen for connected socket
 * @param(int) socket, the peer socket
 * @param(int) max connection number
 */
int listen(int s, int backlog)
{
	return sys_listen(s, backlog);
}

/**
 * recv, recv msg from the peer, only for conected socket
 * @param(int) the peer socket
 * @param(void *) the pointer to the data buffer
 * @param(int) data size of the buffer
 * @param(int) flags for the function
 */
int recv(int s, void *mem, size_t len, int flags)
{
	return sys_recv(s, mem, len, flags);
}

/*int sys_read(int s, void *mem, size_t len)
{
	return sys_sys_read(s, mem, len);
}*/

/**
 * recvfrom, recv msg from the peer
 * @param(int) socket, the peer socket
 * @param(void *) the pointer to the data to be stored
 * @param(int) the data size of the buffer
 * @param(int) flag for the function
 * @param(sockaddr*) which contains the socket address of the socket to be recved
 */
int recvfrom(int s, void *mem, size_t len, int flags, struct sockaddr *from, socklen_t *fromlen)
{
	return sys_recvfrom(s, mem, len, flags, from, fromlen);
}

/**
 * send, send msg to the peer, only for conected socket
 * @param(int) the peer socket
 * @param(void *) the pointer to the data to be sent
 * @param(int) data size to be sent
 * @param(int) flags for the function
 */
int send(int s, const void *dataptr, size_t size, int flags)
{
	return sys_send(s, dataptr, size, flags);
}

/**
 * sendto, send msg to the peer
 * @param(int) socket, the peer socket
 * @param(void *) the pointer to the data to be sent
 * @param(int) the data size to be sent
 * @param(int) flag for the function
 * @param(sockaddr*) which contains the socket address of the socket 
 */
int sendto(int s, const void *dataptr, size_t size, int flags, const struct sockaddr *to, socklen_t tolen)
{
	return sys_sendto(s, dataptr, size, flags, to, tolen);
}

/**
 * socket, create a socket
 * @param(int) domain, domain for the socket
 * @param(int) type, type for the socket
 * @param(int) protocol, protocol for the socket
 */
int socket(int domain, int type, int protocol)
{
	return sys_socket(domain, type, protocol);
}

/**
 * sendmsg, send msghdr struct to peer socket
 * @param(int) socket, the peer socket
 * @param(msghdr) the msghdr struct which stores the data to be sent
 * @param(flag) the flag for sendmsg
 */
int sendmsg(int socket, struct msghdr *hdr, int flag) {
    return sys_sendmsg(socket, hdr, flag);
}

/**
 * recvmsg, recv msghdr struct from peer socket
 * @param(int) socket, the peer socket
 * @param(msghdr) the msghdr struct to store the recved data to
 * @param(flag) the flag for recvmsg
 */
int recvmsg(int socket, struct msghdr *hdr, int flag) {
    return sys_recvmsg(socket, hdr, flag);
}

/**
 * create a connected socket pair and stores them to pair
 * currently the function is not available
 * @param(int) domain, domain for socket
 * @param(int) type, type for socket
 * @param(int) protocol, must be 0
 * @param(int[]) the pair array to contain the created socketpair
 */
int socketpair(int domain, int type, int protocol, int pair[]) {
    return sys_socketpair(domain, type, protocol, pair);
}

/*int sys_write(int s, const void *dataptr, size_t size)
{
	return sys_sys_write(s, dataptr, size);
}*/

int select(int maxfdp1, fd_set *readset, fd_set *writeset, fd_set *exceptset, struct timeval *timeout)
{
	return sys_select(maxfdp1, readset, writeset, exceptset, timeout);
}

int ioctl(int s, long cmd, void *argp)
{
	return sys_ioctl(s, cmd, argp);
}


#include <inet.c>
