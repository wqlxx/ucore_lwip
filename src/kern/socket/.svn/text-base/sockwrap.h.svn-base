#ifndef __UCORE_KERN_SOCKWRAP_H_
#define __UCORE_KERN_SOCKWRAP_H_

#include <types.h>
#include <lwip/sockets.h>

int accept_wrap(int s, struct sockaddr *addr, socklen_t *addrlen);
int bind_wrap(int s, const struct sockaddr *name, socklen_t namelen);
int getpeername_wrap(int s, struct sockaddr *name, socklen_t *namelen);
int getsockname_wrap(int s, struct sockaddr *name, socklen_t *namelen);
//int lwip_getsockopt (int s, int level, int optname, void *optval, socklen_t *optlen);
int setsockopt_wrap (int s, int level, int optname, const void *optval, socklen_t optlen);
int connect_wrap(int s, const struct sockaddr *name, socklen_t namelen);
int recv_wrap(int s, void *mem, size_t len, int flags);
//int lwip_read(int s, void *mem, size_t len);
int recvfrom_wrap(int s, void *mem, size_t len, int flags, struct sockaddr *from, socklen_t *fromlen);
int send_wrap(int s, const void *dataptr, size_t size, int flags);
int sendto_wrap(int s, const void *dataptr, size_t size, int flags, const struct sockaddr *to, socklen_t tolen);
//int lwip_write(int s, const void *dataptr, size_t size);
//int lwip_select(int maxfdp1, fd_set *readset, fd_set *writeset, fd_set *exceptset, struct timeval *timeout);
//int lwip_ioctl(int s, long cmd, void *argp);
int sendmsg_wrap(int s, struct msghdr * hdr, int flag);
int recvmsg_wrap(int s, struct msghdr * hdr, int flag);
int socketpair_wrap(int domain, int type, int protocol, int socket_vector[]);


#endif // __UCORE_KERN_SOCKWRAP_H_
