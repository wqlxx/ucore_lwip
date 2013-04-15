#ifndef __USER_LIBS_SYSCALL_H__
#define __USER_LIBS_SYSCALL_H__

#include <types.h>
#include <lwip/sockets.h>
#include <sockets.h>

int sys_exit(int error_code);
int sys_fork(void);
int sys_wait(int pid, int *store);
int sys_exec(const char *name, int argc, const char **argv);
int sys_yield(void);
int sys_sleep(unsigned int time);
int sys_kill(int pid);
size_t sys_gettime(void);
int sys_getpid(void);
int sys_brk(uintptr_t *brk_store);
int sys_mmap(uintptr_t *addr_store, size_t len, uint32_t mmap_flags);
int sys_munmap(uintptr_t addr, size_t len);
int sys_shmem(uintptr_t *addr_store, size_t len, uint32_t mmap_flags);
int sys_putc(int c);
int sys_pgdir(void);
sem_t sys_sem_init(int value);
int sys_sem_post(sem_t sem_id);
int sys_sem_wait(sem_t sem_id, unsigned int timeout);
int sys_sem_free(sem_t sem_id);
int sys_sem_get_value(sem_t sem_id, int *value_store);
int sys_send_event(int pid, int event, unsigned int timeout);
int sys_recv_event(int *pid_store, int *event_store, unsigned int timeout);

struct mboxbuf;
struct mboxinfo;

int sys_mbox_init(unsigned int max_slots);
int sys_mbox_send(int id, struct mboxbuf *buf, unsigned int timeout);
int sys_mbox_recv(int id, struct mboxbuf *buf, unsigned int timeout);
int sys_mbox_free(int id);
int sys_mbox_info(int id, struct mboxinfo *info);

struct stat;
struct dirent;

int sys_open(const char *path, uint32_t open_flags);
int sys_close(int fd);
int sys_read(int fd, void *base, size_t len);
int sys_write(int fd, void *base, size_t len);
int sys_seek(int fd, off_t pos, int whence);
int sys_fstat(int fd, struct stat *stat);
int sys_fsync(int fd);
int sys_chdir(const char *path);
int sys_getcwd(char *buffer, size_t len);
int sys_mkdir(const char *path);
int sys_link(const char *path1, const char *path2);
int sys_rename(const char *path1, const char *path2);
int sys_unlink(const char *path);
int sys_getdirentry(int fd, struct dirent *dirent);
int sys_dup(int fd1, int fd2);
int sys_pipe(int *fd_store);
int sys_mkfifo(const char *name, uint32_t open_flags);
// socket
int sys_accept(int s, struct sockaddr *addr, socklen_t *addrlen);
int sys_bind(int s, const struct sockaddr *name, socklen_t namelen);
int sys_shutdown(int s, int how);
int sys_getpeername (int s, struct sockaddr *name, socklen_t *namelen);
int sys_getsockname (int s, struct sockaddr *name, socklen_t *namelen);
int sys_getsockopt (int s, int level, int optname, void *optval, socklen_t *optlen);
int sys_setsockopt (int s, int level, int optname, const void *optval, socklen_t optlen);
int sys_sockclose(int s);
int sys_connect(int s, const struct sockaddr *name, socklen_t namelen);
int sys_listen(int s, int backlog);
int sys_recv(int s, void *mem, size_t len, int flags);
//int sys_read(int s, void *mem, size_t len);
int sys_recvfrom(int s, void *mem, size_t len, int flags,
      struct sockaddr *from, socklen_t *fromlen);
int sys_send(int s, const void *dataptr, size_t size, int flags);
int sys_sendto(int s, const void *dataptr, size_t size, int flags,
    const struct sockaddr *to, socklen_t tolen);
int sys_socket(int domain, int type, int protocol);
//int sys_write(int s, const void *dataptr, size_t size);
int sys_select(int maxfdp1, fd_set *readset, fd_set *writeset, fd_set *exceptset,
                struct timeval *timeout);
int sys_ioctl(int s, long cmd, void *argp);
int sys_sendmsg(int, struct msghdr*, int);
int sys_recvmsg(int, struct msghdr*, int);
int sys_socketpair(int, int, int, int *);

#endif /* !__USER_LIBS_SYSCALL_H__ */

