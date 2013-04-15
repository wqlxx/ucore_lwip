#include <sockwrap.h>
#include <error.h>
#include <proc.h>
#include <vmm.h>
#include <slab.h>
#include <string.h>

// the file is a layer between the network syscall and the LWIP function call
// for those functions who has pointer-type parameters, the actual data must
// be copied to the kernel space before invoking the lwip calls.
// That is because lwip functions will leave the function to be executed to 
// the tcpip thread, at that time, the pointer is no longer valid.

int accept_wrap(int s, struct sockaddr *addr, socklen_t *addrlen)
{
    if (addr == NULL || addrlen == NULL)
        return -E_INVAL;

    socklen_t k_addrlen;

    struct mm_struct *mm = current->mm;

    int ret = 1;
    // copy the data from user space to the kernel space safely
    lock_mm(mm);
    {
        if (copy_from_user(mm, &k_addrlen, addrlen, sizeof(socklen_t), 1))
            ret = 0;
    }
    unlock_mm(mm);

    if (ret)
        return -E_INVAL;

    if (!user_mem_check(mm, (uintptr_t)addr, k_addrlen, 1))
        return -E_INVAL;

    char *k_addr = (char *)kmalloc(k_addrlen);

    lock_mm(mm);
    {
        copy_from_user(mm, k_addr, addr, k_addrlen, 1);
    }
    unlock_mm(mm);

    int retval = lwip_accept(s, (struct sockaddr *)k_addr, &k_addrlen);

    lock_mm(mm);
    {
        copy_to_user(mm, addr, k_addr, k_addrlen);
        copy_to_user(mm, addrlen, &k_addrlen, sizeof(socklen_t));
    }
    unlock_mm(mm);
    kfree(k_addr);

    return retval;
}

int bind_wrap(int s, const struct sockaddr *name, socklen_t namelen)
{
    if (namelen <= 0)
        return -E_INVAL;

    struct mm_struct *mm = current->mm;
    if (!user_mem_check(mm, (uintptr_t)name, namelen, 0))
        return -E_INVAL;

    char *k_name = (char *)kmalloc(namelen);
    lock_mm(mm);
    {
        copy_from_user(mm, k_name, name, namelen, 0);
    }
    unlock_mm(mm);

    int retval = lwip_bind(s, (const struct sockaddr *)k_name, namelen);
    kfree(k_name);
    return retval;
}

int getpeername_wrap(int s, struct sockaddr *name, socklen_t *namelen) {
    if (name == NULL || namelen == NULL) {
        return -E_INVAL;
    }
    // check for restrict param
    if ((void *)name == (void *)namelen) {
        return -E_INVAL;
    }
    struct mm_struct * mm = current->mm;
    socklen_t k_namelen;
    int ok = 0;
    lock_mm(mm);
    ok = copy_from_user(mm, &k_namelen, namelen, sizeof(socklen_t), 1);
    unlock_mm(mm);
    if (!ok) {
        return -E_INVAL;
    }
    if (!user_mem_check(mm, (uintptr_t)name, k_namelen, 1)) {
        return -E_INVAL;
    }
    struct sockaddr * k_name = (struct sockaddr *)kmalloc(k_namelen);
    int retval = lwip_getpeername(s, k_name, &k_namelen);
    lock_mm(mm);
    copy_to_user(mm, name, k_name, k_namelen);
    copy_to_user(mm, namelen, &k_namelen, sizeof(socklen_t));
    unlock_mm(mm);
    kfree(k_name);
    return retval;
}

int getsockname_wrap(int s, struct sockaddr *name, socklen_t *namelen) {
    if (name == NULL || namelen == NULL) {
        return -E_INVAL;
    }
    // check for restrict param
    if ((void *)name == (void *)namelen) {
        return -E_INVAL;
    }
    struct mm_struct * mm = current->mm;
    socklen_t k_namelen;
    int ok = 0;
    lock_mm(mm);
    ok = copy_from_user(mm, &k_namelen, namelen, sizeof(socklen_t), 1);
    unlock_mm(mm);
    if (!ok) {
        return -E_INVAL;
    }
    if (!user_mem_check(mm, (uintptr_t)name, k_namelen, 1)) {
        return -E_INVAL;
    }
    struct sockaddr * k_name = (struct sockaddr *)kmalloc(k_namelen);
    int retval = lwip_getsockname(s, k_name, &k_namelen);
    lock_mm(mm);
    copy_to_user(mm, name, k_name, k_namelen);
    copy_to_user(mm, namelen, &k_namelen, sizeof(socklen_t));
    unlock_mm(mm);
    kfree(k_name);
    return retval;
}

//int lwip_getsockopt (int s, int level, int optname, void *optval, socklen_t *optlen);
int setsockopt_wrap (int s, int level, int optname, const void *optval, socklen_t optlen)
{
    if (optlen <= 0)
        return -E_INVAL;

    struct mm_struct *mm = current->mm;
    if (!user_mem_check(mm, (uintptr_t)optval, optlen, 0))
        return -E_INVAL;

    char *k_optval = (char *)kmalloc(optlen);
    lock_mm(mm);
    {
        copy_from_user(mm, k_optval, optval, optlen, 0);
    }
    unlock_mm(mm);

    int retval = lwip_setsockopt(s, level, optname, k_optval, optlen);
    kfree(k_optval);
    return retval;
}

int connect_wrap(int s, const struct sockaddr *name, socklen_t namelen) {
    if (name == NULL || namelen == 0) {
        return -E_INVAL;
    }
    struct mm_struct *mm = current->mm;
    if (!user_mem_check(mm, (uintptr_t)name, namelen, 0)) {
        return -E_INVAL;
    }
    struct sockaddr *k_name = (struct sockaddr *)kmalloc(namelen);
    lock_mm(mm);
    copy_from_user(mm, k_name, name, namelen, 0);
    unlock_mm(mm);

    int ret = lwip_connect(s, k_name, namelen);
    kfree(k_name);
    return ret;
}

int recv_wrap(int s, void *mem, size_t len, int flags)
{
    if (len <= 0)
        return -E_INVAL;

    struct mm_struct *mm = current->mm;
    if (!user_mem_check(mm, (uintptr_t)mem, len, 1))
        return -E_INVAL;

    char *k_mem = (char *)kmalloc(len);
    // recv: no need to copy from user
    
    int retval = lwip_recv(s, k_mem, len, flags);
    lock_mm(mm);
    {
        copy_to_user(mm, mem, k_mem, len);
    }
    unlock_mm(mm);
    kfree(k_mem);
    return retval;
}

//int lwip_read(int s, void *mem, size_t len);

int recvfrom_wrap(int s, void *mem, size_t len, int flags, struct sockaddr *from, socklen_t *fromlen) {
    if (len <= 0) {
        return -E_INVAL;
    }
    if (mem == NULL || from == NULL || fromlen == NULL) {
        return -E_INVAL;
    }
    // check for restrict paramter
    if ((void *)mem == (void *)from || (void *)mem == (void *)fromlen || (void *)from == (void *)fromlen) {
        return -E_INVAL;
    }
    struct mm_struct *mm = current->mm;
    socklen_t k_fromlen;
    int ok = 0;
    lock_mm(mm);
    {
        ok = copy_from_user(mm, &k_fromlen, fromlen, sizeof(socklen_t), 1);
    }
    unlock_mm(mm);
    if (!ok) {
        return -E_INVAL;
    }
    if (!user_mem_check(mm, (uintptr_t)mem, len, 1) || 
        !user_mem_check(mm, (uintptr_t)from, k_fromlen, 1) ) {
        return -E_INVAL;
    }
    char * k_mem = (char *)kmalloc(len);
    struct sockaddr * k_from = (struct sockaddr *)kmalloc(k_fromlen);
    int retval = lwip_recvfrom(s, k_mem, len, flags, k_from, &k_fromlen);
    lock_mm(mm);
    {
        copy_to_user(mm, mem, k_mem, len);
        copy_to_user(mm, from, k_from, k_fromlen);
        copy_to_user(mm, fromlen, &k_fromlen, sizeof(socklen_t));
    }
    unlock_mm(mm);
    kfree(k_mem);
    kfree(k_from);
    return retval;
}

int send_wrap(int s, const void *dataptr, size_t size, int flags)
{
    if (size <= 0)
        return -E_INVAL;

    struct mm_struct *mm = current->mm;
    if (!user_mem_check(mm, (uintptr_t)dataptr, size, 0))
        return -E_INVAL;

    char *k_dataptr = (char *)kmalloc(size);
    lock_mm(mm);
    {
        copy_from_user(mm, k_dataptr, dataptr, size, 0);
    }
    unlock_mm(mm);

    int retval = lwip_send(s, k_dataptr, size, flags);
    kfree(k_dataptr);

    return retval;
}

int sendto_wrap(int s, const void *dataptr, size_t size, int flags, const struct sockaddr *to, socklen_t tolen) {
    if (size <= 0) {
        return -E_INVAL;
    }
    struct mm_struct *mm = current->mm;
    if (!user_mem_check(mm, (uintptr_t)dataptr, size, 0)) {
        return -E_INVAL;
    }
    char * k_dataptr = (char *)kmalloc(size);
    lock_mm(mm);
    {
        copy_from_user(mm, k_dataptr, dataptr, size, 0);
    }
    unlock_mm(mm);
    struct sockaddr * k_to = (struct sockaddr *)kmalloc(tolen);
    lock_mm(mm);
    {
        copy_from_user(mm, k_to, to, tolen, 0);
    }
    unlock_mm(mm);

    int retval = lwip_sendto(s, k_dataptr, size, flags, k_to, tolen);
    kfree(k_dataptr);
    kfree(k_to);

    return retval;
}
//int lwip_write(int s, const void *dataptr, size_t size);
//int lwip_select(int maxfdp1, fd_set *readset, fd_set *writeset, fd_set *exceptset, struct timeval *timeout);
//int lwip_ioctl(int s, long cmd, void *argp);

int sendmsg_wrap(int s, struct msghdr * hdr, int flag) {
    char *buf, *p;
    uint32_t i = 0, total = 0;
    struct iovec *iov = hdr->msg_iov;
    for (i = 0; i < hdr->msg_iovlen; ++i) {
        total += iov[i].iov_len;
    }
    buf = (char *)kmalloc(total);
    p = buf;

    struct mm_struct *mm = current->mm;

    struct sockaddr * k_to = NULL;
    if (hdr->msg_namelen > 0) {
        k_to = (struct sockaddr *)kmalloc(hdr->msg_namelen);
        lock_mm(mm);
            copy_from_user(mm, k_to, hdr->msg_name, hdr->msg_namelen, 0);
        unlock_mm(mm);
    }

    lock_mm(mm);
        for (i = 0; i < hdr->msg_iovlen; ++i) {
            copy_from_user(mm, p, iov[i].iov_base, iov[i].iov_len, 0);
            p += iov[i].iov_len;
        }
    unlock_mm(mm);

    int ret = lwip_sendto(s, buf, total, flag, k_to, hdr->msg_namelen);
    kfree(buf);
    return ret;
}

int recvmsg_wrap(int s, struct msghdr * hdr, int flag) {
    char *buf, *p;
    struct iovec *iov = hdr->msg_iov;
    uint32_t i = 0, total = 0;
    for (i = 0; i < hdr->msg_iovlen; i++) {
        total += iov[i].iov_len;
    }
    if (total == 0) {
        return 0;
    }
    buf = (char *)kmalloc(total);

    uint32_t k_namelen = hdr->msg_namelen;
    struct sockaddr * k_from = NULL;
    if (k_namelen > 0) {
        k_from = (struct sockaddr *)kmalloc(k_namelen);
    }

    int ret, nb;
    nb = ret = lwip_recvfrom(s, buf, total, flag, k_from, &k_namelen);
    p = buf;

    struct mm_struct *mm = current->mm;
    lock_mm(mm);
    {
        while(nb > 0) {
            uint32_t cnt = nb < iov->iov_len ? nb : iov->iov_len;
            copy_to_user(mm, iov->iov_base, p, cnt);
            p += cnt;
            nb -= cnt;
            ++iov;
        }
    }
    unlock_mm(mm);

    kfree(buf);
    return ret;
}

int socketpair_wrap(int domain, int type, int protocol, int pair[]) {
    // server
    int server = lwip_socket(domain, type, protocol);

    struct sockaddr_in serv_addr, cli_addr;
    memset(&serv_addr, 0, sizeof(serv_addr));
    serv_addr.sin_family = domain;
    serv_addr.sin_addr.s_addr = htonl(INADDR_ANY);
    serv_addr.sin_port = htons(100);
    uint32_t addr_size = sizeof(serv_addr);
    if (lwip_bind(server, (struct sockaddr*)&serv_addr, addr_size) < 0) {
        lwip_close(server);
        return -1;
    }
    lwip_listen(server, 1);

    pair[0] = lwip_socket(domain, type, protocol);
    if (pair[0] <= 0) {
        lwip_close(server);
        return -2;
    }
    memset(&cli_addr, 0, sizeof(cli_addr));
    cli_addr.sin_family = domain;
    cli_addr.sin_addr.s_addr = INADDR_LOOPBACK;
    cli_addr.sin_port = serv_addr.sin_port;

    struct trapframe *tf = current->tf;
    uintptr_t stack = tf->tf_esp;

    int pid = do_fork(0, stack, tf);
    cprintf("sockwrap pid = %u\n", pid);
    if (pid == 0) { // child
        cprintf("sockwrap enter child");
        if (lwip_connect(pair[0], (struct sockaddr*)&cli_addr, sizeof(cli_addr)) < 0) {
            lwip_close(server);
            lwip_close(pair[0]);
            return -3;
        }
        cprintf("sockwrap pair[0] = %d\n", pair[0]);
        do_exit(0);
    } else {
        cprintf("sockwrap enter parent");
        pair[1] = lwip_accept(server, (struct sockaddr *)&serv_addr, &addr_size);
        cprintf("sockwrap pair[1] = %d\n", pair[1]);
        if (pair[1] <= 0) {
            lwip_close(server);
            lwip_close(pair[0]);
            return -4;
        }
        //lwip_close(server);
    }
    return 0;
}


